//==============================================================================
//  tb_icon_rom.v —— osd_icon_rom（位图 ROM）专项验证
//==============================================================================
//
//  【2026-10-28 改版】ROM 从「2560 B / 4bpp 覆盖率」改成「25728 B / RGB565 直出」，
//  图标 10 张（5×2 态）+ logo 64×41 在**同一个分区里一次搬完**。本台跟着改：
//  地址宽度 10 → 13、NBYTES 2560 → 25728、抽查点改成"图标区 / logo 区"各一处。
//
//  【要验什么】
//    ① 字节拼装方向：`shf <= {fl_data, shf[31:8]}` 必须让**文件里的第一个
//       字节落在字的最低字节**（低字节 = 先收到的字节）。这是 BRAM 写口
//       最容易搞反的地方，反了就是"图标和 logo 的花纹整体错位"。
//    ② `data_pgm` 未编程检测：Flash 那片区域是擦除态（全 0xFF）时
//       `ready` 必须为 0 —— 否则 0xFFFF 的 16bpp 会渲染成**纯白块**。
//    ③ 真实 osd_bitmaps.bin 能被完整搬进 RAM（用 $fgetc 逐字节喂）。
//    ④ 地址映射：word0 = icon0 普通态左上角、word512 = icon0 激活态左上角、
//       word5120 = logo 左上角、word6431 = logo 右下角。
//
//  【怎么跑】
//    cd src/user_source/hdl_source/APP/OSD
//    iverilog -g2005 -o /tmp/tb_icon.vvp tb_icon_rom.v osd_icon_rom.v
//    vvp /tmp/tb_icon.vvp
//
//==============================================================================
`timescale 1ns / 1ps

module tb_icon_rom;

localparam integer NBYTES = 30976;
localparam integer NWORD  = NBYTES / 4;      // 7744

reg clk = 0, rst = 1;
always #10 clk = ~clk;

//------------------------------------------------------------------------------
//  DUT 端口
//------------------------------------------------------------------------------
wire        fl_req;
wire [23:0] fl_addr;
wire [15:0] fl_len;
reg         fl_rdy   = 1;
reg  [7:0]  fl_data  = 0;
reg         fl_valid = 0;
reg         fl_done  = 0;

reg  [12:0] ra_addr = 0;
wire [31:0] ra_data;
wire        ready;

osd_icon_rom #(
    .BASE       (24'h050000),
    .NBYTES     (NBYTES),
    .POR_CYCLES (40)          // 仿真：40 拍就够，不用真等 8ms
) u_dut (
    .clk      (clk),
    .rst      (rst),
    .fl_req   (fl_req),
    .fl_addr  (fl_addr),
    .fl_len   (fl_len),
    .fl_rdy   (fl_rdy),
    .fl_data  (fl_data),
    .fl_valid (fl_valid),
    .fl_done  (fl_done),
    .ra_addr  (ra_addr),
    .ra_data  (ra_data),
    .ce      (1'b1),          // 单测恒推进
    .ready    (ready)
);

//------------------------------------------------------------------------------
//  Flash 内容模型
//------------------------------------------------------------------------------
reg [7:0] flash [0:NBYTES-1];

// `fl_req` 是 DUT 的寄存器输出（单拍），这里靠 "看到 req 就进接收态" 响应
reg        rx_act  = 0;
reg [15:0] rx_left = 0;
reg [15:0] rx_tot  = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fl_valid <= 1'b0;
        fl_done  <= 1'b0;
        rx_act   <= 1'b0;
        rx_left  <= 16'd0;
        rx_tot   <= 16'd0;
    end else begin
        fl_valid <= 1'b0;
        fl_done  <= 1'b0;
        if (fl_req && !rx_act) begin
            rx_tot  <= fl_len;
            rx_left <= fl_len;
            rx_act  <= 1'b1;
        end else if (rx_act && (rx_left != 16'd0)) begin
            fl_valid <= 1'b1;
            fl_data  <= flash[rx_tot - rx_left];
            rx_left  <= rx_left - 16'd1;
            if (rx_left == 16'd1) begin
                fl_done <= 1'b1;              // 与最后一个 valid 同拍
                rx_act  <= 1'b0;
            end
        end
    end
end

//------------------------------------------------------------------------------
//  检查工具
//------------------------------------------------------------------------------
integer errors = 0;
integer i;

task chk;
    input [8*32-1:0] name;
    input integer got;
    input integer exp;
    begin
        if (got === exp)
            $display("   ok       %0s = %0d", name, got);
        else begin
            $display("  *** FAIL  %0s = %0d (exp %0d)", name, got, exp);
            errors = errors + 1;
        end
    end
endtask

// 读一个字（拍 1 送地址，拍 2 出数据）
task rd_word;
    input  [12:0] a;
    output [31:0] d;
    begin
        @(negedge clk);
        ra_addr = a;
        @(posedge clk);          // ra_q <= mem[ra_addr]
        @(negedge clk);
        d = ra_data;
    end
endtask

reg [31:0] wdata;
reg [31:0] wexp;

// 逐字比对（期望 = 文件里 4 个字节按"低字节在前"拼）
task chk_word;
    input [8*28-1:0] name;
    input [12:0]     a;
    begin
        rd_word(a, wdata);
        wexp = {flash[a*4+3], flash[a*4+2], flash[a*4+1], flash[a*4]};
        if (wdata === wexp) $display("   ok       %0s [%0d] = %08h", name, a, wdata);
        else begin
            $display("  *** FAIL  %0s [%0d] = %08h (exp %08h)", name, a, wdata, wexp);
            errors = errors + 1;
        end
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    //------------------------------------------------------------------
    //  T1：未编程 Flash（全 0xFF）⇒ ready 必须为 0
    //------------------------------------------------------------------
    $display("========================================");
    $display("=== T1：Flash 全 0xFF（未烧 osd_bitmaps）===");
    for (i = 0; i < NBYTES; i = i + 1) flash[i] = 8'hFF;

    rst = 1;
    repeat (4) @(negedge clk);
    rst = 0;

    repeat (NBYTES + 200) @(negedge clk);
    chk("T1 ready==0", ready, 0);
    // 虽然 ready=0，但数据确实搬进去了（BRAM 里全是 0xFF ⇒ 16bpp 纯白）
    rd_word(13'd0, wdata);
    chk("T1 mem[0]==FFFFFFFF", (wdata === 32'hFFFF_FFFF), 1);

    //------------------------------------------------------------------
    //  T2：可预测 pattern ⇒ 验证字节拼装方向 + 首尾字
    //------------------------------------------------------------------
    $display("========================================");
    $display("=== T2：flash[i] = i&0xFF ⇒ 验证拼装方向 ===");
    for (i = 0; i < NBYTES; i = i + 1) flash[i] = i[7:0];

    rst = 1;
    repeat (4) @(negedge clk);
    rst = 0;
    repeat (NBYTES + 200) @(negedge clk);
    chk("T2 ready==1", ready, 1);
    $display("    [info] fl_len=%0d fl_addr=%06h", fl_len, fl_addr);
    chk("T2 fl_len",  fl_len,  NBYTES);
    chk("T2 fl_addr", fl_addr, 24'h050000);

    chk_word("T2 mem", 13'd0);
    chk_word("T2 mem", 13'd1);
    chk_word("T2 mem", 13'd5119);       // 图标区最后一字
    chk_word("T2 mem", 13'd5120);       // logo 首字
    chk_word("T2 mem", 13'd7743);       // logo 末字

    //------------------------------------------------------------------
    //  T3：真实 osd_bitmaps.bin ⇒ 四个关键点
    //------------------------------------------------------------------
    $display("========================================");
    $display("=== T3：真实 osd_bitmaps.bin ===");
    begin : t3_load
        integer fd, r, k;
        fd = $fopen("../../../../../icons/osd_bitmaps.bin", "rb");
        if (fd == 0) begin
            $display("   skip     T3（找不到 icons/osd_bitmaps.bin）");
        end else begin
            for (k = 0; k < NBYTES; k = k + 1) begin
                r = $fgetc(fd);
                flash[k] = (r < 0) ? 8'hFF : r[7:0];
            end
            $fclose(fd);

            rst = 1;
            repeat (4) @(negedge clk);
            rst = 0;
            repeat (NBYTES + 200) @(negedge clk);
            chk("T3 ready", ready, 1);

            chk_word("T3 icon0-nor",  13'd0);
            chk_word("T3 icon0-act",  13'd512);    // 激活态起点
            chk_word("T3 icon4-act",  13'd5119);   // 最后一个图标末字
            chk_word("T3 logo-origin",13'd5120);
            chk_word("T3 logo-last",  13'd7743);

            // 语义抽查：icon0 普通态左上角应是底板色（不是纯黑、也不是白）
            rd_word(13'd0, wdata);
            $display("    [info] icon0(0,0) 565 = %04h / %04h", wdata[15:0], wdata[31:16]);
            chk("T3 icon plate not black", (wdata[15:0] != 16'h0000), 1);
            chk("T3 icon plate not white", (wdata[15:0] != 16'hFFFF), 1);
            // logo 左上角必须 = A192 + #0E0E0E（和左栏面板逐位相同 ⇒ 看不出框）
            rd_word(13'd5120, wdata);
            if (wdata === 32'hC00E_0E0E) $display("   ok       T3 logo(0,0) = A192 #0E0E0E");
            else begin
                $display("  *** FAIL  T3 logo(0,0) = %08h (exp C00E0E0E)", wdata);
                errors = errors + 1;
            end
        end
    end

    $display("========================================");
    if (errors == 0) $display("=== tb_icon_rom ALL PASS ===");
    else             $display("=== tb_icon_rom %0d FAIL ===", errors);
    $finish;
end

endmodule
