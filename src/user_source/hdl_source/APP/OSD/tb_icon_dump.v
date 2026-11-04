`timescale 1ns/1ps
//==============================================================================
//  tb_icon_dump.v —— 把 osd_icon_rom 的全部字导出来（不是断言台，是"取证台"）
//==============================================================================
//  【为什么需要】
//    osd_icon_rom 的存储在 2026-10-28 从「行为 RAM + 推断」改成了
//    「8 个显式 EG_PHY_BRAM32K（4 个深度 bank × 低/高 16 位）」。
//    这种改法引入了一整套**手写的地址译码**：
//        bank    = addr[12:11]
//        bank 内 = addr[10:0]
//        写使能  = we_p && (wa_p[12:11] == bank)
//        读输出  = mux4(dob[b], ra_bank_q)
//    抽样断言（首字/尾字/几个点）**抓不到地址别名** ——
//    如果 bank 译码错，只有 1/4 的字会错，抽样很可能正好躲开。
//
//  【用法】跑两遍，diff：
//      # 参考：行为 RAM 分支
//      iverilog -g2012 -DIVERILOG_SIM -o a.vvp -s tb_icon_dump tb_icon_dump.v \
//          osd_icon_rom.v -o a.vvp
//      vvp a.vvp > dump_sim.txt
//      # 被测：综合分支（显式原语）+ 行为桩
//      sed 's/ifdef IVERILOG_SIM/ifdef FORCE_SYNTH_RAM/' osd_icon_rom.v > /tmp/icon_synth.v
//      iverilog -g2012 -DFORCE_SYNTH_RAM -o b.vvp -s tb_icon_dump tb_icon_dump.v \
//          /tmp/icon_synth.v tools/bram32k_stub.v
//      vvp b.vvp > dump_synth.txt
//      diff dump_sim.txt dump_synth.txt && echo IDENTICAL
//
//  输出格式：每行 `<十进制地址> <8 位十六进制数据>`，便于 diff。
//==============================================================================
module tb_icon_dump;

localparam integer NBYTES = 30976;
localparam integer NWORD  = NBYTES / 4;      // 7744

reg clk = 0, rst = 1;
always #10 clk = ~clk;

wire        fl_req;
wire [23:0] fl_addr;
wire [15:0] fl_len;
reg         fl_rdy   = 1;
reg  [7:0]  fl_data  = 0;
reg         fl_valid = 0;
reg         fl_done  = 0;

reg  [12:0] ra_addr = 0;
reg         ce      = 1'b1;
wire [31:0] ra_data;
wire        ready;

osd_icon_rom #(
    .BASE       (24'h050000),
    .NBYTES     (NBYTES),
    .POR_CYCLES (40)
) u_dut (
    .clk(clk), .rst(rst),
    .fl_req(fl_req), .fl_addr(fl_addr), .fl_len(fl_len),
    .fl_rdy(fl_rdy), .fl_data(fl_data), .fl_valid(fl_valid), .fl_done(fl_done),
    .ra_addr(ra_addr), .ra_data(ra_data),
    .ce     (ce),
    .ready  (ready)
);

//------------------------------------------------------------------------------
//  Flash 内容模型（真实 osd_bitmaps.bin）
//------------------------------------------------------------------------------
reg [7:0] flash [0:NBYTES-1];

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
                fl_done <= 1'b1;
                rx_act  <= 1'b0;
            end
        end
    end
end

//------------------------------------------------------------------------------
integer fd, r, k, a, n_err = 0;
reg [31:0] hold_v;
initial begin
    for (k = 0; k < NBYTES; k = k + 1) flash[k] = 8'hFF;
    fd = $fopen("../../../../../icons/osd_bitmaps.bin", "rb");
    if (fd == 0) begin
        $display("FATAL: 找不到 icons/osd_bitmaps.bin");
        $finish;
    end
    for (k = 0; k < NBYTES; k = k + 1) begin
        r = $fgetc(fd);
        flash[k] = (r < 0) ? 8'hFF : r[7:0];
    end
    $fclose(fd);

    rst = 1;
    repeat (4) @(negedge clk);
    rst = 0;
    repeat (NBYTES + 300) @(negedge clk);
    if (ready !== 1'b1) begin
        $display("FATAL: ready 没起来（装载失败）");
        $finish;
    end

    // ---- T4：读口停顿冻结（ce=0 时换地址，数据必须不变）----
    //  【为什么加这一项】所有测试台的 pixel_ready 恒为 1，**从来没人验证过
    //    "背压停顿时读口会不会跟着往前走"**。上板那两条竖直虚线就是这个问题：
    //    ce=0 那拍 x/y 已前进，BRAM 输出寄存器没有 valid，dob 多走一格
    //    ⇒ 坐标 N 配位图 N+1。
    ce = 1'b1; ra_addr = 13'd100;
    repeat (3) @(negedge clk);
    hold_v = ra_data;
    ce = 1'b0;
    for (a = 0; a < 8; a = a + 1) begin
        ra_addr = 13'd3000 + a[12:0] * 13'd97;   // 停顿时乱换地址
        @(negedge clk);
        if (ra_data !== hold_v) begin
            n_err = n_err + 1;
            $display("  *** FAIL  T4 停顿冻结：ce=0 时数据变了  addr=%0d got=%08x exp=%08x",
                     ra_addr, ra_data, hold_v);
        end
    end
    ce = 1'b1;
    @(negedge clk);
    if (ra_data === hold_v)
        $display("   ok       T4 停顿冻结：ce=0 期间 ra_data 保持不变");
    else
        $display("   (info)   T4 ce 恢复后数据更新（预期）");

    // 逐字扫读：地址在 negedge 落定，数据下一拍 negedge 采（BRAM 1 拍延迟）
    for (a = 0; a < NWORD; a = a + 1) begin
        ra_addr = a[12:0];
        @(negedge clk);                 // 地址已被 posedge 采走
        @(negedge clk);                 // 此时 ra_data 对应地址 a
        $display("%0d %08x", a, ra_data);
    end
    $display("T4 失败数 = %0d", n_err);
    $finish;
end

endmodule
