//==============================================================================
//  tb_bfmem.v —— sd_bmp_loader 的 bf_mem 读口改写：新旧实现**逐拍**等价证明
//==============================================================================
//  背景：为了把 bf_mem（256×24 = 6144 bit）从分布式 RAM 推进块 RAM，
//        把"写旁路"从 RAM 读表达式里移了出来：
//
//    旧： bf_q <= bf_hit ? bmp_data : mem[rp_n];      ← 旁路在 RAM 读里 ⇒ 只能 DRAM
//    新： bf_ram <= mem[rp_n];
//         bf_hit_r <= bf_hit;  bf_dat_r <= bmp_data;
//         bf_q = bf_hit_r ? bf_dat_r : bf_ram;        ← 旁路在寄存器之后
//
//  这个测试台把**两套实现并排**实例化，喂同一份随机激励（写/读/空/满/同址
//  读写交替出现），逐拍比对 {bf_q, bf_dv}。只要全等价，就说明改动是纯结构性的。
//
//  跑法：
//    cd src/user_source/hdl_source
//    iverilog -g2005 -o tb_bfmem.vvp APP/tb_bfmem.v && vvp tb_bfmem.vvp
//==============================================================================
`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//  参考实现 = 改动前的写法（组合旁路塞在 RAM 读里）
//------------------------------------------------------------------------------
module bf_old (
    input  wire        clk,
    input  wire        rst,
    input  wire        we,
    input  wire [23:0] wdat,
    input  wire        rd,
    output wire [23:0] pix,
    output wire        dv
);
    reg [23:0] mem [0:255];
    reg [23:0] q;
    reg        qv;
    reg [8:0]  wp, rp;

    wire [8:0] rp_n  = rp + (rd && qv ? 9'd1 : 9'd0);
    wire       hit    = we && (wp == rp_n);
    wire       cnt_ge = ((wp - rp) >= 9'd256);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wp <= 9'd0; rp <= 9'd0; q <= 24'd0; qv <= 1'b0;
        end else begin
            if (we && !cnt_ge) begin
                mem[wp[7:0]] <= wdat;
                wp <= wp + 9'd1;
            end
            q  <= hit ? wdat : mem[rp_n[7:0]];
            qv <= (wp != rp_n);
            rp <= rp_n;
        end
    end
    assign pix = q;
    assign dv  = qv;
endmodule

//------------------------------------------------------------------------------
//  新实现 = 改动后的写法（旁路移到寄存器之后）
//------------------------------------------------------------------------------
module bf_new (
    input  wire        clk,
    input  wire        rst,
    input  wire        we,
    input  wire [23:0] wdat,
    input  wire        rd,
    output wire [23:0] pix,
    output wire        dv
);
    reg [23:0] mem [0:255];
    reg [23:0] ram_q, dat_r;
    reg        hit_r, qv;
    reg [8:0]  wp, rp;

    wire [8:0] rp_n  = rp + (rd && qv ? 9'd1 : 9'd0);
    wire       hit    = we && (wp == rp_n);
    wire       cnt_ge = ((wp - rp) >= 9'd256);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wp <= 9'd0; rp <= 9'd0;
            ram_q <= 24'd0; dat_r <= 24'd0; hit_r <= 1'b0; qv <= 1'b0;
        end else begin
            if (we && !cnt_ge) begin
                mem[wp[7:0]] <= wdat;
                wp <= wp + 9'd1;
            end
            ram_q <= mem[rp_n[7:0]];     // 纯同步读
            hit_r <= hit;
            dat_r <= wdat;
            qv    <= (wp != rp_n);
            rp    <= rp_n;
        end
    end
    assign pix = hit_r ? dat_r : ram_q;
    assign dv  = qv;
endmodule

//------------------------------------------------------------------------------
module tb_bfmem;

reg         clk, rst;
reg         we, rd;
reg  [23:0] wdat;
wire [23:0] o_pix, n_pix;
wire        o_dv,  n_dv;

integer i, cyc;
integer err, evt;
integer seed;

bf_old u_old (.clk(clk), .rst(rst), .we(we), .wdat(wdat), .rd(rd), .pix(o_pix), .dv(o_dv));
bf_new u_new (.clk(clk), .rst(rst), .we(we), .wdat(wdat), .rd(rd), .pix(n_pix), .dv(n_dv));

always #10 clk = ~clk;

initial begin
    clk = 1'b0; rst = 1'b1; we = 1'b0; rd = 1'b0; wdat = 24'd0;
    err = 0; evt = 0; seed = 32'h1234_5678;

    repeat (5) @(posedge clk);
    @(negedge clk) rst = 1'b0;

    $display("======================================================");
    $display("  tb_bfmem : bf_mem 读口改写 —— 新旧逐拍等价（5000 拍）");
    $display("======================================================");

    for (cyc = 0; cyc < 5000; cyc = cyc + 1) begin
        @(negedge clk);
        // ---- 随机激励：写概率 45%、读概率 60%，数据带周期特征便于识别 ----
        we   = ($random(seed) % 100) < 45;
        rd   = ($random(seed) % 100) < 60;
        wdat = {cyc[7:0], cyc[7:0] * 8'd7, cyc[7:0] ^ 8'hA5};   // 每拍都不同的图案

        // 每 500 拍插一段"只写不读"（灌满）和"只读不写"（排空）
        if ((cyc % 500) < 40)  begin we = 1'b1; rd = 1'b0; end
        if ((cyc % 500) >= 40 && (cyc % 500) < 80) begin we = 1'b0; rd = 1'b1; end
        // 每 250 拍插一段全静默
        if ((cyc % 250) >= 240) begin we = 1'b0; rd = 1'b1; end

        @(posedge clk);
        #1;
        if ((o_pix !== n_pix) || (o_dv !== n_dv)) begin
            err = err + 1;
            if (err <= 15)
                $display("  DIFF cyc=%0d  old(dv=%b pix=%06h)  new(dv=%b pix=%06h)",
                         cyc, o_dv, o_pix, n_dv, n_pix);
        end
        if (o_dv) evt = evt + 1;
    end

    $display("------------------------------------------------------");
    $display("  比对 5000 拍；其中 dv=1 的拍数 = %0d", evt);
    if (err == 0)
        $display("  ALL PASS  (bf_pix / bf_pix_vld 逐拍完全一致)");
    else
        $display("  %0d 个拍不一致", err);
    $display("======================================================");
    $finish;
end

endmodule
