`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_scale.v —— 画布渲染器的几何/写地址/像素来源断言
//==============================================================================
//  纪律：先让它 FAIL（改坏一处就报错），再改 RTL。
//  断言全部用"不变量"：
//    ① 写地址严格 +1，覆盖 0..CW*CH-1，一个字都不重复/不遗漏
//    ② 内容区抽样像素 == 源图按最近邻映射回来的像素
//    ③ 黑边区域必须是 0
//==============================================================================
module tb_scale_a;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         start = 0;
reg  [15:0] src_w = 0, src_h = 0;
reg         stretch = 0;
reg         fmt = 1;
reg         pix_vld = 0;
reg  [23:0] pix = 0;
wire        pix_rd;
wire        we;
wire [20:0] woff;
wire [31:0] wdata;
wire        busy, done;

bmp_scale #(.CW(640), .CH(480)) dut (
    .clk(clk), .rst(rst),
    .start(start), .src_w(src_w), .src_h(src_h),
    .stretch(stretch), .fmt(fmt),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .we(we), .woff(woff), .wdata(wdata),
    .busy(busy), .done(done)
);

// pix_rd 是组合信号：如果直接在 @(posedge) 之后读，读到的已经是"吃完之后"
// 的值（0）⇒ 喂数循环永远等不到，仿真挂死。打一拍再用。
reg prd;
always @(posedge clk) prd <= pix_rd;

integer fails = 0, checks = 0;
task chk;
    input [8*48:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            $fflush;
            fails = fails + 1;
        end else begin
            $display("   ok       %0s = %0d", name, got);
            $fflush;
        end
    end
endtask

// 源像素：{x[7:0], y[7:0], x^y}，一眼能看出"这个像素来自源图哪一点"
function [23:0] srcpix;
    input integer x, y;
    begin
        srcpix = {x[7:0], y[7:0], (x[7:0] ^ y[7:0])};
    end
endfunction

// 画布捕获：只抓 16 个"探针地址"，不存整幅（仿真快 100 倍以上）
reg [20:0] pr_addr [0:15];
reg [31:0] pr_val  [0:15];
integer wr_cnt;
integer addr_err;
integer pi;
always @(posedge clk) begin
    if (we) begin
        if (woff !== wr_cnt[20:0]) addr_err = addr_err + 1;
        for (pi = 0; pi < 16; pi = pi + 1)
            if (woff === pr_addr[pi]) pr_val[pi] = wdata;
        wr_cnt = wr_cnt + 1;
    end
end

task set_probe;
    input integer idx, x, y;
    begin
        pr_addr[idx] = y*640 + x;
        pr_val[idx]  = 32'hDEAD_BEEF;
    end
endtask

// 喂源图
task feed_all;
    input integer W, H;
    integer x, y;
    begin
        for (y = 0; y < H; y = y + 1) begin
            for (x = 0; x < W; x = x + 1) begin
                @(negedge clk);
                pix_vld = 1'b1;
                pix     = srcpix(x, y);
                @(posedge clk);
                while (!prd) @(posedge clk);
            end
        end
        @(negedge clk);
        pix_vld = 1'b0;
    end
endtask

task run_case;
    input integer W, H;
    input         str;
    begin
        wr_cnt = 0; addr_err = 0;

        @(negedge clk);
        src_w = W; src_h = H; stretch = str; fmt = 1;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        feed_all(W, H);          // 所有源像素都被吃掉了
        // 再等冲刷收尾（最长 ~1.5M 拍）
        begin : waitd
            integer t;
            for (t = 0; t < 3000000; t = t + 1) begin
                @(posedge clk);
                if (done) t = 3000000;
            end
        end
        repeat (20) @(posedge clk);
        chk("addr strictly linear", addr_err, 0);
        chk("write count",           wr_cnt, 307200);
    end
endtask

// 取画布像素的 RGB（888 字 = {FF,R,G,B}）
function [23:0] pv;
    input integer idx;
    begin
        pv = pr_val[idx][23:0];
    end
endfunction

integer i;

initial begin
    $display("=========== bmp_scale 几何/像素断言 ===========");
    rst = 1; repeat (5) @(posedge clk); rst = 0; repeat (5) @(posedge clk);

    //--------------------------------------------------------------- A. 320×240 → 满屏 2 倍
    $display("--- A. 320x240 contain（dw=640 dh=480）---");
    set_probe(0,   0,   0); set_probe(1, 639, 479);
    set_probe(2, 100,  50); set_probe(3, 639,   0);
    run_case(320, 240, 0);
    chk("A0 TL == src(0,0)",     pv(0), srcpix(0,0));
    chk("A1 (639,479) == src(319,239)", pv(1), srcpix(319,239));
    chk("A2 (100,50) == src(50,25)",    pv(2), srcpix(50,25));
    chk("A3 (639,0) == src(319,0)",     pv(3), srcpix(319,0));

    $display("======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL / %0d checks", fails, checks);
    $finish;
end

endmodule
