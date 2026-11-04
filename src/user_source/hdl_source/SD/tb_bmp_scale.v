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
module tb_bmp_scale;

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
    .stall(1'b0),                       // ★ 2026-10-27：新增端口，本台测 1:1 不背压
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
// ★ 2026-10-28：888 画布**全量**捕获（查"有没有洞 / 有没有错像素"）
//   上板现象"切 565→HD 时整幅平移 >4px、320 模式右上角一个黑点"，
//   原来只抽 3 个探针，这类问题完全看不见。
reg  [23:0] wmem  [0:307199];
reg         wseen [0:307199];
reg         full888 = 0;
integer     g_miss, g_bad, gi;

task full888_clr;
    begin
        for (gi = 0; gi < 307200; gi = gi + 1) wseen[gi] = 1'b0;
        g_miss = 0; g_bad = 0;
    end
endtask

task full888_chk;
    integer a;
    begin
        g_miss = 0; g_bad = 0;
        for (a = 0; a < 307200; a = a + 1) begin
            if (!wseen[a]) g_miss = g_miss + 1;
            else if (wmem[a] !== srcpix(a % 640, a / 640)) begin
                if (g_bad < 6)
                    $display("        [888] a=%0d (%0d,%0d) got=%h exp=%h",
                             a, a % 640, a / 640, wmem[a], srcpix(a % 640, a / 640));
                g_bad = g_bad + 1;
            end
        end
    end
endtask

always @(posedge clk) begin
    if (we) begin
        if (full888 && woff < 307200) begin
            wmem[woff]  = wdata[23:0];
            wseen[woff] = 1'b1;
        end
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

// 565 画布：第 k 行第 n 字（第 k 行存在 (239-k)*320 处，行翻转）
task set_probe5;
    input integer idx, n, k;
    begin
        pr_addr[idx] = (239-k)*160 + n;
        pr_val[idx]  = 32'hDEAD_BEEF;
    end
endtask

// 流式源：永远把"下一个像素"摆在 pix 上，pix_rd=1 就前进
reg [15:0] fx, fy;
integer    FDW, FDH;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        fx <= 16'd0; fy <= 16'd0; pix_vld <= 1'b0; pix <= 24'd0;
    end else begin
        if (pix_rd) begin
            if (fx + 16'd1 >= FDW) begin fx <= 16'd0; fy <= fy + 16'd1; end
            else                         fx <= fx + 16'd1;
        end
        pix_vld <= (fy < FDH);
        pix     <= srcpix(fx, fy);
    end
end

// 565 通路的地址监视：块内 +1，块间 -320（行翻转），首块起点 239*320
reg     en565 = 0;
integer blk_err, wcnt5, exp_addr;
always @(posedge clk) begin
    if (en565 && we) begin
        if (wcnt5 == 0) exp_addr = 239*160;
        if (woff !== exp_addr) blk_err = blk_err + 1;
        exp_addr = exp_addr + 1;
        wcnt5    = wcnt5 + 1;
        if ((wcnt5 % 160) == 0) exp_addr = exp_addr - 320;   // 下一行起点（再降到 -160）
    end
end

// 565 打包：{高半=奇列, 低半=偶列}
function [15:0] to565;
    input [23:0] c;
    begin
        to565 = {c[23:19], c[15:10], c[7:3]};
    end
endfunction

// ★ 2026-10-28 根因修复：2:1 横向平均必须**逐通道**算。
//   【旧写法】t = {1'b0,a} + {1'b0,b}; t[24:1] —— 把打包的 R/G/B 当整数相加，
//     G_s>=256 会进位到 R、右移时 R_s 的奇偶位又会落进 G 的 MSB：
//         avg(24'h010000, 24'h000000) 旧写法得到 24'h008000（凭空冒出 G=128）
//     ⇒ 上板表现：整幅蓝/绿彩色麻点（结构还认得出）。
//   【为什么当初没抓到】期望函数抄了 RTL 的表达式 ⇒ 把 bug 写成了期望（假绿）。
//   现在按通道算，任何一个通道的和都不会跨到邻居。
function [23:0] avg2pix;
    input [23:0] a, b;
    reg   [8:0]  sr, sg, sb;
    begin
        sr = {1'b0, a[23:16]} + {1'b0, b[23:16]};
        sg = {1'b0, a[15:8]}  + {1'b0, b[15:8]};
        sb = {1'b0, a[7:0]}   + {1'b0, b[7:0]};
        avg2pix = {sr[8:1], sg[8:1], sb[8:1]};
    end
endfunction

task run_case;
    input integer W, H;
    input         str;
    input         f;
    begin
        wr_cnt = 0; addr_err = 0; en565 = (f == 0);

        @(negedge clk);
        FDW = W; FDH = H;
        fx = 16'd0; fy = 16'd0;
        src_w = W; src_h = H; stretch = str; fmt = f;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // 源由上面的流式 always 驱动，这里只等 done。
        // ★ 1080p 源要消费 1080 行 × 1920 像素（丢行也要吃源流），约 4M 拍，
        //   上限给到 12M；超时明确报出来，不要静默当成"跑完了"。
        begin : waitd
            integer t;
            for (t = 0; t < 12000000; t = t + 1) begin
                @(posedge clk);
                if (done) t = 12000000;
            end
            if (!done) begin
                $display("  *** TIMEOUT: 12M 周期没等到 done (st=%0d oy=%0d sy=%0d sy_t=%0d)",
                         dut.st, dut.oy, dut.sy, dut.sy_t);
                $fflush;
                fails = fails + 1;
            end
        end
        repeat (20) @(posedge clk);
        en565 = 0;
        if (f) begin
            chk("addr strictly linear", addr_err, 0);
            chk("write count",           wr_cnt, 307200);
        end else begin
            chk("565 word count", wr_cnt, 38400);   // 160 字/行 × 240 行
            chk("565 addr pattern", blk_err, 0);
        end
    end
endtask

// 888 字 = {FF,R,G,B}：只比低 24 位（高 8 位是填充 FF）
function [23:0] pv;
    input integer idx;
    begin
        pv = pr_val[idx][23:0];
    end
endfunction

// 565 字 = {高半(奇列), 低半(偶列)}：整 32 位
function [31:0] pw;
    input integer idx;
    begin
        pw = pr_val[idx];
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
    run_case(320, 240, 0, 1);
    chk("A0 TL == src(0,0)",     pv(0), srcpix(0,0));
    chk("A1 (639,479) == src(319,239)", pv(1), srcpix(319,239));
    chk("A2 (100,50) == src(50,25)",    pv(2), srcpix(50,25));
    chk("A3 (639,0) == src(319,0)",     pv(3), srcpix(319,0));

    //--------------------------------------------------------------- B. 1920×1080 → 640×360 居中
    $display("--- B. 1920x1080 contain（dw=640 dh=360 dy0=60）---");
    set_probe(0, 100,   0); set_probe(1, 100,  59);
    set_probe(2,   0,  60); set_probe(3, 639,  60);
    set_probe(4,   0, 419); set_probe(5, 100, 420);
    run_case(1920, 1080, 0, 1);
    chk("B0 row0 is black",   pv(0),   24'h000000);
    chk("B1 row59 is black",  pv(1),   24'h000000);
    chk("B2 row60 == src row0", pv(2), srcpix(0,0));
    chk("B3 (639,60) == src(1917,0)", pv(3), srcpix(1917,0));
    // 映射约定是 floor((k)*sh/dh)：k=359 ⇒ 359*1080/360 = 1077
    //（B3/D1 也都是 floor：639*1920/640=1917、479*1080/480=1077，此处原写 1079 是笔误）
    chk("B4 row419 == src row 1077",  pv(4), srcpix(0,1077));
    chk("B5 row420 is black", pv(5), 24'h000000);

    //--------------------------------------------------------------- C. 100×400 → 120×480 居中
    $display("--- C. 100x400 contain（dw=120 dh=480 dx0=260）---");
    set_probe(0,   0, 10); set_probe(1, 259, 10);
    set_probe(2, 260,  0); set_probe(3, 379,  0);
    set_probe(4, 380, 10);
    run_case(100, 400, 0, 1);
    chk("C0 col0 is black",   pv(0), 24'h000000);
    chk("C1 col259 is black", pv(1), 24'h000000);
    chk("C2 col260 == src col0", pv(2), srcpix(0,0));
    chk("C3 col379 == src col99", pv(3), srcpix(99,0));
    chk("C4 col380 is black", pv(4), 24'h000000);

    //--------------------------------------------------------------- D. 拉伸
    $display("--- D. 1920x1080 stretch（dw=640 dh=480）---");
    // ★ 原来断言"row0 不是黑"，但 srcpix(0,0) = {0,0,0} 本身就是 0x000000，
    //   这条永远不会成立。改成有意义的：行 0 = 源行 0，且抽一列比映射。
    set_probe(0, 100,   0); set_probe(1, 639, 479);
    run_case(1920, 1080, 1, 1);
    chk("D0 (100,0) == src(300,0)", pv(0), srcpix(300,0));
    chk("D1 (639,479) == src(1917,1077)", pv(1), srcpix(1917,1077));

    //--------------------------------------------------------------- E. 565 缩略图
    //  640x480 源 → 320x240 画布（2:1 抽列 + 抽行 + 行翻转）
    //    第 k 行第 n 字 = {rgb565(src(2n+1, 2k)), rgb565(src(2n, 2k))}
    $display("--- E. 640x480 -> 565 画布（2:1，行翻转）---");
    set_probe5(0,   0,   0); set_probe5(1, 159,   0);
    set_probe5(2, 100, 240-1);
    blk_err = 0; wcnt5 = 0;
    run_case(640, 480, 0, 0);
    chk("E0 word0 == {avg(2,3),avg(0,1)}",
        pw(0), {to565(avg2pix(srcpix(2,0),srcpix(3,0))),
                to565(avg2pix(srcpix(0,0),srcpix(1,0)))});
    chk("E1 word159 row0 == {avg(638,639),avg(636,637)}",
        pw(1), {to565(avg2pix(srcpix(638,0),srcpix(639,0))),
                to565(avg2pix(srcpix(636,0),srcpix(637,0)))});
    chk("E2 last row == avg src row 478",
        pw(2), {to565(avg2pix(srcpix(402,478),srcpix(403,478))),
                to565(avg2pix(srcpix(400,478),srcpix(401,478)))});

    // ★ 2026-10-28：**通道串位**专项断言。
    //   srcpix(x,y) = {R=x, G=y, B=x^y}，所以第 0 行 G 恒 0、B=R=x。
    //   · 旧写法（整字相加再右移）在 R_s 为奇数时会把 G 的 MSB 置 1
    //     ⇒ 565 的 G6 变成 0x20（8 位 0x80）—— 这就是上板的彩色麻点。
    //   · 这条断言只看 G，不依赖 avg2pix 的实现，能独立抓住串位。
    //   （Verilog 不允许 `pw(0)[26:21]` 这种"函数调用后位选"，先存进 w0。）
    begin : e34
        reg [31:0] w0;
        w0 = pw(0);
        chk("E3 row0 G=0 both halves (no bleed)",
            {w0[26:21], w0[10:5]}, 12'h000);
        //   word0 低半 = avg(src(0,0), src(1,0))：
        //     R=(0+1)>>1=0, G=0, B=(0+1)>>1=0 ⇒ 整个 565 字必须是 0
        chk("E4 word0 low half = 0", w0[15:0], 16'h0000);
    end

    //--------------------------------------------------------------- G. 888 全量
    $display("--- G. 640x480 -> 888 画布（全量 307200 字）---");
    full888 = 1; full888_clr;
    run_case(640, 480, 0, 1);
    full888_chk;
    full888 = 0;
    chk("G0 no hole",     g_miss, 0);
    chk("G1 all pixels",  g_bad,  0);

    $display("======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL / %0d checks", fails, checks);
    $finish;
end

endmodule
