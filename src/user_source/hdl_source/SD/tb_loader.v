`timescale 1ns/1ps
//==============================================================================
//  tb_loader.v —— 装载链路联仿：bmp_read → bf FIFO → bmp_scale → 写 FIFO
//==============================================================================
//  验证目标（都是"不变量"，先让它们 FAIL 再改 RTL）：
//    ① 写口一共吐出 CLR_LEN + N 个字：先 CLR_LEN 个预清常量，再 N 个画布字
//    ② 两段地址都严格 +1（预清 0..CLR_LEN-1，画布 0..N-1）
//    ③ 抽样画布字 == 源图按 floor 映射回来的像素（含 dx0 黑边）
//    ④ src_w_o/src_h_o = 头部解析出的分辨率
//  SD 卡用"逐字节吐扇区"的从机模型（同 tb_bmp_read）。
//==============================================================================
module tb_loader;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz（写口也用同一拍，便于观察）

reg rst = 1;

//----------------------------- DUT 端口 ---------------------------------------
reg         load_start = 0;
reg  [31:0] load_offset = 0;
reg  [31:0] load_length = 0;
reg  [20:0] tex_base = 21'd1703940;      // 与工程里 TEX0 一致
reg         load_fmt = 1;
reg         load_fit = 0;
wire [15:0] src_w_o, src_h_o;
wire        sd_ready, ready, can_load, busy, done, err;
wire [3:0]  state_code, ld_prog;
wire        wr_credit, wr_vld;
wire [9:0]  wr_used;
wire [20:0] wr_addr;
wire [31:0] wr_data;

wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0, sd_end = 0;

sd_bmp_loader #(
    .CLK_FREQ_HZ (50_000_000),
    .IMG_W (640), .IMG_H (480),
    .OUT_W (320), .OUT_H (240)          // ★ 与 top_tf_hdmi_audio.v 的真实例化一致
) dut (
    .clk(clk), .rst(rst), .mclk(clk),
    .load_start(load_start), .load_offset(load_offset), .load_length(load_length),
    .tex_base(tex_base), .load_fmt(load_fmt),
    .sd_ready(sd_ready), .ready(ready), .can_load(can_load),
    .busy(busy), .done(done), .err(err),
    .state_code(state_code), .ld_prog(ld_prog),
    .load_fit(load_fit),
    .src_w_o(src_w_o), .src_h_o(src_h_o),
    .wr_credit(wr_credit),
    .wr_vld(wr_vld), .wr_used(wr_used), .wr_addr(wr_addr), .wr_data(wr_data),
    .wr_ack(wr_vld),                 // 消费者永远当场取走
    .wr_settled(1'b1),
    .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_dv),
    .sd_sec_read_end(sd_end),
    .sd_init_done(1'b1)
);

//----------------------------- SD 从机模型 ------------------------------------
reg [7:0]  mem [0:1048575];          // 1MB
reg [1:0]  sbusy = 0;
reg [15:0] si = 0;
reg [31:0] saddr;

always @(posedge clk) begin
    if (sd_sec_read && !sbusy) begin sbusy <= 1; saddr <= sd_sec_read_addr; end
end
always @(negedge clk) begin
    if (sbusy == 1) begin
        if (si < 512) begin
            sd_data <= mem[saddr*512 + si];
            sd_dv   <= 1'b1;
            si      <= si + 1;
        end else begin
            sd_dv  <= 1'b0;
            sd_end <= 1'b1;
            si     <= 0;
            sbusy  <= 2;
        end
    end else if (sbusy == 2) begin
        sd_end <= 1'b0;
        sbusy  <= 0;
    end
end

//----------------------------- 断言与统计 --------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*48:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            $fflush; fails = fails + 1;
        end else begin
            $display("   ok       %0s = %0d", name, got);
            $fflush;
        end
    end
endtask

function [23:0] srcpix;             // 与 tb_bmp_scale 同款源图
    input integer x, y;
    begin
        srcpix = {x[7:0], y[7:0], (x[7:0] ^ y[7:0])};
    end
endfunction

// ★ 2026-10-28：逐通道平均（不能用整字相加 —— 那是"彩色麻点"的根因）
function [23:0] avgp;
    input [23:0] a, b;
    reg   [8:0]  sr, sg, sb;
    begin
        sr = {1'b0, a[23:16]} + {1'b0, b[23:16]};
        sg = {1'b0, a[15:8]}  + {1'b0, b[15:8]};
        sb = {1'b0, a[7:0]}   + {1'b0, b[7:0]};
        avgp = {sr[8:1], sg[8:1], sb[8:1]};
    end
endfunction

function [15:0] to565;
    input [23:0] c;
    begin
        to565 = {c[23:19], c[15:10], c[7:3]};
    end
endfunction

// 造一张 W×H 的 24bit BMP 到 mem[0..]
task make_bmp;
    input integer W, H;
    integer off, x, y;
    reg [31:0] nbytes;
    reg [23:0] px;
    begin
        nbytes = 54 + W*H*3;        // W*3 是 4 的倍数时无行填充（W 取 100/320 都除不尽…见下）
        for (off = 0; off < nbytes; off = off + 1) mem[off] = 8'h00;
        mem[0] = 8'h42; mem[1] = 8'h4D;
        mem[2] = nbytes[7:0]; mem[3] = nbytes[15:8];
        mem[4] = nbytes[23:16]; mem[5] = nbytes[31:24];
        mem[10] = 54;
        mem[18] = W[7:0]; mem[19] = W[15:8];
        mem[22] = H[7:0]; mem[23] = H[15:8];
        mem[28] = 24; mem[30] = 0;
        for (y = 0; y < H; y = y + 1)
            for (x = 0; x < W; x = x + 1) begin
                // BMP 行自下而上、每行 4 字节对齐
                off = 54 + (H-1-y)*(((W*3+3)/4)*4) + x*3;
                px = srcpix(x, y);                  // 函数返回值不能直接做位选
                mem[off+0] = px[7:0];               // B
                mem[off+1] = px[15:8];              // G
                mem[off+2] = px[23:16];             // R
            end
        load_length = nbytes;
    end
endtask

//----------------------------- 写口监视 ---------------------------------------
localparam [31:0] PREFILL = 32'hFFFF_00FF;
reg done_seen = 0, err_seen = 0;
always @(posedge clk) begin
    if (done) done_seen <= 1'b1;
    if (err)  err_seen  <= 1'b1;
end

integer wcnt, pre_cnt, img_cnt, addr_err, pre_err;
reg     in_img = 0;
reg     is565 = 0;

// ★ 2026-10-28：565 纹理的**覆盖 + 全量数据**检查
//   【为什么要它】上板现象"320 模式右上角一个黑点"，而原来只对了 3 个字，
//   漏写（洞）根本发现不了：addr_err 只比"顺序对不对"，不比"有没有缺"。
//   这里给每个相对地址打 seen，并在结束时数：漏写多少个字、写错多少个字。
reg     chk888 = 0;       // 1 = 本单做 888 全量比对
reg     seen8 [0:307199];
integer miss8, data8, a8, c8, r8;
reg [23:0] exp_w8;
reg     mode565 = 0;      // 0 = 320 源（1:1 抽偶列）  1 = 640 源（逐通道平均）
reg     seen5 [0:38399];
integer miss5, data5;
reg [31:0] exp_w5;
integer    r5, c5, sy5;

task clr_seen;
    integer q;
    begin
        for (q = 0; q < 38400; q = q + 1) seen5[q] = 1'b0;
        miss5 = 0; data5 = 0;
    end
endtask

task chk_seen;                        // 收工后统计
    integer q;
    begin
        miss5 = 0;
        for (q = 0; q < 38400; q = q + 1) if (!seen5[q]) miss5 = miss5 + 1;
    end
endtask
integer img_row, img_col;
reg [20:0] exp_a;

// 抽样：相对地址 → 数据
reg [20:0] s_addr [0:7];
reg [31:0] s_val  [0:7];
integer si2;
always @(posedge clk) begin
    if (wr_vld) begin
        if (!in_img) begin
            // 预清段
            if (pre_cnt + 1 >= dut.CLR_LEN) in_img <= 1;   // 本拍是最后一个预清字
            if (wr_data !== PREFILL) begin
                if (pre_err < 4) $display("        [pre] #%0d data=%h", pre_cnt, wr_data);
                pre_err = pre_err + 1;
            end
            if (wr_addr !== (tex_base + pre_cnt)) begin
                if (addr_err < 4) $display("        [pre] #%0d addr=%0d exp=%0d",
                                           pre_cnt, wr_addr, tex_base + pre_cnt);
                addr_err = addr_err + 1;
            end
            pre_cnt = pre_cnt + 1;
        end else begin
            if (is565) begin
                exp_a = (239 - img_row)*160 + img_col;
                // ★ 覆盖 + 数据全量比对（纹理行 r 存源行 239-r；字 n 取源列 2n/2n+1）
                if (exp_a < 38400) begin
                    seen5[exp_a] = 1'b1;
                    r5  = exp_a / 160;
                    c5  = exp_a % 160;
                    sy5 = r5;          // srcpix 用**文件行**：纹理行 r ← 文件行 r
                    if (mode565 == 0)
                        exp_w5 = {to565(srcpix(2*c5 + 1, sy5)), to565(srcpix(2*c5, sy5))};
                    else   // 640 源：纵向 2:1（纹理行 r ← 源行 2r）、横向逐通道平均
                        exp_w5 = {to565(avgp(srcpix(4*c5 + 2, 2*r5), srcpix(4*c5 + 3, 2*r5))),
                                  to565(avgp(srcpix(4*c5    , 2*r5), srcpix(4*c5 + 1, 2*r5)))};
                    if (wr_data !== exp_w5) begin
                        if (data5 < 4)
                            $display("        [565] a=%0d got=%h exp=%h",
                                     exp_a, wr_data, exp_w5);
                        data5 = data5 + 1;
                    end
                end
                if (img_col == 159) begin img_col = 0; img_row = img_row + 1; end
                else                    img_col = img_col + 1;
            end else begin
                exp_a = img_cnt[20:0];
                // ★ 2026-10-28：888 纹理全量比对（源行 = 479 - 纹理行，纵向翻转）
                if (chk888 && exp_a < 307200) begin
                    seen8[exp_a] = 1'b1;
                    r8 = exp_a / 640;
                    c8 = exp_a % 640;
                    exp_w8 = srcpix(c8, 479 - r8);
                    if (wr_data[23:0] !== exp_w8) begin
                        if (data8 < 6)
                            $display("        [888] a=%0d (c=%0d,r=%0d) got=%h exp=%h",
                                     exp_a, c8, r8, wr_data[23:0], exp_w8);
                        data8 = data8 + 1;
                    end
                end
            end
            if (wr_addr !== tex_base + exp_a) begin
                if (addr_err < 4)
                    $display("        [img] #%0d addr=%0d exp=%0d data=%h",
                             img_cnt, wr_addr, tex_base + exp_a, wr_data);
                addr_err = addr_err + 1;
            end
            for (si2 = 0; si2 < 8; si2 = si2 + 1)
                if (exp_a === s_addr[si2]) s_val[si2] = wr_data;
            img_cnt = img_cnt + 1;
        end
        wcnt = wcnt + 1;
    end
end

task probe;                          // 相对地址探针
    input integer idx, rel;
    begin
        s_addr[idx] = rel;
        s_val[idx]  = 32'hDEAD_BEEF;
    end
endtask

task run_load;                       // 等 done/err
    input integer limit;
    integer t;
    begin
        @(negedge clk); load_start = 1; @(negedge clk); load_start = 0;
        for (t = 0; t < limit; t = t + 1) begin
            @(posedge clk);
            if (done || err) t = limit;
        end
        repeat (10) @(posedge clk);
        $display("        [stat] wcnt=%0d pre=%0d img=%0d err_code=%0d st=%0d fmt=%0d clr_len=%0d",
                 wcnt, pre_cnt, img_cnt, dut.err_code, state_code, load_fmt, dut.CLR_LEN);
        if (!done_seen && !err_seen) begin
            $display("  *** TIMEOUT (st=%0d prog=%0h)", state_code, ld_prog);
            $fflush; fails = fails + 1;
        end
    end
endtask

task reset_stats;
    begin
        wcnt = 0; pre_cnt = 0; img_cnt = 0; addr_err = 0; pre_err = 0;
        in_img = 0; img_row = 0; img_col = 0; done_seen = 0; err_seen = 0;
    end
endtask

integer k;

initial begin
    $display("=========== 装载链路联仿 ===========");
    rst = 1; repeat (8) @(posedge clk); rst = 0; repeat (5) @(posedge clk);

    //----------------------------------------------------------- S1: 100x400 → 888
    $display("--- S1. 100x400 contain → 888（dw=120 dx0=260 dy0=0）---");
    make_bmp(100, 400);
    reset_stats;
    load_fmt = 1; load_fit = 0; is565 = 0;
    probe(0, 0*640 + 260);           // 内容区左上
    probe(1, 0*640 + 259);           // 左黑边
    probe(2, 0*640 + 380);           // 右黑边
    probe(3, 479*640 + 260);         // 最后一行内容
    run_load(4000000);
    chk("S1 done",            done_seen, 1);
    chk("S1 no err",          err_seen, 0);
    chk("S1 src_w",           src_w_o, 100);
    chk("S1 src_h",           src_h_o, 400);
    chk("S1 img words",       img_cnt, 307200);
    chk("S1 addr/prefill err", addr_err + pre_err, 0);
    // ★ 存储约定（与旧装载器逐字一致）：888 纹理按 **BMP 文件行序** 存，
    //   纹理行 0 = 文件第 0 行 = 图像**最底**行。所以纹理行 r == 图像行 H-1-r。
    chk("S1 (260,0) == src(0,399)",   s_val[0][23:0], srcpix(0,399));
    chk("S1 (259,0) black",           s_val[1][23:0], 24'h000000);
    chk("S1 (380,0) black",           s_val[2][23:0], 24'h000000);
    // 纹理最后一行 k=479 ⇒ 文件行 floor(479*400/480)=399 ⇒ 图像行 0
    chk("S1 (260,479) == src(0,0)",   s_val[3][23:0], srcpix(0,0));

    //----------------------------------------------------------- S2: 320x240 → 565
    $display("--- S2. 320x240 → 565 画布（1:1，行翻转）---");
    make_bmp(320, 240);
    reset_stats;
    load_fmt = 0; load_fit = 0; is565 = 1;
    clr_seen;
    $display("        [S2 setup] fmt=%0d CLR_LEN=%0d len=%0d", load_fmt, dut.CLR_LEN, load_length);
    probe(0, 239*160 + 0);           // k=0（纹理最上面）第一个字
    probe(1, 239*160 + 159);
    probe(2, 0*160 + 100);           // k=239（纹理最下面）
    run_load(4000000);
    chk("S2 done",      done_seen, 1);
    chk("S2 no err",    err_seen, 0);
    chk("S2 src_w",     src_w_o, 320);
    chk("S2 src_h",     src_h_o, 240);
    chk("S2 img words", img_cnt, 38400);
    chk("S2 addr/prefill err", addr_err + pre_err, 0);
    // 565：纹理行 k 存文件第 k 行（行翻转），⇒ 图像行 239-k
    // 1 字 = 2 个 RGB565 像素：字 n = {565 像素 2n+1, 2n} = {画布列 4n+2, 4n}
    chk("S2 tex-row0 word0",   s_val[0], {to565(srcpix(1,239)),   to565(srcpix(0,239))});
    chk("S2 tex-row0 word159", s_val[1], {to565(srcpix(319,239)), to565(srcpix(318,239))});
    chk("S2 tex-row239 word100", s_val[2], {to565(srcpix(201,0)), to565(srcpix(200,0))});
    chk_seen;
    chk("S2 no hole (all 38400 written)", miss5, 0);
    chk("S2 all words match src",         data5, 0);

    //----------------------------------------------------------- S3: 640x480 → 565
    //  ★ 2026-10-28：这是**上板真实组合**（苹果 640×480 → 320×240 缩略图），
    //    走 bmp_scale 的 avg2 分支。原来只抽 3 个字比对，查不出"某个字没写/
    //    差一个字"这种问题；这里做 38400 字的覆盖 + 逐字数据比对。
    $display("--- S3. 640x480 → 565 画布（2:1 逐通道平均，全量比对）---");
    make_bmp(640, 480);
    reset_stats;
    load_fmt = 0; load_fit = 0; is565 = 1; mode565 = 1;
    clr_seen;
    run_load(8000000);
    chk("S3 done",      done_seen, 1);
    chk("S3 no err",    err_seen, 0);
    chk("S3 img words", img_cnt, 38400);
    chk("S3 addr err",  addr_err + pre_err, 0);
    chk_seen;
    chk("S3 no hole (all 38400 written)", miss5, 0);
    chk("S3 all words match src",         data5, 0);
    mode565 = 0;

    //----------------------------------------------------------- S4: 640x480 → 888
    //  ★ 上板真实组合的另一半：HD 纹理。bmp_scale 逐字写 640×480，这里查
    //    "有没有洞 / 有没有错位" —— 高清图只要偏几列，切 565→HD 就会整幅平移。
    $display("--- S4. 640x480 → 888 画布（全量 307200 字比对）---");
    make_bmp(640, 480);
    reset_stats;
    load_fmt = 1; load_fit = 0; is565 = 0; chk888 = 1;
    begin : s4clr
        integer q;
        for (q = 0; q < 307200; q = q + 1) seen8[q] = 1'b0;
        miss8 = 0; data8 = 0;
    end
    run_load(16000000);
    chk("S4 done",      done_seen, 1);
    chk("S4 no err",    err_seen, 0);
    chk("S4 img words", img_cnt, 307200);
    chk("S4 addr err",  addr_err + pre_err, 0);
    begin : s4stat
        integer q;
        for (q = 0; q < 307200; q = q + 1) if (!seen8[q]) miss8 = miss8 + 1;
    end
    chk("S4 no hole (all 307200 written)", miss8, 0);
    chk("S4 all pixels match src",         data8, 0);
    chk888 = 0;

    $display("======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL / %0d checks", fails, checks);
    $finish;
end
endmodule
