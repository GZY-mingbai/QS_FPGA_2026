`timescale 1ns / 1ps
//==============================================================================
//  tb_quad_ready.v —— 文件页四宫格「quad_ready 起不来」（交接书 Q4）回归
//==============================================================================
//  2026-10-31 新建。顶层 top_tf_hdmi_audio.v 里：
//      quad_mode_r <= quad_ready & osd_file_page        （每帧采一次）
//  quad_ready = quad_page_ctrl.O_ready。只要它不起来，A 层就一直读相册那块
//  ⇒ 文件页四格全是透明/黑 —— 正是上板现象。
//
//  quad_page_ctrl 的 I_ld_done / I_ld_busy 要的是 **quad_loader 整页**的
//  done/busy（端口段标题就写着 "quad_loader"）。本台按顶层同样的拓扑接：
//      fake lister → quad_page_ctrl → quad_loader → fake bmp_loader
//  并用宏切换两种接法：
//      默认          I_ld_done = ql_done, I_ld_busy = ql_busy   （修复后）
//      -DOLD_WIRING  I_ld_done = bmp 每张的 done, I_ld_busy = bmp busy（修复前）
//  判据：进文件页后 O_ready 必须起来、O_loaded == 4、4 次装载落在 4 个不同的槽；
//        翻页后 ready 先落再起。
//
//  跑法：
//    iverilog -g2012 -o qr.vvp -s tb_quad_ready tb_quad_ready.v quad_page_ctrl.v quad_loader.v
//    vvp qr.vvp
//==============================================================================
module tb_quad_ready;

localparam [20:0] QBASE = 21'd1048576;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  file_page = 0, flip_nx = 0, flip_pv = 0;

// ---- quad_page_ctrl ↔ lister ----
wire        lst_start;
wire [15:0] lst_offset;
reg         lst_busy = 0, lst_done = 0;
reg  [2:0]  lst_cnt = 0;
reg         lst_more = 0;

// ---- quad_page_ctrl ↔ quad_loader ----
wire        ql_start, ql_busy, ql_done;
wire [2:0]  ql_loaded;
wire        qp_ab, ql_filled;
wire [3:0]  ql_bmp;
wire        own, ready, has_more, gen;
wire [15:0] off;
wire [2:0]  vcnt;

// ---- quad_loader ↔ fake bmp_loader ----
wire        bmp_start;
wire [31:0] bmp_off, bmp_len;
wire [20:0] bmp_base;
reg         bmp_busy = 0, bmp_done = 0;
wire [1:0]  rd_slot;

`ifdef OLD_WIRING
wire pc_ld_done = bmp_done;     // 修复前：bmp_loader 每张的 done
wire pc_ld_busy = bmp_busy;
`else
wire pc_ld_done = ql_done;      // 修复后：quad_loader 整页 done
wire pc_ld_busy = ql_busy;
`endif

quad_page_ctrl #(.N_ITEMS(4), .STUCK_CYC(1 << 20)) u_pc (
    .clk(clk), .rst(rst),
    .I_file_page(file_page), .I_flip_next(flip_nx), .I_flip_prev(flip_pv),
    .O_lst_start(lst_start), .O_lst_offset(lst_offset),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done), .I_lst_cnt(lst_cnt),
    .I_lst_has_more(lst_more),
    .O_ld_start(ql_start), .I_ld_busy(pc_ld_busy), .I_ld_done(pc_ld_done),
    .I_ld_loaded(ql_loaded), .I_ld_filled(ql_filled), .I_ld_bmp(ql_bmp), .O_ld_abort(qp_ab), .I_cur(2'd0), .O_has_prev(), .O_rank(),
    .O_own(own), .O_ready(ready), .O_has_more(has_more), .O_offset(off),
    .O_gen(gen), .O_valid_cnt(vcnt)
);

quad_loader #(.SLOT_BASE(QBASE), .SLOT_STEP(40960), .N_ITEMS(4)) u_ql (
    .clk(clk), .rst(rst),
    .I_start(ql_start), .I_abort(qp_ab), .I_hold(1'b0), .O_bmp(ql_bmp), .O_filled(ql_filled), .I_valid_cnt(lst_cnt), .I_slot_valid(4'hF),
    .O_busy(ql_busy), .O_done(ql_done), .O_loaded(ql_loaded),
    .O_rd_slot(rd_slot),
    .I_file_off(32'h0010_0000 * (rd_slot + 1) + {lst_offset, 12'd0}),
    .I_file_len(32'd230454), .I_file_type(4'd1),
    .O_ph_start(), .O_ph_kind(), .O_ph_base(), .I_ph_busy(1'b0), .I_ph_done(1'b0),
    .O_ld_start(bmp_start), .O_ld_off(bmp_off), .O_ld_len(bmp_len), .O_ld_base(bmp_base),
    .I_ld_ready(1'b1), .I_ld_busy(bmp_busy), .I_ld_done(bmp_done), .I_ld_err(1'b0)
);

// ---- fake lister：start 后 50 拍 done，每页 4 项，共 3 页 ----
integer lcnt = 0;
always @(posedge clk) begin
    lst_done <= 0;
    if (lst_start) begin lst_busy <= 1; lcnt <= 0; end
    else if (lst_busy) begin
        lcnt <= lcnt + 1;
        if (lcnt == 50) begin
            lst_busy <= 0; lst_done <= 1;
            lst_cnt  <= 3'd4;
            lst_more <= (lst_offset < 16'd8);
        end
    end
end

// ---- fake bmp_loader：start 后 300 拍 done（中间留空档，模拟真装载的间隙）----
integer bcnt = 0, nld = 0;
reg [20:0] bases [0:15];
always @(posedge clk) begin
    bmp_done <= 0;
    if (bmp_start) begin
        bmp_busy <= 1; bcnt <= 0;
        if (nld < 16) bases[nld] <= bmp_base;
        nld <= nld + 1;
    end else if (bmp_busy) begin
        bcnt <= bcnt + 1;
        if (bcnt == 300) begin bmp_busy <= 0; bmp_done <= 1; end
    end
end

// ---- 判据 ----
integer errors = 0, checks = 0;
task chk;
    input [8*48:1] name;
    input integer got, exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            errors = errors + 1;
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
        end else $display("  ok    %0s = %0d", name, got);
    end
endtask

task wait_ready;
    input [8*48:1] name;
    integer t;
    begin
        t = 0;
        while (!ready && t < 50000) begin @(posedge clk); t = t + 1; end
        chk(name, ready, 1);
    end
endtask

integer k, n0, distinct;
initial begin
`ifdef OLD_WIRING
    $display("== tb_quad_ready (OLD wiring: page ctrl <- bmp per-image done) ==");
`else
    $display("== tb_quad_ready (page ctrl <- quad_loader page done) ==");
`endif
    repeat (10) @(posedge clk); rst = 0;
    repeat (10) @(posedge clk);

    // 进文件页 → 扫目录 → 装 4 张 → ready
    file_page = 1;
    wait_ready("page0 ready rises");
    repeat (2000) @(posedge clk);                // 装载拖尾也走完
    chk("page0 ready stays", ready, 1);
    chk("page0 loaded", ql_loaded, 4);
    chk("page0 loads issued", nld, 4);
    distinct = 1;
    for (k = 1; k < 4; k = k + 1)
        if (bases[k] == bases[k-1]) distinct = 0;
    chk("page0 4 distinct slots", distinct, 1);
    chk("slot3 base", bases[3], QBASE + 3 * 40960);

    // 翻页：ready 先落，再起来
    n0 = nld;
    @(negedge clk); flip_nx = 1; @(negedge clk); flip_nx = 0;
    repeat (5) @(posedge clk);
    chk("flip: ready drops", ready, 0);
    wait_ready("page1 ready rises");
    repeat (2000) @(posedge clk);
    chk("page1 loads issued", nld - n0, 4);
    chk("page1 offset", off, 4);

    // 离开文件页：归属交还
    file_page = 0;
    repeat (2000) @(posedge clk);
    chk("leave: own released", own, 0);

    $display("======================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", checks);
    else             $display("  FAILED    %0d of %0d checks", errors, checks);
    $finish;
end

endmodule
