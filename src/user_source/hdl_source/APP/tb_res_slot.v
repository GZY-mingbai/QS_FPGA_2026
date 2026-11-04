//==============================================================================
//  tb_res_slot.v —— img_switch 的「3 槽分辨率缓存」专项回归（2026-11-02）
//==============================================================================
//
//  【要证的事】信息条的分辨率必须**在换图当拍**跟着变，而且是"目标那张图"的
//  分辨率，不是"最近一次装载"的。
//
//  【为什么这条断言有必要】`ld_src_w/h` 只是**一份活寄存器**：装完一张缩略图
//  就变成那张图的 W/H，而切图之后立刻会预取**邻居**（O_tl_req）。所以：
//      · 按"最近一次装载"取 ⇒ 后退翻页时会拿到邻居的分辨率（错的）；
//      · 按环槽存 3 份 ⇒ 任何方向都对。
//  本台就是构造"最近一次装载 ≠ 目标图"的场景，把第二种行为钉死。
//
//  【场景】图 k 的源分辨率 = (320+16k) × (240+8k)，各不相同 ⇒ 看数字就知道
//  拿到的是哪一张。
//      开  机：装 cur(0) → 装 next(1)
//      断言1：按 next → 分辨率 = 图1（当拍）
//      之后  ：预取 next(2) 装进 slot2 ⇒ **分辨率必须还是图1**（不能被预取改）
//      断言2：再按 next → 图2
//      断言3：按 prev → 图1（★ 这条就是"最近一次装载"方案会错的场景：
//              最近装的是邻居，不是图1）
//      断言4：再按 prev → 图0（窗口头，不滑窗）
//
//  【跑法】
//    cd src/user_source/hdl_source/APP
//    iverilog -g2012 -s tb_res_slot -o /tmp/rs.vvp tb_res_slot.v img_switch.v
//    vvp /tmp/rs.vvp | grep -aE "FAIL|ALL PASS"
//==============================================================================
`timescale 1ns/1ps

module tb_res_slot;

parameter [20:0] T0 = 21'd100, T1 = 21'd200, T2 = 21'd300;

reg clk = 0, rst = 1;
always #5 clk = ~clk;

// ---- 换图请求 ----
reg        I_next = 0, I_prev = 0, I_jump = 0;
reg  [7:0] I_jump_idx = 0;
reg        I_pf_en = 1;

// ---- 相册 ----
wire [15:0] O_base;
reg         I_any = 1;
reg  [2:0]  I_nimg = 3'd4;
reg         I_win4 = 1;
reg         I_alb_ready = 1;

// ---- 缩略图装载 ----
wire        O_tl_req;
wire [1:0]  O_tl_rel;
wire [20:0] O_tl_addr;
reg         I_tl_take = 0, I_tl_fin = 0, I_tl_ok = 1, I_tl_err = 0, I_tl_abort = 0;

// ---- 结果 ----
wire        O_go;
wire        O_dir;
wire [1:0]  O_cur_rel;
wire [20:0] O_cur_addr, O_old_addr;
wire        O_cur_ok;
wire [7:0]  O_cur;
wire [12:0] O_res_w, O_res_h;

//==============================================================================
//  装载器模型：W/H 跟着"这次装的是哪张图"变（就是 ld_src_w/h 的真实行为）
//==============================================================================
//  取单那一拍把"图号"锁下来（真机上 ld_off 就是这一拍锁的），之后 base 变了
//  也不影响本次装载的头信息。
reg  [7:0] ld_img;
reg        ld_busy;
reg  [3:0] ld_cnt;
wire [15:0] ld_w = 16'd320 + ({8'd0, ld_img} << 4);   // 320 + 16k
wire [15:0] ld_h = 16'd240 + ({8'd0, ld_img} << 3);   // 240 + 8k

// ★ 收单必须**组合**（真机里 tl_take = 序列器的组合条件，与 tl_req 同拍），
//   否则 register 一拍之后 O_tl_rel 已经翻到下一张（nc → nn），
//   img_switch 锁的 ts 和本台锁的图号就不是同一张了 —— 那是台的错，不是 RTL 的。
wire ld_take = O_tl_req && !ld_busy;

initial begin
    $dumpfile("tb_res_slot.vcd");
    $dumpvars(0, tb_res_slot);
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        I_tl_take <= 1'b0; I_tl_fin <= 1'b0; ld_busy <= 1'b0; ld_cnt <= 4'd0;
        ld_img <= 8'd0;
    end else begin
        I_tl_take <= 1'b0;
        I_tl_fin  <= 1'b0;
        if (!ld_busy) begin
            if (ld_take) begin
                I_tl_take <= 1'b1;
                ld_img    <= O_base[7:0] + {6'd0, O_tl_rel};
                ld_busy   <= 1'b1;
                ld_cnt    <= 4'd0;
            end
        end else begin
            ld_cnt <= ld_cnt + 4'd1;
            if (ld_cnt == 4'd7) begin
                I_tl_fin <= 1'b1;
                ld_busy  <= 1'b0;
            end
        end
    end
end

img_switch #(.THUMB0_ADDR(T0), .THUMB1_ADDR(T1), .THUMB2_ADDR(T2)) dut (
    .clk(clk), .rst(rst),
    .I_next(I_next), .I_prev(I_prev), .I_jump(I_jump), .I_jump_idx(I_jump_idx),
    .I_pf_en(I_pf_en),
    .O_base(O_base), .I_any(I_any), .I_nimg(I_nimg), .I_win4(I_win4),
    .I_alb_ready(I_alb_ready), .O_alb_ok(),
    .O_tl_req(O_tl_req), .O_tl_rel(O_tl_rel), .O_tl_addr(O_tl_addr),
    .I_tl_take(I_tl_take), .I_tl_fin(I_tl_fin), .I_tl_ok(I_tl_ok),
    .I_tl_err(I_tl_err), .I_tl_abort(I_tl_abort),
    .I_src_w(ld_w), .I_src_h(ld_h),
    .O_res_w(O_res_w), .O_res_h(O_res_h),
    .O_go(O_go), .O_dir(O_dir), .O_cur_rel(O_cur_rel),
    .O_cur_addr(O_cur_addr), .O_old_addr(O_old_addr),
    .O_cur_ok(O_cur_ok), .O_cur(O_cur)
);

//==============================================================================
//  断言工具
//==============================================================================
integer fails = 0, checks = 0;

task chk_res;
    input [8*28:1] name;
    input [7:0]    img;          // 期望是第几张图的分辨率
    begin
        checks = checks + 1;
        if ((O_res_w !== (13'd320 + {5'd0, img, 4'b0000})) ||
            (O_res_h !== (13'd240 + {5'd0, img, 3'b000}))) begin
            fails = fails + 1;
            $display("*** FAIL  %0s : got %0d x %0d, expect %0d x %0d (img %0d)",
                     name, O_res_w, O_res_h,
                     320 + 16*img, 240 + 8*img, img);
        end else
            $display("ok    %0s : %0d x %0d (img %0d)", name, O_res_w, O_res_h, img);
    end
endtask

task nframe;
    input integer n;
    integer i;
    begin for (i = 0; i < n; i = i + 1) @(posedge clk); end
endtask

// 等"装载空闲"（没有单在飞、没有预取在等）
//  ★ 必须连 `ld_on` / `I_tl_fin` 一起等：`I_tl_fin` 那一拍装的是"物理上刚落地"，
//    而 img_switch 的 ld_on 要下一拍才清 ⇒ 只等本台的 ld_busy 会在最后一拍
//    提前返回，于是"按 next"落在装载还没登记完的那一拍（vn=0，白按一次）。
task idle_wait;
    input integer maxn;
    integer i;
    begin
        i = 0;
        @(posedge clk);
        // ★ 还要等"相册重扫"窗口过去（alb_ok=0 期间预取是停摆的），
        //   否则换图后立刻返回，预取还没发就按下一次键 ⇒ 白按（vn/vp=0）。
        while ((ld_busy || O_tl_req || I_tl_fin || dut.ld_on ||
                !I_alb_ready || dut.bpend) && (i < maxn)) begin
            @(posedge clk); i = i + 1;
        end
        nframe(3);
    end
endtask

//==============================================================================
//  相册重扫模型
//==============================================================================
//  ★ 滑窗（base±1）之后 album_src 会拉低 O_ready 再拉高，期间 alb_ok=0 ⇒ 预取
//    停摆。不建模这一段，换图之后的预取就永远等不到（tv 一直是 0），台子会
//    把"没按动"误报成"分辨率不对"。
reg alb_pend;
reg [3:0] alb_cnt;
always @(posedge clk or posedge rst) begin
    if (rst) begin I_alb_ready <= 1'b1; alb_pend <= 1'b0; alb_cnt <= 4'd0; end
    else if (dut.bpend && !alb_pend) begin
        I_alb_ready <= 1'b0; alb_pend <= 1'b1; alb_cnt <= 4'd0;
    end else if (alb_pend) begin
        alb_cnt <= alb_cnt + 4'd1;
        if (alb_cnt == 4'd7) begin I_alb_ready <= 1'b1; alb_pend <= 1'b0; end
    end
end

//==============================================================================
//  换图：单拍 → 当拍（提交之前）查 O_go → 提交 → 等 1 拍再查分辨率
//==============================================================================
task key_go;
    input next_not_prev;
    begin
        @(negedge clk);
        if (next_not_prev) I_next = 1'b1; else I_prev = 1'b1;
        #1;
        $display("      [dbg] cs=%0d cr=%0d tv=%b dv=%b sn=%0d vn=%0d ld_on=%0d go=%0d",
                 dut.cs, dut.cr, dut.tv, dut.dv, dut.sn, dut.vn, dut.ld_on, O_go);
        // O_go 是与 I_next 同拍的组合信号；它在 posedge（cs 更新）之后就会落
        if (!O_go) begin
            fails = fails + 1;
            $display("*** FAIL  O_go did not assert (next=%0d, cur=%0d, rel=%0d)",
                     next_not_prev, O_cur, O_cur_rel);
        end
        @(posedge clk);            // cs / base / cr 在这一拍更新
        @(negedge clk);
        I_next = 1'b0; I_prev = 1'b0;
        @(posedge clk);
        #1;                        // 分辨率是组合读，已经跟着 cs 变了
    end
endtask

//==============================================================================
//  主场景
//==============================================================================
initial begin
    repeat (4) @(posedge clk);
    rst = 0;
    nframe(4);

    // 开 机：装 cur(图0) → 预取 next(图1)
    idle_wait(200);
    $display("--- A. boot: cur=图%0d ---", O_cur);
    chk_res("A1 boot res = img0", 8'd0);

    // 断言1：按 next → 图1（当拍就变）
    key_go(1'b1);
    chk_res("B1 next -> img1", 8'd1);

    // 预取 next(图2) 装进另一个槽；分辨率**不许**被它改掉
    idle_wait(200);
    chk_res("B2 after prefetch, still img1", 8'd1);

    // 断言2：再按 next → 图2
    key_go(1'b1);
    chk_res("C1 next -> img2", 8'd2);
    idle_wait(200);
    chk_res("C2 after prefetch, still img2", 8'd2);

    // ★ 断言3：按 prev → 图1。这就是"最近一次装载"方案会错的场景：
    //   刚刚装完的是邻居（图3 的预取），不是图1。
    key_go(1'b0);
    chk_res("D1 prev -> img1 (the key case)", 8'd1);
    idle_wait(200);
    chk_res("D2 after prefetch, still img1", 8'd1);

    // 断言4：再按 prev → 图0（窗口头）
    key_go(1'b0);
    chk_res("E1 prev -> img0", 8'd0);

    nframe(10);
    $display("");
    if (fails == 0) $display("*** ALL PASS (%0d checks)", checks);
    else            $display("*** %0d/%0d FAIL", fails, checks);
    $finish;
end

endmodule
