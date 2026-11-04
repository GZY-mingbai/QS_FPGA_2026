//==============================================================================
//  tb_trans.v —— 键按下 → S_ARM → S_TRANS → S_HDF → S_HD 的端到端回归
//==============================================================================
//
//  【为什么必须单独建这个台】
//  现有三个台都盖不到这条路径：
//    · tb_ctrl      用的是假帧源（ps_start/busy 自己造），没有 App 的帧节奏
//    · tb_app_full  只跑到 state 5（上电进 HD），从不按键
//    · tb_viewer    只测场景层
//  于是下面这四类 bug 全是从仿真盲区漏到板子上的（都真实发生过）：
//    ① 按键那一拍就踢 HD 装载 ⇒ loader 的第一步是 LCS 预清（洋红 0xFFFF00FF），
//       而此刻 a_hd 还是 1、A 层正在读 HD 槽 ⇒ 屏幕闪一下紫
//    ② S_HDF 只看 `busy` 电平 ⇒ 把"上一帧"当成新帧提交，888 从没被合成过
//       ⇒ 过渡结束/上电后停在放大 2 倍的缩略图上
//    ③ S_TRANS 没有超时 ⇒ 任何一帧 stall 就把 t 永久冻结，状态卡在 3
//    ④ 从 S_TRANS 降级出去时 b_en 没关 ⇒ B 层永久参与 pixel_ready
//
//  【纹理源故意做慢、做抖】
//  板上 SDRAM 是"16 个时隙里分 4 个给写、纹理读要和显示读抢"，
//  而原来的仿真桩是"ack 完立刻给 4 个字"。这里给每层加
//    · ack 前的随机延迟
//    · 逐字之间的随机间隔
//    · 两层抢同一个服务令牌（任一时刻只有一层能被服务）
//  这样 need_pix 才会真的被拉长 —— 这也是 ps_gen_image.v:146 里写的
//  "仿真不复现（仿真纹理源太快）" 的解法。
//
//  跑法：
//    iverilog -g2005 -s tb_trans -o /tmp/tb_trans.vvp \
//        APP/App.v APP/viewer_ctrl.v APP/viewer_scene.v APP/viewer_fx.v \
//        APP/ps_merge_alpha8.v APP/ps_to_rgb24.v APP/app_scan_gen.v \
//        APP/img_layer_ctrl.v APP/img_layer_hd.v APP/ps_gen_image.v APP/img_afifo.v \
//        frame_writer.v APP/tb_trans.v && vvp /tmp/tb_trans.vvp
//
//==============================================================================

`timescale 1ns / 1ps

module tb_trans;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;
parameter  integer T_END = 3;                   // 仿真压缩：真实是 300

parameter integer TEX_JIT  = 0;                 // 逐字随机间隔上限（拍）；0=不限速
parameter integer SLOW_K   = 1;                 // 纹理服务整体降速倍数
parameter integer FX_TARGET = 0;                // 过渡前先按几次 key_fx（0/1/2/3）
// ★ 2026-10-23：把板上的配额/帧周期也做成参数 —— 之前固定 127/127（解除限速），
//   和板上 48/96 差 2.6 倍，所以复现不出板上的现象。
parameter integer SCENE_QUOTA = 127;
parameter integer QUOTA_565   = 127;
parameter integer FRAME_CYC   = 200000;

localparam [20:0] T0_BASE = 21'd614400;
localparam [20:0] T1_BASE = 21'd652800;
localparam [20:0] T2_BASE = 21'd691200;
localparam [20:0] HD_BASE = 21'd729600;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

//------------------------------------------------------------------------------
//  DUT：App + frame_writer
//------------------------------------------------------------------------------
wire        ps_start;
wire        ps_new_frame;
wire [23:0] ps_data;
wire        ps_valid;
wire        app_pipe_ce;
wire        app_busy;

wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
wire [1:0]  tex_pix_wr;
wire [63:0] tex_pix_data;

reg  [1:0]  ack_r = 0;
reg  [1:0]  pwr_r = 0;
reg  [63:0] pd_r  = 0;
wire [1:0]  tex_req_ack  = ack_r;
assign tex_pix_wr   = pwr_r;
assign tex_pix_data = pd_r;

reg         key_next = 0, key_prev = 0, key_fx = 0;

wire        ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg         ld_done = 0;
wire        fb_fmt565;
reg         wr_stall = 0;

wire [1:0]  st_img, st_fx;
wire [3:0]  st_state;          // ★ 2026-10-31：3→4 位（状态码已到 11）

wire        app_write_req;
wire [1:0]  write_buf_idx;
reg         app_write_req_ack = 0;
wire        app_write_en;
wire [31:0] app_write_data;
reg         write_finish_pulse = 0;
wire [1:0]  disp_buf_idx;
wire        display_valid;
wire        fw_commit_tgl;
reg         disp_adopt_sync = 0;
reg  [15:0] adopt_dly = 0;

localparam [31:0] IMG0_OFFSET = 32'd8843264;
localparam [31:0] IMG1_OFFSET = 32'd11620352;
localparam [31:0] IMG2_OFFSET = 32'd9768960;

App #(
    .IMG_W (IMG_W), .IMG_H (IMG_H),
    // ★ 2026-10-31：App 后来把快效果（fx0/1/2）拆成 T_END_F（默认 60）；只压 T_END
    //   压不到 fx0 ⇒ 过渡要 60×FRAME_CYC 拍，本台 3M 拍的等待必超时。两个一起压。
    .T_END (T_END), .T_END_F (T_END), .ARM_FRAMES (0),
    // ★ 仿真解除配额限速：888 帧从 819200 拍降到 307200 拍（约 2.6×）
    .SCENE_QUOTA (SCENE_QUOTA), .QUOTA_565 (QUOTA_565),
    .FRAME_PERIOD_CYCLES (FRAME_CYC),       // 仿真放快：4ms/帧
    .THUMB0_ADDR (T0_BASE), .THUMB1_ADDR (T1_BASE),
    .THUMB2_ADDR (T2_BASE), .HD_ADDR (HD_BASE)
) u_app (
    .clk (clk), .rst (rst), .mclk (mclk),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT）
    .I_img_off0 (IMG0_OFFSET), .I_img_off1 (IMG1_OFFSET), .I_img_off2 (IMG2_OFFSET),
    .I_img_off3(32'd0),
    .ps_start (ps_start), .ps_new_frame (ps_new_frame),
    .ps_data (ps_data), .ps_valid (ps_valid), .pipe_ce (app_pipe_ce),
    .busy (app_busy),
    .tex_req_vld (tex_req_vld), .tex_req_addr (tex_req_addr),
    .tex_req_mask (tex_req_mask), .tex_req_ack (tex_req_ack),
    .tex_pix_wr (tex_pix_wr), .tex_pix_data (tex_pix_data),
    .key_prev (key_prev), .key_next (key_next), .key_fx (key_fx),
    .ld_req (ld_req), .ld_fmt (ld_fmt), .ld_off (ld_off), .ld_len (ld_len),
    .ld_base (ld_base),
    .ld_can_load (1'b1), .ld_done (ld_done), .ld_err (1'b0),
    .ld_sd_ready (1'b1),
    .fmt565 (fb_fmt565), .wr_stall (wr_stall),
    .st_img (st_img), .st_fx (st_fx),
    // ★ 2026-10-31：App 端口漂移修补 —— st_state 改名 dbg_state（4 位，状态已到 11），
    //   I_have 改名 I_alb_have；以下新增输入口原先悬空（Z），全部钉成确定值。
    .dbg_state (st_state),
    .I_alb_any (1'b1), .I_alb_ready (1'b1), .I_alb_win4 (1'b0), .I_quad_mode (1'b0), .I_quad_vm (4'd0),
    .I_nm_char (8'd0), .I_nm_blen (4'd0), .I_nm_elen (4'd0), .I_nm_len4 (20'd0), .I_nm_vld (1'b0),
    .I_fp_vcnt (3'd0), .I_fp_has_more (1'b0), .I_fp_gen (1'b0),
    .I_fp_has_prev (1'b0), .O_fp_open (), .I_jump (1'b0), .I_jump_idx (8'd0), .O_ld_abort (),
    .key_menu (1'b0), .key_level (4'hF), .bright_eff(7'd50), .flash_miso (1'b1),
    .bp_req (1'b0), .bp_addr (24'd0), .bp_len (16'd0),
    .ld_src_w (16'd640), .ld_src_h (16'd480), .music_done (1'b0)
);

frame_writer #(.IMG_W (IMG_W), .IMG_H (IMG_H), .NPX (NPX), .PIPE (4)) u_fw (
    .clk (clk), .rst (rst),
    .ps_start (ps_start), .ps_new_frame (ps_new_frame),
    .ps_data (ps_data), .ps_valid (ps_valid), .pipe_ce (app_pipe_ce),
    .write_req (app_write_req), .write_buf_idx (write_buf_idx), .write_len (),
    .write_req_ack (app_write_req_ack),
    .write_en (app_write_en), .write_data (app_write_data),
    .write_finish_pulse (write_finish_pulse),
    .disp_buf_idx (disp_buf_idx), .display_valid (display_valid),
    .disp_commit_tgl (fw_commit_tgl), .disp_adopt (disp_adopt_sync),
    .busy (app_busy), .done (),
    .px_x (), .px_y (), .out_x (), .out_y ()
);

//------------------------------------------------------------------------------
//  显示侧采纳号（模拟等场边界）
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin disp_adopt_sync <= 1'b0; adopt_dly <= 16'd0; end
    else if (fw_commit_tgl !== disp_adopt_sync) begin
        if (adopt_dly == 16'd5000) begin disp_adopt_sync <= fw_commit_tgl; adopt_dly <= 16'd0; end
        else adopt_dly <= adopt_dly + 16'd1;
    end else adopt_dly <= 16'd0;
end

//------------------------------------------------------------------------------
//  模型 1：装载器
//------------------------------------------------------------------------------
reg [9:0] ldcnt = 0;
reg       ldbusy = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin ld_done <= 1'b0; ldcnt <= 0; ldbusy <= 0; end
    else begin
        ld_done <= 1'b0;
        if (ld_req) begin ldbusy <= 1'b1; ldcnt <= 0; end
        else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 400) begin ld_done <= 1'b1; ldbusy <= 1'b0; end
        end
    end
end

//------------------------------------------------------------------------------
//  模型 2：慢速 + 抖动的纹理源（两层抢一个服务令牌）
//------------------------------------------------------------------------------
function [31:0] tex_word;
    input [20:0] a;
    integer w, row, col, cw, p0, p1;
    reg [4:0] r5_0, r5_1, b5;
    reg [5:0] g6_0, g6_1;
    begin
        if (a >= HD_BASE) begin
            w   = a - HD_BASE;
            row = w / IMG_W;
            col = w % IMG_W;
            tex_word = {8'hFF, col[7:0], row[7:0], 8'h5A};
        end else begin
            w   = a - ((a >= T2_BASE) ? T2_BASE :
                       (a >= T1_BASE) ? T1_BASE : T0_BASE);
            row = w / 160;
            cw  = w % 160;
            p0  = cw * 2;
            p1  = p0 + 1;
            r5_0 = (p0 >> 4) & 31;  g6_0 = p0 & 63;
            r5_1 = (p1 >> 4) & 31;  g6_1 = p1 & 63;
            b5   = (row >> 3) & 31;
            tex_word = {r5_1, g6_1, b5, r5_0, g6_0, b5};
        end
    end
endfunction

integer srv_st0, srv_st1;
reg [20:0] srv_a0, srv_a1;
reg [3:0]  srv_k0, srv_k1;
// ★ 2026-10-31：build F 起每个请求 = 一个 4 字组 + 4 位组掩码，调度器**只回掩码里的字**
//   （组内偏移升序）。原模型不看掩码、每次回满 4 字 ⇒ 层收到没要的像素、计数错位
//   ⇒ 两层都"卡住 warmed=0"、过渡走不完。现在按 sdr_port_sched 的契约回字。
reg [3:0]  srv_m0, srv_m1;
integer    gap0, gap1;

// 服务令牌：0 = layer0 可以动，1 = layer1 可以动
reg tok = 0;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        srv_st0 <= 0; srv_st1 <= 0; ack_r <= 0; pwr_r <= 0; pd_r <= 0;
        gap0 <= 0; gap1 <= 0; srv_k0 <= 0; srv_k1 <= 0;
        srv_a0 <= 0; srv_a1 <= 0; tok <= 0; srv_m0 <= 0; srv_m1 <= 0;
    end else begin
        pwr_r <= 2'b00;
        ack_r <= 2'b00;

        // ---- layer0 ----
        case (srv_st0)
        // ★ 2026-10-31：令牌只在"对方正在被服务完"时交接 ⇒ 一层不再发请求后，
        //   另一层永远拿不到令牌（死锁：B 层 rdy 永远 0、帧写不完、过渡走不完）。
        //   改成：对方**空闲且没在请求**时也可以直接服务；两层仍然互斥。
        0: if (tex_req_vld[0] && srv_st1 == 0 && (tok == 1'b0 || !tex_req_vld[1])) begin
               srv_a0  <= {tex_req_addr[20:2], 2'b00};     // 组首（调度器也只看 [20:2]）
               srv_m0  <= tex_req_mask[3:0];
               srv_st0 <= 1;
               gap0    <= ($random & 32'h7FFF_FFFF) % (SLOW_K * 8) + SLOW_K * 4;
           end
        1: if (gap0 != 0) gap0 <= gap0 - 1;
           else begin ack_r <= ack_r | 2'b01; srv_k0 <= 0; srv_st0 <= 2; end
        2: if (!srv_m0[srv_k0[1:0]]) begin               // 掩码外的字：不投，直接跳过
               if (srv_k0 == 3) begin srv_st0 <= 0; tok <= tex_req_vld[1]; end
               else             srv_k0 <= srv_k0 + 1;
           end else if (gap0 != 0) gap0 <= gap0 - 1;
           else begin
               pwr_r[0]   <= 1'b1;
               pd_r[31:0] <= tex_word(srv_a0 + srv_k0);
               gap0       <= ($random & 32'h7FFF_FFFF) % (TEX_JIT + 1);
               // ★ 只有 layer1 真的在等，才把令牌交出去；否则 layer0 永久饿死
               if (srv_k0 == 3) begin srv_st0 <= 0; tok <= tex_req_vld[1]; end
               else             srv_k0 <= srv_k0 + 1;
           end
        default: srv_st0 <= 0;
        endcase

        // ---- layer1 ----
        case (srv_st1)
        0: if (tex_req_vld[1] && srv_st0 == 0 && (tok == 1'b1 || !tex_req_vld[0])) begin
               srv_a1  <= {tex_req_addr[41:23], 2'b00};
               srv_m1  <= tex_req_mask[7:4];
               srv_st1 <= 1;
               gap1    <= ($random & 32'h7FFF_FFFF) % (SLOW_K * 8) + SLOW_K * 4;
           end
        1: if (gap1 != 0) gap1 <= gap1 - 1;
           else begin ack_r <= ack_r | 2'b10; srv_k1 <= 0; srv_st1 <= 2; end
        2: if (!srv_m1[srv_k1[1:0]]) begin
               if (srv_k1 == 3) begin srv_st1 <= 0; tok <= 1'b0; end
               else             srv_k1 <= srv_k1 + 1;
           end else if (gap1 != 0) gap1 <= gap1 - 1;
           else begin
               pwr_r[1]    <= 1'b1;
               pd_r[63:32] <= tex_word(srv_a1 + srv_k1);
               gap1        <= ($random & 32'h7FFF_FFFF) % (TEX_JIT + 1);
               if (srv_k1 == 3) begin srv_st1 <= 0; tok <= 1'b0; end
               else             srv_k1 <= srv_k1 + 1;
           end
        default: srv_st1 <= 0;
        endcase
    end
end

//------------------------------------------------------------------------------
//  模型 3：写引擎（按字数计）
//------------------------------------------------------------------------------
reg [19:0] wcnt;
reg [5:0]  fdelay;
reg        wr_fmt_l, pk_ph, fin_armed;
integer    finishes = 0;
wire [19:0] words_target = wr_fmt_l ? 20'd153600 : 20'd307200;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        app_write_req_ack <= 1'b0; wcnt <= 0; fdelay <= 0;
        write_finish_pulse <= 1'b0; wr_fmt_l <= 0; pk_ph <= 0; fin_armed <= 0;
    end else begin
        write_finish_pulse <= 1'b0;
        if (app_write_req && !app_write_req_ack) begin
            app_write_req_ack <= 1'b1;
            wcnt <= 0; fdelay <= 0; pk_ph <= 0; fin_armed <= 0;
            wr_fmt_l <= fb_fmt565;
        end else if (app_write_req_ack && !app_write_req) begin
            app_write_req_ack <= 1'b0;
        end
        if (app_write_en) begin
            if (wr_fmt_l) begin
                if (pk_ph) wcnt <= wcnt + 20'd1;
                pk_ph <= ~pk_ph;
            end else wcnt <= wcnt + 20'd1;
        end
        if (wcnt >= words_target) fin_armed <= 1'b1;
        if (fin_armed) begin
            if (fdelay == 6'd20) begin
                write_finish_pulse <= 1'b1;
                finishes = finishes + 1;
                fdelay <= 6'd21; fin_armed <= 1'b0;
            end else if (fdelay < 6'd20) fdelay <= fdelay + 6'd1;
        end
    end
end

//------------------------------------------------------------------------------
//  检查
//------------------------------------------------------------------------------
integer errors = 0;
integer last_state = -1;
integer ticks_tr   = 0;        // S_TRANS 里数到几个 tick
integer ld_ovl     = 0;        // ld_req 与 a_hd 同时为 1 的拍数（洋红窗口）
integer ph0, ph1, ph2;         // S_HDF 三相是否都走过
integer stall_seen = 0;
integer hdf_cyc    = 0;
// ★ 2026-10-23 新增：装载参数错配计数
//   【要抓的 bug】ld_kick 只给脉冲、不给 ld_off/ld_img_d 时，装载器会用上一次的
//   参数 —— 把"第 0 张"装进高清槽，同时把标签记成目标张，于是 hd_ready 反而为真、
//   握手全部通过，最后屏幕上"过渡结束又回到第一张图"。这种错不报任何错，只能断言。
integer ld_param_bad = 0;
integer ld_hd_cnt    = 0;
reg [31:0] exp_off;

// ★ 2026-10-23：卡帧定位。记录"某一层的 layer_ready 连续为 0"的最长拍数，
//   并在超长时把两层的 warmed / rq_idx / ds_idx / FIFO 水位一次性打出来。
//   上板现象"fx0/fx3 卡住"在理想纹理源下不一定复现，所以这里只在
//   TEX_JIT>0（抖动源）时才有意义。
integer ta_wait = 0, tb_wait = 0, wait_max = 0;
integer dumped = 0;
wire ta_rdy = u_app.u_scene.u_ta.layer_ready;
wire tb_rdy = u_app.u_scene.u_tb.layer_ready;

function [31:0] img_off_f;
    input [1:0] i;
    begin
        case (i)
            2'd0:    img_off_f = IMG0_OFFSET;
            2'd1:    img_off_f = IMG1_OFFSET;
            default: img_off_f = IMG2_OFFSET;
        endcase
    end
endfunction

always @(posedge clk) begin
    if (!rst) begin
        // ---- ① 洋红窗口：HD 槽**正在被读**的时候绝不能发装载请求 ----
        //   判据必须带 compose_en：S_ARM 相位 0 里 a_hd 还是 1（A 指着高清槽），
        //   但 compose_en=0、没有帧在飞 ⇒ 一个像素都不会被读走，预清看不见。
        if (ld_req && u_app.u_ctrl.a_hd && u_app.u_ctrl.compose_en) ld_ovl = ld_ovl + 1;

        // ---- ①b 装载参数必须整套对准目标图（只查高清装载） ----
        // ★ 2026-11-04：装载请求改由序列器按当前窗口现取（ld_kick/ld_img_d/tgt_idx 已删）。
        //   判据：每次发出的高清请求，偏移 = 当前图（cur_abs）的偏移。
        if (ld_req === 1'b1 && ld_fmt === 1'b1) begin
            ld_hd_cnt = ld_hd_cnt + 1;
            exp_off   = img_off_f(u_app.u_ctrl.cur_abs[1:0]);
            if (ld_off !== exp_off) begin
                $display("  [FAIL] 高清装载偏移错：ld_off=%0d 应为 %0d（图 %0d）",
                         ld_off, exp_off, u_app.u_ctrl.cur_abs);
                ld_param_bad = ld_param_bad + 1;
            end
            if (ld_base !== HD_BASE) begin
                $display("  [FAIL] 高清装载落点错：ld_base=%0d 应为 %0d", ld_base, HD_BASE);
                ld_param_bad = ld_param_bad + 1;
            end
        end

        if (st_state !== last_state) begin
            $display("  [t=%0t] state %0d -> %0d  img=%0d fmt565=%b a_hd=%b live=%b",
                     $time, last_state, st_state, st_img, fb_fmt565,
                     u_app.u_ctrl.a_hd, display_valid);
            $fflush();
            last_state = st_state;
        end

        if (st_state == 3'd3) begin
            if (u_app.u_ctrl.tick) begin
                $display("  [tick] t=%0d cyc=%0t tstep=%0d", u_app.u_ctrl.t, $time, u_app.u_ctrl.tstep);
                ticks_tr = ticks_tr + 1;
            end
            if (u_app.u_ctrl.trans_stall) stall_seen = stall_seen + 1;

            // ---- 卡帧探针 ----
            if (!ta_rdy) ta_wait = ta_wait + 1; else ta_wait = 0;
            if (!tb_rdy) tb_wait = tb_wait + 1; else tb_wait = 0;
            if (ta_wait > wait_max) wait_max = ta_wait;
            if (tb_wait > wait_max) wait_max = tb_wait;

            if (dumped < 2 && (ta_wait == 3000 || tb_wait == 3000)) begin
                dumped = dumped + 1;
                $display("  [探针] 层卡住 3000 拍：ta_rdy=%b tb_rdy=%b ready_i=%b", ta_rdy, tb_rdy, u_app.u_scene.pixel_ready_i);
                $display("        ps_start=%b compose_en=%b app_busy=%b ps_new_frame=%b fw_buf_free=%b fw_state=%0d commit=%b adopt=%b",
                         ps_start, u_app.compose_en, app_busy, ps_new_frame,
                         u_fw.buf_free, u_fw.state, fw_commit_tgl, disp_adopt_sync);
                $display("        A(thumb): warmed=%b en=%b rq_idx=%0d ds_idx=%0d count=%0d rq_x=%0d rq_y=%0d box=(%0d,%0d) %0dx%0d k=%0d",
                    u_app.u_scene.u_ta.u_gen.warmed, u_app.u_scene.u_ta.u_gen.f_en,
                    u_app.u_scene.u_ta.u_gen.rq_idx, u_app.u_scene.u_ta.u_gen.ds_idx,
                    u_app.u_scene.u_ta.u_gen.pix_count,
                    u_app.u_scene.u_ta.u_gen.rq_x, u_app.u_scene.u_ta.u_gen.rq_y,
                    $signed(u_app.u_scene.u_ta.u_gen.f_x0), $signed(u_app.u_scene.u_ta.u_gen.f_y0),
                    u_app.u_scene.u_ta.u_gen.f_rw, u_app.u_scene.u_ta.u_gen.f_rh,
                    $signed(u_app.u_scene.u_ta.u_gen.f_kx));
                $display("        B(thumb): warmed=%b en=%b rq_idx=%0d ds_idx=%0d count=%0d rq_x=%0d rq_y=%0d box=(%0d,%0d) %0dx%0d k=%0d",
                    u_app.u_scene.u_tb.u_gen.warmed, u_app.u_scene.u_tb.u_gen.f_en,
                    u_app.u_scene.u_tb.u_gen.rq_idx, u_app.u_scene.u_tb.u_gen.ds_idx,
                    u_app.u_scene.u_tb.u_gen.pix_count,
                    u_app.u_scene.u_tb.u_gen.rq_x, u_app.u_scene.u_tb.u_gen.rq_y,
                    $signed(u_app.u_scene.u_tb.u_gen.f_x0), $signed(u_app.u_scene.u_tb.u_gen.f_y0),
                    u_app.u_scene.u_tb.u_gen.f_rw, u_app.u_scene.u_tb.u_gen.f_rh,
                    $signed(u_app.u_scene.u_tb.u_gen.f_kx));
                $fflush();
            end
        end

        if (st_state == 3'd4) begin
            hdf_cyc = hdf_cyc + 1;
            case (u_app.u_ctrl.hdf_ph)
            2'd0: ph0 = ph0 + 1;
            2'd1: ph1 = ph1 + 1;
            2'd2: ph2 = ph2 + 1;
            endcase
        end
    end
end

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
//  心跳：每 20 万拍打一行，用来区分"仿真慢"和"真卡死"
reg [19:0] hb = 20'd0;
always @(posedge clk) begin
    if (!rst) begin
        if (hb == 20'd199_999) begin
            hb <= 20'd0;
            $display("      ... %0d 万拍  state=%0d img=%0d ticks=%0d finishes=%0d",
                     (hb/10000), st_state, st_img, ticks_tr, finishes);
            $fflush();
        end else hb <= hb + 20'd1;
    end
end

initial begin
    ph0 = 0; ph1 = 0; ph2 = 0; hdf_cyc = 0;
    repeat (20) @(posedge clk);
    rst = 0;

    //---- 上电：等进 HD（5）----
    fork : wait_hd
        begin wait (st_state == 3'd5); disable wait_hd; end
        begin
            repeat (4000000) @(posedge clk);
            $display("  [FAIL] 上电进不了 HD（停在 %0d）", st_state);
            errors = errors + 1; disable wait_hd;
        end
    join
    if (errors != 0) begin
        $display("\n  [中止] 前面已失败，不再往下跑（避免级联误报）");
        $display("\n==== errors = %0d ====", errors);
        $display("FAIL");
        $finish;
    end
    $display("  [ ok ] 上电进入 HD，img=%0d，fmt565=%b，live=%b", st_img, fb_fmt565, display_valid);
    if (display_valid !== 1'b1) begin
        $display("  [FAIL] 进 HD 了但 display_valid=0"); errors = errors + 1;
    end
    if (fb_fmt565 !== 1'b0) begin
        $display("  [FAIL] 进 HD 了但 fmt565 还是 1（不是 888）"); errors = errors + 1;
    end

    //---- 切到指定效果（默认 fx0；上板实测 fx0/fx3 会卡）----
    if (FX_TARGET != 0) begin
        repeat (FX_TARGET) begin
            @(negedge clk); key_fx = 1; @(negedge clk); key_fx = 0;
            repeat (10) @(posedge clk);
        end
        $display("  [ok] 已切到 fx%0d（st_fx=%0d）", FX_TARGET, st_fx);
        if (st_fx !== FX_TARGET[1:0]) begin
            $display("  [FAIL] fx 没切到 %0d（=%0d）", FX_TARGET, st_fx);
            errors = errors + 1;
        end
    end

    //---- 按 key_next ----
    ph0 = 0; ph1 = 0; ph2 = 0;
    @(negedge clk); key_next = 1; @(negedge clk); key_next = 0;

    // 必须经过 S_ARM(7)
    fork : wait_arm
        begin wait (st_state == 3'd7); disable wait_arm; end
        begin
            repeat (200000) @(posedge clk);
            $display("  [FAIL] 没进 S_ARM（停在 %0d）", st_state);
            errors = errors + 1; disable wait_arm;
        end
    join
    $display("  [ ok ] 进入 S_ARM(7)");
    if (errors != 0) begin
        $display("\n  [中止] 前面已失败，不再往下跑（避免级联误报）");
        $display("\n==== errors = %0d ====", errors);
        $display("FAIL");
        $finish;
    end


    // 必须进 S_TRANS(3)
    fork : wait_tr
        begin wait (st_state == 3'd3); disable wait_tr; end
        begin
            repeat (2000000) @(posedge clk);
            $display("  [FAIL] 没进 S_TRANS（停在 %0d）", st_state);
            errors = errors + 1; disable wait_tr;
        end
    join
    $display("  [ ok ] 进入 S_TRANS(3)");
    if (errors != 0) begin
        $display("\n  [中止] 前面已失败，不再往下跑（避免级联误报）");
        $display("\n==== errors = %0d ====", errors);
        $display("FAIL");
        $finish;
    end


    // 过渡必须走满 T_END+1 个 tick，且不许触发看门狗
    fork : wait_tr_end
        begin wait (st_state == 3'd4); disable wait_tr_end; end
        begin
            repeat (3000000) @(posedge clk);
            $display("  [FAIL] 过渡结束不了（停在 %0d，ticks=%0d）", st_state, ticks_tr);
            errors = errors + 1; disable wait_tr_end;
        end
    join
    $display("       过渡共 %0d 个 tick（期望 %0d）", ticks_tr, T_END + 1);
    if (ticks_tr < T_END) begin
        $display("  [FAIL] 过渡只走了 %0d 帧，少于 T_END+1", ticks_tr);
        errors = errors + 1;
    end
    if (stall_seen != 0) begin
        $display("  [FAIL] 过渡看门狗被触发过 %0d 拍 —— 有帧 stall 了", stall_seen);
        errors = errors + 1;
    end

    // 回到 HD
    fork : wait_hd2
        begin wait (st_state == 3'd5); disable wait_hd2; end
        begin
            repeat (1200000) @(posedge clk);
            $display("  [FAIL] 过渡后回不到 HD（停在 %0d）", st_state);
            errors = errors + 1; disable wait_hd2;
        end
    join
    $display("  [ ok ] 回到 S_HD(5)，img=%0d，fmt565=%b，live=%b", st_img, fb_fmt565, display_valid);

    // ★ 三相检查放在"到达 S_HD 之后"：如果在刚进 S_HDF 的那一拍就去读计数器，
    //   监控块还没跑，会读到全 0（这是本 tb 第一版的 bug）。
    if (ph0 == 0 || ph1 == 0 || ph2 == 0) begin
        $display("  [FAIL] S_HDF 三相没走全（ph0=%0d ph1=%0d ph2=%0d，停留 %0d 拍）⇒ 提交的是旧帧",
                 ph0, ph1, ph2, hdf_cyc);
        errors = errors + 1;
    end else $display("  [ ok ] S_HDF 三相齐全（旧帧排空 %0d 拍 → 新帧开工 %0d 拍 → 提交 %0d 拍）",
                      ph0, ph1, ph2);
    if (st_img !== 2'd1)  begin $display("  [FAIL] 图片号=%0d，应为 1", st_img); errors=errors+1; end
    if (fb_fmt565 !== 1'b0) begin $display("  [FAIL] 结束后不是 888"); errors=errors+1; end
    if (display_valid !== 1'b1) begin $display("  [FAIL] display_valid=0"); errors=errors+1; end

    // ① 洋红窗口
    if (ld_ovl != 0) begin
        $display("  [FAIL] ld_req 与 a_hd 同时为高 %0d 拍 ⇒ HD 槽在读取时被预清（洋红闪）",
                 ld_ovl);
        errors = errors + 1;
    end else $display("  [ ok ] 装载请求从未与 a_hd 重叠（无洋红窗口）");

    if (ld_param_bad != 0) begin
        $display("  [FAIL] 高清装载参数错配 %0d 次 ⇒ 会把别的图装进高清槽", ld_param_bad);
        errors = errors + 1;
    end else $display("  [ ok ] %0d 次高清装载，标签/偏移/落点全部对准目标图", ld_hd_cnt);

    $display("\n==== errors = %0d ====", errors);
    if (errors == 0) $display("PASS");
    else             $display("FAIL");
    $finish;
end

initial begin
    #3_000_000_000;
    $display("TIMEOUT  state=%0d ticks=%0d", st_state, ticks_tr);
    $finish;
end

endmodule
