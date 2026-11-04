//==============================================================================
//  tb_app_full.v —— App + frame_writer 端到端（"display_valid 到底会不会起来"）
//==============================================================================
//
//  【为什么写这个】
//  上板全屏黑，怀疑 display_valid 一直不拉高。这条链是：
//
//      viewer_ctrl.compose_en
//        → App.ps_new_frame（1 拍脉冲）
//          → frame_writer.req_latch
//            → write_req / ack → ps_start
//              → 307200 次 (ps_start & pixel_ready) 的 take
//                → 307200 个 write_en
//                  → write_finish_pulse
//                    → S_WAIT 提交 → display_valid = 1
//
//  链上任一环卡住，屏幕就是纯黑。这里把 **纹理源（理想）** 和
//  **写引擎（按格式折半计数）** 都换成玩具模型，只保留 App + frame_writer，
//  于是"卡在哪一环"可以被单独暴露出来。
//
//  【三个玩具模型】
//    1. 装载器：ld_can_load=1，ld_req 后 400 拍回 ld_done
//    2. 纹理：仿 tb_viewer，ack 后 4 拍回一组 4 字
//    3. 写口：write_req→ack；数 write_en，数够了就回 write_finish_pulse
//       （565 一帧 153600 字，888 一帧 307200 字 —— 这正是写引擎该看到的字数）
//
//  【跑法】
//    iverilog -g2005 -s tb_app_full -o ap.vvp \
//        APP/App.v APP/viewer_ctrl.v APP/viewer_scene.v APP/viewer_fx.v \
//        APP/ps_merge_alpha8.v APP/ps_to_rgb24.v APP/app_scan_gen.v \
//        APP/img_layer_ctrl.v APP/img_layer_hd.v APP/ps_gen_image.v APP/img_afifo.v \
//        frame_writer.v APP/tb_app_full.v
//    vvp ap.vvp
//
//==============================================================================

`timescale 1ns / 1ps

module tb_app_full;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;

localparam [20:0] T0_BASE = 21'd614400;
localparam [20:0] T1_BASE = 21'd652800;
localparam [20:0] HD_BASE = 21'd729600;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

//------------------------------------------------------------------------------
//  DUT：App
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
wire [1:0]  tex_req_ack;
wire [1:0]  tex_pix_wr;
wire [63:0] tex_pix_data;

wire        ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg         ld_done;
localparam [31:0] IMG0_OFFSET = 32'd8843264;
localparam [31:0] IMG1_OFFSET = 32'd11620352;
localparam [31:0] IMG2_OFFSET = 32'd9768960;

wire        fb_fmt565;
reg         wr_stall = 0;

wire [1:0]  st_img, st_fx;
wire [3:0]  st_state;          // ★ 2026-10-31：3→4 位（状态码已到 11）

// frame_writer 侧
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

App #(
    .IMG_W (IMG_W), .IMG_H (IMG_H),
    .FRAME_PERIOD_CYCLES (200000),       // 仿真放快一点：4ms/帧
    .THUMB0_ADDR (T0_BASE), .THUMB1_ADDR (T1_BASE),
    .THUMB2_ADDR (21'd691200), .HD_ADDR (HD_BASE)
) u_app (
    .clk (clk), .rst (rst), .mclk (mclk),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT）
    .I_img_off0 (IMG0_OFFSET), .I_img_off1 (IMG1_OFFSET), .I_img_off2 (IMG2_OFFSET),
    .I_img_off3(32'd0),
    .ps_start (ps_start),
    .ps_new_frame (ps_new_frame),
    .ps_data (ps_data),
    .ps_valid (ps_valid),
    .pipe_ce (app_pipe_ce),
    .busy (app_busy),
    .tex_req_vld (tex_req_vld), .tex_req_addr (tex_req_addr),
    .tex_req_mask (tex_req_mask), .tex_req_ack (tex_req_ack),
    .tex_pix_wr (tex_pix_wr), .tex_pix_data (tex_pix_data),
    .key_prev (1'b0), .key_next (1'b0), .key_fx (1'b0),
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

frame_writer #(
    .IMG_W (IMG_W), .IMG_H (IMG_H), .NPX (NPX), .PIPE (4)
) u_fw (
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

// 显示侧采纳号：比 commit 晚 5000 拍（模拟"等自己的场边界"）
always @(posedge clk or posedge rst) begin
    if (rst) begin
        disp_adopt_sync <= 1'b0;
        adopt_dly <= 16'd0;
    end else if (fw_commit_tgl !== disp_adopt_sync) begin
        if (adopt_dly == 16'd5000) begin
            disp_adopt_sync <= fw_commit_tgl;
            adopt_dly <= 16'd0;
        end else adopt_dly <= adopt_dly + 16'd1;
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
//  模型 2：纹理源（仿 tb_viewer：ack 后 4 拍回一组 4 字）
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
            w   = a - ((a >= T1_BASE) ? T1_BASE : T0_BASE);
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

reg  [1:0]  ack_r = 0, pwr_r = 0;
reg  [63:0] pd_r  = 0;
assign tex_req_ack = ack_r;
assign tex_pix_wr  = pwr_r;
assign tex_pix_data = pd_r;

integer srv_st0, srv_st1;
reg [20:0] srv_a0, srv_a1;
reg [3:0]  srv_k0, srv_k1;
reg [3:0]  gap0, gap1;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        srv_st0 <= 0; srv_st1 <= 0; ack_r <= 0; pwr_r <= 0; pd_r <= 0;
        gap0 <= 0; gap1 <= 0; srv_k0 <= 0; srv_k1 <= 0;
        srv_a0 <= 0; srv_a1 <= 0;
    end else begin
        pwr_r <= 2'b00; ack_r <= 2'b00;
        case (srv_st0)
        0: if (tex_req_vld[0]) begin
               srv_a0 <= tex_req_addr[20:0]; srv_st0 <= 1; gap0 <= 6;
           end
        1: if (gap0 != 0) gap0 <= gap0 - 1;
           else begin ack_r <= ack_r | 2'b01; srv_k0 <= 0; srv_st0 <= 2; end
        2: begin
               pwr_r[0] <= 1'b1; pd_r[31:0] <= tex_word(srv_a0 + srv_k0);
               if (srv_k0 == 3) srv_st0 <= 0; else srv_k0 <= srv_k0 + 1;
           end
        default: srv_st0 <= 0;
        endcase
        case (srv_st1)
        0: if (tex_req_vld[1]) begin
               srv_a1 <= tex_req_addr[41:21]; srv_st1 <= 1; gap1 <= 6;
           end
        1: if (gap1 != 0) gap1 <= gap1 - 1;
           else begin ack_r <= ack_r | 2'b10; srv_k1 <= 0; srv_st1 <= 2; end
        2: begin
               pwr_r[1] <= 1'b1; pd_r[63:32] <= tex_word(srv_a1 + srv_k1);
               if (srv_k1 == 3) srv_st1 <= 0; else srv_k1 <= srv_k1 + 1;
           end
        default: srv_st1 <= 0;
        endcase
    end
end

//------------------------------------------------------------------------------
//  模型 3：写引擎（按"字数"计数：565 每 2 像素 1 字，888 每像素 1 字）
//------------------------------------------------------------------------------
//  这就是 frame_fifo_write 的 len_words 语义，也是 frame_read_write 里
//  wfifo_we = wr_fmt_r ? pk_we : write_en 的等价模型。
reg [19:0] wcnt;
reg [5:0]  fdelay;
reg        wr_fmt_l;
reg        pk_ph;
reg        fin_armed;
integer    finishes = 0;

wire [19:0] words_target = wr_fmt_l ? 20'd153600 : 20'd307200;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        app_write_req_ack  <= 1'b0;
        wcnt   <= 20'd0;
        fdelay <= 6'd0;
        write_finish_pulse <= 1'b0;
        wr_fmt_l <= 1'b0;
        pk_ph    <= 1'b0;
        fin_armed <= 1'b0;
    end else begin
        write_finish_pulse <= 1'b0;

        if (app_write_req && !app_write_req_ack) begin
            app_write_req_ack <= 1'b1;
            wcnt      <= 20'd0;
            fdelay    <= 6'd0;
            pk_ph     <= 1'b0;
            fin_armed <= 1'b0;
            wr_fmt_l  <= fb_fmt565;
        end else if (app_write_req_ack && !app_write_req) begin
            app_write_req_ack <= 1'b0;
        end

        if (app_write_en) begin
            if (wr_fmt_l) begin
                if (pk_ph) wcnt <= wcnt + 20'd1;   // 565：后像素到达才写一个字
                pk_ph <= ~pk_ph;
            end else begin
                wcnt <= wcnt + 20'd1;              // 888：一像素一字
            end
        end

        if (wcnt >= words_target) fin_armed <= 1'b1;
        if (fin_armed) begin
            if (fdelay == 6'd20) begin
                write_finish_pulse <= 1'b1;
                finishes = finishes + 1;
                fdelay   <= 6'd21;                 // 只发一次
                fin_armed <= 1'b0;
            end else if (fdelay < 6'd20) fdelay <= fdelay + 6'd1;
        end
    end
end

//------------------------------------------------------------------------------
//  观测
//------------------------------------------------------------------------------
integer errors = 0;
integer last_state = -1;
integer frm_take, frm_px, frm_srvy, frm_b;

//==============================================================================
//  ★ 2026-10-28：抓"送进帧缓存的像素流"，逐点比对纹理模型
//==============================================================================
//  【为什么加】上板现象："HD 模式下右边有一列 1px 黑边，左边正常"。
//    纹理侧已经证明逐点精确（tb_bmp_read640 / tb_scale_shift），
//    所以问题只能在"图层 → 合并 → frame_writer"这段像素流里。
//    本 tb 的纹理模型是**坐标编码**的：HD 槽第 k 个字 = {col, row, 0x5A}，
//    而 S_HD 时 HD 图层是 1:1 顺序读 ⇒ 送进帧缓存的第 k 个像素必须
//    正好等于 {k%640, k/640, 0x5A}。任何一列黑/错位都会立刻显形。
//------------------------------------------------------------------------------
reg  [31:0] wcap [0:307199];
integer     wc = 0;
reg         cap_en = 0;

always @(posedge clk) begin
    if (cap_en && app_write_en && (wc < 307200)) begin
        wcap[wc] = app_write_data;
        wc = wc + 1;
    end
end

task wait_finish;                 // 等一次 write_finish_pulse（带超时）
    input integer limit;
    integer t;
    begin
        for (t = 0; t < limit; t = t + 1) begin
            @(posedge clk);
            if (write_finish_pulse === 1'b1) t = limit;
        end
    end
endtask

always @(posedge clk) begin
    if (!rst) begin
        if (st_state !== last_state) begin
            $display("  [t=%0t] state %0d -> %0d  img=%0d fmt565=%b a_hd=%b a_en=%b live=%b",
                     $time, last_state, st_state, st_img, fb_fmt565,
                     u_app.u_scene.a_hd, u_app.u_scene.a_en, display_valid);
            last_state = st_state;
        end
        if (ps_valid) frm_take = frm_take + 1;
        if (app_write_en) frm_px = frm_px + 1;
    end
end

// 卡住时每 2ms 报一次"谁在挡 pixel_ready"
always @(posedge clk) begin
    if (!rst && (frm_take < NPX)) begin
        frm_b = frm_b + 1;
        if (frm_b == 100000) begin
            frm_b = 0;
            $display("  [blk t=%0t] st=%0d take=%0d px=%0d | rdy_a=%b rdy_b=%b quota=%b q_left=%0d wr_stall=%b ps_start=%b ps_new=%b",
                     $time, st_state, frm_take, frm_px,
                     u_app.u_scene.rdy_a, u_app.u_scene.rdy_b,
                     u_app.u_scene.quota_ok, u_app.u_scene.q_left,
                     wr_stall, ps_start, ps_new_frame);
        end
    end
end

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    frm_take = 0; frm_px = 0; frm_srvy = 0; frm_b = 0;
    repeat (20) @(posedge clk);
    rst = 0;

    // 等 viewer_ctrl 把第 0 张缩略图装完、开画面
    fork : wait_first_frame
        begin
            wait (ps_new_frame === 1'b1);
            disable wait_first_frame;
        end
        begin
            repeat (120000) @(posedge clk);
            $display("  [FAIL] 5ms 内 ps_new_frame 一次都没来（compose_en 没起来？）");
            errors = errors + 1;
            disable wait_first_frame;
        end
    join
    $display("  [ ok ] ps_new_frame 来了");

    // 等这一帧被收满
    fork : wait_done
        begin
            wait (app_busy === 1'b1);
            disable wait_done;
        end
        begin
            repeat (300000) @(posedge clk);
            $display("  [FAIL] ps_new_frame 之后 busy 一直没起来（帧根本没开）");
            errors = errors + 1;
            disable wait_done;
        end
    join
    $display("  [ ok ] frame_writer 开工，busy=1");

    // 等这一帧提交（display_valid 拉高）
    fork : wait_live
        begin
            wait (display_valid === 1'b1);
            disable wait_live;
        end
        begin
            repeat (1600000) @(posedge clk);
            $display("  [FAIL] 一帧迟迟不提交：take=%0d px=%0d（display_valid 起不来）",
                     frm_take, frm_px);
            errors = errors + 1;
            disable wait_live;
        end
    join
    if (display_valid) $display("  [ ok ] display_valid = 1，display_valid 链路是通的");
    $display("       本帧 ps_valid 拍数=%0d，write_en 拍数=%0d，finishes=%0d",
             frm_take, frm_px, finishes);

    //---- 再验一步：S_LOW 能靠 tick 走到 S_HDF(4) → S_HD(5) ----
    //   这一步专门盯 tick 的产生：场消隐里 pipe_ce 不动，如果 tick 还挂在
    //   pipe_ce 上，状态会永远停在 2。
    fork : wait_boot_hd
        begin
            wait (st_state == 3'd5);
            disable wait_boot_hd;
        end
        begin
            repeat (2500000) @(posedge clk);
            $display("  [FAIL] 停在 state=%0d：tick 没产生（S_LOW/S_TRANS 卡死）", st_state);
            errors = errors + 1;
            disable wait_boot_hd;
        end
    join
    if (st_state == 3'd5) $display("  [ ok ] 走完 S_LOW(2) → S_HDF(4) → S_HD(5)，tick 正常");

    //--------------------------------------------------------------------------
    //  抓一帧 S_HD 的写流，逐点比对
    //--------------------------------------------------------------------------
    if (st_state == 3'd5) begin : hd_stream_chk
        integer q, bad;
        reg [7:0] cx, cy;
        reg [31:0] expw;

        cap_en = 1'b0; wc = 0;
        wait_finish(4000000);                 // 丢掉可能的半截帧
        wc = 0; cap_en = 1'b1;
        wait_finish(4000000);                 // 抓完整的一帧
        cap_en = 1'b0;

        $display("  ---- S_HD 写流：抓到 %0d 字（应为 307200）----", wc);
        if (wc !== 307200) begin
            $display("  [FAIL] 写流字数不对（少 %0d 个）", 307200 - wc);
            errors = errors + 1;
        end else begin
            bad = 0;
            for (q = 0; q < 307200; q = q + 1) begin
                cx = q % 640;
                cy = q / 640;
                expw = {cx, cy, 8'h5A, 8'h00};   // write_data = {ps_data, 8'h00}
                if (wcap[q] !== expw) begin
                    if (bad < 8)
                        $display("        [%0d] (x=%0d,y=%0d) got=%h exp=%h",
                                 q, cx, cy, wcap[q], expw);
                    bad = bad + 1;
                end
            end
            $display("  ---- 逐点不匹配 %0d / 307200 ----", bad);
            if (bad != 0) errors = errors + 1;

            // 每行最后两列单独看（上板就是"右边一列黑"）
            for (q = 0; q < 480; q = q + 160) begin
                cx = 639; cy = q[7:0];
                $display("        row %0d : x=638 %h | x=639 %h (exp %h)",
                         q, wcap[q*640+638], wcap[q*640+639], {cx, cy, 8'h5A, 8'h00});
            end
        end
    end

    $display("\n==== errors = %0d ====", errors);
    if (errors == 0) $display("PASS");
    else             $display("FAIL");
    $finish;
end

initial begin
    #90_000_000;
    $display("TIMEOUT  take=%0d px=%0d st=%0d live=%b", frm_take, frm_px, st_state, display_valid);
    $finish;
end

endmodule
