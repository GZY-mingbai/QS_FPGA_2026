//==============================================================================
//  tb_quad_page.v —— 四宫格【显示通路】全链路仿真（viewer_scene 内部真实电路）
//==============================================================================
//  iverilog -g2012 -o qp.vvp -s tb_quad_page \
//      tb_quad_page.v viewer_scene.v quad_sel.v img_layer_ctrl.v \
//      img_layer_hd.v ps_gen_image.v ps_merge_alpha_pp.v ps_to_rgb24.v \
//      app_scan_gen.v
//  vvp qp.vvp
//
//  【为什么要这个 tb】
//  tb_quad_chain 只验证了"列表 → 装载请求 → 坐标映射"，**没有一行真实的
//  像素生成逻辑**。上板"文件页只有 6 个点"就是死在显示通路上，所以这里
//  直接把 viewer_scene 拉起来，用真实纹理地址喂假 SDRAM，逐像素比对。
//
//  【怎么造纹理】
//  纹理内容只由 (槽号, 行, 字列) 决定，与半字无关 —— 这样"半字选择"这种
//  内部细节不会影响判据，而"槽 ↔ 格子对应关系"和"行/列映射"仍然被逐位验证。
//
//      R5 = 槽号 × 8      ⇒ 一眼看出是不是同一个格子串了槽
//      G6 = 字列 & 63     ⇒ 验证横向映射（左右不镜像）
//      B5 = 源行 >> 3     ⇒ 验证纵向映射（上下不镜像）
//
//  【A 层故意关掉】
//  a_en = 0、a_hd = 0 —— 模拟真实情况：open 文件页时相册里没有"当前图"。
//  viewer_scene 内部必须靠 quad_mode 强制把 A 层打开，否则一像素都不出。
//  这一条就是上板故障的根因回归测试。
//==============================================================================
`timescale 1ns / 1ps

module tb_quad_page;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;

localparam [20:0] QBASE = 21'h100000;      // 1048576，2^20，2^16 对齐
localparam integer SLOT_STEP = 40960;   // 槽间距（字）
localparam integer SRC_W   = 160;          // 纹理字/行（=320 像素）

// 布局（与 quad_sel 参数一致）
localparam integer X0 = 208, Y0 = 100, CP = 176, RP = 160, CW = 160, CH = 120;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
reg         ps_start = 0;
wire        pixel_ready;
wire [23:0] ps_data;
wire        pipe_ce;
reg         fmt565   = 1'b1;
reg         wr_stall = 1'b0;

// A 层：**故意关掉** —— 验证 quad_mode 的强制使能
reg         a_hd = 0;
reg  [31:0] a_pos  = 32'd0;
reg  [31:0] a_size = {16'd640, 16'd480};
reg  [39:0] a_k    = {20'd16384, 20'd32768};
reg  [19:0] a_ori  = 20'd0;
reg  [20:0] a_base = 21'd614400;
reg         a_en   = 0;

// B 层：关
reg  [31:0] b_pos   = {16'd640, 16'd0};
reg  [31:0] b_size  = {16'd640, 16'd480};
reg  [39:0] b_k     = {20'd16384, 20'd32768};
reg  [20:0] b_base  = 21'd652800;
reg         b_en    = 0;
reg  [7:0]  alpha_b = 8'd255;

wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
wire [1:0]  tex_pix_wr;
wire [63:0] tex_pix_data;
reg  [1:0]  ack_r = 0;
reg  [1:0]  pwr_r = 0;
reg  [63:0] pd_r  = 0;
wire [1:0]  tex_req_ack = ack_r;
assign tex_pix_wr   = pwr_r;
assign tex_pix_data = pd_r;

wire [15:0] scan_x, scan_y;

viewer_scene #(.IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRC_W)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(ps_start), .pixel_ready(pixel_ready), .ps_data(ps_data), .pipe_ce(pipe_ce),
    .fmt565(fmt565), .wr_stall(wr_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(a_ori),
    .a_base(a_base), .a_en(a_en),
    .quad_mode(1'b1),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en),
    .alpha_b(alpha_b),
    .scan_x(scan_x), .scan_y(scan_y),
    .osd_on(1'b0), .osd_fade(8'd0), .osd_pix(32'd0),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data)
);

//------------------------------------------------------------------------------
//  纹理内容模型（只依赖 槽号 / 行 / 字列）
//------------------------------------------------------------------------------
function [31:0] tex_word;
    input [20:0] a;
    integer rel, slot, row, col;
    reg [4:0] r5, b5;
    reg [5:0] g6;
    reg [15:0] w;
    begin
        rel  = a - QBASE;
        slot = rel / SLOT_STEP;
        w    = rel % SLOT_STEP;
        row  = w / SRC_W;
        col  = w % SRC_W;
        r5   = slot[4:0] * 5'd8;          // 0/8/16/24
        g6   = col[5:0];
        b5   = (row >> 3) & 31;
        tex_word = {r5, g6, b5, r5, g6, b5};
    end
endfunction

//------------------------------------------------------------------------------
//  假纹理仲裁：每层 4 拍一组，先 ack 再回像素（照抄 tb_viewer 的节拍）
//------------------------------------------------------------------------------
integer    srv_st0;
reg [20:0] srv_a0;
reg [3:0]  srv_k0;
reg [3:0]  gap0;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        srv_st0 <= 0; ack_r <= 0; pwr_r <= 0; pd_r <= 0;
        gap0 <= 0; srv_k0 <= 0; srv_a0 <= 0;
    end else begin
        pwr_r <= 2'b00;
        ack_r <= 2'b00;
        // layer1 永远关，收到请求直接吞掉（否则会卡住 tex_req_ack[1]）
        case (srv_st0)
        0: if (tex_req_vld[0]) begin
               srv_a0  <= tex_req_addr[20:0];
               srv_st0 <= 1;
               gap0    <= 6;
           end
        1: if (gap0 != 0) gap0 <= gap0 - 1;
           else begin
               ack_r   <= ack_r | 2'b01;
               srv_k0  <= 0;
               srv_st0 <= 2;
           end
        2: begin
               pwr_r[0]   <= 1'b1;
               pd_r[31:0] <= tex_word(srv_a0 + srv_k0);
               if (srv_k0 == 3) srv_st0 <= 0;
               else             srv_k0 <= srv_k0 + 1;
           end
        default: srv_st0 <= 0;
        endcase
    end
end

//------------------------------------------------------------------------------
//  期望值
//------------------------------------------------------------------------------
//  把屏幕坐标换算成 (在不在格子内, 槽号, 局部坐标)
task cell_of;
    input  integer px, py;
    output          hit;
    output integer  slot, lx, ly;
    begin
        hit = 0; slot = 0; lx = 0; ly = 0;
        if (py >= Y0 && py < Y0 + CH) begin
            if (px >= X0 && px < X0 + CW)          begin hit=1; slot=0; lx=px-X0;      ly=py-Y0;      end
            else if (px >= X0+CP && px < X0+CP+CW) begin hit=1; slot=1; lx=px-(X0+CP); ly=py-Y0;      end
        end else if (py >= Y0+RP && py < Y0+RP+CH) begin
            if (px >= X0 && px < X0 + CW)          begin hit=1; slot=2; lx=px-X0;      ly=py-(Y0+RP); end
            else if (px >= X0+CP && px < X0+CP+CW) begin hit=1; slot=3; lx=px-(X0+CP); ly=py-(Y0+RP); end
        end
    end
endtask

// 屏幕 (px,py) 上应当出现的 24 位像素（函数里不能调 task，所以自己算一遍）
function [23:0] exp_px;
    input integer px, py;
    integer slot, lx, ly, row, col;
    reg [4:0] r5, b5;
    reg [5:0] g6;
    begin
        slot = -1; lx = 0; ly = 0;
        if (py >= Y0 && py < Y0 + CH) begin
            if (px >= X0 && px < X0 + CW)          begin slot=0; lx=px-X0;      ly=py-Y0;      end
            else if (px >= X0+CP && px < X0+CP+CW) begin slot=1; lx=px-(X0+CP); ly=py-Y0;      end
        end else if (py >= Y0+RP && py < Y0+RP+CH) begin
            if (px >= X0 && px < X0 + CW)          begin slot=2; lx=px-X0;      ly=py-(Y0+RP); end
            else if (px >= X0+CP && px < X0+CP+CW) begin slot=3; lx=px-(X0+CP); ly=py-(Y0+RP); end
        end
        if (slot < 0) begin
            exp_px = 24'h000000;               // 框外 = 透明 = 黑
        end else begin
            // 源坐标：KX=65536 ⇒ 字列 = lx；KY=131072 ⇒ 源行 = 2*ly+1
            col = lx;
            row = 2 * ly + 1;
            r5  = slot[4:0] * 5'd8;
            g6  = col[5:0];
            b5  = (row >> 3) & 31;
            exp_px = {{r5, r5[4:2]}, {g6, g6[5:4]}, {b5, b5[4:2]}};
        end
    end
endfunction

//------------------------------------------------------------------------------
//  出图（把真实 RTL 的 ps_data 按扫描位置拼成 640×480）
//------------------------------------------------------------------------------
reg [23:0] fbuf [0:NPX-1];
integer    fd, i, j, px, py;
reg [23:0] v24;
integer    n_in[0:3];

integer errors = 0;
integer frames = 0;
integer n_bad[0:3];
integer n_out_bad;

//------------------------------------------------------------------------------
//  单帧
//------------------------------------------------------------------------------
task run_frame;
    integer i, g, j, px, py, slot, lx, ly;
    reg hit;
    reg [23:0] exp;
    integer abort;
    begin
        abort = 0;
        for (i = 0; i < 4; i = i + 1) begin n_in[i] = 0; n_bad[i] = 0; end
        n_out_bad = 0;

        ps_start = 1'b1;
        @(posedge clk);
        for (i = 0; (i < NPX) && !abort; i = i + 1) begin
            g = 0;
            while (!(ps_start && pixel_ready) && (g < 200000)) begin
                @(posedge clk);
                g = g + 1;
            end
            if (g >= 200000) begin
                $display("  [FAIL] frame %0d deadlock at pixel %0d", frames, i);
                $display("    ready=%b warmed=%b scan=(%0d,%0d) local=(%0d,%0d)",
                         dut.u_ta.layer_ready, dut.u_ta.u_gen.warmed,
                         dut.sx, dut.sy, dut.ta_x, dut.ta_y);
                $display("    rq=(%0d,%0d) idx rq=%0d ds=%0d cnt=%0d box=%b new=%b",
                         dut.u_ta.u_gen.rq_x, dut.u_ta.u_gen.rq_y,
                         dut.u_ta.u_gen.rq_idx, dut.u_ta.u_gen.ds_idx,
                         dut.u_ta.u_gen.pix_count,
                         dut.u_ta.u_gen.rq_in_box, dut.u_ta.u_gen.rq_new);
                errors = errors + 1;
                abort  = 1;
            end else begin
                // ★ 两个容易踩错的约定（都对过了实测值）：
                //   · 像素流水线延迟 = PIPE = 5（viewer_scene 里 [DSP-PIPE] 4→5），
                //     不是老 tb_viewer 写的 4
                //   · app_scan_gen 从 (0,479) 起、x 先走、回绕时 y 减 1
                //     ⇒ 第 j 个扫描位置对应屏幕行 479 - j/640（自下而上）
                j = i - 5;
                if (j >= 0) begin
                    py = (IMG_H - 1) - (j / IMG_W);   // 屏幕行（0 = 顶）
                    px = j % IMG_W;
                    cell_of(px, py, hit, slot, lx, ly);
                    exp = exp_px(px, py);
                    if (ps_data !== exp) begin
                        if (hit) begin
                            n_bad[slot] = n_bad[slot] + 1;
                            if (n_bad[slot] <= 3)
                                $display("  [FAIL] f%0d cell%0d (%0d,%0d) lx=%0d ly=%0d got %06h exp %06h",
                                         frames, slot, px, py, lx, ly, ps_data, exp);
                        end else begin
                            n_out_bad = n_out_bad + 1;
                            if (n_out_bad <= 3)
                                $display("  [FAIL] f%0d outside (%0d,%0d) got %06h exp 000000",
                                         frames, px, py, ps_data);
                        end
                        errors = errors + 1;
                    end
                    if (hit) n_in[slot] = n_in[slot] + 1;
                    fbuf[py*IMG_W + px] = ps_data;
                end
                @(posedge clk);
            end
        end
        ps_start = 1'b0;
        if (!abort) begin
            $display("  frame %0d done: cell px = %0d %0d %0d %0d  (expect 19200 each)",
                     frames, n_in[0], n_in[1], n_in[2], n_in[3]);
            if (n_in[0] != CW*CH || n_in[1] != CW*CH || n_in[2] != CW*CH || n_in[3] != CW*CH) begin
                $display("  [FAIL] per-cell pixel count wrong");
                errors = errors + 1;
            end
        end
        if (frames == 1) begin
            fd = $fopen("quad_page.ppm", "wb");
            $fwrite(fd, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
            for (j = 0; j < IMG_H; j = j + 1)
                for (i = 0; i < IMG_W; i = i + 1) begin
                    v24 = fbuf[j*IMG_W + i];
                    $fwrite(fd, "%c%c%c", v24[23:16], v24[15:8], v24[7:0]);
                end
            $fclose(fd);
            $display("      wrote quad_page.ppm (real RTL pixels)");
        end
        frames = frames + 1;
        repeat (3000) @(posedge clk);
    end
endtask

initial begin
    repeat (20) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    $display("\n== quad page: viewer_scene with a_en=0 (must be forced on by quad_mode) ==");
    run_frame;
    $display("== a second frame (steady state) ==");
    run_frame;

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d frames)", frames);
    else             $display("  FAIL: %0d errors", errors);
    $display("========================================\n");
    $finish;
end

initial begin
    #600_000_000;
    $display("  TIMEOUT");
    $finish;
end

endmodule
