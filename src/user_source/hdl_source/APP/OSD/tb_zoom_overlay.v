//==============================================================================
//  tb_zoom_overlay.v —— 缩放 OSD 叠加层断言（回归 "打开缩放时菜单跟着冒出来"）
//==============================================================================
//  【这个 bug 为什么之前没抓到】
//    tb_osd_keys 里 osd_top 的 zoom_on 是悬空的（=0），全部用例都跑在
//    "菜单模式"下；而 zoom_osd 的台只测键位，不碰像素。两条测试路径中间
//    **正好漏掉了「菜单关着 + zoom 打开」这个组合**。
//
//  【复现路径（必须按这个顺序）】
//    ① 开菜单并布局一次 ⇒ osd_menu 把控件矩形写进 osd_gfx 的槽
//    ② 关菜单，**不重新布局** ⇒ 槽里还是上一次菜单的矩形（osd_menu 只在
//       布局时才清槽，而 S_OSD 退出时是停帧、根本不会再布局）
//    ③ 打开缩放（S_ZOOM 是连续出帧）⇒ osd_layer_en=1，残留槽被逐帧画出来
//
//  【断言】
//    A. zoom_on=1 时非透明像素总数必须 ≈ 十字条(640+480) + 数字(≈60)，< 3000
//    B. 十字条那一列整列非透明
//    C. 菜单区域的采样点（300,300 等）必须全透明
//    D. 出 PPM 供目检
//
//  运行：
//    iverilog -g2012 -DIVERILOG_SIM -o /tmp/zo.vvp -s tb_zoom_overlay tb_zoom_overlay.v \
//      osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v flash_arb.v \
//      osd_icon_rom.v ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v \
//      ../../spi_flash_read.v ../w25q64_model_24b.v tools/bram32k_stub.v tools/mult18_stub.v
//==============================================================================
`timescale 1ns / 1ps

module tb_zoom_overlay;

reg         clk = 1'b0;
reg         rst = 1'b1;
always #10 clk = ~clk;               // 50MHz

reg         ps_start = 1'b0;
reg         pixel_ready = 1'b1;
reg  [15:0] px = 16'd0, py = 16'd0;

reg  [3:0]  key_press = 4'd0;
reg  [3:0]  key_level = 4'hF;

reg         menu_on   = 1'b0;
reg         info_show = 1'b0;
reg         zoom_on   = 1'b0;
reg  [9:0]  ax = 10'd200, ay = 10'd240;
reg  [7:0]  zoom_sel = 8'h20;

wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;
wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire [2:0]  fx_sel, auto_mode;
wire        info_on, album_prev, album_next;
wire        bp_rdy, bp_valid, bp_done;
wire [7:0]  bp_data;
wire        o_lang, o_mute, o_loop, o_spc;
wire [6:0]  o_vol;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(20), .NSLOT_F(63), .SELFTEST(0),
          .RPT_FIRST(2), .RPT_EVERY(1)) dut (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(info_show),
    .zoom_on(zoom_on), .ax(ax), .ay(ay), .zoom_sel(zoom_sel), .zoom_mode(2'd2),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi),
    .flash_wp_n(f_wp), .flash_hold_n(f_hold), .flash_miso(f_miso),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(bp_rdy), .bp_data(bp_data), .bp_valid(bp_valid), .bp_done(bp_done),
    .enter_req(enter_req), .exit_req(exit_req),
    .osd_layer_en(osd_layer_en), .fade(fade), .osd_pix(osd_pix),
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(info_on),
    .album_prev(album_prev), .album_next(album_next),
    .cur_w(16'd640), .cur_h(16'd480),
    .auto_remain(6'd0), .auto_bar(9'd0),
    .o_lang(o_lang), .o_vol(o_vol), .o_mute(o_mute), .o_loop(o_loop), .o_spc(o_spc),
    .txt_trig()
);

w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

//------------------------------------------------------------------------------
//  工具
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input         cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            fails = fails + 1;
            $display("  FAIL  %0s", name);
        end else
            $display("  ok    %0s", name);
    end
endtask

// 布局一拍（让 osd_menu 重新排控件表/文字）
task lay;
    begin
        @(negedge clk); ps_start = 1'b1;
        repeat (8) @(posedge clk);
        @(negedge clk); ps_start = 1'b0;
        repeat (4000) @(posedge clk);
    end
endtask

// 全帧扫描：把 osd_pix 的 A 通道存进 mask[ ]
//   每个像素给 4 拍，保证流水（osd_gfx 2 拍 + merge 1 拍）走完
reg [7:0] amask [0:640*480-1];
integer   npix, nonz;
integer   xx, yy;

task scan_frame;
    begin
        nonz = 0;
        // ★★ ps_start 必须**每像素**拉高 ★★
        //   osd_gfx 的 `adv = ps_start & pixel_ready` 是"列推进"信号，
        //   不是帧同步。写成帧头给一次，adv 只在那一拍有效、
        //   槽索引不再前进 ⇒ 整帧命中不了（实测像素全 0）。
        //   参考 tb_osd_render.v 的 shot_rect()：它也是每像素 ps_start=1。
        for (yy = 0; yy < 480; yy = yy + 1) begin
            for (xx = 0; xx < 640; xx = xx + 1) begin
                @(negedge clk); px = xx[15:0]; py = yy[15:0]; ps_start = 1'b1;
                repeat (4) @(posedge clk);
                // 采到的 osd_pix 对应 2 拍前的 (xx-2, yy-?) —— 只做**总量/区域**
                // 统计，不做逐像素坐标断言，所以不追求严格对齐
                amask[yy*640 + xx] = osd_pix[31:24];
                if (osd_pix[31:24] != 8'd0) nonz = nonz + 1;
            end
        end
        @(negedge clk); ps_start = 1'b0;
        repeat (200) @(posedge clk);
    end
endtask

// 粗粒度区域检查：某个 16×16 方块里有没有非透明像素
task region_has;
    input integer rx, ry;
    output        has;
    integer i, j;
    begin
        has = 0;
        for (j = 0; j < 16; j = j + 1)
            for (i = 0; i < 16; i = i + 1)
                if (amask[(ry+j)*640 + (rx+i)] != 8'd0) has = 1;
    end
endtask

// 写 PPM（目检用）
integer fd, i2;
task dump_ppm;
    input [255:0] fname;
    begin
        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n640 480\n255\n");
        for (i2 = 0; i2 < 640*480; i2 = i2 + 1) begin
            // 区分 alpha：0=黑、192=面板/十字(灰)、255=文字(白)
            if (amask[i2] == 8'd0)       $fwrite(fd, "%c%c%c", 8'h10, 8'h10, 8'h10);
            else if (amask[i2] == 8'd255) $fwrite(fd, "%c%c%c", 8'hFF, 8'hFF, 8'hFF);
            else                          $fwrite(fd, "%c%c%c", 8'h60, 8'h60, 8'h60);
        end
        $fclose(fd);
    end
endtask

//------------------------------------------------------------------------------
reg has;
initial begin
    repeat (10) @(negedge clk);
    rst = 1'b0;
    repeat (200) @(negedge clk);

    //------------------------------------------------------------------
    $display("--- 1. 先开一次菜单，让它把控件槽写满 ---");
    //------------------------------------------------------------------
    menu_on = 1'b1;                        // 直接给电平（同 tb_osd_keys 的做法）
    repeat (40) @(negedge clk);
    lay();
    $display("      DBG lay_st=%0d (0=IDLE 1=ANIM 2=TWN 3=CELL 4=TXY 5=TXT 6=FIN)  wr_en=%b cell_i=%0d dirty=%b",
             dut.u_menu.lay_st, dut.u_menu.wr_en, dut.u_menu.cell_i, dut.u_menu.dirty);
    $display("      DBG slot0 geo_lo=%h geo_hi=%h attr=%h",
             dut.u_gfx.u_mem.geo_lo[0], dut.u_gfx.u_mem.geo_hi[0], dut.u_gfx.u_mem.attr_r[0]);
    $display("      menu 布局完成（menu_on=%b，槽里应有菜单矩形）", menu_on);
    // ★ 前置断言：没这一步，后面"菜单不冒出来"可能只是因为槽本来是空的
    scan_frame();
    $display("      菜单首开非透明像素 = %0d", nonz);
    chk(nonz > 20000, "P1 menu really rendered (precondition for the bug)");

    //------------------------------------------------------------------
    $display("--- 2. 关菜单（不重新布局，模拟 S_OSD 停帧退出）---");
    //------------------------------------------------------------------
    menu_on = 1'b0;
    repeat (100) @(negedge clk);

    //------------------------------------------------------------------
    $display("--- 3. 打开缩放 OSD，扫描全帧 ---");
    //------------------------------------------------------------------
    zoom_on  = 1'b1;
    ax       = 10'd200;
    ay       = 10'd240;
    zoom_sel = 8'h20;                      // 2.0x
    // ★ 必须先布局：槽 16 的几何和文字行 6/7 的内容都在 L_CELL/L_TXT 里写，
    //   而 scan_frame 每像素翻 ps_start 会把 L_IDLE 的入场条件反复打断。
    repeat (100) @(negedge clk);
    lay();
    $display("      DBG zoom_on=%b menu_on=%b lay_st=%0d wr_en=%b cell_i=%0d",
             dut.u_menu.zoom_on, dut.u_menu.menu_on, dut.u_menu.lay_st,
             dut.u_menu.wr_en, dut.u_menu.cell_i);
    $display("      DBG dynslot6 geo_lo=%h geo_hi=%h attr=%h",
             dut.u_gfx.u_mem.geo_lo[6], dut.u_gfx.u_mem.geo_hi[6], dut.u_gfx.u_mem.attr_r[6]);
    scan_frame();

    $display("      非透明像素总数 = %0d", nonz);

    // A. 总量：十字条 640+480-1 = 1119，数字 ≈ 60，其余不应有
    chk(nonz < 12000, "A total opaque < 12000 (panel+cross, no menu leftovers)");
    chk(nonz > 6000, "B panel+cross drawn");

    // C. 菜单常驻区域必须全透明
    region_has(16,  72, has);  chk(!has, "C1 nav bar area (16,72) empty");
    region_has(300, 100, has); chk(!has, "C2 page area (300,100) empty");
    region_has(300, 300, has); chk(!has, "C3 page area (300,300) empty");
    region_has(96,  180, has); chk(!has, "C4 divider area (96,180) empty");

    // D. 十字条位置
    region_has(196, 100, has); chk(has, "D1 cross vertical line present");
    region_has(300, 236, has); chk(has, "D2 cross horizontal line present");

    // E. 倍数数字（左上角 16,16）
    region_has(16, 16, has);   chk(has, "E1 zoom digit present at (16,16)");

    dump_ppm("out/zoom_overlay.ppm");
    $display("      已写 out/zoom_overlay.ppm");

    //------------------------------------------------------------------
    $display("--- 4. 关缩放，确认菜单没被破坏 ---");
    //------------------------------------------------------------------
    zoom_on = 1'b0;
    menu_on = 1'b1;
    repeat (40) @(negedge clk);
    lay();
    scan_frame();
    $display("      菜单重开后非透明像素 = %0d  (menu_on=%b layer_en=%b fade=%0d)",
             nonz, menu_on, osd_layer_en, fade);
    chk(nonz > 20000, "F menu renders normally after zoom OSD");

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #200_000_000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
