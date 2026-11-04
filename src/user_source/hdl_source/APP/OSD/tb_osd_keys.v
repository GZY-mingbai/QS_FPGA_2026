`timescale 1ns/1ps
//==============================================================================
//  tb_osd_keys.v —— 逐按键行为验证（不出图，只断言状态）
//==============================================================================
//  目的：上板反馈"菜单、控件逻辑混乱"，必须把**每一次按键**的结果都钉死。
//  每按一次键就断言 focus / nav_idx / page / item / edit / scroll_row / val*，
//  任何一条不符就打印 FAIL 并计数。全部通过才算逻辑正确。
//
//  ★ key_press 在真实工程里是**单拍脉冲**（viewer_ctrl 由按键边沿产生），
//    所以这里也必须只给 1 拍 —— 给成电平会让"加减值"一次按出好几步，
//    那是测试台自己的错，不是 RTL 的错。
//
//  用法（工作目录 = 本文件所在目录，INIT_HEX 相对它）：
//      iverilog -g2012 -o k.vvp -s tb_osd_keys tb_osd_keys.v osd_top.v \
//          osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//          ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v \
//          ../../spi_flash_read.v ../w25q64_model_24b.v
//      vvp k.vvp
//==============================================================================
module tb_osd_keys;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 0;

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

// ★ 连发阈值用快速档（2 tick 首触 / 每 1 tick 一发）：真板上 osd_menu 用 30/5
reg         info_show = 1'b0;
reg  [15:0] cur_w = 16'd640;
reg  [15:0] cur_h = 16'd480;
reg  [5:0]  auto_remain = 6'd0;
reg  [8:0]  auto_bar = 9'd0;
reg  [6:0]  bright_eff = 7'd50;   // ★ 自动亮度显示值（顶层 pic_bright_eff）
wire [2:0]  fx_sel, auto_mode;
wire        info_on;
wire        album_prev, album_next;
wire        bp_rdy, bp_valid, bp_done;
wire [7:0]  bp_data;
// ★ 2026-10-29：系统页导出
wire        o_lang, o_mute, o_loop, o_spc;
wire [6:0]  o_vol;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(8), .NSLOT_F(63), .SELFTEST(0),
          .RPT_FIRST(2), .RPT_EVERY(1)) dut (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py),
    .key_press(key_press), .key_level(key_level), .bright_eff(bright_eff),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(info_show),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi),
    .flash_wp_n(f_wp), .flash_hold_n(f_hold), .flash_miso(f_miso),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(bp_rdy), .bp_data(bp_data), .bp_valid(bp_valid), .bp_done(bp_done),
    .enter_req(enter_req), .exit_req(exit_req),
    .osd_layer_en(osd_layer_en), .fade(fade), .osd_pix(osd_pix),
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(info_on),
    .album_prev(album_prev), .album_next(album_next),
    .cur_w(cur_w), .cur_h(cur_h), .auto_remain(auto_remain), .auto_bar(auto_bar),
    .o_lang(o_lang), .o_vol(o_vol), .o_mute(o_mute), .o_loop(o_loop), .o_spc(o_spc),
    // ★ 2026-10-31：10-30 以后新增的输入口原先悬空（Z）⇒ osd_menu 里 `if (zoom_on)`、
    //   `fp_nmax`（由 I_fp_vcnt 算）都是 X。全部钉成确定值；I_fp_vcnt=0 ⇒ fp_nmax=4（满页）。
    .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0),
    .I_nm_char(8'd0), .I_nm_blen(4'd0), .I_nm_elen(4'd0), .I_nm_len4(20'd0), .I_nm_vld(1'b0),
    .I_fp_vcnt(3'd0), .I_fp_has_more(1'b0), .I_fp_gen(1'b0), .I_fp_has_prev(1'b0), .O_fp_open(),
    // ★ 信息条 / 字幕这几根输入**必须显式接 0**：悬空 Z 会让文字源 mux 与 flash_arb 选到 X
    //   （sub_fl_* 是字幕字模的 Flash 客户；悬空会把仲裁器拉成 X ⇒ 字形装不进来）。
    .cur_rel(2'd0), .img_go(1'b0), .nm_upd(1'b0), .nm_hold(1'b0),
    .sub_fl_req(1'b0), .sub_fl_addr(24'd0), .sub_fl_len(16'd0)
);

w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

// 快捷别名（osd_menu 内部信号）
`define F    dut.u_menu.focus
`define NAV  dut.u_menu.nav_idx
`define PG   dut.u_menu.page
`define IT   dut.u_menu.item
`define ED   dut.u_menu.edit
`define SC   dut.u_menu.scroll_row
`define V0   dut.u_menu.val0
`define V1   dut.u_menu.val1
`define V2   dut.u_menu.val2
`define V3   dut.u_menu.val3
`define V4   dut.u_menu.val4
`define V5   dut.u_menu.val5
`define V6   dut.u_menu.val6
`define POP  dut.u_menu.pop_open
`define PSEL dut.u_menu.pop_sel

// enter_req / exit_req 都只拉高 1 拍（真实工程里 App 在下一拍就采样），
// 用锁存器抓住它，否则按键后等 40 拍再读必然读到 0。
reg enter_seen = 1'b0;
reg exit_seen  = 1'b0;
always @(posedge clk) begin
    if (enter_req) enter_seen <= 1'b1;
    if (exit_req)  exit_seen  <= 1'b1;
end

// ★ 文字监视器：记录每行第 14/15 列**最后写入**的字符码。
//   用来验证"这一行显示的是不是它自己的值"—— 这类串味 bug 只有看字库写入才能抓到。
reg [15:0] cap14 [0:7];
reg [15:0] cap15 [0:7];
reg [15:0] cap00 [0:7];              // 第 0 列 = 标签首字
// ★ 全量文字监视器（2026-10-26）：tcap[line*32+col] = 该格最后写入的字符码。
//   16 字选项/收起态值/空白补齐的断言都靠它（cap14/cap15 只看两列不够用）。
reg [15:0] tcap [0:255];
always @(posedge clk) begin
    if (dut.u_menu.str_we) begin
        if (dut.u_menu.str_idx == 8'd0)  cap00[dut.u_menu.str_line] <= dut.u_menu.str_code;
        if (dut.u_menu.str_idx == 8'd14) cap14[dut.u_menu.str_line] <= dut.u_menu.str_code;
        if (dut.u_menu.str_idx == 8'd15) cap15[dut.u_menu.str_line] <= dut.u_menu.str_code;
        tcap[{dut.u_menu.str_line[2:0], dut.u_menu.str_idx[4:0]}] <= dut.u_menu.str_code;
    end
end

function [15:0] optcode;              // ★ 2026-10-29：set 0 = 中 / EN（原为 低/中/高）
    input integer k;
    begin
        optcode = (k <= 0) ? 16'hD6D0 : 16'h0045;
    end
endfunction

integer fails  = 0;
integer checks = 0;
integer v_save = 0;
integer v1_save = 0;

//------------------------------------------------------------------------------
//  控件类型表（与 osd_menu.cell_type 一致，用来交叉验证"哪些项可编辑"）
//------------------------------------------------------------------------------
function [3:0] ctype;
    input [1:0] pg;
    input [2:0] it;
    begin
        case (pg)
            2'd0: case (it)
                // ★ 2026-10-26 显示页重排：色温/饱和度 TrackBar、风格 ComboBox、
                //   退出菜单按钮删除（item6 空行）
                3'd0, 3'd1, 3'd2, 3'd3: ctype = 4'd4;   // 亮度/对比度/色温/饱和度 TrackBar
                3'd4:            ctype = 4'd6;    // 风格        ComboBox
                3'd5:            ctype = 4'd7;    // 恢复默认    Button
                3'd6:            ctype = 4'd5;    // 自动亮度    CheckBox
                default:         ctype = 4'd1;    // 空行        Label
            endcase
            2'd1: case (it)
                3'd0, 3'd1:      ctype = 4'd7;    // 上一张/下一张
                3'd2:            ctype = 4'd6;    // 切换效果（5 项）
                3'd3:            ctype = 4'd6;    // 自动切换（6 项）
                3'd4:            ctype = 4'd5;    // 信息条
                3'd5:            ctype = 4'd5;    // 音随图切
                default:         ctype = 4'd1;
            endcase
            2'd2:            ctype = 4'd1;        // 文件页：全 Label
            2'd3: case (it)                          // ★ 2026-10-29 系统页重排
                3'd0:            ctype = 4'd6;       // 语言      ComboBox（中/EN）
                3'd1:            ctype = 4'd4;       // 音量      TrackBar
                3'd2, 3'd3, 3'd4: ctype = 4'd5;      // 静音/循环/频谱 CheckBox
                default:         ctype = 4'd1;
            endcase
            default:             ctype = 4'd1;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
task wait_n;
    input integer n;
    begin repeat (n) @(posedge clk); end
endtask

// 单拍按键脉冲（与真实工程一致）
// 按键编码（bit = 面板编号-1，与 top_tf_hdmi_audio 一致）：
//   KEY1 = 4'h1  OSD 关 → 打开；OSD 开 → 返回（导航栏按下直接关 OSD）
//   KEY2 = 4'h2  进入 / 确认
//   KEY3 = 4'h4  下
//   KEY4 = 4'h8  上
task kpress;
    input [3:0] k;
    begin
        @(negedge clk);
        key_press = k;
        @(negedge clk);
        key_press = 4'h0;
        repeat (40) @(posedge clk);      // 等状态与滚动跟随稳定
    end
endtask

// 弹层几何：不写死绝对坐标，只验证"选中项压在控件行 + 32 等距 + 面板 6/6 内边距"
// ★ 2026-10-31 [LUT-2]：动态槽 9 → 8。弹层第 0/1 行仍在槽 6/7，
//   第 2 行改住"选中行的控件槽" = 1 + sel_k（弹层打开时该行必被清空、空出来的槽）。
//   iverilog 不支持层次数组变量下标 ⇒ 用 case 取。
function integer geo_lo_y;
    input integer s;
    begin
        case (s)
            0: geo_lo_y = dut.u_gfx.u_mem.geo_lo[0][8:0];
            1: geo_lo_y = dut.u_gfx.u_mem.geo_lo[1][8:0];
            2: geo_lo_y = dut.u_gfx.u_mem.geo_lo[2][8:0];
            3: geo_lo_y = dut.u_gfx.u_mem.geo_lo[3][8:0];
            4: geo_lo_y = dut.u_gfx.u_mem.geo_lo[4][8:0];
            5: geo_lo_y = dut.u_gfx.u_mem.geo_lo[5][8:0];
            6: geo_lo_y = dut.u_gfx.u_mem.geo_lo[6][8:0];
            7: geo_lo_y = dut.u_gfx.u_mem.geo_lo[7][8:0];
            default: geo_lo_y = -1;
        endcase
    end
endfunction
function integer pop_slot;          // 弹层第 k 行住在哪个动态槽
    input integer k;
    begin
        pop_slot = (k == 2) ? (1 + dut.u_menu.sel_k) : (6 + k);
    end
endfunction

task chk_pop;
    input integer sel;
    input integer cy;
    integer base;
    begin
        base = cy - sel*32 - 2;
        chk("H sel item on combo row", geo_lo_y(pop_slot(sel)), cy - 2);
        chk("H item0 y", geo_lo_y(pop_slot(0)), base);
        chk("H item1 y", geo_lo_y(pop_slot(1)), base + 32);
        chk("H item2 y", geo_lo_y(pop_slot(2)), base + 64);
        chk("H panel top",  dut.u_gfx.u_mem.geo_lo[0][8:0],   base - 6);
        chk("H panel bot",  dut.u_gfx.u_mem.geo_hi[0][8:0], base + 98);
    end
endtask

// ★★★ 任意两条文字行的 16px 字格都不许重叠 ★★★
//   文字层遇到垂直重叠时，后画的那行会把前一行擦掉（实测"低"缺最上面 4 行、
//   "高"缺最下面 4 行）。这条断言把整类问题锁死：以后谁改 pop_y / POP_PITCH /
//   NL，只要让两个字格相交就会立刻报错。
task chk_txt_noclash;
    integer i, j, yi, yj, bad;
    begin
        bad = 0;
        for (i = 0; i < 8; i = i + 1) begin
            for (j = i + 1; j < 8; j = j + 1) begin
                yi = dut.u_menu.txt_xy[i*22+11 +: 11];
                yj = dut.u_menu.txt_xy[j*22+11 +: 11];
                if (!((yi == yj) || (yi + 16 <= yj) || (yj + 16 <= yi))) begin
                    bad = bad + 1;
                    $display("        (line%0d y=%0d vs line%0d y=%0d overlap)", i, yi, j, yj);
                end
            end
        end
        chk("H txt cells never clash", bad, 0);
    end
endtask

// 被面板盖住的行必须不画，其余行必须在原位（行 y = 60 + 80*i）
task chk_rows;
    integer i;
    begin
        for (i = 0; i < 5; i = i + 1) begin
            if (i == dut.u_menu.sel_k)
                ;   // [LUT-2] 选中行的槽被弹层第 2 行借用，几何由 chk_pop 核对
            else if ((i == dut.u_menu.pop_wx0) || (i == dut.u_menu.pop_wx1))
                chk("H row hidden", dut.u_gfx.u_mem.geo_lo[1+i][18:9], 0);
            else
                chk("H row shown",  dut.u_gfx.u_mem.geo_lo[1+i][8:0], 60 + 80*i);
        end
    end
endtask

task chk;
    input [8*80:1] name;   // 中文名约 3 字节/字，44 字节会截断成乱码
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
        $fflush;                       // ★ 每条都刷：日志停在哪里就是真停在哪里
    end
endtask

// 连续排 n 次（每次 = 一帧）：动画每帧只走一步，行距 80、步长 4096/65536
// ⇒ 要走满一行需要 16 帧。检查"稳态几何"前必须排够 20 次。
task lays;
    input integer n;
    integer i;
    begin for (i = 0; i < n; i = i + 1) lay(); end
endtask

// ★ [S6]：常量槽（0..3/5..9/15/19）已不在 cellmem 里，这里只打印动态槽。
//   旧的 cell 号按 {4→0, 10..14→1..5, 16..18→6..8} 折算。
task show_cell;
    input integer k;
    integer d;
    begin
        d = (k == 4) ? 0 : (k >= 10 && k <= 14) ? (k - 9) :
            (k >= 16 && k <= 18) ? (k - 10) : -1;
        if (d < 0)
            $display("        c%0d (常量槽，几何在 osd_gfx 常量表里)", k);
        else
            $display("        c%0d→d%0d ty=%0d sty=%0d rad=%0d msk=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
                k, d, dut.u_gfx.u_mem.cell_rd[d][23:20], dut.u_gfx.u_mem.cell_rd[d][19:16],
                dut.u_gfx.u_mem.cell_rd[d][15:12], dut.u_gfx.u_mem.cell_rd[d][11:10],
                dut.u_gfx.u_mem.cell_rd[d][9:0],
                dut.u_gfx.u_mem.geo_lo[d][18:9],  dut.u_gfx.u_mem.geo_lo[d][8:0],
                dut.u_gfx.u_mem.geo_hi[d][18:9], dut.u_gfx.u_mem.geo_hi[d][8:0]);
    end
endtask

task show;
    input [8*20:1] tag;
    begin
        $display("  [%0s] focus=%0d nav=%0d page=%0d item=%0d edit=%0d scroll=%0d | v0..6=%0d %0d %0d %0d %0d",
            tag, `F, `NAV, `PG, `IT, `ED, `SC, `V0, `V1, `V2, `V3, `V4);
    end
endtask

// 布局一拍：产生一个 tick，让控件表/文字重新排一次
task lay;
    begin
        @(negedge clk); ps_start = 1'b1;
        repeat (8) @(posedge clk);
        @(negedge clk); ps_start = 1'b0;
        repeat (800) @(posedge clk);     // L_ANIM+L_TWN(6)+L_CELL(15)+L_TXT(126)+余量
    end
endtask

// 回 NAV（最多按 2 次 LEFT）
task to_nav;
    begin
        if (`F) kpress(4'h1);
        if (`F) kpress(4'h1);
    end
endtask

// NAV 下把 nav_idx 挪到 target（0..3）
task nav_to;
    input integer target;
    integer i;
    begin
        for (i = 0; i < 5; i = i + 1) if (`NAV != 3'd0) kpress(4'h8);
        for (i = 0; i < target; i = i + 1) kpress(4'h4);
    end
endtask

// 等到布局机正在排“显示页第 0 行”，此时 row_val/vdisp 就是亮度行的显示值
task wait_ct0;
    begin
        while (dut.u_menu.ct_row !== 3'd0) @(posedge clk);
        @(negedge clk);
    end
endtask

initial begin
    $display("=========== 逐按键行为验证 ===========");
    #200 rst = 0;
    #100;

    //--------------------------------------------------------------- 上电
    $display("--- A. 上电 ---");
    chk("power focus",  `F,  0);
    chk("power nav",    `NAV, 0);
    chk("power page",   `PG,  0);
    chk("power item",   `IT,  0);
    chk("power scroll", `SC,  0);

    //--------------------------------------------------------------- 进入菜单
    $display("--- B. KEY1 进入菜单（menu_on=0 时）---");
    enter_seen = 1'b0;
    kpress(4'h1);                            // ★ 开菜单永远是 KEY1
    chk("enter_req", enter_seen, 1);
    menu_on = 1;                             // 模拟 App 响应
    wait_n(60);
    lay();
    show("after enter");

    //--------------------------------------------------------------- 5 个导航图标
    $display("--- B2. 导航图标 cell 5..9（type12 / 32x32 / 80 间距）---");
    // ★ 2026-10-29 [S6]：图标 5..9 的几何/属性已搬到 osd_gfx 的常量表，
    //   cellmem 里不再有这些表项。它们由 tb_osd_icons 的 C 组（5×1024 点逐像素）
    //   和 E 组（激活态位图）覆盖，这里只断言"命中开关跟着菜单走"。
    chk("icon hit follows menu_on", dut.u_gfx.c_hit_icon && !dut.u_gfx.menu_on, 0);

    //--------------------------------------------------------------- NAV 上下
    $display("--- C. NAV 态上下：普通页必须即时预览（文件页除外）---");
    kpress(4'h4);  chk("nav after DN1", `NAV, 1);
                   chk("page follows nav", `PG, 1);   // ★ 普通页即时预览
                   chk("item reset",    `IT,  0);
                   chk("scroll reset",  `SC,  0);
    kpress(4'h4);  chk("nav after DN2", `NAV, 2);
    kpress(4'h4);  chk("nav after DN3", `NAV, 3);
    kpress(4'h4);  chk("nav after DN4", `NAV, 4);
    kpress(4'h4);  chk("nav clamp low", `NAV, 4);
    kpress(4'h8);  chk("nav after UP1", `NAV, 3);
    kpress(4'h8);  chk("nav after UP2", `NAV, 2);
    kpress(4'h8);  chk("nav after UP3", `NAV, 1);
    kpress(4'h8);  chk("nav after UP4", `NAV, 0);
    kpress(4'h8);  chk("nav clamp up",  `NAV, 0);
    chk("page still 0", `PG, 0);

    //--------------------------------------------------------------- 进 PAGE
    $display("--- D. KEY4 进入页面 ---");
    kpress(4'h2);
    chk("focus enter",   `F, 1);
    chk("item reset2",   `IT, 0);
    chk("scroll reset2", `SC, 0);
    chk("nav untouched", `NAV, 0);

    //--------------------------------------------------------------- 换行 + 滚动
    $display("--- E. PAGE 态上下换行与滚动跟随（显示页 7 项：item0..6）---");
    kpress(4'h4); chk("item1", `IT, 1); chk("scroll@1", `SC, 0);
    kpress(4'h4); chk("item2", `IT, 2); chk("scroll@2", `SC, 0);
    kpress(4'h4); chk("item3", `IT, 3); chk("scroll@3", `SC, 0);
    kpress(4'h4); chk("item4", `IT, 4); chk("scroll@4", `SC, 0);
    kpress(4'h4); chk("item5", `IT, 5); chk("scroll@5", `SC, 1);
    kpress(4'h4); chk("item6", `IT, 6); chk("scroll@6", `SC, 2);
    kpress(4'h4); chk("item clamp", `IT, 6);     chk("scroll clamp", `SC, 2);
    kpress(4'h8); chk("up to 5", `IT, 5);        chk("scroll keep", `SC, 2);
    kpress(4'h8); chk("up to 4", `IT, 4);        chk("scroll keep2", `SC, 2);
    kpress(4'h8); chk("up to 3", `IT, 3);        chk("scroll keep3", `SC, 2);
    kpress(4'h8); chk("up to 2", `IT, 2);        chk("scroll keep4", `SC, 2);
    kpress(4'h8); chk("up to 1", `IT, 1);        chk("scroll keep5", `SC, 1);
    kpress(4'h8); chk("up to 0", `IT, 0);        chk("scroll home", `SC, 0);
    kpress(4'h8); chk("item clamp0", `IT, 0);
    lay(); show("page0");

    //--------------------------------------------------------------- TrackBar
    $display("--- F. TrackBar(item0) 进入编辑 / 加减（DEF0 亮度=50）---");
    chk("val0 before", `V0, 50);
    kpress(4'h2); chk("edit on", `ED, 1);
    kpress(4'h4); chk("val0 +5", `V0, 55);          // ★ KEY3 增大
    kpress(4'h4); chk("val0 +10", `V0, 60);
    kpress(4'h8); chk("val0 -5", `V0, 55);          // ★ KEY4 减小
    kpress(4'h1); chk("edit off by LEFT", `ED, 0);
    chk("focus kept", `F, 1);

    //--------------------------------------------------------------- 饱和度 TrackBar
    $display("--- G. 饱和度(item3) TrackBar：编辑加减（原 item3 是 CheckBox，改版后滑条）---");
    kpress(4'h4); kpress(4'h4); kpress(4'h4);        // item 0->3
    chk("at item3", `IT, 3);
    chk("val3 init", `V3, 50);
    kpress(4'h2); chk("edit on", `ED, 1);
    kpress(4'h4); chk("val3 +5", `V3, 55);          // ★ KEY3 增大
    kpress(4'h8); chk("val3 -5", `V3, 50);          // ★ KEY4 减小
    kpress(4'h1); chk("edit off", `ED, 0);
    // （CheckBox"KEY2 直接反选不进编辑"由 N 场景在 page1 覆盖）

    //--------------------------------------------------------------- 风格 ComboBox
    $display("--- H. 风格(item4) ComboBox：弹层 + 16 字选项 + 收起态 ---");
    kpress(4'h4);                                    // item3 -> item4
    chk("at item4", `IT, 4);
    chk("val4 init", `V4, 0);                        // 无
    kpress(4'h2);
    chk("combo edit on",    `ED, 1);
    chk("pop open",         `POP, 1);
    chk("pop sel (val4=0)", `PSEL, 0);
    lay();
    chk("H sel=0 anchor", `PSEL, 0);
    chk_pop(0, 380);  chk_rows;  chk_txt_noclash;    // item4 控件顶边 y=380
    // ★ 16 字选项写入：弹层行整行写串，超出串长必须是空白码（防"值串味"）
    //   sel=0（无）：弹层第 6 行 = 第 1 项"黑白"，第 7 行 = 第 2 项"反色"
    chk("H6 opt1 c0=黑", tcap[6*32+0], 16'hBADA);
    chk("H6 opt1 c1=白", tcap[6*32+1], 16'hB0D7);
    chk("H6 opt1 c2 blank", tcap[6*32+2], 16'h0000);
    chk("H7 opt2 c0=反", tcap[7*32+0], 16'hB7B4);
    chk("H7 opt2 c1=色", tcap[7*32+1], 16'hC9AB);
    chk("H7 opt2 c2 blank", tcap[7*32+2], 16'h0000);
    // 锚定行（文字行 5）：选中项"无"1 字，第 15 列必须空白
    chk("H5 anchor c0=无", tcap[5*32+14], 16'hCEDE);
    chk("H5 anchor c1 blank", tcap[5*32+15], 16'h0000);
    chk("H txt line0 title y=24", dut.u_menu.txt_xy[0*22+11 +: 11], 24);
    chk("H txt line5 row4 y=384", dut.u_menu.txt_xy[5*22+11 +: 11], 384);

    // ★ 2026-10-27：KEY4=上（选项 -1）、KEY3=下（选项 +1）—— 用户要求的
    //   "KEY4 向上、KEY3 向下"。这里用 KEY3 往下走到 1、2，再夹紧。
    kpress(4'h4); chk("val4 -> 1", `V4, 1);
    chk("pop sel follows", `PSEL, 1);
    lay(); chk_pop(1, 380); chk_rows; chk_txt_noclash; // 选中黑白：面板整体上移一格
    chk("H5 anchor c0=黑", tcap[5*32+14], 16'hBADA);
    chk("H5 anchor c1=白", tcap[5*32+15], 16'hB0D7);
    chk("H5 anchor c2 blank", tcap[5*32+16], 16'h0000);
    kpress(4'h4); chk("val4 -> 2", `V4, 2);
    chk("pop sel top", `PSEL, 2);
    lay(); chk_pop(2, 380); chk_rows; chk_txt_noclash;
    chk("H5 anchor c0=反", tcap[5*32+14], 16'hB7B4);
    kpress(4'h4); chk("val4 clamp", `V4, 2);
    lays(4);                                          // 等 clamp 后的文字重写
    chk("H5 anchor still 反", tcap[5*32+14], 16'hB7B4);
    kpress(4'h1); chk("combo edit off", `ED, 0);
    chk("pop closed", `POP, 0);
    lays(4);
    // ★ 收起态：值区第 14..15 列 = "反色"完整 2 字，第 16 列空白
    //   （收起态容量 16 字 = 第 14..29 列；2 字选项之后的列不许残留）
    chk("H closed c0=反", tcap[5*32+14], 16'hB7B4);
    chk("H closed c1=色", tcap[5*32+15], 16'hC9AB);
    chk("H closed c2 blank", tcap[5*32+16], 16'h0000);

    //--------------------------------------------------------------- 恢复默认 Button
    $display("--- I. 恢复默认(item5) Button：KEY2 触发动作 + pic_tgl ---");
    kpress(4'h4);                                     // item4 -> item5
    chk("at item5", `IT, 5);
    v1_save = dut.u_menu.pic_tgl;                     // 记录 tgl 现值
    kpress(4'h2);                                     // KEY2 on Button
    chk("I1 button no edit", `ED, 0);
    chk("I2 restore vals", ((`V0==50)&&(`V1==50)&&(`V2==50)&&(`V3==50)&&(`V4==0)) ? 1 : 0, 1);
    chk("I3 tgl toggled", (dut.u_menu.pic_tgl != v1_save) ? 1 : 0, 1);
    lays(6);                                          // 等 DEF0 的文字/滑条重写
    chk("I4 row0 fill back to 50", dut.u_gfx.u_mem.cell_rd[1][9:0], (50*737)>>8);

    //--------------------------------------------------------------- 回 NAV / 切页
    $display("--- J. KEY3 回 NAV，切页要重置 item/scroll ---");
    kpress(4'h1);
    chk("back to nav", `F, 0);
    kpress(4'h4);                                // nav 0 -> 1
    chk("nav=1", `NAV, 1);
    chk("page follows nav", `PG, 1);             // ★ 普通页跟随光标预览
    kpress(4'h2);                                // 确认进 page1
    chk("page1 focus", `F, 1);
    chk("page=1 on confirm", `PG, 1);
    chk("item reset on confirm", `IT, 0);
    chk("scroll reset on confirm", `SC, 0);
    lay();
    kpress(4'h4); kpress(4'h4); kpress(4'h4); kpress(4'h4); kpress(4'h4);
    chk("page1 item5", `IT, 5);
    chk("page1 scroll", `SC, 1);
    kpress(4'h1);                                // 回 NAV
    kpress(4'h4);                                // nav 1 -> 2
    chk("nav=2", `NAV, 2);
    // ★ 2026-11-02 用户口径：文件页和其它页一样**即时预览**，
    //   不再"必须按确定才显示"。原来的断言（导航不换页）是旧行为。
    chk("page follows nav", `PG, 2);
    kpress(4'h2);                                // 确认进 page2（幂等）
    chk("page=2 on confirm", `PG, 2);
    chk("scroll reset on confirm", `SC, 0);
    chk("item reset on confirm", `IT, 0);

    //--------------------------------------------------------------- 四页类型表
    $display("--- K. 四页各自的 item 类型（用于核对可编辑性）---");
    begin : pgloop
        integer p, i, ni;
        for (p = 0; p < 5; p = p + 1) begin
            to_nav;
            nav_to(p);
            chk("nav_to ok", `NAV, p);
            $display("    page%0d:", p);
            if (p == 4) begin
                // 关于页是纯展示页：KEY2 不进页面（页面内容仍即时预览）
                kpress(4'h2);
                chk("about stays nav", `F, 0);
                chk("page ok", `PG, 4);
            end else begin
                kpress(4'h2);                    // 进该页（★ 确认才换页）
                chk("in page", `F, 1);
                chk("page ok",   `PG,  p);
                // 显示页 7 项、相册页 6 项、文件页 1 项、系统页 5 项
                ni = (p == 0) ? 7 : (p == 1) ? 6 : (p == 2) ? 1 : 5;
                for (i = 0; i < ni; i = i + 1) begin
                    chk("item pos", `IT, i);
                    $display("      item%0d type=%0d  editable=%0d",
                             i, ctype(p[1:0], i[2:0]),
                             (ctype(p[1:0], i[2:0]) == 4'd4) ||
                             (ctype(p[1:0], i[2:0]) == 4'd5) ||
                             (ctype(p[1:0], i[2:0]) == 4'd6) ||
                             (ctype(p[1:0], i[2:0]) == 4'd10));
                    if (i != ni-1) kpress(4'h4);
                end
                kpress(4'h1);                    // 回 NAV
            end
        end
    end

    //--------------------------------------------------------------- 相册页 CheckBox
    //  ★ 2026-11-03：删除“适应屏幕”后，信息条=item4，音随图切=item5。
    $display("--- N. 相册页 CheckBox：item4 信息条 / item5 音随图切 ---");
    to_nav; nav_to(1);
    kpress(4'h2);                                    // 进 page1
    chk("p1 in page", `F, 1);
    begin : toi4
        integer i;
        for (i = 0; i < 4; i = i + 1) kpress(4'h4);   // item 0 -> 4
    end
    chk("p1 at item4", `IT, 4);
    chk("info bar init", `V4, 0);
    kpress(4'h2); chk("cb4 no edit", `ED, 0);
    chk("cb4 -> 1", `V4, 1);
    kpress(4'h2); chk("cb4 -> 0", `V4, 0);
    // 下移到 item5（音随图切，出厂默认 1）
    kpress(4'h4);
    chk("p1 at item5", `IT, 5);
    chk("asy init", `V5, 1);
    kpress(4'h2); chk("cb5 no edit", `ED, 0);
    chk("cb5 -> 0", `V5, 0);
    kpress(4'h2); chk("cb5 -> 1", `V5, 1);
    // 自动切换(item3) 是 ComboBox(6 项)：导航态上下键只换行，绝不改值
    begin : b3
        integer i;
        for (i = 0; i < 2; i = i + 1) kpress(4'h8);   // item 5 -> 3
    end
    chk("p1 at item3", `IT, 3);
    v_save = `V3;
    kpress(4'h8); chk("up moves only",   `IT, 2);
    chk("up keeps val", `V3, v_save);
    kpress(4'h4); chk("down moves only", `IT, 3);
    chk("down keeps val", `V3, v_save);

    //--------------------------------------------------------------- 语言 ComboBox
    $display("--- O. 语言(page3 item0) ComboBox：2 项，0..1 钳位 ---");
    to_nav; nav_to(3);
    kpress(4'h2);
    chk("p3 in page", `F, 1);
    chk("p3 item0", `IT, 0);
    chk("lang init", `V0, 0);                 // ★ 2026-10-29 默认中文（原来默认 1）
    kpress(4'h2); chk("lang edit on", `ED, 1);
    chk("lang pop open", `POP, 1);
    // ★ KEY4=上（值 -1）、KEY3=下（值 +1）；nopt 从 3 变 2
    kpress(4'h4); chk("lang 0->1", `V0, 1);
    kpress(4'h4); chk("lang clamp 1", `V0, 1);
    kpress(4'h8); chk("lang 1->0", `V0, 0);
    kpress(4'h8); chk("lang clamp 0", `V0, 0);
    kpress(4'h1); chk("lang edit off", `ED, 0);
    chk("O1 o_lang follows val0", o_lang, `V0);

    //--------------------------------------------------------------- 自动亮度显示
    //  ★ 2026-11-03：自动亮度打开后，亮度滑条/百分比显示 bright_eff，而不是手动 val0。
    $display("--- O2. 自动亮度：亮度行显示 bright_eff ---");
    to_nav; nav_to(0); kpress(4'h2);                 // 进 page0 item0
    begin : o2_to6
        integer i;
        for (i = 0; i < 6; i = i + 1) kpress(4'h4);
    end
    chk("O2 at item6", `IT, 6);
    chk("O2 auto init off", `V6, 0);
    kpress(4'h2);                                    // 打开自动亮度
    chk("O2 auto on", `V6, 1);

    bright_eff = 7'd37;                              // 模拟 ALS 生效值
    lays(2);
    wait_ct0;
    chk("O2 display=37", dut.u_menu.row_val, 37);
    chk("O2 fill=37",    dut.u_menu.vfill, 106);     // (37*737)>>8
    chk("O2 manual kept", `V0, 50);

    bright_eff = 7'd73;
    lays(2);
    wait_ct0;
    chk("O2 display=73", dut.u_menu.row_val, 73);
    chk("O2 fill=73",    dut.u_menu.vfill, 210);     // (73*737)>>8

    kpress(4'h2);                                    // 关自动亮度
    chk("O2 auto off", `V6, 0);
    lays(2);
    wait_ct0;
    chk("O2 back manual", dut.u_menu.row_val, 50);
    bright_eff = 7'd90;
    lays(2);
    wait_ct0;
    chk("O2 manual unaffected", dut.u_menu.row_val, 50);
    bright_eff = 7'd50;                              // 恢复缺省，避免影响后续台本

    //--------------------------------------------------------------- 每页值独立
    //  ★ 原来 7 个值寄存器按行号被 4 个页共用 ⇒ 改一页等于改了别的页。
    $display("--- P. 每页的值必须互不影响 ---");
    to_nav; nav_to(3); kpress(4'h2);                 // 确认进 page3
    chk("P1 page3 lang=0", `V0, 0);
    to_nav; nav_to(0); kpress(4'h2);                 // 确认进 page0
    chk("P2 at item0", `IT, 0);
    v_save = `V0;
    $display("       (page0 亮度当前值 = %0d，再 +15)", v_save);
    kpress(4'h2); kpress(4'h4); kpress(4'h4); kpress(4'h4); kpress(4'h1);   // KEY3 ×3
    chk("P3 bright+15", `V0, v_save + 15);
    to_nav; nav_to(3); kpress(4'h2);                 // 确认进 page3
    chk("P4 page3 val0 unchanged", `V0, 0);           // 语言没被亮度带跑
    to_nav; nav_to(0); kpress(4'h2);                 // 确认回 page0
    chk("P5 back page0 restored", `V0, v_save + 15);

    //--------------------------------------------------------------- 滚到最底部
    //  ★ 这一段专盯"滚动到 page 最下方"：行控件 y、文字行 y、游标 y、弹层位置
    //    都必须还在屏内，且文字行要**跟着 scroll_row 一起偏移**
    //    （原来文字 y 漏加 scroll_row*PITCH ⇒ 一滚动标签就错行、最后一行没字）。
    $display("--- Q. 滚到 page 最下方（item=6, scroll_row=2，显示页 7 项）---");
    to_nav; nav_to(0);
    kpress(4'h2);                                    // 进 page0
    begin : to6
        integer i;
        for (i = 0; i < 6; i = i + 1) kpress(4'h4);
    end
    chk("Q1 at item6", `IT, 6);
    chk("Q2 scroll=2", `SC, 2);
    lays(22);
    show_cell(9); show_cell(10); show_cell(11); show_cell(12); show_cell(13);
    $display("       txt_xy y = %0d %0d %0d %0d %0d %0d",
        dut.u_menu.txt_xy[0*22+11 +: 11], dut.u_menu.txt_xy[1*22+11 +: 11],
        dut.u_menu.txt_xy[2*22+11 +: 11], dut.u_menu.txt_xy[3*22+11 +: 11],
        dut.u_menu.txt_xy[4*22+11 +: 11], dut.u_menu.txt_xy[5*22+11 +: 11]);
    begin : chkq
        integer k;
        // 行控件：窗口第 k 行（k=0..4）应在 y = 60 + 80k
        for (k = 0; k < 5; k = k + 1)
            chk("Q row y", dut.u_gfx.u_mem.geo_lo[1+k][8:0], 60 + 80*k);
        // 文字：第 l 行（l=1..5）应在 y = 64 + 80(l-1)（含 scroll_row 偏移后与控件对齐）
        for (k = 1; k < 6; k = k + 1)
            chk("Q txt y", dut.u_menu.txt_xy[k*22+11 +: 11], 64 + 80*(k-1));
    end
    // 到底了再按下：item / scroll 都不许动
    kpress(4'h4);
    chk("Q3 item clamp at 6", `IT, 6);
    chk("Q4 scroll clamp at 2", `SC, 2);
    // 游标应在最后一行的位置：cy = ROW0 + 4*80 - 6 = 378
    chk("Q5 cursor y", dut.u_menu.cy_t, 378);
    // 一路按回顶部，滚动要跟着回收
    begin : toup
        integer i;
        for (i = 0; i < 6; i = i + 1) kpress(4'h8);
    end
    chk("Q6 item back 0", `IT, 0);
    chk("Q7 scroll back 0", `SC, 0);
    lays(22);
    chk("Q8 txt y line1", dut.u_menu.txt_xy[1*22+11 +: 11], 64);
    chk("Q9 row0 y", dut.u_gfx.u_mem.geo_lo[1][8:0], 60);
    kpress(4'h1);                                    // 回 NAV

    //--------------------------------------------------------------- 上下换行不能改值
    //  ★ 复现用户报的"上下切换会导致控件显示值错位/错乱，连动画都会改值"：
    //    换行后，新选中行的滑条填充/数字必须**立刻**是本行的值，
    //    不允许还顶着上一行的值做过渡动画。
    $display("--- R. 上下换行不能改变任何一行的显示值 ---");
    to_nav; nav_to(0);
    kpress(4'h2);                                    // 进 page0
    lays(22);
    chk("R1 at item0", `IT, 0);
    v_save  = `V0;
    v1_save = `V1;
    $display("       (亮度=%0d 对比度=%0d)", v_save, v1_save);
    chk("R2 row0 fill", dut.u_gfx.u_mem.cell_rd[1][9:0],  (v_save*737)>>8);
    chk("R2b row1 fill", dut.u_gfx.u_mem.cell_rd[2][9:0], (v1_save*737)>>8);
    kpress(4'h4);                                    // ↓ → item1（对比度）
    lays(2);                                         // 只排 2 帧：正好在动画中途
    chk("R3 at item1", `IT, 1);
    chk("R4 new row fill ok", dut.u_gfx.u_mem.cell_rd[2][9:0], (v1_save*737)>>8);
    chk("R5 prev row intact",     dut.u_gfx.u_mem.cell_rd[1][9:0],  (v_save*737)>>8);
    kpress(4'h8);                                    // ↑ 回 item0
    lays(2);
    chk("R6 at item0", `IT, 0);
    chk("R7 row0 fill intact", dut.u_gfx.u_mem.cell_rd[1][9:0],  (v_save*737)>>8);
    chk("R8 row1 fill intact", dut.u_gfx.u_mem.cell_rd[2][9:0], (v1_save*737)>>8);

    // ★ 反过来也要保证：编辑时"值动画"不能被一起修没了
    v_save = `V0;
    kpress(4'h2);                                    // 进编辑（TrackBar）
    chk("R9 enter edit", `ED, 1);
    chk("R10 sv snapped", dut.u_menu.sv_c[15:4], v_save);
    kpress(4'h4);                                    // 值 +5（步进 5，KEY3 增大）
    lays(1);
    chk("R11 sv animating", (dut.u_menu.sv_c[15:4] == v_save) ? 1 : 0, 1);
    lays(20);
    chk("R12 sv settled", dut.u_menu.sv_c[15:4], v_save + 5);
    chk("R13 val +5", `V0, v_save + 5);
    kpress(4'h1);                                    // 退出编辑
    chk("R14 edit off", `ED, 0);

    // ★ 复现用户报的"slider 调大一次数值不变、按确定才变；调大两次只涨一次"：
    //   值寄存器立刻变、滑条也在动，但**文字只在 dirty 时重写**，而值动画要走
    //   十几帧 —— 这期间没有 dirty ⇒ 数字停在按前的进度。
    //   断言方式：值动画在飞的时候，文字版本号必须还在涨（说明每帧都在重写）。
    v_save = `V0;
    kpress(4'h2);                                    // 进编辑
    kpress(4'h4);                                    // 值 +5（步进 5，KEY3 增大）
    lays(2);
    v1_save = dut.u_menu.txt_ver_r;                  // 记下当前文字版本
    lays(3);                                         // 动画还在走
    chk("R15 text keeps refreshing", (dut.u_menu.txt_ver_r != v1_save) ? 1 : 0, 1);
    lays(20);
    chk("R16 value settled", `V0, v_save + 5);
    kpress(4'h1);                                    // 退出编辑

    //--------------------------------------------------------------- 滚到底再进来
    //  ★ 复现用户报的"滚到最底部→回导航栏→再进入，控件错位、文字不显示"。
    $display("--- S. 滚到底 → 回导航栏 → 再进页面 ---");
    to_nav; nav_to(0);
    kpress(4'h2);                                    // 进 page0
    begin : s_dn
        integer i;
        for (i = 0; i < 5; i = i + 1) kpress(4'h4);  // 一路到 item5
    end
    lays(22);
    chk("S1 scroll=1", `SC, 1);
    chk("S2 last row y=380", dut.u_gfx.u_mem.geo_lo[5][8:0], 380);
    kpress(4'h1);                                    // KEY1：页面 → 导航栏
    chk("S3 back to nav", `F, 0);
    kpress(4'h2);                                    // KEY2：再进页面
    chk("S4 item=0", `IT, 0);
    chk("S5 scroll=0", `SC, 0);
    lays(22);
    chk("S6 row0 y=60", dut.u_gfx.u_mem.geo_lo[1][8:0], 60);
    chk("S7 row4 y=380", dut.u_gfx.u_mem.geo_lo[5][8:0], 380);
    chk("S8 txt line1 y=64", dut.u_menu.txt_xy[1*22+11 +: 11], 64);
    chk("S9 txt line5 y=384", dut.u_menu.txt_xy[5*22+11 +: 11], 384);

    //--------------------------------------------------------------- CheckBox 直接反选
    //  （T 场景已删：显示页 item3 原是"高动态"CheckBox，2026-10-26 改为饱和度
    //    TrackBar（G 场景覆盖）；"KEY2 直接反选不进编辑"由 N 场景在 page1 覆盖。）

    //--------------------------------------------------------------- 退出 OSD
    //  ★ 长按退出已取消。现在的规则：
    //    · 在**导航栏**（focus=0）按 KEY1  ⇒ exit_req 单拍（App 收到就关 OSD）
    //    · 在**页面里**按 KEY1            ⇒ 只回导航栏，**不许**关 OSD
    $display("--- L. KEY1 退出规则（导航栏=关 OSD / 页面内=只返回）---");
    to_nav;
    chk("L1 in nav", `F, 0);
    kpress(4'h2);                                    // 进页面
    chk("L2 in page", `F, 1);
    exit_seen = 1'b0;
    kpress(4'h1);                                    // 页面内按 KEY1
    chk("L3 back to nav", `F, 0);
    chk("L4 no exit in page", exit_seen, 0);         // ← 页面里不许退出
    exit_seen = 1'b0;
    kpress(4'h1);                                    // 导航栏按 KEY1
    chk("L5 exit_req in nav", exit_seen, 1);         // ← 这一下才关 OSD
    chk("L6 still nav", `F, 0);
    // 连着按也不该重复触发（单拍脉冲）
    exit_seen = 1'b0;
    wait_n(40);
    chk("L7 single pulse", exit_seen, 0);

    //--------------------------------------------------------------- 弹层落在第 0 行
    //  ★ 用户报："滚到最底下再打开色温 dropdown，弹层顶端顶到标题栏"。
    //  ★ 2026-10-29：系统页改成 5 项后这一页不再滚动，但"弹层落在窗口第 0 行"
    //    这个最坏情况反而更好构造 —— 语言下拉天生就在 sel_k=0。
    //    断言仍是那三条：pop_a 夹到 0、面板不进标题区（y>=48）、被盖住的行清空。
    $display("--- U. 弹层落在窗口第 0 行：不许压标题（page3 语言）---");
    to_nav; nav_to(3);
    kpress(4'h2);                                    // 进 page3，item0
    lays(22);
    chk("U1 at item0", `IT, 0);
    chk("U2 sel_k = 0", dut.u_menu.sel_k, 0);
    kpress(4'h2);                                    // 打开弹层
    lays(6);
    chk("U3 pop open", `POP, 1);
    chk("U4 pop_sel follows val0", `PSEL, `V0);
    chk("U5 pop_a clamped", dut.u_menu.pop_a, 0);
    $display("      pop_y=%0d panel=(%0d..%0d) nrow=%0d opt6=%0d opt7=%0d",
             dut.u_menu.pop_y, dut.u_menu.pop_y - 6,
             dut.u_menu.pop_y + (dut.u_menu.pop_nrow - 1) * 32 + 28 + 6,
             dut.u_menu.pop_nrow,
             dut.u_gfx.u_mem.geo_lo[6][8:0], dut.u_gfx.u_mem.geo_lo[7][8:0]);
    chk("U6 panel top >= 48", ((dut.u_menu.pop_y - 6) >= 48) ? 1 : 0, 1);
    chk("U7 panel bot <= 480",
        ((dut.u_menu.pop_y + (dut.u_menu.pop_nrow - 1) * 32 + 28 + 6) <= 480) ? 1 : 0, 1);
    chk("U8 opt0 inside screen", (dut.u_gfx.u_mem.geo_lo[6][8:0] >= 48) ? 1 : 0, 1);
    $display("      line2 (volume) col0=%04h  (must be D2F4 = 音)", cap00[2]);
    // ★ 2026-10-29：语言下拉只有 2 项 ⇒ 面板只包 2 行（pop_nrow=2），
    //   下一行（音量）**不在**面板底下 ⇒ 它的标签必须还在。
    //   原来面板写死 3 行，这一行会被 pop_wx1 清空，屏幕上"缺一行"。
    chk("U9 volume label kept", cap00[2], 16'hD2F4);
    chk("U9b pop_nrow = 2", dut.u_menu.pop_nrow, 2);
    chk("U9c panel bottom above row1",
        ((dut.u_menu.pop_y + (dut.u_menu.pop_nrow - 1) * 32 + 28 + 6) <= 140) ? 1 : 0, 1);
    chk_txt_noclash;
    kpress(4'h4); lays(4);                           // 切到 EN
    chk("U10 val0 -> 1", `V0, 1);
    chk("U11 pop_a still 0", dut.u_menu.pop_a, 0);
    chk("U12 panel top >= 48", ((dut.u_menu.pop_y - 6) >= 48) ? 1 : 0, 1);
    chk("U13 o_lang = 1", o_lang, 1);
    chk_txt_noclash;
    kpress(4'h8); lays(4);                           // 回中文
    kpress(4'h1); lays(4);                           // 退出编辑

    //--------------------------------------------------------------- 系统页 5 项
    //  ★ 旧断言是"系统页 4 个下拉互不影响"。那一页现在是 1 下拉 + 1 滑条 + 3 勾选，
    //    改成验证：控件类型对、改音量不串味、3 个勾选各自独立、导出信号跟随。
    $display("--- V. system page: 1 combo + 1 slider + 3 checkbox ---");
    to_nav; nav_to(3);
    kpress(4'h2);
    lays(22);
    chk("V1 at item0", `IT, 0);
    chk("V2 item0 ComboBox", dut.u_menu.cell_type(3'd3, 3'd0), 4'd6);
    chk("V3 item1 TrackBar", dut.u_menu.cell_type(3'd3, 3'd1), 4'd4);
    chk("V4 item2 CheckBox", dut.u_menu.cell_type(3'd3, 3'd2), 4'd5);
    chk("V5 item3 CheckBox", dut.u_menu.cell_type(3'd3, 3'd3), 4'd5);
    chk("V6 item4 CheckBox", dut.u_menu.cell_type(3'd3, 3'd4), 4'd5);
    // ---- 音量滑条：100 -> 95（KEY4 = 减），别的值不许动
    kpress(4'h4);
    chk("V7 at item1", `IT, 1);
    chk("V8 vol init 100", `V1, 100);
    v_save  = `V2;
    v1_save = `V3;
    kpress(4'h2);
    kpress(4'h8); lays(4);
    chk("V9 vol 100->95", `V1, 95);
    chk("V10 mute untouched", `V2, v_save);
    chk("V11 loop untouched", `V3, v1_save);
    kpress(4'h1); lays(4);
    chk("V12 o_vol follows val1", o_vol, `V1);
    // ---- 静音：CheckBox 单击翻转，不进编辑态
    kpress(4'h4);
    chk("V13 at item2", `IT, 2);
    v_save = `V2;
    kpress(4'h2); lays(4);
    chk("V14 mute toggled", `V2, (v_save == 0) ? 1 : 0);
    chk("V15 edit not entered", `ED, 0);
    chk("V16 o_mute follows val2", o_mute, `V2);
    kpress(4'h2); lays(4);
    chk("V17 mute back", `V2, v_save);
    // ---- 循环播放：默认勾（保持改造前的行为）
    kpress(4'h4);
    chk("V18 at item3", `IT, 3);
    chk("V19 loop init 1", `V3, 1);
    kpress(4'h2); lays(4);
    chk("V20 loop toggled", `V3, 0);
    chk("V21 o_loop follows val3", o_loop, `V3);
    kpress(4'h2); lays(4);
    chk("V22 loop back", `V3, 1);
    // ---- 频谱条：预留，只存位
    kpress(4'h4);
    chk("V23 at item4", `IT, 4);
    kpress(4'h2); lays(4);
    chk("V24 spc toggled", `V4, 1);
    chk("V25 o_spc follows val4", o_spc, 1);
    kpress(4'h2); lays(4);
    // ---- item_last = 4：再按下键必须停在最后一行
    kpress(4'h4);
    chk("V26 clamp at item4", `IT, 4);
    kpress(4'h1);                                    // 回 NAV

    //--------------------------------------------------------------- Button 外观
    //  ★ 用户报："按钮控件不应该是完全空的，应该和 checkbox 一样放一个圆角矩形，
    //    里面写执行"。
    $display("--- W. Button：圆角矩形 + 执行（恢复默认） ---");
    to_nav; nav_to(0);
    kpress(4'h2);
    begin : w_dn
        integer i;
        for (i = 0; i < 5; i = i + 1) kpress(4'h4);   // item0 -> 5，scroll=1
    end
    lays(22);
    chk("W1 at item5", `IT, 5);
    $display("      line5 (恢复默认) col14=%04h col15=%04h", cap14[5], cap15[5]);
    chk("W2 button col14 = 执", cap14[5], 16'hD6B4);
    chk("W3 button col15 = 行", cap15[5], 16'hD0D0);
    // 按钮行必须有实心圆角矩形：类型 7、有边框、有圆角
    // item5 在 scroll=1 时落在窗口第 4 行 ⇒ 行控件槽 = cell 13
    chk("W4 button type = 7",  dut.u_gfx.u_mem.cell_rd[5][23:20], 7);
    chk("W5 button has radius", ((dut.u_gfx.u_mem.cell_rd[5][15:12] != 0) ? 1 : 0), 1);
    chk("W6 button rect width", dut.u_gfx.u_mem.geo_hi[5][18:9] - dut.u_gfx.u_mem.geo_lo[5][18:9], 296);
    // ★ 2026-10-29：原来这里还有一组"page3 item5 = 升级固件按钮"的同款外观断言。
    //   系统页改成 5 项后这一页已经没有 Button 了，那 4 条断言随之删除；
    //   Button 的外观由上面 W1..W6（显示页"恢复默认"）覆盖，控件像素逻辑是共用的。
    kpress(4'h1);                                    // 回 NAV

    //--------------------------------------------------------------- 长按连发
    //  ★ tb 用快速档 RPT_FIRST=2 / RPT_EVERY=1：按住第 2 个 local_tick 起连发、
    //    每 1 tick 一发。按住 8 个 tick ⇒ 恰好 +3（第 3/5/7 拍各一发）。
    //    导航态**不连发**（连发只注入编辑态的值加减路径）。
    $display("--- X. 长按连发 ---");
    to_nav; nav_to(0);
    kpress(4'h2);                                    // 进 page0 item0
    kpress(4'h2);                                    // 进编辑
    chk("X1 edit on", `ED, 1);
    v_save = `V0;
    key_level = 4'hB;                                // KEY3 按住（bit2=0）⇒ 滑条连发增大
    repeat (8 * 833334) @(posedge clk);              // 8 个 local_tick
    key_level = 4'hF;                                // 松开
    repeat (4 * 833334) @(posedge clk);              // 等动画/文字重写收尾
    chk("X2 repeated +15", `V0, v_save + 15);
    chk("X3 edit still on", `ED, 1);
    kpress(4'h1);                                    // 退出编辑
    chk("X4 edit off", `ED, 0);
    key_level = 4'h7;                                // 导航态按住：不许连发
    repeat (8 * 833334) @(posedge clk);
    key_level = 4'hF;
    repeat (2000) @(posedge clk);
    chk("X5 nav no repeat", `IT, 0);
    kpress(4'h1);                                    // 回 NAV

    //--------------------------------------------------------------- 页面板 / 分割线
    //  ★ 这两条是"上板才有、出图看不出来"的杂像素/多余圆角，根因全在这几个数值：
    //    分割线必须和面板**同透过程度**（style 10 的 level 已由 tb_osd_style 守住），
    //    页面板必须从 y=0 开始、radius=0、且不再自带 1px 边框（Solid）。
    //  cell_r 位布局：{gt[62:59], gs[58:55], grad[54:51], gpm[50:49], gv[48:39],
    //                  gx1[38:29], gy1[28:19], gx0[18:9], gy0[8:0]}
    $display("--- D. 页面板 / 分割线 几何 ---");
    // ★ 2026-10-29 [S6]：遮罩(0)/左栏(1)/分割线(2)/页面板(3) 已经是 osd_gfx 里的
    //   **常量矩形**（不再有 cellmem 表项），所以改成"打一个像素看命中索引 + 看
    //   读回来的几何/属性"，比读表更接近真实像素通路。
    //   rd_geo 位布局 {en, x1[37:28], y1[27:19], x0[18:9], y0[8:0]},
    //   rd_attr 位布局 {type[23:20], style[19:16], rad[15:12], mask[11:10], v0[9:0]}
    px = 16'd97; py = 16'd200; repeat (3) @(negedge clk);
    chk("D1 divider hit idx=2",   dut.u_gfx.hit_idx, 2);
    chk("D2 divider x0 = 96",     dut.u_gfx.rd_geo[18:9],  96);
    chk("D3 divider x1 = 99 (3px)", dut.u_gfx.rd_geo[37:28], 99);
    chk("D4 divider style = 10",  dut.u_gfx.rd_attr[19:16], 10);
    chk("D5 divider type = Solid",dut.u_gfx.rd_attr[23:20], 0);
    px = 16'd200; py = 16'd200; repeat (3) @(negedge clk);
    chk("D6 page panel hit idx=3", dut.u_gfx.hit_idx, 3);
    chk("D7 page panel x0 = 99",  dut.u_gfx.rd_geo[18:9], 99);
    chk("D8 page panel y0 = 0",   dut.u_gfx.rd_geo[8:0],  0);
    chk("D9 page panel rad = 0",  dut.u_gfx.rd_attr[15:12], 0);
    chk("D10 page panel style = 1", dut.u_gfx.rd_attr[19:16], 1);
    px = 16'd50; py = 16'd200; repeat (3) @(negedge clk);
    chk("D11 left bar hit idx=1", dut.u_gfx.hit_idx, 1);
    chk("D12 left bar x1 = 96",   dut.u_gfx.rd_geo[37:28], 96);
    // 分割线和面板必须**紧贴**（不留 1px 缝）：常量里 99 与 99 相接
    chk("D13 divider/panel 相接", 99, 99);

    //--------------------------------------------------------------- 汇总
    $display("======================================");
    if (fails == 0)
        $display("  ALL PASS  (%0d checks)", checks);
    else
        $display("  FAILED    %0d of %0d checks", fails, checks);
    $display("======================================");
    $finish;
end

endmodule
