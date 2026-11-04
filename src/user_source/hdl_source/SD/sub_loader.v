`timescale 1ns/1ps
//=============================================================================
//  sub_loader.v —— 动态字幕装载器（SD 卡 TXT → 字符码 → Flash 字模 → BRAM）
//=============================================================================
//
//  【它干什么】读"与当前图同名"的 TXT（8.3 短名，P001.BMP ↔ P001.TXT），解析成
//    字符码，再把每个字的 16×16 字模从 SPI Flash 字库读出来、写进 dyn_subtitle
//    的字形 RAM。
//
//  ★ 2026-11-07 改版：双缓冲 + 解析/装字解耦 + 省 LUT
//    · 码表、字形 RAM 都分 front / back 两个 bank（两块 BRAM32K 本来就有 2048 字，
//      原来只用了一半）。front = 正在显示的字幕，back = 正在装的新字幕。
//    · 新字幕：解析进 back 码表 → 首批 min(len,64) 个字模装进 back 字形 bank →
//      **提交**（翻 bank、换长度、O_commit_tgl 翻转）。提交之前旧字幕照常滚动，
//      它的懒加载 / 间隙重装也照常进行（优先级高于新字幕的首批装载）。
//    · TXT 解析是独立的小状态机：SD 扇区在读的时候，字模装载器照样能给旧字幕
//      补字，互不等待。
//    · 原来的 gb_addr 两级流水（3 个 14 位加法器 + 24 位三选一）换成串行算址：
//      一个 13 位加法器逐区累加 94，最多 ~85 拍（1.7us）≪ 一个字模的 Flash 读。
//    · 帧计数改成"超前 192 帧的相位" fa，懒加载 / 间隙 / 回卷都化成
//      fa[12:4] 上的 9 位比较或相等判定，不再有 13 位减法器。
//
//  【为什么只读一个扇区】fat_dir_lister 不给文件长度（没有 FAT 链），所以只拿
//    "文件首扇区"：512 B = 256 个汉字（ASCII 也最多收 256 个，码表只有 256 项）。
//
//  【文本格式】ANSI / GB2312：
//      · 0x20..0x7E 单字节 ASCII（空格保留）
//      · 0xA1..0xFE 双字节：{区码, 位码}
//      · 0x00 / 0x0D / 0x0A ⇒ 单行结束（后面的字节全部忽略）
//      · 其余（0x01..0x1F、0x7F、0x80..0xA0、0xFF）跳过
//      · 一个有效字符都没有 ⇒ len=0，提交后不显示字幕
//
//  【调度：每个 bank 64 个字形槽】屏幕上同时最多 41 个字。
//    · 槽号 slot = 文本位置 % 64；
//    · 首批装 char 0..min(len,64)-1，装完才提交；
//    · len > 64 时：char k(≥64) 在 fa[12:4] ≥ k（即提前 192 帧）时装进槽 k%64；
//      旧占用者 char k−64 早在提前 368 帧时就已滚出屏幕；
//    · 文字整段滚出屏幕后的 16 帧窗口（fa[12:4] == len+53）里把 0..63 重装，
//      给下一圈用（64 字 × ~100us ≪ 267ms）。len ≤ 64 时什么都不用补。
//
//  【帧计数】dyn_subtitle 每帧翻一次 vs_tgl，这里 2FF + 边沿检测得到 frame_tick。
//    提交时 fa 置 192，dyn_subtitle 在下一个 vs 沿把 pm 置 −640 ⇒ 两边同步（±1 帧）。
//    fa = phase + 192；回卷点 phase = 640+len×16+31 ⇔ fa = 16×(len+53)+15。
//
//  【重复启动】I_start 可在任何时刻到达，参数当拍锁存；正在读的 SD 扇区 / Flash
//    字模会跑完再重启（中途撤 req 会让仲裁器错位）。重启只作废 back bank，
//    front（正在显示的）不受影响。
//
//  【时钟域】sd_card_clk（50MHz）。字模 BRAM 的端口 A 也在这个域。
//=============================================================================

module sub_loader (
    input  wire        clk,         // sd_card_clk 50MHz
    input  wire        rst,

    // ---- 启动（单拍，可在忙碌时到达）----
    input  wire        I_start,
    input  wire        I_txt_found, // 本次扫描是否找到同名 TXT
    input  wire [22:0] I_txt_sec,   // 该 TXT 的文件首扇区（LBA）

    // ---- SD 客户 3 ----
    output reg         O_sd_req,
    output wire [31:0] O_sd_addr,   // ★ 扇区号（与 fat_dir_lister 一致）
    input  wire [7:0]  I_sd_data,
    input  wire        I_sd_dv,
    input  wire        I_sd_end,

    // ---- Flash 客户 3（字库）----
    output reg         O_fl_req,
    output wire [23:0] O_fl_addr,
    output wire [15:0] O_fl_len,    // 恒 32（一个字模 32 字节）
    input  wire        I_fl_rdy,
    input  wire [7:0]  I_fl_data,
    input  wire        I_fl_valid,
    input  wire        I_fl_done,

    // ---- 帧脉冲（video_clk 域，每帧翻转）----
    input  wire        I_vs_tgl,

    // ---- 字模写口（→ dyn_subtitle 的 BRAM 端口 A）----
    output reg  [10:0] O_wr_addr,   // {bank, slot[5:0], row[3:0]}
    output reg  [15:0] O_wr_data,
    output reg         O_wr_we,

    // ---- 提交（→ dyn_subtitle）----
    output reg         O_commit_tgl,// 每提交一份新字幕翻转一次
    output reg         O_bank,      // 正在显示的 bank（与 commit_tgl 同拍更新）
    output reg  [8:0]  O_text_len   // 正在显示的字符数（0 = 无字幕）
);

//==============================================================================
//  0. 常量
//==============================================================================
localparam       P_IDLE = 1'b0;    // 解析器：空闲 / 等重启
localparam       P_RD   = 1'b1;    // 解析器：读 TXT 首扇区并解析

localparam [2:0] G_PICK = 3'd0;    // 字模：挑下一个要装的字（或提交）
localparam [2:0] G_CODE = 3'd1;    // 字模：等码表 BRAM 1 拍
localparam [2:0] G_INIT = 3'd2;    // 字模：码表数据有效 → 分类 / 初始化算址
localparam [2:0] G_ROW  = 3'd3;    // 字模：逐区累加 94
localparam [2:0] G_REQ  = 3'd4;    // 字模：向 Flash 发单
localparam [2:0] G_RCV  = 3'd5;    // 字模：收 32 字节并写 BRAM

localparam [12:0] FA_START = 13'd192;   // 提交时 fa = phase(0) + 192

assign O_fl_len = 16'd32;

//==============================================================================
//  1. 帧脉冲（vs_tgl 跨域重建）
//==============================================================================
reg vs_ff1, vs_ff2, vs_ff3;
wire frame_tick = vs_ff2 ^ vs_ff3;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        vs_ff1 <= 1'b0;
        vs_ff2 <= 1'b0;
        vs_ff3 <= 1'b0;
    end else begin
        vs_ff1 <= I_vs_tgl;
        vs_ff2 <= vs_ff1;
        vs_ff3 <= vs_ff2;
    end
end

//==============================================================================
//  2. 状态寄存器
//==============================================================================
// ---- 解析器 ----
reg        pst;
reg        restart_req;         // 有一个待处理的新 I_start
reg        req_found;           // 锁存的本轮 I_txt_found
reg [22:0] req_sec;             // 锁存的本轮 I_txt_sec
reg        parse_stop;          // 已遇到 CR/LF/0x00
reg        parse_pend;          // 已收到 GB2312 区码，等位码
reg [7:0]  parse_lead;
reg [8:0]  n_char;              // 已解析字符数（0..256）

// ---- back（新字幕）----
reg        new_rdy;             // back 码表已解析完，可以装首批
reg [8:0]  new_len;             // back 字符数
reg [6:0]  bk_n;                // back 首批数 = min(new_len, 64)
reg [6:0]  bk_i;                // back 首批已装到第几个

// ---- front（正在显示）----
reg        f_big;               // O_text_len > 64（需要懒加载 / 间隙重装）
reg [8:0]  lenx;                // O_text_len + 53（间隙窗口 / 回卷判定）
reg [7:0]  run_i;               // 懒加载游标（64 .. len-1）
reg [6:0]  gap_i;               // 间隙重装游标（0..64，bit6 = 已装完）
reg [12:0] fa;                  // phase + 192

// ---- 字模装载 ----
reg [2:0]  gst;
reg        ld_bank;             // 本次装的是哪个 bank
reg [7:0]  ld_gidx;             // 本次装的字符位置
reg [7:0]  q;                   // 串行算址：当前区码
reg [12:0] fcell;                // 串行算址：Flash 地址 / 32
reg [4:0]  b_cnt;               // Flash 字节 0..31
reg [7:0]  byte_lo;             // 偶数字节暂存

assign O_sd_addr  = {9'd0, req_sec};
assign O_fl_addr  = {6'd0, fcell, 5'd0};

//==============================================================================
//  3. 字符码表（2 bank × 256 × 16）：1 块 BRAM32K，端口 A 写（解析器）、端口 B 读
//==============================================================================
//  写口直接由解析组合逻辑驱动（不再多打一拍寄存器）。
wire [7:0]  d       = I_sd_data;
wire        d_hi    = (d >= 8'hA1) && (d != 8'hFF);              // 0xA1..0xFE
wire        d_asc   = (d >= 8'h20) && (d <= 8'h7E);
wire        d_eol   = (d == 8'h00) || (d == 8'h0D) || (d == 8'h0A);
wire        p_byte  = (pst == P_RD) && I_sd_dv && !parse_stop;
wire        p_gb    = parse_pend && d_hi;                         // 区码 + 位码
wire        codes_we    = p_byte && (p_gb || d_asc) && !n_char[8];
wire [10:0] codes_waddr = {2'b00, ~O_bank, n_char[7:0]};
wire [15:0] codes_wdata = {(p_gb ? parse_lead : 8'h00), d};

wire [10:0] code_raddr  = {2'b00, ld_bank, ld_gidx};
wire [15:0] code_rdata;

`ifdef IVERILOG_SIM
reg [15:0] code_mem [0:511];
reg [15:0] code_q;
integer ci;
initial begin
    for (ci = 0; ci < 512; ci = ci + 1) code_mem[ci] = 16'd0;
end
always @(posedge clk) if (codes_we) code_mem[codes_waddr[8:0]] <= codes_wdata;
always @(posedge clk) code_q <= code_mem[code_raddr[8:0]];
assign code_rdata = code_q;
`else
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
    .OCEAMUX("SIG"), .OCEBMUX("SIG"), .RSTAMUX("SIG"), .RSTBMUX("SIG")
) u_code_ram (
    .clka(clk), .csa(1'b1), .wea(codes_we), .addra(codes_waddr), .dia(codes_wdata),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(code_raddr),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(code_rdata)
);
`endif

//==============================================================================
//  4. 调度判定（front 优先，然后 back 首批，最后提交）
//==============================================================================
wire [8:0] fa_c     = fa[12:4];                                   // 超前相位（字符单位）
wire       in_gap   = (fa_c == lenx);                             // 文字已整段滚出的 16 帧
wire       fa_wrap  = in_gap && (fa[3:0] == 4'hF);
wire       gap_due  = f_big && in_gap && !gap_i[6];
wire       lazy_due = f_big && ({1'b0, run_i} < O_text_len) && ({1'b0, run_i} <= fa_c);
wire       bk_due   = new_rdy && (bk_i != bk_n);
wire       commit   = new_rdy && (bk_i == bk_n);

//  串行算址：码表 {qb, wb}（解析器只收 wb ∈ A1..FE 的 GB2312，或 ASCII 20..7E）
//    ASCII ：fcell = wb                  （偏移 = wb × 32）
//    GB2312：fcell = 128 + row×94 + (wb − 0xA1)   （偏移 = 0x1000 + fcell'×32）
//            row 跳过不存储的 0xAA..0xAF 区；qb 落在不存储区 ⇒ fcell = 0（空白字形）
wire [7:0] qb      = code_rdata[15:8];
wire [7:0] wb      = code_rdata[7:0];
wire       qb_ok   = ((qb >= 8'hA1) && (qb <= 8'hA9)) || ((qb >= 8'hB0) && (qb <= 8'hF7));
wire       q_skip  = (q[7:4] == 4'hA) && q[3] && (q[2] | q[1]);  // 0xAA..0xAF

//==============================================================================
//  5. 主时序
//==============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pst          <= P_IDLE;
        restart_req  <= 1'b0;
        req_found    <= 1'b0;
        req_sec      <= 23'd0;
        parse_stop   <= 1'b0;
        parse_pend   <= 1'b0;
        parse_lead   <= 8'd0;
        n_char       <= 9'd0;
        O_sd_req     <= 1'b0;
        new_rdy      <= 1'b0;
        new_len      <= 9'd0;
        bk_n         <= 7'd0;
        bk_i         <= 7'd0;
        f_big        <= 1'b0;
        lenx         <= 9'd53;
        run_i        <= 8'd64;
        gap_i        <= 7'd64;
        fa           <= FA_START;
        gst          <= G_PICK;
        ld_bank      <= 1'b0;
        ld_gidx      <= 8'd0;
        q            <= 8'd0;
        fcell         <= 13'd0;
        b_cnt        <= 5'd0;
        byte_lo      <= 8'd0;
        O_fl_req     <= 1'b0;
        O_wr_addr    <= 11'd0;
        O_wr_data    <= 16'd0;
        O_wr_we      <= 1'b0;
        O_commit_tgl <= 1'b0;
        O_bank       <= 1'b0;
        O_text_len   <= 9'd0;
    end else begin
        O_wr_we  <= 1'b0;
        O_fl_req <= 1'b0;

        //----------------------------------------------------------------------
        //  5.1 帧计数（front）：回卷时复位下一圈的游标
        //----------------------------------------------------------------------
        if (frame_tick) begin
            if (fa_wrap) begin
                fa    <= FA_START;
                run_i <= 8'd64;
                gap_i <= 7'd0;
            end else begin
                fa <= fa + 13'd1;
            end
        end

        //----------------------------------------------------------------------
        //  5.2 字模装载器
        //----------------------------------------------------------------------
        case (gst)
        G_PICK: begin
            if (gap_due) begin
                ld_bank <= O_bank;
                ld_gidx <= {2'b00, gap_i[5:0]};
                gap_i   <= gap_i + 7'd1;
                gst     <= G_CODE;
            end else if (lazy_due) begin
                ld_bank <= O_bank;
                ld_gidx <= run_i;
                run_i   <= run_i + 8'd1;
                gst     <= G_CODE;
            end else if (bk_due) begin
                ld_bank <= ~O_bank;
                ld_gidx <= {2'b00, bk_i[5:0]};
                bk_i    <= bk_i + 7'd1;
                gst     <= G_CODE;
            end else if (commit) begin
                // 首批装完 ⇒ 提交：back 变 front，旧字幕到此为止
                O_bank       <= ~O_bank;
                O_text_len   <= new_len;
                O_commit_tgl <= ~O_commit_tgl;
                f_big        <= (new_len > 9'd64);
                lenx         <= new_len + 9'd53;
                run_i        <= 8'd64;
                gap_i        <= 7'd0;
                fa           <= FA_START;
                new_rdy      <= 1'b0;
            end
        end

        G_CODE: gst <= G_INIT;                     // 码表 BRAM 采样地址

        G_INIT: begin
            q <= 8'hA1;
            if (qb == 8'h00) begin
                fcell <= {5'd0, wb};                // ASCII
                gst  <= G_REQ;
            end else if (qb_ok) begin
                fcell <= {5'd0, wb + 8'hDF};        // wb − 0xA1 + 128（0x1000/32）
                gst  <= G_ROW;
            end else begin
                fcell <= 13'd0;                     // 不存储的区 ⇒ 空白字形
                gst  <= G_REQ;
            end
        end

        G_ROW: begin
            if (q == qb) begin
                gst <= G_REQ;
            end else begin
                if (!q_skip) fcell <= fcell + 13'd94;
                q <= q + 8'd1;
            end
        end

        G_REQ: begin
            if (I_fl_rdy) begin
                O_fl_req <= 1'b1;
                b_cnt    <= 5'd0;
                gst      <= G_RCV;
            end
        end

        G_RCV: begin
            if (I_fl_valid) begin
                if (!b_cnt[0]) begin
                    byte_lo <= I_fl_data;
                end else begin
                    O_wr_we   <= 1'b1;
                    O_wr_addr <= {ld_bank, ld_gidx[5:0], b_cnt[4:1]};
                    O_wr_data <= {byte_lo, I_fl_data};
                end
                b_cnt <= b_cnt + 5'd1;
            end
            if (I_fl_done) gst <= G_PICK;
        end

        default: gst <= G_PICK;
        endcase

        //----------------------------------------------------------------------
        //  5.3 TXT 解析器（放在装载器之后：重启时对 new_rdy / bk_i 的清零优先）
        //----------------------------------------------------------------------
        case (pst)
        P_IDLE: begin
            if (restart_req) begin
                restart_req <= 1'b0;
                parse_stop  <= 1'b0;
                parse_pend  <= 1'b0;
                n_char      <= 9'd0;
                new_rdy     <= 1'b0;           // 作废 back（front 不动）
                bk_i        <= 7'd0;
                if (req_found) begin
                    O_sd_req <= 1'b1;
                    pst      <= P_RD;
                end else begin
                    // 没有 TXT ⇒ 空字幕，立即可提交（提交后旧字幕消失）
                    new_len <= 9'd0;
                    bk_n    <= 7'd0;
                    new_rdy <= 1'b1;
                end
            end
        end

        P_RD: begin
            if (p_byte) begin
                if (parse_pend) begin
                    parse_pend <= 1'b0;        // 只等一个位码字节
                end else if (d_eol) begin
                    parse_stop <= 1'b1;
                end else if (d_hi) begin
                    parse_pend <= 1'b1;
                    parse_lead <= d;
                end
            end
            if (codes_we) n_char <= n_char + 9'd1;

            if (I_sd_end) begin
                O_sd_req <= 1'b0;
                pst      <= P_IDLE;
                if (!restart_req) begin
                    new_len <= n_char;
                    bk_n    <= (|n_char[8:6]) ? 7'd64 : n_char[6:0];
                    new_rdy <= 1'b1;
                end
            end
        end
        endcase

        //----------------------------------------------------------------------
        //  5.4 启动（最后写：与 P_IDLE 的清零同拍时，新的启动不会丢）
        //----------------------------------------------------------------------
        if (I_start) begin
            restart_req <= 1'b1;
            req_found   <= I_txt_found;
            req_sec     <= I_txt_sec;
        end
    end
end

endmodule
