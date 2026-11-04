//==============================================================================
//  quad_loader.v —— 四宫格装载调度（把 4 项列表依次灌进 4 个 SDRAM 槽）
//==============================================================================
//
//  【它干什么】
//  lister 给出 4 项文件列表后，把第 0..3 项依次交给 bmp_loader，
//  每项装进一个独立的 SDRAM 槽，装完 4 个（或列表用完）就报完成。
//
//  【为什么不需要任务队列】
//  列表本身就是待办清单：一个 2 位指针 slot 往下走就行。
//
//  【为什么串行】
//  SD 卡只有一条总线，4 张图必须一张一张来。每张 160×120×3B ≈ 57.6KB
//  ≈ 113 扇区 ≈ 41ms，4 张约 164ms。
//
//  【槽基址】
//  base = SLOT_BASE + (slot << SLOT_SHIFT)，SLOT_SHIFT=14 即每槽 16384 字。
//  160×120 565 实际只用 9600 字（2 像素/字），浪费 41% 换来零成本地址计算。
//
//  【失败处理】
//  某一项 bmp_loader 报 err（头不对 / 卡读失败）→ 跳过它继续下一项，
//  那个槽保持原样（显示时会看到上一页的残留，所以外层应该在开始前清一次
//  或者接受短暂残影）。这里只保证**不会卡死**。
//
//==============================================================================

module quad_loader #(
    parameter [20:0] SLOT_BASE   = 21'd0,
    // ★ 槽间距（字）。用 40960 而不是 65536：
    //   ps_gen_image 的 quad_map 把槽号折进了 sy（sy = slot*256 + 2·ly+1），
    //   于是地址 = sy*160 + x = slot*40960 + ... ⇒ 槽间距必须是 160*256 = 40960。
    //   好处：请求地址上不需要再叠一个加法器（那个加法器正好落在最差路径的锥里）。
    parameter integer SLOT_STEP  = 40960,
    parameter integer N_ITEMS    = 4
)(
    input  wire        clk,
    input  wire        rst,

    //------------------------------------------------------------------
    //  启动
    //------------------------------------------------------------------
    input  wire        I_start,             // 单拍：开始装这一页
    input  wire [2:0]  I_valid_cnt,         // 本页有几项有效（lister 给）
    input  wire [3:0]  I_slot_valid,        // 哪几项有效
    output reg         O_busy,
    output reg         O_done,              // 单拍：全部装完
    output reg  [2:0]  O_loaded,            // 实际装成功几张
    // ★ 2026-11-04：两遍装载 + 可中途放弃
    //   第 0 遍：每格先写占位（BMP/其它 = 纯黑，WAV/TXT = 占位图），4 格 ≈ 3ms，
    //            写完置 O_filled ⇒ 上层马上可以显示四宫格（没装好的格子是黑的，不露陈货）；
    //   第 1 遍：再逐张读卡装 BMP（慢，每张几百 ms）。
    //   I_abort：翻页 / 离开页面时放弃本页（当前这一张由 bmp_loader 自己尽快收尾）。
    input  wire        I_abort,
    input  wire        I_hold,              // 1 = 顶层借用列表读口这一拍，S_NEXT 别读
    output reg  [3:0]  O_bmp,               // 本页哪几格是 BMP（第 0 遍填好，给"打开"算图序号）
    output reg         O_filled,            // 1 = 第 0 遍完成

    //------------------------------------------------------------------
    //  列表读口（接 fat_dir_lister）
    //------------------------------------------------------------------
    output reg  [1:0]  O_rd_slot,
    input  wire [31:0] I_file_off,
    input  wire [31:0] I_file_len,
    input  wire [3:0]  I_file_type,     // 0=其他 1=BMP 2=WAV 3=TXT（跟 O_rd_slot 同步）

    //------------------------------------------------------------------
    //  占位图生成器（wav / txt 走这里，不读卡）
    //------------------------------------------------------------------
    output reg         O_ph_start,
    output reg  [1:0]  O_ph_kind,       // 2=WAV 3=TXT
    output reg  [20:0] O_ph_base,
    input  wire        I_ph_busy,
    input  wire        I_ph_done,

    //------------------------------------------------------------------
    //  装载器控制（接 bmp_loader）
    //------------------------------------------------------------------
    output reg         O_ld_start,
    output reg  [31:0] O_ld_off,
    output reg  [31:0] O_ld_len,
    output reg  [20:0] O_ld_base,
    input  wire        I_ld_ready,
    input  wire        I_ld_busy,
    input  wire        I_ld_done,
    input  wire        I_ld_err
);

localparam [2:0] S_IDLE = 3'd0;
localparam [2:0] S_ISSUE = 3'd1;   // 发 start
localparam [2:0] S_WAIT  = 3'd2;   // 等 done/err
localparam [2:0] S_NEXT  = 3'd3;   // 决定下一项
localparam [2:0] S_PH    = 3'd4;   // 占位图生成（wav/txt）

reg [2:0] st;
reg [2:0] slot;          // ★ 3 位：2 位会在 3+1 时回绕成 0，永远到不了结束
reg       pass;          // 0 = 写占位，1 = 装 BMP
reg       ab;            // 已收到放弃请求（下一次回到 S_NEXT 就收工）
wire      is_bmp = (I_file_type == 4'd1);
wire      is_ph  = (I_file_type == 4'd2) || (I_file_type == 4'd3);
// slot*40960 = slot<<15 + slot<<13（2 位变量，两个移位相加，约 10 LUT）
wire [20:0] slot_base = SLOT_BASE + (({{18{1'b0}}, slot} << 15) + ({{18{1'b0}}, slot} << 13));

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st         <= S_IDLE;
        slot       <= 2'd0;
        O_rd_slot  <= 2'd0;
        O_ld_start <= 1'b0;
        O_ld_off   <= 32'd0;
        O_ld_len   <= 32'd0;
        O_ld_base  <= 21'd0;
        O_busy     <= 1'b0;
        O_done     <= 1'b0;
        O_loaded   <= 3'd0;
        O_ph_start <= 1'b0;
        O_ph_kind  <= 2'd2;
        O_ph_base  <= 21'd0;
        pass       <= 1'b0;
        ab         <= 1'b0;
        O_bmp      <= 4'd0;
        O_filled   <= 1'b0;
    end else begin
        if (I_start)      ab <= 1'b0;
        else if (I_abort) ab <= 1'b1;
        O_ld_start <= 1'b0;
        O_done     <= 1'b0;
        O_ph_start <= 1'b0;

        case (st)
        //----------------------------------------------------------------------
        S_IDLE: begin
            O_busy <= 1'b0;
            if (I_start) begin
                slot     <= 2'd0;
                O_rd_slot<= 2'd0;     // 先选第 0 项，下一拍 S_NEXT 再锁数据
                O_loaded <= 3'd0;
                O_busy   <= 1'b1;
                pass     <= 1'b0;
                O_bmp    <= 4'd0;
                O_filled <= 1'b0;
                st       <= S_NEXT;
            end
        end

        //----------------------------------------------------------------------
        //  S_NEXT：跳过无效项，找到下一个要装的
        //----------------------------------------------------------------------
        S_NEXT: begin
            if (I_hold) begin
                // 顶层这一拍借走了列表读口（读光标那格），数据不是本格的，等一拍
            end else if (ab || slot >= N_ITEMS || slot >= I_valid_cnt[2:0] || !I_slot_valid[slot]) begin
                if (!pass && !ab) begin
                    // 第 0 遍完：从第 0 格开始第 1 遍
                    pass      <= 1'b1;
                    O_filled  <= 1'b1;
                    slot      <= 3'd0;
                    O_rd_slot <= 2'd0;
                end else begin
                    O_busy <= 1'b0;
                    O_done <= 1'b1;
                    st     <= S_IDLE;
                end
            end else if (!pass) begin
                // 第 0 遍：WAV/TXT 画占位图（算"装成功"），其余先刷成纯黑
                O_bmp[slot[1:0]] <= is_bmp;
                O_ph_kind  <= is_ph ? I_file_type[1:0] : 2'd0;
                O_ph_base  <= slot_base;
                O_ph_start <= 1'b1;
                st         <= S_PH;
            end else if (is_bmp) begin
                // 注意：O_rd_slot 已在上一拍设好，这里锁到的才是本项数据
                O_ld_off  <= I_file_off;
                O_ld_len  <= I_file_len;
                O_ld_base <= slot_base;
                st        <= S_ISSUE;
            end else begin
                slot      <= slot + 3'd1;
                O_rd_slot <= slot[1:0] + 2'd1;
            end
        end

        //----------------------------------------------------------------------
        //  S_ISSUE：bmp_loader ready 才发 start
        //----------------------------------------------------------------------
        S_ISSUE: begin
            if (ab) begin
                st <= S_NEXT;                    // 还没发：直接收工
            end else if (I_ld_ready && !I_ld_busy) begin
                O_ld_start <= 1'b1;
                st         <= S_WAIT;
            end
        end

        //----------------------------------------------------------------------
        //  S_WAIT：等 done / err
        //----------------------------------------------------------------------
        S_WAIT: begin
            // ★ 2026-11-04：bmp_loader 头校验失败走 L_ERR —— **只发 err、不发 done**。
            //   原来只等 done（注释说"done 也会跟着来"，实际不来）⇒ 坏 BMP 卡死整页。
            if (I_ld_done || I_ld_err) begin
                if (!I_ld_err) O_loaded <= O_loaded + 3'd1;
                slot      <= slot + 2'd1;
                O_rd_slot <= slot[1:0] + 2'd1;   // 提前选下一项
                st        <= S_NEXT;
            end
        end

        //----------------------------------------------------------------------
        //  S_PH：等占位图写完（38400 字 ≈ 768µs，锁一帧都不够，仍在淡入前）
        //----------------------------------------------------------------------
        //  ★ 2026-11-03：完成判据改成"busy 已落下"（不再分 done / 兜底两条）。
        //    ph_writer 搬到了 100MHz（写口和 SDRAM 调度器同域），它的 O_done 只有
        //    一个 100MHz 拍，本模块在 50MHz 下可能采不到 ⇒ 原来走"兜底"分支、
        //    O_loaded 不 +1 ⇒ 整页都是 wav/txt 时 loaded=0 ⇒ quad_ready=0 ⇒ 占位图不显示。
        //    busy 高 38400+ 拍，跨域采样不会漏；ph_writer 的 kind 只有 2/3 两种，
        //    原来"kind 非法"那条兜底本来就走不到。I_ph_done 保留端口，不再使用。
        S_PH: begin
            if (!I_ph_busy && !O_ph_start) begin
                if (O_ph_kind[1]) O_loaded <= O_loaded + 3'd1;   // 黑底不算"装成功"
                slot      <= slot + 2'd1;
                O_rd_slot <= slot[1:0] + 2'd1;
                st        <= S_NEXT;
            end
        end

        default: st <= S_IDLE;
        endcase
    end
end

endmodule
