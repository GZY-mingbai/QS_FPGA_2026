//==============================================================================
//  sd_bus_arbiter4.v —— SD 卡扇区级总线仲裁（4 客户）
//==============================================================================
//
//  【为什么要有它】
//  SD 卡只有一组 SPI 引脚，但四个模块都要读卡：
//      0. 图像装载（bmp_loader，切图/文件页缩略图）
//      1. 音频播放（sd_audio_reader，WAV 预取）
//      2. FAT 目录扫描（fat_dir_lister，翻页时扫根目录）
//      3. 文本读取（后续功能，现在留口）
//
//  【为什么粒度是"一个扇区"】
//  sd_card_sec_read_write 的语义是"拉高 req → 吐 512 字节 → 单拍 end →
//  回 S_WAIT_READ_WRITE 重新锁地址"。end 之后控制器里没有任何半截状态，
//  所以**在 end 之后换人最安全**，也不需要改控制器一行。
//
//  【★ A_GAP 为什么不能省 ★】
//  控制器在 S_READ_END 之后回到 S_WAIT_READ_WRITE，**下一拍重新锁存
//  sd_sec_read_addr**。如果那一拍 req 还是高、而地址已经换成下一个客户，
//  控制器就会拿错地址再读一个扇区 —— **静默，数据全错，不报错**。
//  所以换人时必须撤 req 一拍。这一拍 20ns，相对 360us 的扇区时间可忽略。
//
//  【优先级】
//      1. IMG            永远最高（有 1.34s 装载看门狗）
//      2. AUD urgent     FIFO 低水位，见底就爆音
//      3. FAT            用户翻页，有交互延迟感
//      4. TXT            打开文档，一次性动作
//      5. AUD 非 urgent  FIFO 还有水，能等
//
//  【FAT 配额：防饿死，也防拖垮图像】
//  FAT 扫 125 个扇区要 45ms，如果它连续霸占总线，图像装载会明显变慢。
//  所以 FAT 连续占满 FAT_QUOTA 个扇区后，强制让一拍给 IMG/AUD。
//
//  【音频限速：为什么放在仲裁器】
//  这是"公平性"的职责，与音频内部状态无关。音频再出毛病，图像也不会被拖垮。
//
//  【时钟域】全部在 sd_card_clk（50MHz），纯组合 mux，不需要同步器。
//
//==============================================================================

module sd_bus_arbiter4 #(
    //--------------------------------------------------------------------------
    //  超时：一个扇区正常 ~360us，这里给 20ms（50MHz × 1e6），约 55 倍余量
    //--------------------------------------------------------------------------
    parameter integer TIMEOUT_CYC = 1_000_000,
    //--------------------------------------------------------------------------
    //  音频限速：每 AUD_GAP_CYC 最多拿 1 个扇区。0 = 关闭
    //    48000 字/秒 = 每 2.67ms 一个扇区（128 字），给 1.33ms 是 2 倍余量
    //--------------------------------------------------------------------------
    parameter integer AUD_GAP_CYC = 66_000,
    //--------------------------------------------------------------------------
    //  FAT 配额：连续占满这么多个扇区后让一拍
    //--------------------------------------------------------------------------
    parameter integer FAT_QUOTA   = 8,
    //--------------------------------------------------------------------------
    //  文本客户开关：0 = 不例化（综合器会把这个口整个裁掉）
    //--------------------------------------------------------------------------
    parameter         ENA_TXT     = 1
)(
    input  wire        I_clk,               // sd_card_clk 50MHz
    input  wire        I_rst,               // 高有效
    input  wire        I_sd_init_done,      // 卡初始化完成前不授权

    //------------------------------------------------------------------
    //  客户 0：图像装载
    //------------------------------------------------------------------
    input  wire        I_img_req,
    input  wire [31:0] I_img_addr,
    output wire [7:0]  O_img_data,
    output wire        O_img_dv,
    output wire        O_img_end,

    //------------------------------------------------------------------
    //  客户 1：音频
    //------------------------------------------------------------------
    input  wire        I_aud_req,
    input  wire [31:0] I_aud_addr,
    input  wire        I_aud_urgent,        // FIFO 低于低水位 → 抢占
    output wire [7:0]  O_aud_data,
    output wire        O_aud_dv,
    output wire        O_aud_end,

    //------------------------------------------------------------------
    //  客户 2：FAT 目录扫描
    //------------------------------------------------------------------
    input  wire        I_fat_req,
    input  wire [31:0] I_fat_addr,
    output wire [7:0]  O_fat_data,
    output wire        O_fat_dv,
    output wire        O_fat_end,

    //------------------------------------------------------------------
    //  客户 3：文本读取（预留）
    //------------------------------------------------------------------
    input  wire        I_txt_req,
    input  wire [31:0] I_txt_addr,
    output wire [7:0]  O_txt_data,
    output wire        O_txt_dv,
    output wire        O_txt_end,

    //------------------------------------------------------------------
    //  到 sd_card_top
    //------------------------------------------------------------------
    output reg         O_sd_sec_read,
    output reg  [31:0] O_sd_sec_addr,
    input  wire [7:0]  I_sd_sec_data,
    input  wire        I_sd_sec_dv,
    input  wire        I_sd_sec_end,

    //------------------------------------------------------------------
    //  调试
    //------------------------------------------------------------------
    output reg         O_timeout,           // 粘滞：发生过一次等不到 end
    output wire [3:0]  O_owner              // one-hot：bit0=IMG 1=AUD 2=FAT 3=TXT
);

//==============================================================================
//  状态编码：one-hot
//==============================================================================
//  用 one-hot 而不是二进制，省掉"二进制 → one-hot"的译码器，
//  而且 dv/end 分发可以直接 `& st[0]`，不需要比较器。
//==============================================================================
localparam [3:0] S_IDLE = 4'b0000;

reg  [3:0]  st;
reg         gap;                // A_GAP：撤 req 一拍
reg  [31:0] addr_lat;           // 授权瞬间锁存的地址

//==============================================================================
//  客户请求（TXT 可裁剪）
//==============================================================================
wire img_req = I_img_req;
wire aud_req = I_aud_req;
wire fat_req = I_fat_req;
wire txt_req = ENA_TXT ? I_txt_req : 1'b0;

//==============================================================================
//  音频限速
//==============================================================================
reg  [16:0] aud_gap_cnt;
reg         aud_ok;                             // 1 = 允许音频再拿一个扇区

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        aud_gap_cnt <= 17'd0;
        aud_ok      <= 1'b1;
    end else if (!aud_ok) begin
        if (aud_gap_cnt == AUD_GAP_CYC[16:0]) begin   // ★ [AREA] 0 起逐一递增、到点清零 ⇒ == 与 >= 等价
            aud_ok      <= 1'b1;
            aud_gap_cnt <= 17'd0;
        end else
            aud_gap_cnt <= aud_gap_cnt + 17'd1;
    end else if (st[1]) begin
        aud_ok      <= 1'b0;                    // 刚给过一个，开始计时
        aud_gap_cnt <= 17'd0;
    end
end

//==============================================================================
//  FAT 配额
//==============================================================================
reg  [3:0] fat_cnt;
wire fat_quota_hit = (fat_cnt >= FAT_QUOTA[3:0]);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)
        fat_cnt <= 4'd0;
    else if (st[2])
        fat_cnt <= fat_cnt + 4'd1;
    else if (st != S_IDLE)
        fat_cnt <= 4'd0;                        // 别人在传 → 清零
end

//==============================================================================
//  优先级（纯组合）
//==============================================================================
//  ★ 配额只在"有人抢"时才生效 ★
//  如果无条件用 quota_hit 挡 FAT，那么"只有 FAT 在请求"时 grant 恒为 0，
//  fat_cnt 在 IDLE 态又不递增也不清零 ⇒ **永久死锁**。
//  所以只有 IMG/AUD 在等的时候才让 FAT 停一拍。
//==============================================================================
wire fat_block  = fat_quota_hit && (img_req || aud_req);

wire g_img      = img_req;
wire g_aud_urg  = !g_img && aud_req && aud_ok &&  I_aud_urgent;
wire g_fat      = !g_img && !g_aud_urg && fat_req && !fat_block;
wire g_txt      = !g_img && !g_aud_urg && !g_fat && txt_req;
wire g_aud_norm = !g_img && !g_aud_urg && !g_fat && !g_txt && aud_req && aud_ok;

wire [3:0] grant = {g_txt, g_fat, (g_aud_urg | g_aud_norm), g_img};

//==============================================================================
//  地址：one-hot 或门合并
//==============================================================================
//  因为 grant 是 one-hot（最多一位为 1），所以按位与再或就是"选一个"。
//  比 4:1 mux 省：每 bit 4 个 AND + 3 个 OR 打包进 2 个 LUT4；
//  4:1 mux 要 3 级 LUT2。
//==============================================================================
wire [31:0] addr_oh = ({32{grant[0]}} & I_img_addr)
                    | ({32{grant[1]}} & I_aud_addr)
                    | ({32{grant[2]}} & I_fat_addr)
                    | ({32{grant[3]}} & I_txt_addr);

//==============================================================================
//  超时
//==============================================================================
reg  [19:0] to_cnt;
wire        to_hit = (to_cnt == TIMEOUT_CYC[19:0]);   // ★ [AREA] 0 起逐一递增、到点清零 ⇒ == 与 >= 等价

//==============================================================================
//  主状态机
//==============================================================================
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        st            <= S_IDLE;
        gap           <= 1'b0;
        addr_lat      <= 32'd0;
        O_sd_sec_read <= 1'b0;
        O_sd_sec_addr <= 32'd0;
        to_cnt        <= 20'd0;
        O_timeout     <= 1'b0;
    end else begin
        if (gap) begin
            // A_GAP：req 已经拉低一拍，回空闲
            gap <= 1'b0;
            st  <= S_IDLE;
        end else if (st == S_IDLE) begin
            to_cnt <= 20'd0;
            if (I_sd_init_done && (grant != 4'd0)) begin
                addr_lat      <= addr_oh;
                O_sd_sec_addr <= addr_oh;
                O_sd_sec_read <= 1'b1;
                st            <= grant;
            end
        end else begin
            // 已授权，等 end 或超时
            if (I_sd_sec_end || to_hit) begin
                O_sd_sec_read <= 1'b0;
                if (to_hit)
                    O_timeout <= 1'b1;
                to_cnt <= 20'd0;
                gap    <= 1'b1;
            end else begin
                to_cnt <= to_cnt + 20'd1;
            end
        end
    end
end

//==============================================================================
//  数据 / 结束信号分发（纯组合，同域）
//==============================================================================
//  超时那一拍也要给客户一个 end，否则它会一直等在"等 end"的状态里。
//==============================================================================
assign O_img_data = I_sd_sec_data;
assign O_aud_data = I_sd_sec_data;
assign O_fat_data = I_sd_sec_data;
assign O_txt_data = I_sd_sec_data;

assign O_img_dv   = I_sd_sec_dv && st[0];
assign O_aud_dv   = I_sd_sec_dv && st[1];
assign O_fat_dv   = I_sd_sec_dv && st[2];
assign O_txt_dv   = I_sd_sec_dv && st[3];

assign O_img_end  = st[0] && (I_sd_sec_end || to_hit);
assign O_aud_end  = st[1] && (I_sd_sec_end || to_hit);
assign O_fat_end  = st[2] && (I_sd_sec_end || to_hit);
assign O_txt_end  = st[3] && (I_sd_sec_end || to_hit);

assign O_owner    = st;

endmodule
