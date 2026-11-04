//==============================================================================
//  sd_sector_file_model.v —— 扇区级"电子假卡"（★ 只用于仿真 ★）
//==============================================================================
//
//  【为什么不用 sd_card_model.v】
//  那个模型在 SPI 位级响应命令：一张 640×480 BMP 有 900KB，位级仿真要跑几天。
//  本模型直接接在 sd_bus_arbiter4 的**控制器侧**（O_sd_sec_read / I_sd_sec_*），
//  每个扇区只做一次 $fseek + $fread，把 512 字节按真卡的节拍吐出来。
//
//  【和真卡的差别（只有两处）】
//    · 没有 CMD0/CMD8/ACMD41 初始化流程 —— init_done 直接给（SPI 初始化另有
//      tb_sd_card 用 sd_card_model.v 覆盖）
//    · 读延迟是固定值（参数 LAT_CYC，默认 360µs @50MHz），真卡在 200~500µs 之间抖
//
//  【用法】
//      sd_sector_file_model #(.INIT_FILE("sim_data/fat32_10img.img")) u_card (
//          .clk(sd_clk), .rst(rst),
//          .O_init_done(sd_init_done),
//          .I_sec_read(arb_sd_read), .I_sec_addr(arb_sd_addr),
//          .O_sec_data(arb_sd_data), .O_sec_dv(arb_sd_dv), .O_sec_end(arb_sd_end),
//          .O_rd_pulse(rd_pulse), .O_rd_lba(rd_lba));
//
//  【注意】读越界返回 0xFF（真卡空白区就是 0xFF），不会让仿真读到 x。
//==============================================================================

module sd_sector_file_model #(
    parameter         INIT_FILE = "",
    parameter integer LAT_CYC   = 18000,        // 360µs @50MHz
    parameter integer INIT_CYC  = 200,
    parameter [31:0]  MAX_LBA   = 32'd131071,
    // ★ 一次性预载多少字节（0 = 不预载，退回逐扇区 $fread）
    //   逐扇区 $fread 在 Windows 上约 60ms/次（sys 时间），一张 640×480 要 2 分钟。
    //   预载 12MB 约 3s，之后访问全是内存读 ⇒ 仿真从"分钟/张"降到"秒/张"。
    parameter integer PRELOAD   = 6*1024*1024
)(
    input  wire        clk,
    input  wire        rst,

    output reg         O_init_done,

    input  wire        I_sec_read,
    input  wire [31:0] I_sec_addr,
    output reg  [7:0]  O_sec_data,
    output reg         O_sec_dv,
    output reg         O_sec_end,

    // 观测：每读一个扇区给一拍脉冲 + 扇区号（TB 用它统计"每张图被读了多少扇区"）
    output reg         O_rd_pulse,
    output reg  [31:0] O_rd_lba
);

localparam [1:0] C_IDLE = 2'd0, C_WAIT = 2'd1, C_DATA = 2'd2, C_END = 2'd3;

reg [1:0]  st;
reg [31:0] lba;
integer    fd;
integer    lat;
integer    cnt;
integer    init_cnt;
reg  [7:0] sbuf [0:511];
integer    nread;
integer    i;
reg  [7:0] pmem [0:PRELOAD-1];      // 预载窗口（0..PRELOAD-1 字节）

// 打开镜像（整个仿真只开一次）
initial begin
    fd = 0;
    nread = 0;
    if (INIT_FILE != "") begin
        fd = $fopen(INIT_FILE, "rb");
        if (fd == 0)
            $display("  [sd_sector] !! 打不开卡镜像 %s", INIT_FILE);
        else begin
            nread = $fread(pmem, fd);
            $fclose(fd);
            $display("  [sd_sector] 卡镜像已预载 %0d 字节：%s", nread, INIT_FILE);
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st         <= C_IDLE;
        lba        <= 32'd0;
        lat        <= 0;
        cnt        <= 0;
        init_cnt   <= 0;
        O_init_done<= 1'b0;
        O_sec_data <= 8'hFF;
        O_sec_dv   <= 1'b0;
        O_sec_end  <= 1'b0;
        O_rd_pulse <= 1'b0;
        O_rd_lba   <= 32'd0;
    end else begin
        O_rd_pulse <= 1'b0;

        if (init_cnt < INIT_CYC) begin
            init_cnt    <= init_cnt + 1;
            O_init_done <= 1'b0;
        end else
            O_init_done <= 1'b1;

        case (st)
        C_IDLE: begin
            O_sec_dv  <= 1'b0;
            O_sec_end <= 1'b0;
            if (I_sec_read && O_init_done) begin
                lba <= (I_sec_addr > MAX_LBA) ? MAX_LBA : I_sec_addr;
                lat <= LAT_CYC;
                st  <= C_WAIT;
            end
        end

        C_WAIT: begin
            if (lat > 0) lat <= lat - 1;
            else begin
                // 从预载窗口拷 512 字节（越界一律 0xFF，和真卡空白区一致）
                for (i = 0; i < 512; i = i + 1) begin
                    if ((lba * 512 + i) < nread) sbuf[i] = pmem[lba * 512 + i];
                    else                         sbuf[i] = 8'hFF;
                end
                O_rd_pulse <= 1'b1;
                O_rd_lba   <= lba;
                cnt        <= 0;
                st         <= C_DATA;
            end
        end

        C_DATA: begin
            O_sec_dv   <= 1'b1;
            O_sec_data <= sbuf[cnt];
            if (cnt == 511) begin
                // 第 512 个字节也要 dv=1（下一拍进 C_END 才撤）
                O_sec_end <= 1'b1;
                st        <= C_END;
            end else
                cnt <= cnt + 1;
        end

        C_END: begin
            O_sec_dv  <= 1'b0;
            O_sec_end <= 1'b0;
            // 等请求撤掉再接下一条（仲裁器在见到 end 的下一拍撤）
            if (!I_sec_read) st <= C_IDLE;
        end

        default: st <= C_IDLE;
        endcase
    end
end

endmodule
