`timescale 1ns/1ps
//==============================================================================
//  tb_audio_prefill_deadlock.v —— 【死锁回归】预灌期间必须始终抢占总线
//==============================================================================
//
//  【要复现的那个 bug（2026-10-27 上板无声）】
//  播放器要"FIFO 里有 PREFILL=512 个字"才肯开播；开播之前没人读 FIFO，
//  水位只涨不跌。板上开机时图像装载器一直抢总线，音频只抢到 3 个扇区
//  （364 字）就被"图像优先"切走；此时 364 > URGENT_LEVEL(256)，
//  urgent 掉 0 ⇒ 再也抢不到总线 ⇒ 水位永远停在 364 ⇒ 永远到不了 512
//  ⇒ 永远不开播 ⇒ 永久静音，SW2 怎么拨都没反应。
//
//  【本用例】把条件开到最恶劣：
//    · play 从上电就是 1（板上 SW2 常拨在播放位）
//    · 图像客户从 init_done 起**一直**请求，永不让总线空闲
//    · FIFO 只有 1024 深，预灌门槛 512
//  正确实现必须在几十毫秒内完成预灌并开始出声。
//
//  【跑法】（工程根目录）
//    iverilog -o tb.vvp src/user_source/hdl_source/audio/tb_audio_prefill_deadlock.v \
//        src/user_source/hdl_source/audio/audio_wav_path.v \
//        src/user_source/hdl_source/audio/sd_audio_reader.v \
//        src/user_source/hdl_source/audio/wav_player.v \
//        src/user_source/hdl_source/SD/sd_bus_arbiter.v \
//        src/user_source/hdl_source/APP/img_afifo.v && vvp tb.vvp
//==============================================================================
module tb_audio_prefill_deadlock;

localparam BASE  = 100;
localparam SKIP  = 44;
localparam TOTAL = 200000;          // 够长，用例期间不会回卷

//------------------------------------------------------------------------------
//  时钟
//------------------------------------------------------------------------------
reg clk = 0, clk_v = 0;
always #10 clk   = ~clk;            // 50MHz（SD 侧）
always #20 clk_v = ~clk_v;          // 25MHz（音频输出侧）

reg rst = 1;
reg play = 1;                       // ★ 上电即播放

//------------------------------------------------------------------------------
//  假卡（与 tb_audio_wav_path 同款：第 S 扇区第 i 字节 = (S + i) & 0xFF）
//------------------------------------------------------------------------------
wire        sd_read;
wire [31:0] sd_addr;
wire [7:0]  sd_data;
wire        sd_dv, sd_end;

fake_sd_b #(.LAT(3)) u_card (
    .clk(clk), .rst(rst), .read(sd_read), .addr(sd_addr),
    .dout(sd_data), .dv(sd_dv), .endp(sd_end),
    .served_addr(), .served()
);

//------------------------------------------------------------------------------
//  仲裁器：图像客户从 init_done 起一直请求（模拟持续装载）
//------------------------------------------------------------------------------
reg         img_req = 0;
reg  [31:0] img_addr = 32'd0;
wire        aud_req, aud_urgent, aud_dv, aud_end;
wire [31:0] aud_addr;
wire [7:0]  aud_data;
wire        img_end_w;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        img_req <= 1'b0; img_addr <= 32'd0;
    end else begin
        img_req <= 1'b1;                       // ★ 永不撤销
        if (img_end_w) img_addr <= img_addr + 32'd1;
    end
end

sd_bus_arbiter u_arb (
    .I_clk(clk), .I_rst(rst), .I_sd_init_done(1'b1),
    .I_img_req(img_req), .I_img_addr(img_addr),
    .O_img_data(), .O_img_dv(), .O_img_end(img_end_w),
    .I_aud_req(aud_req), .I_aud_addr(aud_addr), .I_aud_urgent(aud_urgent),
    .O_aud_data(aud_data), .O_aud_dv(aud_dv), .O_aud_end(aud_end),
    .O_sd_sec_read(sd_read), .O_sd_sec_addr(sd_addr),
    .I_sd_sec_data(sd_data), .I_sd_sec_dv(sd_dv), .I_sd_sec_end(sd_end),
    .O_timeout(), .O_owner()
);

//------------------------------------------------------------------------------
//  被测通道
//------------------------------------------------------------------------------
wire        avalid, aunder, adrop, aplaying;
wire [23:0] aleft, aright;
wire [10:0] alevel;

audio_wav_path #(
    .BASE_SECTOR(BASE), .SKIP_BYTES(SKIP), .TOTAL_WORDS(TOTAL),
    .FIFO_AW(10), .PREFILL(512), .REQ_LEVEL(768), .URGENT_LEVEL(256)
) u_path (
    .I_clk_sd(clk), .I_rst_sd(rst),
    .I_clk_v(clk_v), .I_rst_v(rst),
    .I_play_async(play),
    .O_sd_req(aud_req), .O_sd_addr(aud_addr), .O_sd_urgent(aud_urgent),
    .I_sd_data(aud_data), .I_sd_dv(aud_dv), .I_sd_end(aud_end),
    .O_audio_valid(avalid), .O_audio_left(aleft), .O_audio_right(aright),
    .O_underrun(aunder), .O_drop(adrop), .O_playing(aplaying),
    .O_fifo_level(alevel)
);

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
integer checks = 0, fails = 0;
task chk(input [8*40-1:0] name, input [63:0] got, input [63:0] exp);
    begin
        checks = checks + 1;
        if (got !== exp) begin
            fails = fails + 1;
            $display("  FAIL  %0s : got %0d exp %0d", name, got, exp);
        end else
            $display("  ok    %0s = %0d", name, got);
    end
endtask

function [7:0] byte_at(input integer gi);
    integer sc, of;
    begin
        sc = BASE + gi / 512;
        of = gi % 512;
        byte_at = (sc[7:0] + of[7:0]) & 8'hFF;
    end
endfunction

integer smp = 0, errs = 0, t_start = 0;
reg [23:0] el, er;
reg [7:0]  b0, b1, b2, b3;

always @(posedge clk_v) begin
    if (avalid && aplaying) begin
        // 假卡的图案是"每个扇区各自从 (S+i) 起算"，跨过 512 字节边界时
        // 基准扇区号要 +1，不能当成连续字节流。
        b0 = byte_at(SKIP + 4*smp + 0);
        b1 = byte_at(SKIP + 4*smp + 1);
        b2 = byte_at(SKIP + 4*smp + 2);
        b3 = byte_at(SKIP + 4*smp + 3);
        el = {b1, b0, 8'h00};        // 小端：L16 = b1<<8|b0
        er = {b3, b2, 8'h00};
        if (smp < 200 && (aleft !== el || aright !== er)) begin
            errs = errs + 1;
            if (errs <= 5)
                $display("  SAMPLE %0d: got %06h/%06h exp %06h/%06h",
                         smp, aleft, aright, el, er);
        end
        smp = smp + 1;
    end
end

integer i;
initial begin
    repeat (10) @(posedge clk);
    rst = 0;                        // ★ play 一直是 1
    t_start = $time;

    // 最多等 30ms 必须开播（正确实现约 2~3ms）
    for (i = 0; i < 1500000 && aplaying !== 1'b1; i = i + 1) @(posedge clk_v);

    chk("P1 prefilled and playing", aplaying, 1);
    $display("      (reset -> playing: %0d us)", ($time - t_start)/1000);

    // 再收 200 个样本
    for (i = 0; i < 400000 && smp < 200; i = i + 1) @(posedge clk_v);

    chk("P2 got 200 samples",       smp,    200);
    chk("P3 sample data correct",   errs,   0);
    chk("P4 no fifo drop",          adrop,  0);
    chk("P5 no underrun",           aunder, 0);

    if (fails == 0) $display("\n  ALL PASS  (%0d checks)\n", checks);
    else            $display("\n  FAILED    %0d of %0d checks\n", fails, checks);
    $finish;
end

initial begin #60_000_000; $display("\n  *** TIMEOUT ***\n"); $finish; end
endmodule


//==============================================================================
//  假卡：第 S 扇区第 i 字节 = (S + i) & 0xFF
//  end 用组合输出 —— 仲裁器在 end 那一拍就要撤 req，否则会多读一个扇区
//==============================================================================
module fake_sd_b #(
    parameter integer LAT = 3
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        read,
    input  wire [31:0] addr,
    output reg  [7:0]  dout,
    output reg         dv,
    output wire        endp,
    output reg  [31:0] served_addr,
    output reg         served
);

reg [2:0]  st;
reg [31:0] a;
reg [9:0]  i;

function [7:0] byte_of;
    input [31:0] sa;
    input [9:0]  si;
    begin
        byte_of = sa[7:0] + si[7:0];
    end
endfunction

assign endp = (st == 3'd3);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= 3'd0; a <= 32'd0; i <= 10'd0;
        dout <= 8'd0; dv <= 1'b0;
        served_addr <= 32'd0; served <= 1'b0;
    end else begin
        dv     <= 1'b0;
        served <= 1'b0;

        case (st)
        3'd0: begin
            if (read) begin
                a           <= addr;
                served_addr <= addr;
                served      <= 1'b1;
                i           <= 10'd0;
                st          <= 3'd1;
            end
        end
        3'd1: begin
            if (i == LAT) begin
                i  <= 10'd0;
                st <= 3'd2;
            end else
                i <= i + 10'd1;
        end
        3'd2: begin
            dout <= byte_of(a, i);
            dv   <= 1'b1;
            if (i == 10'd511) begin
                i  <= 10'd0;
                st <= 3'd3;
            end else
                i <= i + 10'd1;
        end
        3'd3: st <= 3'd0;
        default: st <= 3'd0;
        endcase
    end
end

endmodule
