//==============================================================================
//  tb_audio_wav_path.v —— SD 卡 WAV 播放通道的断言式系统测试
//==============================================================================
//
//  【被测对象】sd_bus_arbiter + audio_wav_path（含 FIFO / reader / wav_player）
//
//  【假卡】fake_sd_b 按扇区返回固定图案：第 S 扇区第 i 字节 = (S + i) & 0xFF
//    于是"扇区 100 跳过 44 字节后的第一个样本字"= 0x9190_9392，可以直接比对。
//    （小端：先到的 0x90 是 L 的低字节，所以 L16 = 0x9190、R16 = 0x9392）
//
//  【验证目标】
//    E 组：跳 header —— 第一个写入 FIFO 的字正好是 0x9190_9392
//    F 组：扇区序列 100→101→102→(回到 100)，循环成立
//    G 组：每轮恰好 300 个字（117 + 128 + 55），不多不少
//    H 组：48kHz 节拍 —— 480 个采样点 = 10.000ms（±1us）
//    I 组：位宽 —— 首个输出样本 = {L16, 8'h00} / {R16, 8'h00}
//    J 组：播放中插入图像装载（抢总线 5 个扇区）→ 不欠载、不丢样本
//    K 组：SW2 关闭 → 立即静音（仍送 valid）、FIFO 清空、不再写字；
//          再打开 → 从扇区 100 的第一个样本重新开始
//
//  【运行】
//    iverilog -o tb.vvp tb_audio_wav_path.v sd_audio_reader.v audio_wav_path.v \
//             wav_player.v ../SD/sd_bus_arbiter.v ../APP/img_afifo.v
//==============================================================================
`timescale 1ns/1ps

//==============================================================================
//  假 SD 卡
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

//==============================================================================
module tb_audio_wav_path;

//------------------------------------------------------------------------------
//  断言基础设施
//------------------------------------------------------------------------------
integer fails  = 0;
integer checks = 0;

task chk;
    input [8*80:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask

//------------------------------------------------------------------------------
//  时钟
//------------------------------------------------------------------------------
reg clk_sd = 1'b0;              // 50MHz
reg clk_v  = 1'b0;              // 25MHz
always #10  clk_sd = ~clk_sd;
always #20  clk_v  = ~clk_v;

reg rst      = 1'b1;
reg play     = 1'b0;            // 对应 SW2（1 = 播放）
reg init_done= 1'b0;

//------------------------------------------------------------------------------
//  参数
//------------------------------------------------------------------------------
localparam integer BASE  = 100;
localparam integer SKIP  = 44;
localparam integer WORDS = 300;
localparam integer PREF  = 128;

//------------------------------------------------------------------------------
//  互联
//------------------------------------------------------------------------------
wire        aud_req;
wire [31:0] aud_addr;
wire        aud_urgent;
wire [7:0]  aud_data;
wire        aud_dv, aud_end;

// 图像客户（本测试里只用来制造总线竞争，不检查它的数据）
reg  img_req = 1'b0;
reg  [31:0] img_addr = 32'd1000;
wire [7:0]  img_data;
wire        img_dv, img_end;
wire        img_end_w;
assign img_end_w = img_end;

wire        sd_read;
wire [31:0] sd_addr;
wire [7:0]  sd_data;
wire        sd_dv, sd_end;
wire        to_flag;
wire [1:0]  owner;

wire        a_valid;
wire [23:0] a_left, a_right;
wire        a_underrun, a_drop, a_playing;

sd_bus_arbiter #(
    .TIMEOUT_CYC (200000)        // 4ms，远大于一个扇区（~520 拍）
) u_arb (
    .I_clk          (clk_sd),
    .I_rst          (rst),
    .I_sd_init_done (init_done),

    .I_img_req      (img_req),
    .I_img_addr     (img_addr),
    .O_img_data     (img_data),
    .O_img_dv       (img_dv),
    .O_img_end      (img_end),

    .I_aud_req      (aud_req),
    .I_aud_addr     (aud_addr),
    .I_aud_urgent   (aud_urgent),
    .O_aud_data     (aud_data),
    .O_aud_dv       (aud_dv),
    .O_aud_end      (aud_end),

    .O_sd_sec_read  (sd_read),
    .O_sd_sec_addr  (sd_addr),
    .I_sd_sec_data  (sd_data),
    .I_sd_sec_dv    (sd_dv),
    .I_sd_sec_end   (sd_end),

    .O_timeout      (to_flag),
    .O_owner        (owner)
);

audio_wav_path #(
    .BASE_SECTOR  (BASE),
    .SKIP_BYTES   (SKIP),
    .TOTAL_WORDS  (WORDS),
    .FIFO_AW      (10),
    .PREFILL      (PREF),
    .REQ_LEVEL    (768),
    .URGENT_LEVEL (256)
) u_path (
    .I_clk_sd      (clk_sd),
    .I_rst_sd      (rst),
    .I_clk_v       (clk_v),
    .I_rst_v       (rst),
    .I_play_async  (play),

    .O_sd_req      (aud_req),
    .O_sd_addr     (aud_addr),
    .O_sd_urgent   (aud_urgent),
    .I_sd_data     (aud_data),
    .I_sd_dv       (aud_dv),
    .I_sd_end      (aud_end),

    .O_audio_valid (a_valid),
    .O_audio_left  (a_left),
    .O_audio_right (a_right),
    .O_underrun    (a_underrun),
    .O_drop        (a_drop),
    .O_playing     (a_playing)
);

fake_sd_b u_card (
    .clk         (clk_sd),
    .rst         (rst),
    .read        (sd_read),
    .addr        (sd_addr),
    .dout        (sd_data),
    .dv          (sd_dv),
    .endp        (sd_end),
    .served_addr (),
    .served      ()
);

//------------------------------------------------------------------------------
//  观测
//------------------------------------------------------------------------------
integer wr_cnt      = 0;        // FIFO 写入字数（累计）
integer sec_cnt     = 0;        // 卡已接受的扇区数（累计）
integer aud_cnt     = 0;        // 音频 valid 数（累计）
integer img_ends    = 0;        // 图像扇区结束数（J 组用）
integer sec_addr_log [0:15];    // 前 16 个扇区地址
integer sec_wr_log   [0:15];    // 每个扇区开始时的 wr_cnt

reg [31:0] first_word;
reg        first_word_ok = 1'b0;
reg [23:0] first_left, first_right;
reg        first_lr_ok = 1'b0;
integer    mute_bad   = 0;      // 静音期间出现非零样本
integer    wr_bad     = 0;      // 停止期间还在写 FIFO
integer    t_first    = 0;
integer    t_480      = 0;
reg        t480_ok    = 1'b0;

// 静音检查窗口：play 拉低后，等 5 拍（跨域同步只需 3 拍）再开始看
//   【为什么不能立刻看】wav_player 的 I_play 是同步过来的，比 play 晚 2~3 拍；
//   那一两拍里输出还可能是上一个非零样本（正常现象，不是静音失败）。
reg [4:0] play_dly;
always @(posedge clk_v) play_dly <= {play_dly[3:0], play};
wire check_mute = ~play_dly[4];

task log_sector;
    input [31:0] a;
    begin
        if (sec_cnt < 16) begin
            sec_addr_log[sec_cnt] = a;
            sec_wr_log[sec_cnt]   = wr_cnt;
        end
    end
endtask

// FIFO 写口 snoop
always @(posedge clk_sd) begin
    if (u_path.fifo_we) begin
        if (!play) wr_bad = wr_bad + 1;
        if (!first_word_ok && play) begin
            first_word    = u_path.fifo_din;
            first_word_ok = 1'b1;
        end
        wr_cnt = wr_cnt + 1;
    end
    if (u_card.served) begin
        log_sector(u_card.served_addr);
        sec_cnt = sec_cnt + 1;
    end
    if (img_end) img_ends = img_ends + 1;
end

// 音频输出 snoop
always @(posedge clk_v) begin
    if (a_valid) begin
        aud_cnt = aud_cnt + 1;

        // 第一个样本：检查 16→24 位宽（左移 8 位）
        if (a_playing && !first_lr_ok) begin
            first_left  = a_left;
            first_right = a_right;
            first_lr_ok = 1'b1;
            t_first     = $time;
            aud_cnt     = 1;
        end

        // 第 481 个样本：应恰好过了 10ms
        if (first_lr_ok && !t480_ok && (aud_cnt == 481)) begin
            t_480   = $time;
            t480_ok = 1'b1;
        end

        // 静音检查（关播 5 拍之后开始看，避开跨域同步延迟）
        if (check_mute && ((a_left !== 24'sd0) || (a_right !== 24'sd0)))
            mute_bad = mute_bad + 1;
    end
end

//------------------------------------------------------------------------------
//  用例
//------------------------------------------------------------------------------
integer t0, t1, k, wrap_wr;

initial begin
    $dumpfile("tb_audio_wav_path.vcd");
    $dumpvars(0, tb_audio_wav_path);

    rst = 1'b1; play = 1'b0; init_done = 1'b0; img_req = 1'b0;
    repeat (8) @(posedge clk_sd);
    rst = 1'b0;
    repeat (5) @(posedge clk_sd);

    // 关播状态先跑一会儿：不应该有任何写、任何请求
    repeat (200) @(posedge clk_sd);
    chk("A1 no request while stopped", aud_req, 0);
    chk("A2 no fifo write while stopped", wr_cnt, 0);

    //--------------------------------------------------------------------------
    //  开始播放
    //--------------------------------------------------------------------------
    init_done = 1'b1;
    @(negedge clk_sd);
    play = 1'b1;

    // 等到预灌完成、真正出声
    wait (a_playing === 1'b1);
    repeat (5) @(posedge clk_v);

    //--------------------------------------------------------------------------
    //  E 组：跳 header + 首字内容
    //--------------------------------------------------------------------------
    chk("E1 first fifo word after skip", first_word, 32'h9190_9392);
    chk("E2 first sector requested",     sec_addr_log[0], BASE);

    //--------------------------------------------------------------------------
    //  F/G 组：扇区序列与每轮字数（等第 4 个扇区 = 回到 100）
    //--------------------------------------------------------------------------
    wait (sec_cnt >= 4);
    repeat (2) @(posedge clk_sd);
    chk("F1 sector 0", sec_addr_log[0], BASE);
    chk("F2 sector 1", sec_addr_log[1], BASE + 1);
    chk("F3 sector 2", sec_addr_log[2], BASE + 2);
    chk("F4 wraps back to base", sec_addr_log[3], BASE);
    chk("G1 words per pass", sec_wr_log[3], WORDS);

    //--------------------------------------------------------------------------
    //  H 组：48000Hz 节拍（480 个样本 = 10ms）
    //--------------------------------------------------------------------------
    wait (t480_ok === 1'b1);
    t0 = t_480 - t_first;                   // 单位 ns
    chk("H1 480 samples span (ns, exp 10000000)",
        (t0 > 9_999_000 && t0 < 10_001_000) ? 1 : 0, 1);

    //--------------------------------------------------------------------------
    //  I 组：位宽（首个样本 = 首字的高低 16 位各左移 8）
    //--------------------------------------------------------------------------
    chk("I1 left  sample 24bit",  first_left,  24'h91_9000);
    chk("I2 right sample 24bit",  first_right, 24'h93_9200);

    //--------------------------------------------------------------------------
    //  J 组：播放中插入图像装载（图像连抢 20 个扇区）
    //--------------------------------------------------------------------------
    //  【这里验什么】音频 FIFO 有 21ms 存量，200us 的总线竞争**不该**让它欠载；
    //  真正验的是"仲裁把总线借给图像时，音频输出没有断、没有丢样本，
    //  之后还能继续预取"。优先级逻辑本身在 tb_sd_bus_arbiter 里逐条验过。
    //--------------------------------------------------------------------------
    k = aud_cnt;
    @(negedge clk_sd);
    img_req = 1'b1;
    wait (img_ends >= 20);
    @(negedge clk_sd);
    img_req = 1'b0;
    repeat (100) @(posedge clk_sd);

    chk("J1 audio kept playing during image load", (aud_cnt > k) ? 1 : 0, 1);
    chk("J2 no underrun",  a_underrun, 0);
    chk("J3 no fifo drop", a_drop,     0);

    //--------------------------------------------------------------------------
    //  K 组：关闭 → 静音 / 清 FIFO / 不再写；再打开 → 从头开始
    //--------------------------------------------------------------------------
    @(negedge clk_sd);
    play = 1'b0;
    repeat (60) @(posedge clk_v);
    chk("K1 not playing after switch off", a_playing, 0);
    chk("K2 fifo drained (rcount=0)",      u_path.u_fifo.rcount, 0);

    k = wr_cnt;
    repeat (400) @(posedge clk_sd);
    chk("K3 no write while stopped",  wr_cnt - k, 0);
    chk("K4 audio output muted",      mute_bad,   0);
    chk("K5 no write during mute window", wr_bad, 0);

    // 重新打开：应该又从扇区 100 的第一个样本开始
    first_word_ok = 1'b0;
    first_lr_ok   = 1'b0;
    sec_cnt       = 0;
    @(negedge clk_sd);
    play = 1'b1;

    wait (sec_cnt >= 1);
    repeat (2) @(posedge clk_sd);
    chk("K6 restart from base sector", sec_addr_log[0], BASE);
    chk("K7 restart first word",       first_word, 32'h9190_9392);

    wait (a_playing === 1'b1);
    repeat (5) @(posedge clk_v);
    chk("K8 playing again", a_playing, 1);

    chk("L1 arbitration never timed out", to_flag, 0);

    //--------------------------------------------------------------------------
    //  汇总
    //--------------------------------------------------------------------------
    repeat (10) @(posedge clk_sd);
    if (fails == 0) $display("\n  ALL PASS  (%0d checks)", checks);
    else            $display("\n  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

// 看门狗
initial begin
    #60_000_000;                        // 60ms
    $display("\n  *** TIMEOUT  simulation hung ***");
    $finish;
end

endmodule
