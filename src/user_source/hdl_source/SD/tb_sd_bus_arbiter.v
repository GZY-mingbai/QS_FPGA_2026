//==============================================================================
//  tb_sd_bus_arbiter.v —— SD 扇区级仲裁器的断言式单元测试
//==============================================================================
//
//  【验证目标】（对应设计里的四条硬约束）
//    A 组：init_done 之前绝不授权
//    B 组：音频不 urgent 时图像优先，且音频完全拿不到总线
//    C 组：音频 urgent 时抢占成功，图像被暂时饿住；音频撤请求后图像恢复
//    D 组：授权互斥、数据和 end 只发给被授权者、地址在一段读期间不变、
//          每段读结束后 sd_sec_read 必须拉低
//    E 组：客户卡住时有超时兜底，强制给 end，不会让装载器永久等死
//
//  【假卡】fake_sd_a 的行为与 sd_card_top 一致：
//    只在 read 上升沿锁地址 → 几拍命令延时 → 512 字节逐拍吐
//    → end 单拍 → 回空闲
//  数据内容 = (地址低 8 位 + 字节序号) & 0xFF，用来确认"谁读的扇区给了谁"。
//
//  【运行】
//    iverilog -o tb.vvp tb_sd_bus_arbiter.v sd_bus_arbiter.v && vvp tb.vvp
//==============================================================================
`timescale 1ns/1ps

//==============================================================================
//  假 SD 卡（只支持单扇区读）
//==============================================================================
module fake_sd_a #(
    parameter integer LAT = 3               // 命令延时（拍）
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        read,
    input  wire [31:0] addr,
    input  wire        stall,               // 1 = 装死（测超时）
    output reg  [7:0]  dout,
    output reg         dv,
    output wire        endp,                // ★ 组合输出（与 sd_card_top 一致：
                                            //   它也是 assign end = (state==S_READ_END)）
    output reg  [31:0] served_addr,         // 本次读锁存的地址
    output reg         served               // 锁存那一拍（单拍）
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

//  end 必须是组合输出：真实 sd_card_top 用的是
//      assign sd_sec_read_end = (state == S_READ_END);
//  如果这里用寄存器打一拍，end 会晚一个周期，而那一拍状态已经回到 IDLE、
//  read 还是高 ⇒ 假卡会**误接受一次新请求**，后面整条数据流全错位。
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
            if (read && !stall) begin
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
        3'd3: begin
            // endp 是本状态的组合输出，这里只负责回空闲
            st <= 3'd0;
        end
        default: st <= 3'd0;
        endcase
    end
end

endmodule

//==============================================================================
module tb_sd_bus_arbiter;

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
//  时钟与激励
//------------------------------------------------------------------------------
reg clk = 1'b0;
always #10 clk = ~clk;                  // 50MHz

reg rst       = 1'b1;
reg init_done = 1'b0;
reg stall     = 1'b0;

// 客户 0：图像。行为与 bmp_read 一致 —— 地址在 end 之后 +1
reg  img_req = 1'b0;
reg  [31:0] img_addr = 32'd100;
wire [7:0]  img_data;
wire        img_dv, img_end;

// 客户 1：音频。地址同样在 end 之后 +1
reg  aud_req    = 1'b0;
reg  [31:0] aud_addr = 32'd500;
reg  aud_urgent = 1'b0;
wire [7:0]  aud_data;
wire        aud_dv, aud_end;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        img_addr <= 32'd100;
        aud_addr <= 32'd500;
    end else begin
        if (img_end) img_addr <= img_addr + 32'd1;
        if (aud_end) aud_addr <= aud_addr + 32'd1;
    end
end

wire        sd_read;
wire [31:0] sd_addr;
wire [7:0]  sd_data;
wire        sd_dv, sd_end;
wire        to_flag;
wire [1:0]  owner;

//------------------------------------------------------------------------------
//  待测模块
//  【超时取值】假卡一个扇区约 520 拍（3 拍命令 + 512 字 + end），
//  这里给 2000 拍（40us）—— 远大于正常扇区，又比真实 20ms 短，便于仿真。
//------------------------------------------------------------------------------
sd_bus_arbiter #(
    .TIMEOUT_CYC (2000),
    .AUD_GAP_CYC (0)            // ★ 本台测的是"授权/地址/end"协议，把音频限速关掉
) u_dut (
    .I_clk          (clk),
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

//------------------------------------------------------------------------------
//  假卡
//------------------------------------------------------------------------------
wire [7:0]  card_data;
wire        card_dv, card_end;
wire [31:0] card_addr;
wire        card_served;

fake_sd_a u_card (
    .clk         (clk),
    .rst         (rst),
    .read        (sd_read),
    .addr        (sd_addr),
    .stall       (stall),
    .dout        (card_data),
    .dv          (card_dv),
    .endp        (card_end),
    .served_addr (card_addr),
    .served      (card_served)
);

assign sd_data = card_data;
assign sd_dv   = card_dv;
assign sd_end  = card_end;

//------------------------------------------------------------------------------
//  计数器与不变量监视
//------------------------------------------------------------------------------
integer img_end_cnt = 0;
integer aud_end_cnt = 0;
integer srv_cnt     = 0;

integer noinit_read    = 0;    // init 之前出现 read
integer mixed_dv       = 0;    // 同一拍两个客户都收到数据
integer img_addr_bad   = 0;    // 图像收到的数据不属于它请求的扇区
integer aud_addr_bad   = 0;    // 音频同上
integer addr_moved     = 0;    // 一段读期间地址变了
integer no_gap         = 0;    // end 之后下一拍 read 还是高
integer owner_mismatch = 0;    // 客户收到 dv 但 owner 不是它

reg        prev_read;
reg        prev_end;
reg [31:0] cur_addr;

always @(posedge clk) begin
    if (img_end) img_end_cnt <= img_end_cnt + 1;
    if (aud_end) aud_end_cnt <= aud_end_cnt + 1;
    if (card_served) srv_cnt <= srv_cnt + 1;

    if (sd_read && !init_done) noinit_read <= noinit_read + 1;
    if (img_dv && aud_dv)      mixed_dv    <= mixed_dv + 1;

    if (img_dv && (img_addr !== card_addr)) img_addr_bad <= img_addr_bad + 1;
    if (aud_dv && (aud_addr !== card_addr)) aud_addr_bad <= aud_addr_bad + 1;

    if (img_dv && (owner !== 2'd1)) owner_mismatch <= owner_mismatch + 1;
    if (aud_dv && (owner !== 2'd2)) owner_mismatch <= owner_mismatch + 1;

    // 地址在一段读期间不许变
    if (sd_read && prev_read && (sd_addr !== cur_addr)) addr_moved <= addr_moved + 1;
    if (sd_read && !prev_read) cur_addr <= sd_addr;

    // end 之后必须至少有一拍 read=0（否则控制器来不及重新锁地址）
    if (prev_end && sd_read) no_gap <= no_gap + 1;

    prev_read <= sd_read;
    prev_end  <= sd_end;
end

//------------------------------------------------------------------------------
//  用例
//------------------------------------------------------------------------------
integer base_img, base_aud, quiet;

// 把总线彻底放空：两个客户都撤请求，并要求**连续 4 拍** owner=0 且 read=0。
//   【为什么不能只等 owner==0】arbiter 在 A_IDLE 只停一拍；如果撤请求的那一拍
//   刚好和"授权边沿"撞上（initial 块与 always 块同一时刻求值的竞态），
//   下一拍 owner 就变 1 了。只等一次会漏掉这个正在飞的扇区，
//   基线就会提前拍快照 ⇒ 后面那条 end 被算进新阶段（假失败）。
task quiesce;
    begin
        img_req = 1'b0;
        aud_req = 1'b0;
        quiet   = 0;
        while (quiet < 4) begin
            @(posedge clk);
            if ((owner == 2'd0) && (sd_read == 1'b0))
                quiet = quiet + 1;
            else
                quiet = 0;
        end
    end
endtask

initial begin
    $dumpfile("tb_sd_bus_arbiter.vcd");
    $dumpvars(0, tb_sd_bus_arbiter);

    rst = 1'b1; init_done = 1'b0;
    img_req = 1'b0; aud_req = 1'b0; aud_urgent = 1'b0;
    repeat (6) @(posedge clk);
    rst = 1'b0;

    //--------------------------------------------------------------------------
    //  A 组：init_done 之前不授权
    //--------------------------------------------------------------------------
    img_req = 1'b1; aud_req = 1'b1; aud_urgent = 1'b1;
    repeat (40) @(posedge clk);
    chk("A1 no read before init_done", noinit_read, 0);
    chk("A2 owner stays idle before init", owner, 0);
    chk("A3 no sector served before init", srv_cnt, 0);

    init_done = 1'b1;
    repeat (5) @(posedge clk);

    //--------------------------------------------------------------------------
    //  B 组：音频不 urgent → 图像优先
    //--------------------------------------------------------------------------
    quiesce;
    @(negedge clk);
    img_req = 1'b1; aud_req = 1'b1; aud_urgent = 1'b0;
    base_img = img_end_cnt;
    base_aud = aud_end_cnt;
    wait (img_end_cnt >= base_img + 3);
    repeat (2) @(posedge clk);
    chk("B1 image got 3 sectors",           img_end_cnt - base_img, 3);
    chk("B2 audio got none when not urgent", aud_end_cnt - base_aud, 0);

    //--------------------------------------------------------------------------
    //  C 组：音频 urgent → 抢占
    //--------------------------------------------------------------------------
    quiesce;
    @(negedge clk);
    img_req = 1'b1; aud_req = 1'b1; aud_urgent = 1'b1;
    base_img = img_end_cnt;
    base_aud = aud_end_cnt;
    wait (aud_end_cnt >= base_aud + 3);
    repeat (2) @(posedge clk);
    chk("C1 audio got 3 sectors when urgent", aud_end_cnt - base_aud, 3);
    chk("C2 image starved while audio urgent", img_end_cnt - base_img, 0);

    //--------------------------------------------------------------------------
    //  C 组续：音频撤请求 → 图像恢复
    //--------------------------------------------------------------------------
    quiesce;
    @(negedge clk);
    img_req = 1'b1; aud_req = 1'b0; aud_urgent = 1'b0;
    base_img = img_end_cnt;
    wait (img_end_cnt >= base_img + 2);
    repeat (2) @(posedge clk);
    chk("C3 image resumes after audio stops", img_end_cnt - base_img, 2);

    //--------------------------------------------------------------------------
    //  D 组：互斥 / 地址 / 间隙不变量
    //--------------------------------------------------------------------------
    chk("D1 never two clients at once",       mixed_dv,       0);
    chk("D2 image data matches its sector",   img_addr_bad,   0);
    chk("D3 audio data matches its sector",   aud_addr_bad,   0);
    chk("D4 address stable within a sector",  addr_moved,     0);
    chk("D5 read deasserted after end",       no_gap,         0);
    chk("D6 dv only goes to granted owner",   owner_mismatch, 0);
    chk("D7 both clients actually got data",  (img_end_cnt > 0) && (aud_end_cnt > 0), 1);

    //--------------------------------------------------------------------------
    //  E 组：超时兜底（假卡装死，图像一直请求）
    //--------------------------------------------------------------------------
    quiesce;
    stall   = 1'b1;
    img_req = 1'b1;
    base_img = img_end_cnt;
    wait (to_flag === 1'b1);
    repeat (4) @(posedge clk);
    chk("E1 timeout flag raised",            to_flag, 1);
    chk("E2 client released by forced end",  img_end_cnt - base_img, 1);
    chk("E3 still deasserted read after end", no_gap, 0);

    stall = 1'b0;

    //--------------------------------------------------------------------------
    //  汇总
    //--------------------------------------------------------------------------
    repeat (5) @(posedge clk);
    if (fails == 0) $display("\n  ALL PASS  (%0d checks)", checks);
    else            $display("\n  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

// 看门狗：跑飞就报错退出
initial begin
    #2_000_000;
    $display("\n  *** TIMEOUT  simulation hung ***");
    $finish;
end

endmodule
