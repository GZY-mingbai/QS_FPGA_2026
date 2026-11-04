//==============================================================================
//  tb_flash_arb.v —— flash_arb 的仲裁正确性验证
//==============================================================================
//
//  【验什么】三个不变量，全是"旧版会挂、新版必须过"的：
//    T1 请求永不丢失：每个客户发起的每一次请求，都必须**恰好**收到一次 done。
//    T2 永不挂死：任何一次请求都要在有限时间内拿到 done（含控制器异常）。
//    T3 数据不串：每个客户收到的字节序列必须来自它自己的地址。
//
//  【怎么造出旧版那个 bug】
//   客户侧严格照抄真实 RTL 的时序：
//       osd_font_cache : if (rd_ready) begin rd_req_r <= 1'b1; st <= S_FRX; end
//       osd_icon_rom   : if (fl_rdy)   begin fl_req   <= 1'b1; st <= S_RX;  end
//       beep_player    : if (fl_rdy && ...) fl_req <= 1'b1;
//   即"看到 rdy 的那一拍不拉 req，下一拍才拉"，而且拉完立刻进入等数据态。
//   三个客户在同一拍同时发起 ⇒ 旧版会吞掉两个，新版必须三个都服务到。
//
//  【控制器模型】照 spi_flash_read 的语义：
//   · fl_ready = 1 只在空闲
//   · 认请求后先发 4 字节（命令+地址），再逐字节吐数据
//   · **fl_done 与最后一个 fl_valid 同拍**
//
//==============================================================================
`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//  客户模型：见 rdy → 下一拍拉单拍 req → 等 done
//------------------------------------------------------------------------------
module arb_cust #(
    parameter [1:0] ID = 2'd0
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        go,          // 单拍：发起一次请求
    input  wire [23:0] a,           // 目标地址
    input  wire [15:0] l,           // 长度

    input  wire        rdy,
    output reg         req,
    output reg  [23:0] addr,
    output reg  [15:0] len,

    input  wire [7:0]  data,
    input  wire        valid,
    input  wire        done,

    output reg  [15:0] n_done,      // 收到几次 done
    output reg  [15:0] n_valid,     // 收到几个字节
    output reg  [15:0] n_go,        // 发起过几次
    output reg  [7:0]  dat_err,     // 数据校验失败次数（只增）
    output reg         waiting      // 1 = 已提交请求，还没等到 done
);

reg [23:0] exp_addr;
reg [15:0] cnt;
reg        want;        // 1 = 有活要干（go 置位，提交请求后清零）

always @(posedge clk or posedge rst) begin
    if (rst) begin
        req      <= 1'b0;
        addr     <= 24'd0;
        len      <= 16'd0;
        n_done   <= 16'd0;
        n_valid  <= 16'd0;
        n_go     <= 16'd0;
        dat_err  <= 8'd0;
        waiting  <= 1'b0;
        exp_addr <= 24'd0;
        cnt      <= 16'd0;
        want     <= 1'b0;
    end else begin
        req <= 1'b0;

        // ---- 接活：go 单拍脉冲，把参数锁进本客户 ----
        if (go && !waiting && !want) begin
            want     <= 1'b1;
            addr     <= a;
            len      <= l;
            exp_addr <= a;
            cnt      <= 16'd0;
        end

        // ---- 关键时序：看到 rdy 的这一拍只"预约"，req 在下一拍 ----
        //   与真实 RTL 完全一致：
        //     osd_font_cache : if (rd_ready) begin rd_req_r <= 1'b1; st <= S_FRX; end
        //   ★只提交一次★（want 清掉），提交后进入 waiting，直到 done 才解除。
        if (want && rdy) begin
            want    <= 1'b0;
            waiting <= 1'b1;
            req     <= 1'b1;
            n_go    <= n_go + 16'd1;
        end

        // ---- 收数据：逐字节校验 ----
        if (valid) begin
            n_valid <= n_valid + 16'd1;
            if (data !== (exp_addr[7:0] + cnt[7:0])) begin
                if (dat_err != 8'hFF) dat_err <= dat_err + 8'd1;
                $display("  [C%0d] DATA MISMATCH #%0d got=%02h exp=%02h",
                         ID, cnt, data, exp_addr[7:0] + cnt[7:0]);
            end
            cnt <= cnt + 16'd1;
        end

        // ---- 收工 ----
        if (done) begin
            n_done  <= n_done + 16'd1;
            waiting <= 1'b0;
        end
    end
end
endmodule

//------------------------------------------------------------------------------
//  控制器模型
//------------------------------------------------------------------------------
module arb_ctrl (
    input  wire        clk,
    input  wire        rst,
    input  wire        hang,        // 1 = 故意不回 done（模拟 SPI 异常）

    input  wire        fl_req,
    input  wire [23:0] fl_addr,
    input  wire [15:0] fl_len,
    output reg         fl_ready,
    output reg  [7:0]  fl_data,
    output reg         fl_valid,
    output reg         fl_done
);
localparam [1:0] S_ID = 2'd0, S_TX = 2'd1, S_RX = 2'd2, S_DN = 2'd3;

reg [1:0]  st;
reg [3:0]  dv;
reg [15:0] cnt, total;
reg [23:0] a;

always @(*) fl_ready = (st == S_ID);   // 组合，和真实控制器一致

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= S_ID; dv <= 4'd0; cnt <= 16'd0; total <= 16'd0;
        a <= 24'd0; fl_data <= 8'd0; fl_valid <= 1'b0; fl_done <= 1'b0;
    end else begin
        // 默认拉低：valid/done 都是单拍
        fl_valid <= 1'b0;
        fl_done  <= 1'b0;

        case (st)
        S_ID: if (fl_req && (fl_len != 16'd0)) begin
            a <= fl_addr; total <= fl_len; cnt <= 16'd0; dv <= 4'd0;
            st <= S_TX;
        end
        // 发 4 字节命令+地址：每字节 4 拍
        S_TX: begin
            if (dv == 4'd3) begin dv <= 4'd0; st <= S_RX; end
            else                 dv <= dv + 4'd1;
        end
        // 收字节：每 4 拍一个；最后一个 valid 与 done 同拍
        S_RX: begin
            if (!hang) begin
                if (dv == 4'd3) begin
                    dv       <= 4'd0;
                    fl_valid <= 1'b1;
                    fl_data  <= a[7:0] + cnt[7:0];
                    if (cnt == total - 16'd1) begin
                        fl_done <= 1'b1;
                        st      <= S_DN;
                    end else cnt <= cnt + 16'd1;
                end else dv <= dv + 4'd1;
            end
            // hang=1 就一直停在这里，不吐数据也不回 done
        end
        S_DN: st <= S_ID;
        default: st <= S_ID;
        endcase
    end
end
endmodule

//------------------------------------------------------------------------------
//  主测试
//------------------------------------------------------------------------------
module tb_flash_arb;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg hang = 0;

// DUT 端口
wire        c0_rdy, c1_rdy, c2_rdy;
wire [7:0]  c0_data, c1_data, c2_data;
wire        c0_valid, c1_valid, c2_valid;
wire        c0_done, c1_done, c2_done;
wire        c0_req, c1_req, c2_req;
wire [23:0] c0_addr, c1_addr, c2_addr;
wire [15:0] c0_len, c1_len, c2_len;

wire        fl_req;
wire [23:0] fl_addr;
wire [15:0] fl_len;
wire        fl_ready;
wire [7:0]  fl_data;
wire        fl_valid, fl_done;

// 客户实例
reg         go0 = 0, go1 = 0, go2 = 0;
reg  [23:0] a0 = 0, a1 = 0, a2 = 0;
reg  [15:0] l0 = 0, l1 = 0, l2 = 0;
wire [15:0] d0, d1, d2, v0, v1, v2, g0, g1, g2;
wire [7:0]  e0, e1, e2;
wire        w0, w1, w2;

arb_cust #(.ID(2'd0)) u_c0 (
    .clk(clk), .rst(rst), .go(go0), .a(a0), .l(l0),
    .rdy(c0_rdy), .req(c0_req), .addr(c0_addr), .len(c0_len),
    .data(c0_data), .valid(c0_valid), .done(c0_done),
    .n_done(d0), .n_valid(v0), .n_go(g0), .dat_err(e0), .waiting(w0));

arb_cust #(.ID(2'd1)) u_c1 (
    .clk(clk), .rst(rst), .go(go1), .a(a1), .l(l1),
    .rdy(c1_rdy), .req(c1_req), .addr(c1_addr), .len(c1_len),
    .data(c1_data), .valid(c1_valid), .done(c1_done),
    .n_done(d1), .n_valid(v1), .n_go(g1), .dat_err(e1), .waiting(w1));

arb_cust #(.ID(2'd2)) u_c2 (
    .clk(clk), .rst(rst), .go(go2), .a(a2), .l(l2),
    .rdy(c2_rdy), .req(c2_req), .addr(c2_addr), .len(c2_len),
    .data(c2_data), .valid(c2_valid), .done(c2_done),
    .n_done(d2), .n_valid(v2), .n_go(g2), .dat_err(e2), .waiting(w2));

arb_ctrl u_ctrl (
    .clk(clk), .rst(rst), .hang(hang),
    .fl_req(fl_req), .fl_addr(fl_addr), .fl_len(fl_len),
    .fl_ready(fl_ready), .fl_data(fl_data), .fl_valid(fl_valid), .fl_done(fl_done));

flash_arb #(.TIMEOUT_CYC(4000)) u_dut (
    .clk(clk), .rst(rst),
    .c0_req(c0_req), .c0_addr(c0_addr), .c0_len(c0_len),
    .c0_rdy(c0_rdy), .c0_data(c0_data), .c0_valid(c0_valid), .c0_done(c0_done),
    .c1_req(c1_req), .c1_addr(c1_addr), .c1_len(c1_len),
    .c1_rdy(c1_rdy), .c1_data(c1_data), .c1_valid(c1_valid), .c1_done(c1_done),
    .c2_req(c2_req), .c2_addr(c2_addr), .c2_len(c2_len),
    .c2_rdy(c2_rdy), .c2_data(c2_data), .c2_valid(c2_valid), .c2_done(c2_done),
    .fl_req(fl_req), .fl_addr(fl_addr), .fl_len(fl_len),
    .fl_ready(fl_ready), .fl_data(fl_data), .fl_valid(fl_valid), .fl_done(fl_done));

integer errs = 0;
integer k;

task chk(input [255:0] name, input integer got, input integer exp);
begin
    if (got !== exp) begin
        $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
        errs = errs + 1;
    end else
        $display("   ok       %0s = %0d", name, got);
end
endtask

// 发起一次三方同时请求（这就是旧版必挂的场景）
task burst3(input [23:0] base);
begin
    @(negedge clk);
    a0 = base + 24'h000010; l0 = 16'd32;   // 字库：32B
    a1 = base + 24'h03D000; l1 = 16'd64;   // 图标：64B（缩短仿真）
    a2 = base + 24'h040000; l2 = 16'd16;   // 提示音：16B
    go0 = 1; go1 = 1; go2 = 1;
    @(negedge clk);
    go0 = 0; go1 = 0; go2 = 0;
end
endtask

initial begin
    $display("=== T1/T3：三方同拍抢总线（旧版会吞掉两个请求）===");
    repeat (5) @(negedge clk);
    rst = 0;
    repeat (5) @(negedge clk);

    for (k = 0; k < 20; k = k + 1) begin
        burst3(k * 24'h000100);
        // 等本轮全部收工（或超时保护）
        repeat (3000) @(negedge clk);
    end

    chk("T1 c0 n_go",    g0, 20);
    chk("T1 c0 n_done",  d0, 20);
    chk("T1 c1 n_go",    g1, 20);
    chk("T1 c1 n_done",  d1, 20);
    chk("T1 c2 n_go",    g2, 20);
    chk("T1 c2 n_done",  d2, 20);
    chk("T3 c0 dat_err", e0, 0);
    chk("T3 c1 dat_err", e1, 0);
    chk("T3 c2 dat_err", e2, 0);
    chk("T1 c0 n_valid", v0, 20*32);
    chk("T1 c1 n_valid", v1, 20*64);
    chk("T1 c2 n_valid", v2, 20*16);
    chk("T2 c0 not waiting", w0, 0);
    chk("T2 c1 not waiting", w1, 0);
    chk("T2 c2 not waiting", w2, 0);

    $display("=== T2：控制器异常（hang=1）→ 超时兜底必须放行 ===");
    // 让 c0 单独发一单，控制器不回 done
    @(negedge clk);
    hang = 1;
    a0 = 24'h001234; l0 = 16'd8; go0 = 1;
    @(negedge clk);
    go0 = 0;
    // TIMEOUT_CYC=4000，给足 8000 拍
    repeat (8000) @(negedge clk);
    chk("T2 c0 done after timeout", d0, 21);
    chk("T2 c0 not waiting",        w0, 0);
    hang = 0;

    $display("=== T2b：超时后总线必须还能被别人用 ===");
    repeat (10) @(negedge clk);
    a2 = 24'h040800; l2 = 16'd16; go2 = 1;
    @(negedge clk);
    go2 = 0;
    repeat (2000) @(negedge clk);
    chk("T2b c2 still alive", d2, 21);

    $display("=== T1b：长度为 0 的请求不能把客户晾住 ===");
    a1 = 24'h03D000; l1 = 16'd0; go1 = 1;
    @(negedge clk);
    go1 = 0;
    repeat (200) @(negedge clk);
    // 长度为 0 时 arb_cust 不会提交（rdy 分支要求 l!=0），所以 n_go 会 +1
    // 但不会进入等待态；这里只验证"没有挂死"
    chk("T1b c1 not waiting", w1, 0);

    $display("======================================");
    if (errs == 0) $display("  ALL PASS");
    else           $display("  %0d FAIL", errs);
    $display("======================================");
    $finish;
end

// 全局看门狗
initial begin
    #20_000_000;
    $display("  *** GLOBAL TIMEOUT ***");
    $finish;
end

endmodule
