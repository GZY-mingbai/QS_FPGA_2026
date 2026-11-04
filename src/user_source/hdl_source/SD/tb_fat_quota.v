//==============================================================================
//  tb_fat_quota.v —— fat_quota_gate 单元台（限流不许变成"全停"）
//==============================================================================
//  背景：顶层原来是
//      fat_req_g = lst_sd_req & ~((lst_own == OWN_MM) & ld_busy);
//  —— "match 扫描 + 装载器忙"期间 FAT 客户的请求被**整个摘掉**。lister 的上界
//  `sec_left` 是按扇区递减的 ⇒ 一个扇区都拿不到就没有上界 ⇒ O_busy 永久 1
//  ⇒ 相册永远重扫不了 ⇒ alb_ready 永久 0 ⇒ 之后再也发不出装载。
//
//  本台守四条：
//    Q1 不在限流窗口（i_block=0）⇒ 完全透传
//    Q2 在窗口内 ⇒ 恰好每 N 个 IMG 扇区放 1 个 FAT 扇区（不是 0 个）
//    Q3 放行后 `o_req` 必须保持到该 FAT 扇区结束（中途撤请求会让 lister 读到残余字节）
//    Q5 退出窗口 ⇒ 计数/pass 清零、立刻透传
//
//  跑法：iverilog -g2012 -o q.vvp -s tb_fat_quota tb_fat_quota.v fat_quota_gate.v && vvp q.vvp
//==============================================================================
`timescale 1ns/1ps

module tb_fat_quota;

localparam integer N = 8;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  i_block = 0, i_req = 0, i_img_end = 0, i_fat_end = 0;
wire o_req;

fat_quota_gate #(.N(N)) dut (
    .clk(clk), .rst(rst),
    .i_block(i_block), .i_req(i_req),
    .i_img_end(i_img_end), .i_fat_end(i_fat_end),
    .o_req(o_req)
);

integer errors = 0;
integer i, npass;

task chk;
    input exp;
    input [255:0] t;
    begin
        #1;                                  // 组合赋值落地
        if (o_req !== exp) begin
            $display("  [FAIL] %0s: o_req=%b expect %b (block=%b req=%b cnt=%0d pass=%b)",
                     t, o_req, exp, i_block, i_req, dut.cnt, dut.pass);
            errors = errors + 1;
        end else
            $display("  ok   %0s: o_req=%b", t, o_req);
    end
endtask

task img_end;
    begin
        @(negedge clk); i_img_end = 1;
        @(negedge clk); i_img_end = 0;
        @(negedge clk);
    end
endtask

task fat_end;
    begin
        @(negedge clk); i_fat_end = 1;
        @(negedge clk); i_fat_end = 0;
        @(negedge clk);
    end
endtask

initial begin
    $display("== tb_fat_quota: N=%0d ==", N);
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    //------------------------------------------------------------------ Q1
    $display("  -- Q1 passthrough outside window --");
    i_block = 0; i_req = 0; chk(0, "Q1a");
    i_req = 1;              chk(1, "Q1b");
    i_block = 1; i_req = 0; chk(0, "Q1c");
    i_block = 0; i_req = 1; chk(1, "Q1d");

    //------------------------------------------------------------------ Q2
    $display("  -- Q2 one pass per %0d IMG sectors --", N);
    i_block = 1; i_req = 1;
    @(negedge clk);
    for (i = 1; i <= N; i = i + 1) begin
        img_end;
        if (i < N) chk(0, "Q2 pre-quota");
        else       chk(1, "Q2 at-quota");
    end

    //------------------------------------------------------------------ Q3
    $display("  -- Q3 pass held until FAT sector ends --");
    i_req = 0; chk(0, "Q3a");
    i_req = 1; chk(1, "Q3b");
    fat_end;
    chk(0, "Q3c");
    i_req = 1; chk(0, "Q3d");

    //------------------------------------------------------------------ Q4
    $display("  -- Q4 long run: >= K/N passes over K IMG sectors --");
    npass = 0;
    for (i = 0; i < 10 * N; i = i + 1) begin
        img_end;
        if (o_req) begin
            npass = npass + 1;
            fat_end;
        end
    end
    if (npass < 10) begin
        $display("  [FAIL] Q4 only %0d passes (expect >= 10)", npass);
        errors = errors + 1;
    end else
        $display("  ok   Q4 %0d passes over %0d IMG sectors", npass, 10 * N);

    //------------------------------------------------------------------ Q5
    $display("  -- Q5 window exit clears state --");
    i_block = 0; i_req = 1; chk(1, "Q5a");
    if (dut.pass !== 1'b0 || dut.cnt !== 0) begin
        $display("  [FAIL] Q5b pass/cnt not cleared (pass=%b cnt=%0d)", dut.pass, dut.cnt);
        errors = errors + 1;
    end else
        $display("  ok   Q5b cleared");

    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #2_000_000;
    $display("TIMEOUT");
    $display("== errors=%0d FAIL ==", errors + 1);
    $finish;
end

endmodule
