//==============================================================================
//  tb_quad_loader.v —— 四宫格装载调度单元测试
//==============================================================================
//  iverilog -g2012 -o ql.vvp -s tb_quad_loader tb_quad_loader.v quad_loader.v && vvp ql.vvp
//
//  T1 正常 4 项：base 依次是 SLOT0..3，off 依次是 0x1000..0x4000
//  T2 第 2 项 err：跳过，loaded=3
//  T3 valid_cnt=2：只装 2 张
//==============================================================================
`timescale 1ns/1ps

module tb_quad_loader;

localparam [20:0] SBASE = 21'h100000;
localparam integer SH   = 40960;   // ★ 槽间距（字）：160 字/行 × 256 行

reg clk = 0, rst = 1;
reg start = 0;
reg [2:0] valid_cnt = 4;
reg [3:0] slot_valid = 4'hF;
wire busy, done;
wire [2:0] loaded;

wire [1:0] rd_slot;
reg  [31:0] file_off, file_len;

wire ld_start;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg  ld_ready = 1, ld_busy = 0, ld_done = 0, ld_err = 0;

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin errors = errors + 1; $display("  FAIL [%0d] %0s", ncase, msg); end
    end
endtask

// 列表：组合输出，跟着 rd_slot
always @* begin
    case (rd_slot)
        2'd0: begin file_off = 32'h1000; file_len = 32'd1000; end
        2'd1: begin file_off = 32'h2000; file_len = 32'd2000; end
        2'd2: begin file_off = 32'h3000; file_len = 32'd3000; end
        default: begin file_off = 32'h4000; file_len = 32'd4000; end
    endcase
end

quad_loader #(
    .SLOT_BASE(SBASE), .SLOT_STEP(SH), .N_ITEMS(4)
) dut (
    .clk(clk), .rst(rst),
    .I_start(start), .I_abort(1'b0), .I_hold(1'b0), .O_bmp(), .O_filled(), .I_valid_cnt(valid_cnt), .I_slot_valid(slot_valid),
    .O_busy(busy), .O_done(done), .O_loaded(loaded),
    .O_rd_slot(rd_slot), .I_file_off(file_off), .I_file_len(file_len),
    .I_file_type(4'd1),
    .O_ph_start (),
    .O_ph_kind  (),
    .O_ph_base  (),
    .I_ph_busy  (1'b0),
    .I_ph_done  (1'b0),
    .O_ld_start(ld_start), .O_ld_off(ld_off), .O_ld_len(ld_len), .O_ld_base(ld_base),
    .I_ld_ready(ld_ready), .I_ld_busy(ld_busy), .I_ld_done(ld_done), .I_ld_err(ld_err)
);

always #10 clk = ~clk;



//------------------------------------------------------------------------------
//  bmp_loader 模型：start → busy 3 拍 → done（可注入 err）
//------------------------------------------------------------------------------
integer n_load = 0;
reg [20:0] rec_base [0:3];
reg [31:0] rec_off  [0:3];
reg inject_err = 0;
integer err_idx = -1;
integer waitc = 0;
reg running = 0;

always @(posedge clk) begin
    ld_done <= 0;
    ld_err  <= 0;          // ★ 与真 bmp_loader 一致：err 是单拍（原来是粘滞电平）
    if (rst) begin
        running <= 0;
        ld_busy <= 0;
    end else if (!running) begin
        ld_busy <= 0;
        if (ld_start) begin
            running <= 1;
            ld_busy <= 1;
            waitc   <= 3;
            if (n_load < 4) begin
                rec_base[n_load] <= ld_base;
                rec_off[n_load]  <= ld_off;
            end
            n_load <= n_load + 1;
        end
    end else begin
        if (waitc != 0) waitc <= waitc - 1;
        else begin
            ld_done <= 1;
            ld_err  <= (n_load - 1 == err_idx) ? 1'b1 : 1'b0;
            ld_busy <= 0;
            running <= 0;
        end
    end
end

task run_page;
    input [2:0] vc;
    begin
        @(posedge clk); start = 1;
        @(posedge clk); start = 0;
        wait (done);          // done 只 1 拍，轮询会漏
        repeat (2) @(posedge clk);
    end
endtask

initial begin
    $dumpfile("tb_ql.vcd");
    $dumpvars(0, tb_quad_loader);

    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    //======================================================================
    $display("\n[T1] load all 4");
    n_load = 0; err_idx = -1; valid_cnt = 4; slot_valid = 4'hF;
    run_page(3'd4);
    $display("      n_load=%0d loaded=%0d", n_load, loaded);
    chk(n_load == 4,     "T1 issued 4 loads");
    chk(loaded == 4,     "T1 loaded == 4");
    chk(rec_base[0] == SBASE + 0*SH, "T1 slot0 base");
    chk(rec_base[1] == SBASE + 1*SH, "T1 slot1 base");
    chk(rec_base[2] == SBASE + 2*SH, "T1 slot2 base");
    chk(rec_base[3] == SBASE + 3*SH, "T1 slot3 base");
    chk(rec_off[0] == 32'h1000 && rec_off[1] == 32'h2000 &&
        rec_off[2] == 32'h3000 && rec_off[3] == 32'h4000, "T1 offsets in order");

    //======================================================================
    $display("[T2] item2 errors");
    n_load = 0; err_idx = 2; valid_cnt = 4; slot_valid = 4'hF;
    run_page(3'd4);
    $display("      n_load=%0d loaded=%0d", n_load, loaded);
    chk(n_load == 4,   "T2 still issued 4 (no deadlock)");
    chk(loaded == 3,   "T2 loaded == 3 (one skipped)");

    //======================================================================
    $display("[T3] only 2 items");
    n_load = 0; err_idx = -1; valid_cnt = 2; slot_valid = 4'b0011;
    run_page(3'd2);
    $display("      n_load=%0d loaded=%0d", n_load, loaded);
    chk(n_load == 2,   "T3 only 2 loaded");
    chk(loaded == 2,   "T3 loaded == 2");

    //======================================================================
    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #2_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
