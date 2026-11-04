//==============================================================================
//  tb_sd_bus_arbiter4.v —— 4 客户仲裁器单元测试
//==============================================================================
//  iverilog -g2012 -o arb4.vvp -s tb_sd_bus_arbiter4 \
//      tb_sd_bus_arbiter4.v sd_bus_arbiter4.v && vvp arb4.vvp
//
//  注意：一个扇区 512 字节 = 512 拍，任何"等几拍再看"的写法都不够，
//        一律用 wait(sd_read) / wait(xxx_end) 事件等待。
//==============================================================================
`timescale 1ns/1ps

module tb_sd_bus_arbiter4;

reg clk = 0;
reg rst = 1;
reg init_done = 0;

reg         img_req = 0, aud_req = 0, aud_urg = 0, fat_req = 0, txt_req = 0;
reg  [31:0] img_addr = 0, aud_addr = 0, fat_addr = 0, txt_addr = 0;
wire [7:0]  img_d, aud_d, fat_d, txt_d;
wire img_dv, aud_dv, fat_dv, txt_dv;
wire img_end, aud_end, fat_end, txt_end;

wire        sd_read;
wire [31:0] sd_addr;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0, sd_end = 0;

wire        to;
wire [3:0]  owner;

integer errors = 0;
integer ncase  = 0;

task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("  FAIL [%0d] %0s", ncase, msg);
        end
    end
endtask

sd_bus_arbiter4 #(
    .TIMEOUT_CYC (1000),
    .AUD_GAP_CYC (20),
    .FAT_QUOTA   (3),
    .ENA_TXT     (1)
) dut (
    .I_clk(clk), .I_rst(rst), .I_sd_init_done(init_done),

    .I_img_req(img_req), .I_img_addr(img_addr),
    .O_img_data(img_d), .O_img_dv(img_dv), .O_img_end(img_end),

    .I_aud_req(aud_req), .I_aud_addr(aud_addr), .I_aud_urgent(aud_urg),
    .O_aud_data(aud_d), .O_aud_dv(aud_dv), .O_aud_end(aud_end),

    .I_fat_req(fat_req), .I_fat_addr(fat_addr),
    .O_fat_data(fat_d), .O_fat_dv(fat_dv), .O_fat_end(fat_end),

    .I_txt_req(txt_req), .I_txt_addr(txt_addr),
    .O_txt_data(txt_d), .O_txt_dv(txt_dv), .O_txt_end(txt_end),

    .O_sd_sec_read(sd_read), .O_sd_sec_addr(sd_addr),
    .I_sd_sec_data(sd_data), .I_sd_sec_dv(sd_dv), .I_sd_sec_end(sd_end),

    .O_timeout(to), .O_owner(owner)
);

always #10 clk = ~clk;

//------------------------------------------------------------------------------
//  SD 卡行为模型
//------------------------------------------------------------------------------
reg  [31:0] lat_addr;
reg         reading = 0;
reg  [3:0]  wait_cnt = 0;
integer     bc;
reg         no_resp = 0;

always @(posedge clk) begin
    sd_dv  <= 0;
    sd_end <= 0;
    if (rst) begin
        reading  <= 0;
        wait_cnt <= 0;
    end else if (!reading) begin
        if (sd_read && !no_resp) begin
            lat_addr <= sd_addr;
            reading  <= 1;
            wait_cnt <= 4;
            bc       <= 0;
        end
    end else begin
        if (wait_cnt != 0)
            wait_cnt <= wait_cnt - 1;
        else if (bc < 512) begin
            sd_dv   <= 1;
            sd_data <= bc[7:0];
            bc      <= bc + 1;
        end else begin
            sd_end  <= 1;
            reading <= 0;
        end
    end
end

// 每个客户收到的字节数
integer img_bytes = 0, aud_bytes = 0, fat_bytes = 0, txt_bytes = 0;
always @(posedge clk) begin
    if (img_dv) img_bytes <= img_bytes + 1;
    if (aud_dv) aud_bytes <= aud_bytes + 1;
    if (fat_dv) fat_bytes <= fat_bytes + 1;
    if (txt_dv) txt_bytes <= txt_bytes + 1;
end

//------------------------------------------------------------------------------
//  A_GAP 监控：sd_read 每次上升沿前，必须有一拍低电平
//------------------------------------------------------------------------------
reg  sd_read_q = 0;
reg  gap_ok = 1;
integer low_run = 0;
always @(posedge clk) begin
    sd_read_q <= sd_read;
    if (!sd_read) low_run <= low_run + 1;
    else          low_run <= 0;
    if (sd_read && !sd_read_q && low_run == 0 && !rst)
        gap_ok <= 0;
end

//------------------------------------------------------------------------------
//  事件等待（带超时）
//------------------------------------------------------------------------------
task wait_img_end;
    begin : w_img
        integer t;
        t = 0;
        while (!img_end && t < 20000) begin @(posedge clk); t = t + 1; end
        if (t >= 20000) begin errors = errors + 1; $display("  FAIL timeout img_end"); end
    end
endtask

task wait_fat_end;
    begin : w_fat
        integer t;
        t = 0;
        while (!fat_end && t < 20000) begin @(posedge clk); t = t + 1; end
        if (t >= 20000) begin errors = errors + 1; $display("  FAIL timeout fat_end"); end
    end
endtask

//------------------------------------------------------------------------------
initial begin
    $dumpfile("tb_arb4.vcd");
    $dumpvars(0, tb_sd_bus_arbiter4);

    repeat (5) @(posedge clk);
    rst = 0;
    init_done = 0;
    repeat (2) @(posedge clk);
    init_done = 1;
    repeat (2) @(posedge clk);

    //======================================================================
    $display("\n[T1] single client IMG");
    img_addr = 32'h0000_1234;
    img_req  = 1;
    wait (sd_read);  #1;
    chk(sd_addr == 32'h0000_1234, "T1 addr == img_addr");
    wait_img_end;
    img_req = 0;                    // end 后必须立刻拉低，否则会重读同一扇区
    repeat (4) @(posedge clk);
    $display("      dbg: img_bytes=%0d", img_bytes);
    chk(img_bytes == 512, "T1 IMG got 512 bytes");
    chk(aud_bytes == 0 && fat_bytes == 0 && txt_bytes == 0, "T1 others got nothing");
    repeat (6) @(posedge clk);

    //======================================================================
    $display("[T2] priority IMG > FAT");
    img_addr = 32'h0000_2000;
    fat_addr = 32'h0000_3000;
    img_req  = 1;
    fat_req  = 1;
    wait (sd_read); repeat (3) @(posedge clk);
    $display("      dbg T2: sd_addr=%h expect=%h owner=%b", sd_addr, img_addr, owner); chk(sd_addr == 32'h0000_2000, "T2 first grant is IMG");
    wait_img_end;
    img_req = 0;                    // end 后必须立刻拉低，否则会重读同一扇区
    repeat (4) @(posedge clk);
    wait (sd_read); repeat (3) @(posedge clk);
    chk(sd_addr == 32'h0000_3000, "T2 FAT takes over after IMG");
    fat_req = 0;
    repeat (4) @(posedge clk);
    // 让当前扇区跑完
    repeat (600) @(posedge clk);

    //======================================================================
    $display("[T3] AUD urgent preempts FAT");
    aud_bytes = 0; fat_bytes = 0;
    fat_addr = 32'h0000_4000;
    aud_addr = 32'h0000_5000;
    fat_req  = 1;
    aud_req  = 1;
    aud_urg  = 1;
    wait (sd_read); repeat (3) @(posedge clk);
    chk(sd_addr == 32'h0000_5000, "T3 urgent AUD wins");
    repeat (600) @(posedge clk);
    aud_urg = 0;
    aud_req = 0;
    repeat (600) @(posedge clk);
    fat_req = 0;
    repeat (20) @(posedge clk);

    //======================================================================
    $display("[T4] FAT quota does not deadlock");
    fat_bytes = 0;
    fat_addr  = 32'h0000_6000;
    fat_req   = 1;
    repeat (6000) @(posedge clk);
    chk(fat_bytes > 512*5, "T4 FAT alone keeps running");
    fat_req = 0;
    repeat (600) @(posedge clk);

    //======================================================================
    $display("[T5] A_GAP");
    chk(gap_ok == 1, "T5 req must drop >=1 cycle between clients");

    //======================================================================
    $display("[T6] timeout fallback");
    no_resp  = 1;
    txt_addr = 32'h0000_7000;
    txt_req  = 1;
    repeat (2000) @(posedge clk);
    chk(to == 1, "T6 O_timeout set when card silent");
    txt_req = 0;
    no_resp = 0;
    wait (owner == 4'b0000);        // 等第二次 TXT 超时并回 IDLE
    repeat (4) @(posedge clk);

    //======================================================================
    $display("[T7] TXT lowest priority");
    txt_addr = 32'h0000_8000;
    fat_addr = 32'h0000_9000;
    txt_req  = 1;
    fat_req  = 1;
    wait (sd_read); repeat (3) @(posedge clk);
    chk(sd_addr == 32'h0000_9000, "T7 FAT before TXT");
    txt_req = 0;
    fat_req = 0;
    repeat (600) @(posedge clk);

    //======================================================================
    $display("\n========================================");
    if (errors == 0)
        $display("  ALL PASS  (%0d checks)", ncase);
    else
        $display("  FAIL: %0d / %0d checks", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #20_000_000;
    $display("  FAIL: simulation watchdog timeout");
    $finish;
end

endmodule
