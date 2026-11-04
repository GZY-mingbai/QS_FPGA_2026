//==============================================================================
//  tb_flash_programmer.v —— 端到端仿真
//==============================================================================
//
//  模拟上位机（flash_tool.py）：按协议组帧 → 通过 UART 发给 FPGA →
//  收响应 → 校验。Flash 用 w25q64_model 行为模型。
//
//  跑法（在 sim/ 目录下）：
//    iverilog -g2012 -o tb.out tb_flash_programmer.v w25q64_model.v \
//             ../src/top.v ../src/cmd_parser.v ../src/spi_flash_ctrl.v \
//             ../src/uart_rx.v ../src/uart_tx.v
//    vvp tb.out
//
//==============================================================================
`timescale 1ns/1ps

module tb_flash_programmer;

//==============================================================================
//  时钟 / 复位 / 连线
//==============================================================================
reg  clk = 1'b0;
always #10 clk = ~clk;              // 50MHz

reg  rst_n        = 1'b0;
reg  uart_rx_line = 1'b1;
wire uart_tx_line;

wire flash_cs_n, flash_sclk, flash_mosi, flash_wp_n, flash_hold_n;
wire flash_miso;

top u_top (
    .clk          (clk),
    .rst_n        (rst_n),
    .uart_rx      (uart_rx_line),
    .uart_tx      (uart_tx_line),
    .flash_cs_n   (flash_cs_n),
    .flash_sclk   (flash_sclk),
    .flash_mosi   (flash_mosi),
    .flash_miso   (flash_miso),
    .flash_wp_n   (flash_wp_n),
    .flash_hold_n (flash_hold_n)
);

w25q64_model u_flash (
    .cs_n (flash_cs_n),
    .sclk (flash_sclk),
    .mosi (flash_mosi),
    .miso (flash_miso)
);

//==============================================================================
//  UART 收发（921600 8N1）
//==============================================================================
localparam BIT_NS = 1085;           // 1/921600 s ≈ 1085.07 ns

task uart_send(input [7:0] b);
    integer i;
    begin
        uart_rx_line = 1'b0;        // 起始位
        #BIT_NS;
        for (i = 0; i < 8; i = i + 1) begin
            uart_rx_line = b[i];
            #BIT_NS;
        end
        uart_rx_line = 1'b1;        // 停止位
        #BIT_NS;
    end
endtask

//------------------------------------------------------------------
//  后台 UART 接收器
//  DUT 收完帧后可能"立刻"开始回响应；若主进程等发完 stop 位才去等下降沿，
//  就会漏掉起始位。所以这里用常驻 always 块连续接收，主进程只从队列取。
//  位定时：50MHz / 921600 ≈ 54.25 拍，取 54 拍。
//------------------------------------------------------------------
localparam RX_IDLE  = 2'd0,
           RX_START = 2'd1,
           RX_DATA  = 2'd2,
           RX_STOP  = 2'd3;

reg  [1:0] rxst = RX_IDLE;
reg  [6:0] rxt  = 7'd0;
reg  [3:0] rxn  = 4'd0;
reg  [7:0] rxs  = 8'd0;
reg        tx_prev = 1'b1;

reg  [7:0] rxq [0:65535];
integer    rxqw = 0;        // 写指针（后台块）
integer    rxqr = 0;        // 读指针（主进程）

always @(posedge clk) begin
    tx_prev <= uart_tx_line;
    if (!u_top.rstn) begin
        // 复位期间不接收，避免把 uart_tx 的复位电平当成起始位
        rxst <= RX_IDLE;
        rxt  <= 7'd0;
        rxn  <= 4'd0;
        rxs  <= 8'd0;
    end else begin
        case (rxst)
            RX_IDLE: if (tx_prev == 1'b1 && uart_tx_line == 1'b0) begin
                         rxst <= RX_START;
                         rxt  <= 7'd81;                 // 1.5 位 → bit0 中心
                     end
            RX_START: if (rxt == 7'd0) begin
                          rxs  <= {uart_tx_line, rxs[7:1]};   // 采 bit0
                          rxn  <= 4'd1;
                          rxt  <= 7'd54;
                          rxst <= RX_DATA;
                      end else rxt <= rxt - 7'd1;
            RX_DATA: if (rxt == 7'd0) begin
                         rxs <= {uart_tx_line, rxs[7:1]};
                         if (rxn == 4'd7) begin
                             rxst <= RX_STOP;
                             rxt  <= 7'd27;        // 半位后检查 stop 位
                         end else begin
                             rxn  <= rxn + 4'd1;
                             rxt  <= 7'd54;
                         end
                     end else rxt <= rxt - 7'd1;
            RX_STOP: if (rxt == 7'd0) begin
                         // 不用固定拍数等 stop 位（会累积误差、漏掉下一字节起始位），
                         // 而是等线真正变高（进入 stop 位）就收工。
                         if (uart_tx_line == 1'b1) begin
                             rxq[rxqw] = rxs;
                             rxqw = rxqw + 1;
                             rxst <= RX_IDLE;
                         end
                     end else rxt <= rxt - 7'd1;
        endcase
    end
end

//==============================================================================
//  CRC-16/CCITT-FALSE
//==============================================================================
function [15:0] crc_step;
    input [15:0] c;
    input [7:0]  d;
    reg   [15:0] cc;
    integer k;
    begin
        cc = c ^ {d, 8'h00};
        for (k = 0; k < 8; k = k + 1)
            cc = cc[15] ? ((cc << 1) ^ 16'h1021) : (cc << 1);
        crc_step = cc;
    end
endfunction

//==============================================================================
//  帧构造 / 收发
//==============================================================================
reg [7:0] txb [0:8191];
reg [7:0] rxb [0:8191];
integer   txn;
integer   rlen;
integer   rstatus;

task build_req(input [7:0] cmd, input [31:0] addr, input [31:0] len);
    begin
        txb[0]  = 8'hAA;
        txb[1]  = 8'h55;
        txb[2]  = cmd;
        txb[3]  = addr[7:0];
        txb[4]  = addr[15:8];
        txb[5]  = addr[23:16];
        txb[6]  = addr[31:24];
        txb[7]  = len[7:0];
        txb[8]  = len[15:8];
        txb[9]  = len[23:16];
        txb[10] = len[31:24];
        txn     = 11;
    end
endtask

task put_data(input [7:0] b);
    begin
        txb[txn] = b;
        txn = txn + 1;
    end
endtask

task finish_req;
    integer k;
    reg [15:0] c;
    begin
        c = 16'hFFFF;
        for (k = 2; k < txn; k = k + 1)
            c = crc_step(c, txb[k]);
        txb[txn]     = c[7:0];
        txb[txn + 1] = c[15:8];
        txn = txn + 2;
    end
endtask

task send_frame;
    integer k;
    begin
        for (k = 0; k < txn; k = k + 1)
            uart_send(txb[k]);
    end
endtask

task recv_resp;
    integer k;
    begin
        // 帧头 AA 55
        while (rxqw - rxqr < 2) #1000;
        if (rxq[rxqr]     !== 8'hAA) $display("  [SYNC] 期望 AA，收到 %02X", rxq[rxqr]);
        if (rxq[rxqr + 1] !== 8'h55) $display("  [SYNC] 期望 55，收到 %02X", rxq[rxqr + 1]);
        rxqr = rxqr + 2;
        // STATUS + LEN(4,小端)
        while (rxqw - rxqr < 5) #1000;
        rstatus = rxq[rxqr];
        rlen = rxq[rxqr+1] | (rxq[rxqr+2] << 8) | (rxq[rxqr+3] << 16) | (rxq[rxqr+4] << 24);
        rxqr = rxqr + 5;
        // DATA + CRC(2)
        while (rxqw - rxqr < rlen + 2) #1000;
        for (k = 0; k < rlen; k = k + 1)
            rxb[k] = rxq[rxqr + k];
        rxqr = rxqr + rlen + 2;
    end
endtask

//==============================================================================
//  测试
//==============================================================================
reg [7:0] pat [0:4095];
integer   errs, rd_err, j;

initial begin
    errs = 0;

    // 复位
    rst_n = 1'b0;
    uart_rx_line = 1'b1;
    #2000;
    rst_n = 1'b1;
    #700000;                        // 等上电复位完成（~0.65ms）

    //--------------------------------------------------------------
    $display("=== Test1: PING ===");
    build_req(8'h05, 32'd0, 32'd0); finish_req; send_frame; recv_resp;
    if (rstatus == 0 && rlen == 1 && rxb[0] == 8'h55)
        $display("  [ OK ] 回环 0x55");
    else begin
        $display("  [FAIL] status=%0d len=%0d data=%02X", rstatus, rlen, rxb[0]);
        errs = errs + 1;
    end

    $display("=== Test2: READ_ID ===");
    build_req(8'h01, 32'd0, 32'd0); finish_req; send_frame; recv_resp;
    if (rstatus == 0 && rlen == 3)
        $display("  [ OK ] ID = %02X %02X %02X", rxb[0], rxb[1], rxb[2]);
    else begin
        $display("  [FAIL] status=%0d len=%0d", rstatus, rlen);
        errs = errs + 1;
    end

    //--------------------------------------------------------------
    $display("=== Test3: CHIP_ERASE ===");
    build_req(8'h04, 32'd0, 32'd0); finish_req; send_frame; recv_resp;
    if (rstatus == 0)
        $display("  [ OK ] 擦除完成");
    else begin
        $display("  [FAIL] status=%0d", rstatus);
        errs = errs + 1;
    end

    //--------------------------------------------------------------
    $display("=== Test4: WRITE 512 字节 @0x0000（跨 2 页）===");
    for (j = 0; j < 512; j = j + 1)
        pat[j] = (j * 7 + 3) & 8'hFF;
    build_req(8'h03, 32'h00000000, 32'd512);
    for (j = 0; j < 512; j = j + 1)
        put_data(pat[j]);
    finish_req; send_frame; recv_resp;
    if (rstatus == 0)
        $display("  [ OK ] 写入完成");
    else begin
        $display("  [FAIL] status=%0d", rstatus);
        errs = errs + 1;
    end

    //--------------------------------------------------------------
    $display("=== Test5: READ 512 字节 @0x0000 ===");
    build_req(8'h02, 32'h00000000, 32'd512); finish_req; send_frame; recv_resp;
    if (rstatus != 0) begin
        $display("  [FAIL] status=%0d", rstatus);
        errs = errs + 1;
    end else if (rlen != 512) begin
        $display("  [FAIL] 长度 %0d != 512", rlen);
        errs = errs + 1;
    end else begin
        rd_err = 0;
        for (j = 0; j < 512; j = j + 1)
            if (rxb[j] !== pat[j]) begin
                if (rd_err < 8)
                    $display("    差异 @%0d: 读回 %02X 期望 %02X", j, rxb[j], pat[j]);
                rd_err = rd_err + 1;
            end
        if (rd_err == 0)
            $display("  [ OK ] 512 字节全部匹配");
        else begin
            $display("  [FAIL] %0d 字节不符", rd_err);
            errs = errs + 1;
        end
    end

    //--------------------------------------------------------------
    $display("=== Test6: 非对齐地址写读（0x0100，验证预取偏移）===");
    for (j = 0; j < 300; j = j + 1)
        pat[j] = (j ^ 8'hA5) & 8'hFF;
    build_req(8'h03, 32'h00000100, 32'd300);
    for (j = 0; j < 300; j = j + 1)
        put_data(pat[j]);
    finish_req; send_frame; recv_resp;
    build_req(8'h02, 32'h00000100, 32'd300); finish_req; send_frame; recv_resp;
    if (rstatus != 0 || rlen != 300) begin
        $display("  [FAIL] status=%0d len=%0d", rstatus, rlen);
        errs = errs + 1;
    end else begin
        rd_err = 0;
        for (j = 0; j < 300; j = j + 1)
            if (rxb[j] !== pat[j]) rd_err = rd_err + 1;
        if (rd_err == 0)
            $display("  [ OK ] 300 字节全部匹配");
        else begin
            $display("  [FAIL] %0d 字节不符", rd_err);
            errs = errs + 1;
        end
    end

    //--------------------------------------------------------------
    $display("=== Test7: READ len=0 应被拒绝（防 16 位下溢）===");
    build_req(8'h02, 32'h00000000, 32'd0); finish_req; send_frame; recv_resp;
    if (rstatus == 8'h03)                       // ST_BAD_LEN
        $display("  [ OK ] 正确返回 BAD_LEN(3)");
    else begin
        $display("  [FAIL] status=%0d（期望 3）", rstatus);
        errs = errs + 1;
    end

    //--------------------------------------------------------------
    $display("=== Test8: WRITE len=0 应被拒绝 ===");
    build_req(8'h03, 32'h00000000, 32'd0); finish_req; send_frame; recv_resp;
    if (rstatus == 8'h03)
        $display("  [ OK ] 正确返回 BAD_LEN(3)");
    else begin
        $display("  [FAIL] status=%0d（期望 3）", rstatus);
        errs = errs + 1;
    end

    //--------------------------------------------------------------
    $display("=== Test9: WRITE/READ 4096 字节（整块边界，最容易踩的坑）===");
    for (j = 0; j < 4096; j = j + 1)
        pat[j] = (j * 13 + 5) & 8'hFF;
    build_req(8'h03, 32'h00002000, 32'd4096);
    for (j = 0; j < 4096; j = j + 1)
        put_data(pat[j]);
    finish_req; send_frame; recv_resp;
    if (rstatus != 0) begin
        $display("  [FAIL] 写入 status=%0d", rstatus);
        errs = errs + 1;
    end else begin
        $display("  [ OK ] 写入完成");
        build_req(8'h02, 32'h00002000, 32'd4096); finish_req; send_frame; recv_resp;
        if (rstatus != 0 || rlen != 4096) begin
            $display("  [FAIL] 读回 status=%0d len=%0d", rstatus, rlen);
            errs = errs + 1;
        end else begin
            rd_err = 0;
            for (j = 0; j < 4096; j = j + 1)
                if (rxb[j] !== pat[j]) begin
                    if (rd_err < 4)
                        $display("    差异 @%0d: 读回 %02X 期望 %02X", j, rxb[j], pat[j]);
                    rd_err = rd_err + 1;
                end
            if (rd_err == 0)
                $display("  [ OK ] 4096 字节全部匹配");
            else begin
                $display("  [FAIL] %0d 字节不符", rd_err);
                errs = errs + 1;
            end
        end
    end

    //--------------------------------------------------------------
    if (errs == 0) $display("\n==== 全部测试通过 ====");
    else           $display("\n==== %0d 项失败 ====", errs);
    $finish;
end

// 超时保护
initial begin
    #150000000;                     // 150ms
    $display("\n[超时] 仿真未在 150ms 内结束");
    $finish;
end

endmodule
