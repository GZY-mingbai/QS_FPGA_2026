//==============================================================================
//  top.v —— FlashProgrammer 顶层
//==============================================================================
//
//  【系统构成】
//    上位机 ──USB(CH340)──▶ uart_rx ──▶ cmd_parser ──▶ spi_flash_ctrl ──▶ W25Q64
//                  ◀──────── uart_tx ◀──┘
//
//  【时钟】直接用板上 50MHz 晶振（引脚 R7），不用 PLL。
//    串口波特率由 CLK_FREQ/BAUD 分频得到，SPI 时钟由 SCLK_DIV 分频得到。
//
//  【复位】两路"或"在一起，任一有效就复位：
//    · 外部按键 rst_n（引脚 A2，按下为低）
//    · 内部上电复位计数器（约 0.65ms，等 FPGA 配置完成、时钟稳定）
//
//  【引脚】（详见 src/pin.adc）
//    clk = R7     uart_rx = F12   uart_tx = D12
//    flash: cs=P8  sclk=M9  mosi=P7  miso=N8  wp=P9  hold=R9
//
//==============================================================================

module top (
    input  wire clk,            // 50MHz 晶振      R7
    input  wire rst_n,          // 复位按键，低有效 A2
    input  wire uart_rx,        // 串口输入         F12
    output wire uart_tx,        // 串口输出         D12

    // Flash 物理接口
    output wire flash_cs_n,     // P8
    output wire flash_sclk,     // M9
    output wire flash_mosi,     // P7
    input  wire flash_miso,     // N8
    output wire flash_wp_n,     // P9
    output wire flash_hold_n    // R9
);

//==============================================================================
//  参数
//==============================================================================
parameter CLK_FREQ = 50000000;      // 50MHz
parameter BAUD     = 921600;        // 串口波特率
parameter SCLK_DIV = 2;             // SPI 时钟 = 50M/(2*2) = 12.5MHz

//==============================================================================
//  复位：上电复位 + 外部按键
//==============================================================================
reg [15:0] por_cnt;
wire       por_done = por_cnt[15];          // 约 0.65ms

always @(posedge clk) begin
    if (!por_done)
        por_cnt <= por_cnt + 16'd1;
end

wire rst_n_all = rst_n & por_done;

// 打两拍同步（去亚稳态）
reg [1:0] rst_sync;
always @(posedge clk) begin
    rst_sync <= {rst_sync[0], rst_n_all};
end
wire rstn = rst_sync[1];

//==============================================================================
//  UART 接收
//==============================================================================
wire        rx_tvalid;
wire [7:0]  rx_tdata;
wire        rx_tready;
wire        rx_overflow;

uart_rx #(
    .CLK_FREQ  (CLK_FREQ),
    .BAUD_RATE (BAUD),
    .PARITY    ("NONE"),
    .FIFO_EA   (0)
) u_rx (
    .rstn       (rstn),
    .clk        (clk),
    .i_uart_rx  (uart_rx),
    .o_tready   (rx_tready),
    .o_tvalid   (rx_tvalid),
    .o_tdata    (rx_tdata),
    .o_overflow (rx_overflow)
);

//==============================================================================
//  UART 发送
//==============================================================================
wire        tx_tvalid;
wire [7:0]  tx_tdata;
wire        tx_tready;

uart_tx #(
    .CLK_FREQ   (CLK_FREQ),
    .BAUD_RATE  (BAUD),
    .PARITY     ("NONE"),
    .STOP_BITS  (1),            // ★ 必须显式设 1，模块默认是 2
    .BYTE_WIDTH (1),
    .FIFO_EA    (0)
) u_tx (
    .rstn      (rstn),
    .clk       (clk),
    .i_tready  (tx_tready),
    .i_tvalid  (tx_tvalid),
    .i_tdata   (tx_tdata),
    .i_tkeep   (1'b1),
    .i_tlast   (1'b0),
    .o_uart_tx (uart_tx)
);

//==============================================================================
//  命令解析器
//==============================================================================
wire [2:0]  fl_op;
wire        fl_start;
wire [23:0] fl_addr;
wire [15:0] fl_len;
wire        fl_busy;
wire        fl_done;
wire [7:0]  fl_rdata;
wire        fl_rvalid;
wire [11:0] fl_waddr;
wire [7:0]  fl_wdata;
wire [7:0]  fl_sr;
wire [3:0]  led_state;      // 状态灯（暂不接引脚，留作调试）

cmd_parser #(
    .CLK_FREQ (CLK_FREQ),
    .BUF_AW   (12)              // 4KB
) u_cmd (
    .clk        (clk),
    .rst_n      (rstn),
    .rx_tvalid  (rx_tvalid),
    .rx_tdata   (rx_tdata),
    .rx_tready  (rx_tready),
    .tx_tvalid  (tx_tvalid),
    .tx_tdata   (tx_tdata),
    .tx_tready  (tx_tready),
    .fl_op      (fl_op),
    .fl_start   (fl_start),
    .fl_addr    (fl_addr),
    .fl_len     (fl_len),
    .fl_busy    (fl_busy),
    .fl_done    (fl_done),
    .fl_rdata   (fl_rdata),
    .fl_rvalid  (fl_rvalid),
    .fl_waddr   (fl_waddr),
    .fl_wdata   (fl_wdata),
    .fl_sr      (fl_sr),
    .led_state  (led_state)
);

//==============================================================================
//  Flash 控制器
//==============================================================================
spi_flash_ctrl #(
    .CLK_FREQ (CLK_FREQ),
    .SCLK_DIV (SCLK_DIV)
) u_flash (
    .clk          (clk),
    .rst_n        (rstn),
    .op           (fl_op),
    .start        (fl_start),
    .addr         (fl_addr),
    .len          (fl_len),
    .busy         (fl_busy),
    .done         (fl_done),
    .rdata        (fl_rdata),
    .rvalid       (fl_rvalid),
    .waddr        (fl_waddr),
    .wdata        (fl_wdata),
    .sr_val       (fl_sr),
    .flash_cs_n   (flash_cs_n),
    .flash_sclk   (flash_sclk),
    .flash_mosi   (flash_mosi),
    .flash_miso   (flash_miso),
    .flash_wp_n   (flash_wp_n),
    .flash_hold_n (flash_hold_n)
);

endmodule
