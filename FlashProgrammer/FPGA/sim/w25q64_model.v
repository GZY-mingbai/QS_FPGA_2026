//==============================================================================
//  w25q64_model.v —— W25Q64 SPI Flash 行为级模型（仅用于仿真）
//==============================================================================
//
//  支持的命令：
//    9Fh  读 JEDEC ID       → EF 40 17（W25Q64）
//    03h  读数据
//    02h  页编程（页内回绕，不跨页 —— 真实芯片行为）
//    06h  写使能
//    05h  读状态（WIP 会保持忙 PROG_NS / ERASE_NS，不是立刻完成）
//    C7h  整片擦除
//
//  SPI Mode 0：上升沿采样 MOSI，下降沿改变 MISO。
//  模型只模拟 64KB（MEM_BYTES），足够验证逻辑；地址取低 16 位。
//
//  【重要】WIP 必须真实保持一段时间，不能恒为 0。
//  真实 W25Q64 页编程要 ~0.7ms，控制器在这段时间里会**反复轮询状态寄存器**
//  （一次轮询 = 05h 命令 + 1 字节状态）。轮询次数是上位机看不到的"隐藏流量"，
//  如果 DUT 把轮询读回的状态字节混进数据通路，只有 WIP 真的保持忙才暴露得出来。
//  这里 PROG_NS 默认 700us，正好复现真实芯片的轮询次数（~280 次）。
//
//==============================================================================
`timescale 1ns/1ps

module w25q64_model #(
    parameter MEM_BYTES = 65536,
    parameter PROG_NS   = 700000,      // 页编程耗时：W25Q64 典型 0.7ms
    parameter ERASE_NS  = 1000000      // 整片擦除：真实 20~100s，仿真取 1ms 够用
)(
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output reg  miso
);

    reg [7:0] mem [0:MEM_BYTES-1];

    reg [7:0]  sr_in;        // MOSI 移位输入
    reg [7:0]  sr_out;       // MISO 移位输出
    reg [3:0]  bitcnt;       // 位计数 0..8
    reg [1:0]  phase;        // 0=命令 1=地址 2=数据 3=忽略
    reg [7:0]  cmd;
    reg [23:0] addr;
    reg [1:0]  addr_cnt;
    reg [2:0]  id_cnt;

    time       busy_until;   // WIP 保持到这个时刻

    integer i;

    initial begin
        for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = 8'hFF;
        sr_in = 0; sr_out = 0; bitcnt = 0; phase = 0;
        cmd = 0; addr = 0; addr_cnt = 0; id_cnt = 0;
        miso = 1'b1;
        busy_until = 0;
    end

    //------------------------------------------------------------------
    //  上升沿：采样 MOSI，按字节推进
    //------------------------------------------------------------------
    always @(posedge sclk) begin
        if (!cs_n) begin
            sr_in  = {sr_in[6:0], mosi};
            bitcnt = bitcnt + 4'd1;
            if (bitcnt == 4'd8) begin
                bitcnt = 4'd0;
                case (phase)
                    //---------- 命令字节 ----------
                    2'd0: begin
                        cmd = sr_in;
                        case (sr_in)
                            8'h9F: begin phase = 2'd2; sr_out = 8'hEF; id_cnt = 0; end
                            8'h03: begin phase = 2'd1; addr_cnt = 0; addr = 24'd0; end
                            8'h02: begin phase = 2'd1; addr_cnt = 0; addr = 24'd0; end
                            8'h06: begin phase = 2'd3; end
                            // 状态寄存器：bit0=WIP。忙时返回 03h（WIP|WEL），闲时 00h
                            8'h05: begin
                                phase  = 2'd2;
                                sr_out = ($time < busy_until) ? 8'h03 : 8'h00;
                            end
                            8'hC7: begin phase = 2'd3; end     // 擦除在 CS 拉高后才开始
                            default: phase = 2'd3;
                        endcase
                    end
                    //---------- 地址（3 字节）----------
                    2'd1: begin
                        addr = {addr[15:0], sr_in};
                        if (addr_cnt == 2'd2) begin      // 收满 3 字节地址
                            phase = 2'd2;
                            if (cmd == 8'h03) begin
                                sr_out = mem[addr[15:0]];
                                addr   = addr + 24'd1;
                            end else begin
                                sr_out = 8'h00;
                            end
                        end else begin
                            addr_cnt = addr_cnt + 2'd1;
                        end
                    end
                    //---------- 数据 ----------
                    2'd2: begin
                        if (cmd == 8'h02) begin
                            mem[addr[15:0]] = sr_in;
                            // 页编程在 256 字节页内回绕，绝不跨页（真实芯片行为）。
                            // 若控制器多发了一个字节，就会覆盖本页开头 —— 这条能让
                            // "页边界算错" 类 bug 直接暴露，而不是被静默吞掉。
                            addr = {addr[23:8], addr[7:0] + 8'd1};
                        end else if (cmd == 8'h03) begin
                            sr_out = mem[addr[15:0]];
                            addr   = addr + 24'd1;
                        end else if (cmd == 8'h9F) begin
                            id_cnt = id_cnt + 3'd1;
                            case (id_cnt)
                                3'd1:    sr_out = 8'h40;
                                3'd2:    sr_out = 8'h17;
                                default: sr_out = 8'hFF;
                            endcase
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    //------------------------------------------------------------------
    //  下降沿：改变 MISO
    //------------------------------------------------------------------
    always @(negedge sclk) begin
        if (!cs_n) begin
            if (phase == 2'd2) begin
                miso   = sr_out[7];
                sr_out = {sr_out[6:0], 1'b0};
            end else begin
                miso = 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    //  CS 上升沿：结束事务。编程/擦除在这里才真正开始计时（芯片内部动作）
    //------------------------------------------------------------------
    always @(posedge cs_n) begin
        if (cmd == 8'h02) begin
            busy_until = $time + PROG_NS;               // 页编程
        end else if (cmd == 8'hC7) begin
            busy_until = $time + ERASE_NS;              // 整片擦除
            for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = 8'hFF;
        end
        bitcnt   = 4'd0;
        phase    = 2'd0;
        addr_cnt = 2'd0;
    end

endmodule
