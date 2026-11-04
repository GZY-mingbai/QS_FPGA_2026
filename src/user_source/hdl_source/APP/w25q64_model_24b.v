//==============================================================================
//  w25q64_model_24b.v —— W25Q64 SPI Flash 行为级模型（24 位寻址版，仅用于仿真）
//==============================================================================
//
//  【为什么有这个文件】
//  上游 FlashProgrammer 工程里那份 `FlashProgrammer/FPGA/sim/w25q64_model.v`
//  只在**低 16 位**寻址（`mem[addr[15:0]]`，只模拟 64KB），而字库
//  `font_gb2312.bin` 有 **247,744 字节（0x3C7C0 ≈ 242KB）**，超出 64KB 装不下。
//  本文件是那份模型的分支：把寻址放宽到 **24 位**、容量按 MEM_BYTES 自动定址，
//  其余行为（命令集、SPI Mode 0 时序、页内回绕、WIP 保持）**逐字保持一致**。
//  这样就不必去改动另一个工程里共享的资产。
//
//  【支持的命令】（与上游一致）
//    9Fh  读 JEDEC ID       → EF 40 17（W25Q64）
//    03h  读数据（地址自动递增，可跨页）
//    02h  页编程（页内回绕，不跨页 —— 真实芯片行为）
//    06h  写使能
//    05h  读状态（WIP 会保持忙 PROG_NS / ERASE_NS，不是立刻完成）
//    C7h  整片擦除
//
//  SPI Mode 0：上升沿采样 MOSI，下降沿改变 MISO。
//
//  【预置数据】
//  用 INIT_HEX 参数指定一个 hex 文件（路径相对**仿真工作目录**），上电时灌进去；
//  参数留空则整片都是 0xFF（= 没写过的 Flash 扇区）。
//      w25q64_model_24b #(.MEM_BYTES(262144),
//                         .INIT_HEX ("tools/out/font256k.hex")) u_flash (...);
//  ★ 不要在 tb 里用 `$readmemh("...", u_flash.mem)` 这种**跨模块层次引用**：
//    iverilog 认，但综合工具的 elaborator 不认（HDL-8007: 'mem' is not declared
//    under prefix 'u_flash'），而这个文件一旦被登记进 TD 工程就会把综合搞挂。
//    把灌数据放进模型自己的 initial 里，还顺带免掉了与"填 0xFF"的竞态。
//
//==============================================================================

`timescale 1ns/1ps

module w25q64_model_24b #(
    parameter MEM_BYTES = 262144,      // 256KB，够装 242KB 字库
    parameter INIT_HEX  = "",          // 非空：上电时从该 hex 文件预置（相对仿真工作目录）
    // ★ 2026-10-27：第二段预置区（图标）。
    //   【为什么需要】以前仿真只灌字库，0x03D000 的图标分区是擦除态 0xFF。
    //   而 osd_icon_rom 现在会判"有没有编程过"（全 0xFF ⇒ ready=0 ⇒ 不画图标），
    //   于是"图标到底是在 RTL 里画错了，还是 Flash 没烧"这两件事在仿真里
    //   完全分不开。有了这一段就可以把真实 icons32.bin 灌进去，单独验 RTL。
    //   INIT_HEX_ICONS 的地址由 ICON_ADDR 指定（默认与 partitions.json 一致）。
    parameter INIT_HEX_ICONS = "",
    parameter [23:0] ICON_ADDR = 24'h03D000,
    parameter PROG_NS   = 700000,      // 页编程耗时：W25Q64 典型 0.7ms
    parameter ERASE_NS  = 1000000      // 整片擦除：真实 20~100s，仿真取 1ms 够用
)(
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output reg  miso
);

    // 地址位宽按容量自动定：MEM_BYTES=262144 → AIDX=18
    localparam AIDX = $clog2(MEM_BYTES);

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

    // ★ 不能再把"裁剪后的地址"引成一个中间 wire（如 `wire aidx = addr[AIDX-1:0]`）：
    //   连续赋值会在**同一时刻的后续 delta** 才更新，而下面地址收满那一拍是
    //   `addr = {...}`（阻塞赋值）紧接着就 `sr_out = mem[...]`，此时中间 wire 还是旧值，
    //   会读到上一个地址（未写入区 = 0xFF）→ 首个数据字节错位一格。
    //   直接写 `mem[addr[AIDX-1:0]]` 就是取当下的 addr，没有这个坑。

    initial begin
        for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = 8'hFF;
        // 预置字库：放在填 0xFF 之后，同一个 initial 里 → 顺序有保证，没有竞态
        if (INIT_HEX != "") $readmemh(INIT_HEX, mem);
        // 预置图标分区（第二段，地址由 ICON_ADDR 指定）
        if (INIT_HEX_ICONS != "") $readmemh(INIT_HEX_ICONS, mem, ICON_ADDR);
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
                                sr_out = mem[addr[AIDX-1:0]];
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
                            mem[addr[AIDX-1:0]] = sr_in;
                            // 页编程在 256 字节页内回绕，绝不跨页（真实芯片行为）。
                            addr = {addr[23:8], addr[7:0] + 8'd1};
                        end else if (cmd == 8'h03) begin
                            sr_out = mem[addr[AIDX-1:0]];
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
