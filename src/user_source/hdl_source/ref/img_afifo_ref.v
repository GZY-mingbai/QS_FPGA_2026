//==============================================================================
//  img_afifo.v —— 图片图层用的异步 FIFO（灰度码指针 + 两拍同步器）
//==============================================================================
//
//  【为什么要异步 FIFO】
//  场景图（含图片图层）跑在 50MHz（sd_card_clk），SDRAM 读仲裁器跑在
//  125MHz（ext_mem_clk）。两者之间要传两类数据：
//      · 请求：图层 → 仲裁器（"帮我读地址 X 的像素"）
//      · 像素：仲裁器 → 图层（"地址 X 的像素给你"）
//  都是多比特数据流，跨时钟域**必须用异步 FIFO**，不能只打两拍。
//
//  【为什么不用 IP 生成器】
//  TD 的 FIFO IP 是 32bit 定宽的，而请求 FIFO 要 21bit。手写一个参数化的
//  小 FIFO 更省事，而且深度只有 16，用分布式 RAM 就够（不占 BRAM）。
//
//  【读侧是 show-ahead】
//  dout 组合输出当前队首，不需要"先读再等一拍"。这对图片图层很重要 ——
//  它要在一拍里决定"用队首像素"还是"再弹一个"。
//
//  【rcount】读域可见的"当前可读个数"（已经过同步器，保守值）。
//  图层用它做流控：要弹的时候必须 rcount >= 1。
//
//  【深度必须是 2 的幂】（灰度码指针的要求）
//
//==============================================================================

//==============================================================================
//  ★★ 2026-10-16：加"不要动我"属性，防止 TD 把几个 FIFO 实例打包/共享 ★★
//==============================================================================
//  【为什么】本地跑完 P&R 后发现 125MHz 域（ext_mem_clk）有一组**真实的时序违例**：
//        ext_mem_clk -> ext_mem_clk   SWNS -1.929ns, STNS -148.733ns, 128 端点
//      而最差那条路径的起点/终点是**两个逻辑上毫无关系的模块**：
//        起点  u_loader/u_wfifo/rgray_reg[0]        （装载器写 FIFO 的读指针）
//        终点  u_app/.../u_img1/u_pix_fifo/ramread0…（图片层像素 FIFO 的 LUTRAM 读）
//      两者之间没有任何信号连接 ⇒ 只可能是**综合/布局阶段把不相关的逻辑塞进了
//      同一个 LUT（LUT5/6 打包），或者做了跨实例的资源共享**，于是"跨模块"的
//      长组合路径被造出来了。
//      这类路径既会拖慢时序，也可能在极端情况下读错数据 —— 而画面上那条
//      "固定 x、每 4 行一次、还会闪"的小亮块，和"整个黑屏"都符合这个特征。
//
//  【做法】给模块加 dont_touch / keep，禁止工具优化或合并它的内部逻辑。
//      代价：LUT 会多用一些（本设计 LUT 已 75.8%），所以必须本地跑完 P&R
//      确认资源与时序都还行，才能上板。
//      如果工具不认这个属性，会直接忽略 —— 不会更坏。
//==============================================================================
(* dont_touch = "true" *)
(* keep_hierarchy = "true" *)
module img_afifo_ref #(
    parameter integer DW = 32,      // 数据位宽
    parameter integer AW = 4,       // 地址位宽 → 深度 = 2^AW
    //--------------------------------------------------------------------------
    //  ★ build 100M-29：写口额外流水级（默认 0 = 原行为）
    //--------------------------------------------------------------------------
    //  【为什么要这个旋钮】`phy_1/final_timing.rpt`（20:30）报了一条**保持违例**：
    //      u_img1/u_req_fifo/waddr_q_reg[7] → u_req_fifo/ramread0.addra[11]
    //      HWNS −0.011ns / HTNS −0.022ns，2 个端点
    //  这是"flop → BRAM 地址脚"的同沿保持路径：地址可能**早变** ⇒ RAM 写到/读到
    //  相邻条目 ⇒ 拿到的请求是别人的（静默，任何计数器都不报）。
    //  【做法】把 地址/数据/使能 再打一拍，让进 BRAM 的那一级来自**扇出=1 的
    //  专用寄存器**（工具才会把它摆到 BRAM 旁边，缩短/稳定这条保持路径）。
    //  【代价】写数据晚 1 拍落地；`wgray_p` 同步再延 1 拍，所以
    //  "读域看见指针"与"数据真正落地"仍然同拍 —— 那条 :243-247 的不变式不破。
    //  【为什么默认 0】它会改变所有实例的写延迟；像素 FIFO 的延迟是**载荷语义**
    //  （pix_head 对齐），不能盲开。本版只在请求 FIFO 上开（见 img_layer_ctrl）。
    //--------------------------------------------------------------------------
    parameter integer WSTAGE = 0
)(
    // ---- 写侧 ----
    input  wire          wclk,
    input  wire          wrst,      // 高有效
    input  wire          we,
    input  wire [DW-1:0] din,
    output wire          wfull,

    // ---- 读侧 ----
    input  wire          rclk,
    input  wire          rrst,      // 高有效
    input  wire          re,
    output wire [DW-1:0] dout,
    output wire          rempty,
    output wire [AW:0]   rcount,
    //--------------------------------------------------------------------------
    //  ★ build 100M-12：写域的水位（"已经用掉几个"）
    //--------------------------------------------------------------------------
    //  【为什么不用 rcount】rcount 是**读域**的二进制值：
    //    ① 采到写域属于跨时钟域，二进制多比特翻位时会读到不相干的中间值；
    //    ② 它是保守**欠报**（wgray_p 比数据晚一拍）⇒ 让人以为"更空"⇒ 更容易溢出。
    //  写域里本来就有同步好的读指针 `rgray_s2`（wfull 就是拿它算的），
    //  直接 gray2bin 再减一下就得到**写域自己的**水位，CDC 干净、方向也是安全的
    //  （wbin 比数据早一拍落地 ⇒ 多报 1 ⇒ 更早刹车）。
    //--------------------------------------------------------------------------
    output wire [AW:0]   wlevel
);

localparam integer DEPTH = (1 << AW);

//------------------------------------------------------------------------------
//  灰度码 → 二进制
//------------------------------------------------------------------------------
//  ★ build 100M-12：函数声明**必须放在模块前部** —— 写域的水位（wlevel）
//    也要用它，而 iverilog 不允许先用后声明。
//------------------------------------------------------------------------------
function [AW:0] gray2bin;
    input [AW:0] g;
    integer      i;
    begin
        gray2bin[AW] = g[AW];
        for (i = AW-1; i >= 0; i = i - 1)
            gray2bin[i] = gray2bin[i+1] ^ g[i];
    end
endfunction

// ★ build W：明确要求落成块 RAM（同步读）★
//   双时钟 + 异步读在 FPGA 上做不到：BRAM 的读是同步的，分布式 RAM 不支持双时钟。
//   原来 `assign dout = mem[rbin]` 是**组合读**，只能落成 LUTRAM，
//   而 LUTRAM 跨时钟读写的实际行为是未定义的 ⇒ 板上有概率读到旧值/重复值
//   （正是"一整组 4 拍拿到同一个值"），而仿真永远正常。
(* ram_style = "block" *) (* ramstyle = "block" *) (* syn_ramstyle = "block" *)
reg [DW-1:0] mem [0:DEPTH-1];

//------------------------------------------------------------------------------
//  指针（二进制 + 灰度码各一份）
//------------------------------------------------------------------------------
reg [AW:0] wbin, wgray;
reg [AW:0] rbin, rgray;

// 灰度码同步链
(* ASYNC_REG = "TRUE" *) reg [AW:0] wgray_s1; // 写指针 → 读域：同步级 1
(* ASYNC_REG = "TRUE" *) reg [AW:0] wgray_s2; // 写指针 → 读域：同步级 2
(* ASYNC_REG = "TRUE" *) reg [AW:0] rgray_s1; // 读指针 → 写域：同步级 1
(* ASYNC_REG = "TRUE" *) reg [AW:0] rgray_s2; // 读指针 → 写域：同步级 2

//------------------------------------------------------------------------------
//  写侧
//------------------------------------------------------------------------------
//  ★ 2026-10-17：RAM 写口打一拍（时序修复）★
//  【为什么】原来的写路径是：wfull（格雷码比较）→ we 门控 → LUTRAM 写地址
//  译码 → DRAM 写端口，全串在一个 8 级组合锥里，125MHz 关不住
//  （syn_1/gate.timing 实测 wgray → pix_fifo 写使能 SWNS −0.396ns，8 级
//  8.29ns）。把"这一拍要写"预先寄存（we_q / waddr_q / din_q），RAM 写口
//  的锥只剩"寄存器 → 地址译码"，深度减半。
//  【为什么语义不变】读侧看到的新写指针本来就要经过 wgray → wgray_s1 →
//  wgray_s2 两拍同步（rcount 是保守值），RAM 数据晚一拍落地永远赶在
//  读者看见指针之前 —— 请求 FIFO 和像素 FIFO 两个用途都成立。
reg            we_q;
reg [AW-1:0]   waddr_q;
reg [DW-1:0]   din_q;

// ★ build 100M-29：WSTAGE=1 时的第二级（只驱动 BRAM，扇出=1）
// ★ 100M-35：这几个寄存器**只驱动 BRAM 的写口**，加 keep 防止工具把它们
//   合并/重定时到别的逻辑里 —— 保持违例（−0.011ns @ dia[0]）只能靠
//   "专用寄存器 + 贴着 RAM 摆放"来收敛。
(* keep = "true" *) reg            we_qq;
(* keep = "true" *) reg [AW-1:0]   waddr_qq;
(* keep = "true" *) reg [DW-1:0]   din_qq;

always @(posedge wclk or posedge wrst) begin
    if (wrst) begin
        we_q    <= 1'b0;
        waddr_q <= {AW{1'b0}};
        din_q   <= {DW{1'b0}};
        we_qq   <= 1'b0;
        waddr_qq<= {AW{1'b0}};
        din_qq  <= {DW{1'b0}};
    end else begin
        we_q    <= we && !wfull;
        waddr_q <= wbin[AW-1:0];
        din_q   <= din;
        we_qq   <= we_q;
        waddr_qq<= waddr_q;
        din_qq  <= din_q;
    end
end

always @(posedge wclk or posedge wrst) begin
    if (wrst) begin
        wbin  <= {(AW+1){1'b0}};
        wgray <= {(AW+1){1'b0}};
    end else if (we && !wfull) begin
        wbin  <= wbin + 1'b1;
        wgray <= ((wbin + 1'b1) >> 1) ^ (wbin + 1'b1);
    end
end

generate
if (WSTAGE != 0) begin : g_wstage
    // ★ 地址/数据/使能再打一拍：进 BRAM 的那一级是扇出=1 的专用寄存器
    always @(posedge wclk or posedge wrst) begin
        if (wrst) begin
            // mem 无需复位（内容由写口决定）
        end else if (we_qq) begin
            mem[waddr_qq] <= din_qq;
        end
    end
end else begin : g_nowstage
    always @(posedge wclk or posedge wrst) begin
        if (wrst) begin
            // mem 无需复位（内容由写口决定）
        end else if (we_q) begin
            mem[waddr_q] <= din_q;
        end
    end
end
endgenerate

// 满：灰度码高两位相反、其余相同
assign wfull = (wgray == {~rgray_s2[AW:AW-1], rgray_s2[AW-2:0]});

// 读指针同步到写域
always @(posedge wclk or posedge wrst) begin
    if (wrst) begin
        rgray_s1 <= {(AW+1){1'b0}};
        rgray_s2 <= {(AW+1){1'b0}};
    end else begin
        rgray_s1 <= rgray;
        rgray_s2 <= rgray_s1;
    end
end

//------------------------------------------------------------------------------
//  读侧
//------------------------------------------------------------------------------
always @(posedge rclk or posedge rrst) begin
    if (rrst) begin
        rbin  <= {(AW+1){1'b0}};
        rgray <= {(AW+1){1'b0}};
    end else if (re && !rempty) begin
        rbin  <= rbin + 1'b1;
        rgray <= ((rbin + 1'b1) >> 1) ^ (rbin + 1'b1);
    end
end

//==============================================================================
//  ★★★ build W：读侧改成【同步读 + 提前一拍取址】（First-Word-Fall-Through）★★★
//==============================================================================
//  【为什么】原来 `assign dout = mem[rbin]` 是组合读 ⇒ 只能落成 LUTRAM ⇒
//  双时钟跨域读写的实际行为未定义 ⇒ 板上偶发读到旧值（一整组 4 拍同一个值），
//  而仿真（reg 数组）永远正常。
//
//  【怎么改】存储器必须是同步读（这样才能落成双时钟 BRAM），
//  但图层侧是"这一拍就要用队首"（`pix_head` 直接进组合逻辑），
//  所以不能简单打一拍（那样队首会晚一拍）。
//  做法：**提前一拍把"下一次要用的地址"读出来** ——
//      rbin_nxt = 本拍弹了就 +1，没弹就保持
//      dout_r  <= mem[rbin_nxt]        （在时钟沿寄存）
//  ⇒ 下一拍 dout 正好等于那时的队首，**语义与组合读完全一致**，图层一行都不用改。
//  代价：地址多一个 +1（1 个 LUT），BRAM 读本身是同步的（这正是我们要的）。
//==============================================================================
wire [AW-1:0] rbin_nxt = (re && !rempty) ? (rbin[AW-1:0] + 1'b1) : rbin[AW-1:0];

reg [DW-1:0] dout_r;

always @(posedge rclk or posedge rrst) begin
    if (rrst) dout_r <= {DW{1'b0}};
    else      dout_r <= mem[rbin_nxt];
end

assign dout   = dout_r;
assign rempty = (rgray == wgray_s2);

//==============================================================================
//  ★★★ 2026-10-18：写指针必须"跟着数据一起"暴露给读域 ★★★
//==============================================================================
//  【这是一个真 bug，不是保险措施】
//  写口为了时序被打了一拍（见上面 we_q/waddr_q/din_q）：
//      t     : we=1，wbin=P
//      t+1   : wbin→P+1、wgray→gray(P+1)      ← 指针先动了
//      t+2   : mem[P] <= din                   ← 数据晚两拍才落地
//  而下面的同步链是直接从 **wgray** 出发的，于是（以请求 FIFO 为例：
//  写 50MHz / 读 125MHz）：
//      t+1+ε      : wgray_s1 抓到 gray(P+1)
//      t+1+ε+8ns  : wgray_s2 也变成 gray(P+1) ⇒ rempty 拉低 ⇒ 读域认为"有货"
//      t+40ns     : mem[P] 才真正写进去
//  读域在 **t+28~36ns** 就认为有数据，比数据落地早 4~12ns ⇒ **读到上一个字的
//  旧内容**（show-ahead 的 dout 直接是 mem[rbin]，没有任何保护）。
//
//  【为什么板上的症状全对得上】
//    · 装载器 FIFO（50MHz→125MHz）中招 ⇒ 写进 SDRAM 的地址/数据偶尔是上一个
//      像素的 ⇒ 纹理数据错 ⇒ **数码管照样 E（文件头校验早过了），图却花**
//    · 图层请求 FIFO（50MHz→125MHz）中招 ⇒ 偶尔去读上一个请求的地址
//      ⇒ 图层拿到错像素 ⇒ **稀疏网点 + 偶发残图**
//    · 帧缓存用的是厂商 IP 的 FIFO（有完整保护）⇒ **基础画面完全正常**
//    · 仿真看不出来：iverilog 的非阻塞赋值让 RAM 写入在同一个时间步就可见，
//      没有"写口打一拍"的真实延迟 ⇒ 这个 bug 在仿真里天然隐身
//
//  【修法】把写指针再打一拍（wgray_p），让"读域能看到的写指针"与
//  "数据真正落地"同拍：
//      t+2 : mem[P] 写入  &&  wgray_p <= gray(P+1)     ← 同拍
//      读域最早在 t+2 之后一个读时钟才看到 ⇒ 数据一定已经在 RAM 里
//  代价：rempty 晚 1 个写时钟解除（读侧只是多等一拍，无副作用）。
//==============================================================================
reg  [AW:0] wgray_p;         // "已落地"的写指针（与 mem 写入同拍）
reg  [AW:0] wgray_p1;        // ★ WSTAGE=1 时的中间级

always @(posedge wclk or posedge wrst) begin
    if (wrst) begin
        wgray_p  <= {(AW+1){1'b0}};
        wgray_p1 <= {(AW+1){1'b0}};
    end else if (WSTAGE != 0) begin
        // 数据在 we_qq 那一拍才落地 ⇒ 暴露给读域的指针也要多延一拍
        wgray_p  <= wgray_p1;
        wgray_p1 <= wgray;
    end else begin
        wgray_p  <= wgray;
    end
end

// 写指针同步到读域（源头改成 wgray_p）
always @(posedge rclk or posedge rrst) begin
    if (rrst) begin
        wgray_s1 <= {(AW+1){1'b0}};
        wgray_s2 <= {(AW+1){1'b0}};
    end else begin
        wgray_s1 <= wgray_p;
        wgray_s2 <= wgray_s1;
    end
end

//------------------------------------------------------------------------------
//  可读个数（读域）：gray2bin(写指针同步值) - 读指针
//  （gray2bin 已挪到模块前部，见 build 100M-12 的说明）
//------------------------------------------------------------------------------
wire [AW:0] wbin_r = gray2bin(wgray_s2);

assign rcount = wbin_r - rbin;

//------------------------------------------------------------------------------
//  ★ build 100M-12：写域水位 = 写指针 − 同步过来的读指针
//------------------------------------------------------------------------------
//  两边的源头都已经在写域里了：
//      wbin     —— 写指针本身（本域）
//      rgray_s2 —— 读指针经两级同步后的副本（wfull 也是用它的）
//  所以这个减法**没有任何跨时钟域问题**，不是在"采一个别的域的二进制值"。
//  多报 1（wbin 比数据早一拍落地）⇒ 更早刹车 ⇒ 只会更安全。
//------------------------------------------------------------------------------
wire [AW:0] rbin_w = gray2bin(rgray_s2);

assign wlevel = wbin - rbin_w;

endmodule
