//==============================================================================
//  als_adc.v —— 环境光传感器（GL5528 光敏电阻 + 10k 分压）→ 目标亮度
//==============================================================================
//
//  【时钟】audio_mclk（12.288MHz，已有的音频 PLL 输出，≤ ADC 上限 16MHz）。
//    不新增 PLL、不新增全局时钟约束（audio_mclk 已在 timing.sdc 里命名）。
//
//  【链路】
//    EG_PHY_ADC(CH1 = M10) 自由连转 ──► code_last（最近一次转换值）
//         │ 每 1ms 取一次
//         ▼
//    lut[code[11:4]]（256×7 BRAM 曲线，tools/gen_als_lut.py 生成）
//         │
//         ├─ 在位判定：code>24 连续 8ms → 在位；code≤16 连续 511ms → 不在位
//         │            1ms 窗口里一个 EOC 都没有（ADC 不工作）→ 立刻不在位
//         └─ EMA：每 16ms  ema += (target − ema) / 16（Q7.4 定点）
//                在位上升沿直接置成目标（不从 0 慢慢爬）
//    输出 tgt[6:0] + tgt_tgl（值变化时翻转）+ ok —— 交给 als_bright 跨到 50MHz。
//
//  【曲线】target = clamp(18 + 72·log10(E)/log10(300), 8, 95)
//          E = 10·(1.5/(3.3/V − 1))^1.25（lux），V = code/4096 × 3.3
//
//  【复位】rst（高有效，顶层接 rst_all）在本模块内做"异步置位 / 同步释放"。
//==============================================================================
module als_adc #(
    parameter integer CLK_HZ   = 12_288_000,
    parameter integer ADC_CH   = 1,           // 1 = M10（ALS_AOUT）；备选 2 = L10
    parameter integer EOC_TMO  = 63,          // 发出 SOC 后等 EOC 的最长拍数
    parameter integer PRES_ON  = 8,           // 连续多少 ms "亮于阈值" 判在位
    parameter integer PRES_OFF = 511,         // 连续多少 ms "暗于阈值" 判不在位
    parameter [11:0]  TH_ON    = 12'd24,      // code > TH_ON  ⇒ 有传感器
    parameter [11:0]  TH_OFF   = 12'd16       // code ≤ TH_OFF ⇒ 像是没接
)(
    input  wire       clk,                    // audio_mclk
    input  wire       rst,
    output reg  [6:0] tgt,                    // 平滑后的目标亮度 0..100
    output reg        tgt_tgl,                // tgt 变化时翻转
    output reg        ok                      // 传感器在位且 ADC 正常
);

//------------------------------------------------------------------
//  本地复位同步（异步置位 / 同步释放）
//------------------------------------------------------------------
reg [1:0] rst_sh;
always @(posedge clk or posedge rst) begin
    if (rst) rst_sh <= 2'b11;
    else     rst_sh <= {rst_sh[0], 1'b0};
end
wire srst = rst_sh[1];

//------------------------------------------------------------------
//  1. ADC：SOC 单拍 → 等 EOC → 锁存 → 立刻再发（超时也重发）
//------------------------------------------------------------------
wire        adc_eoc;
wire [11:0] adc_dout;
reg         adc_soc;
reg  [5:0]  wait_cnt;
reg  [11:0] code_last;
reg         eoc_seen;                         // 本 1ms 窗口里见过 EOC

localparam [2:0] ADC_S = ADC_CH;

// 仿真时由 peripherals/sim/eg_phy_adc_stub.v 提供同名同参数的行为桩
EG_PHY_ADC #(
    .CH0  ((ADC_CH == 0) ? "ENABLE" : "DISABLE"),
    .CH1  ((ADC_CH == 1) ? "ENABLE" : "DISABLE"),
    .CH2  ((ADC_CH == 2) ? "ENABLE" : "DISABLE"),
    .CH3  ((ADC_CH == 3) ? "ENABLE" : "DISABLE"),
    .CH4  ((ADC_CH == 4) ? "ENABLE" : "DISABLE"),
    .CH5  ((ADC_CH == 5) ? "ENABLE" : "DISABLE"),
    .CH6  ((ADC_CH == 6) ? "ENABLE" : "DISABLE"),
    .CH7  ((ADC_CH == 7) ? "ENABLE" : "DISABLE"),
    .VREF ("DISABLE")                         // 用内部参考（= VDDA）
) u_adc (
    .clk  (clk),
    .pd   (1'b0),
    .s    (ADC_S),
    .soc  (adc_soc),
    .eoc  (adc_eoc),
    .dout (adc_dout)
);

reg tick1;                                    // 1ms 节拍（下面产生）

always @(posedge clk) begin
    if (srst) begin
        adc_soc   <= 1'b0;
        wait_cnt  <= 6'd0;
        code_last <= 12'd0;
        eoc_seen  <= 1'b0;
    end else begin
        adc_soc <= 1'b0;
        if (tick1) eoc_seen <= 1'b0;          // 新窗口
        if (adc_eoc) begin
            code_last <= adc_dout;
            eoc_seen  <= 1'b1;
            wait_cnt  <= 6'd0;
            adc_soc   <= 1'b1;                // 自由连转
        end else if (wait_cnt == EOC_TMO[5:0]) begin
            wait_cnt  <= 6'd0;
            adc_soc   <= 1'b1;                // 超时补发（上电第一次也走这里）
        end else begin
            wait_cnt  <= wait_cnt + 6'd1;
        end
    end
end

//------------------------------------------------------------------
//  2. 1ms 节拍 + 16ms 子节拍
//------------------------------------------------------------------
localparam integer MS_DIV = CLK_HZ / 1000;    // 12288
localparam [13:0]  MS_TOP = MS_DIV - 1;
reg [13:0] ms_cnt;
reg [3:0]  ms16;

always @(posedge clk) begin
    if (srst) begin
        ms_cnt <= 14'd0;
        tick1  <= 1'b0;
        ms16   <= 4'd0;
    end else begin
        tick1 <= 1'b0;
        if (ms_cnt == MS_TOP) begin
            ms_cnt <= 14'd0;
            tick1  <= 1'b1;
            ms16   <= ms16 + 4'd1;
        end else
            ms_cnt <= ms_cnt + 14'd1;
    end
end

//------------------------------------------------------------------
//  3. 曲线表（256×7，BRAM）：同步读
//------------------------------------------------------------------
//  tick1 拍锁存 code_s → 下一拍 BRAM 采地址 → 再下一拍 lut_q 有效（s2）
reg  [11:0] code_s;
reg         alive_s;                          // 上一个 1ms 窗口里 ADC 有输出
reg  [6:0]  lut_q;
reg         s1, s2;
reg         s2_16;                            // 本次采样同时是 16ms 边界

reg [6:0] lut_mem [0:255];    /* fehdl force_ram=1, ram_style="bram" */

always @(posedge clk) lut_q <= lut_mem[code_s[11:4]];

always @(posedge clk) begin
    if (srst) begin
        code_s  <= 12'd0;
        alive_s <= 1'b0;
        s1      <= 1'b0;
        s2      <= 1'b0;
        s2_16   <= 1'b0;
    end else begin
        s1 <= tick1;
        s2 <= s1;
        if (tick1) begin
            code_s  <= code_last;
            alive_s <= eoc_seen;
            s2_16   <= (ms16 == 4'd15);
        end
    end
end

//------------------------------------------------------------------
//  4. 在位判定（单个 9 位计数 + 两个阈值）
//------------------------------------------------------------------
//  计数"与当前判定相反的连续毫秒数"：
//    不在位时数 code>TH_ON 的连续次数，到 PRES_ON 翻成在位；
//    在位时数 code≤TH_OFF 的连续次数，到 PRES_OFF 翻成不在位。
//  ADC 没输出（alive_s=0）⇒ 立刻不在位。
reg  [8:0] pres_cnt;
reg        ok_d;
wire       hi = (code_s >  TH_ON);
wire       lo = (code_s <= TH_OFF);

always @(posedge clk) begin
    if (srst) begin
        ok       <= 1'b0;
        pres_cnt <= 9'd0;
    end else if (s2) begin
        if (!alive_s) begin
            ok       <= 1'b0;
            pres_cnt <= 9'd0;
        end else if (!ok) begin
            if (!hi)                              pres_cnt <= 9'd0;
            else if (pres_cnt == PRES_ON - 1)  begin ok <= 1'b1; pres_cnt <= 9'd0; end
            else                                  pres_cnt <= pres_cnt + 9'd1;
        end else begin
            if (!lo)                              pres_cnt <= 9'd0;
            else if (pres_cnt == PRES_OFF - 1) begin ok <= 1'b0; pres_cnt <= 9'd0; end
            else                                  pres_cnt <= pres_cnt + 9'd1;
        end
    end
end

//------------------------------------------------------------------
//  5. EMA（Q7.4）+ 输出
//------------------------------------------------------------------
//  ema += (target·16 − ema) >>> 4 —— 时间常数 16 × 16ms ≈ 0.26s，
//  从全暗到全亮约 1s 走完 98%，观感是"平滑渐变"而不是跳。
reg  [10:0] ema;
wire signed [11:0] ema_err = {1'b0, lut_q, 4'd0} - {1'b0, ema};
wire [10:0] ema_nx = ema + {{3{ema_err[11]}}, ema_err[11:4]};   // 算术右移 4
wire [6:0]  ema_out = ema[10:4] + {6'd0, ema[3]};     // 四舍五入

always @(posedge clk) begin
    if (srst) begin
        ema     <= 11'd0;
        ok_d    <= 1'b0;
        tgt     <= 7'd50;
        tgt_tgl <= 1'b0;
    end else begin
        ok_d <= ok;
        if (ok & ~ok_d)
            ema <= {lut_q, 4'd0};                     // 刚接上：直接置目标
        else if (s2 & s2_16)
            ema <= ema_nx;
        // ok_d：在位上升沿那拍 ema 才被置成目标，下一拍再出 ⇒ 不会先吐一拍旧的 ema
        if (ok && ok_d && (ema_out != tgt)) begin
            tgt     <= ema_out;
            tgt_tgl <= ~tgt_tgl;
        end
    end
end

//------------------------------------------------------------------
//  曲线表初值（tools/gen_als_lut.py --write 生成，不要手改）
//------------------------------------------------------------------
// ==== BEGIN AUTO-GEN: ALS_LUT (tools/gen_als_lut.py) ====
initial begin
    lut_mem[  0]=7'd8 ; lut_mem[  1]=7'd8 ; lut_mem[  2]=7'd8 ; lut_mem[  3]=7'd8 ; lut_mem[  4]=7'd8 ; lut_mem[  5]=7'd8 ; lut_mem[  6]=7'd8 ; lut_mem[  7]=7'd8 ;
    lut_mem[  8]=7'd8 ; lut_mem[  9]=7'd8 ; lut_mem[ 10]=7'd8 ; lut_mem[ 11]=7'd8 ; lut_mem[ 12]=7'd8 ; lut_mem[ 13]=7'd8 ; lut_mem[ 14]=7'd9 ; lut_mem[ 15]=7'd10;
    lut_mem[ 16]=7'd11; lut_mem[ 17]=7'd12; lut_mem[ 18]=7'd13; lut_mem[ 19]=7'd14; lut_mem[ 20]=7'd15; lut_mem[ 21]=7'd16; lut_mem[ 22]=7'd17; lut_mem[ 23]=7'd17;
    lut_mem[ 24]=7'd18; lut_mem[ 25]=7'd19; lut_mem[ 26]=7'd19; lut_mem[ 27]=7'd20; lut_mem[ 28]=7'd21; lut_mem[ 29]=7'd21; lut_mem[ 30]=7'd22; lut_mem[ 31]=7'd22;
    lut_mem[ 32]=7'd23; lut_mem[ 33]=7'd24; lut_mem[ 34]=7'd24; lut_mem[ 35]=7'd25; lut_mem[ 36]=7'd25; lut_mem[ 37]=7'd26; lut_mem[ 38]=7'd26; lut_mem[ 39]=7'd27;
    lut_mem[ 40]=7'd27; lut_mem[ 41]=7'd28; lut_mem[ 42]=7'd28; lut_mem[ 43]=7'd28; lut_mem[ 44]=7'd29; lut_mem[ 45]=7'd29; lut_mem[ 46]=7'd30; lut_mem[ 47]=7'd30;
    lut_mem[ 48]=7'd31; lut_mem[ 49]=7'd31; lut_mem[ 50]=7'd31; lut_mem[ 51]=7'd32; lut_mem[ 52]=7'd32; lut_mem[ 53]=7'd32; lut_mem[ 54]=7'd33; lut_mem[ 55]=7'd33;
    lut_mem[ 56]=7'd34; lut_mem[ 57]=7'd34; lut_mem[ 58]=7'd34; lut_mem[ 59]=7'd35; lut_mem[ 60]=7'd35; lut_mem[ 61]=7'd35; lut_mem[ 62]=7'd36; lut_mem[ 63]=7'd36;
    lut_mem[ 64]=7'd36; lut_mem[ 65]=7'd37; lut_mem[ 66]=7'd37; lut_mem[ 67]=7'd37; lut_mem[ 68]=7'd38; lut_mem[ 69]=7'd38; lut_mem[ 70]=7'd38; lut_mem[ 71]=7'd39;
    lut_mem[ 72]=7'd39; lut_mem[ 73]=7'd39; lut_mem[ 74]=7'd39; lut_mem[ 75]=7'd40; lut_mem[ 76]=7'd40; lut_mem[ 77]=7'd40; lut_mem[ 78]=7'd41; lut_mem[ 79]=7'd41;
    lut_mem[ 80]=7'd41; lut_mem[ 81]=7'd41; lut_mem[ 82]=7'd42; lut_mem[ 83]=7'd42; lut_mem[ 84]=7'd42; lut_mem[ 85]=7'd43; lut_mem[ 86]=7'd43; lut_mem[ 87]=7'd43;
    lut_mem[ 88]=7'd43; lut_mem[ 89]=7'd44; lut_mem[ 90]=7'd44; lut_mem[ 91]=7'd44; lut_mem[ 92]=7'd44; lut_mem[ 93]=7'd45; lut_mem[ 94]=7'd45; lut_mem[ 95]=7'd45;
    lut_mem[ 96]=7'd46; lut_mem[ 97]=7'd46; lut_mem[ 98]=7'd46; lut_mem[ 99]=7'd46; lut_mem[100]=7'd47; lut_mem[101]=7'd47; lut_mem[102]=7'd47; lut_mem[103]=7'd47;
    lut_mem[104]=7'd48; lut_mem[105]=7'd48; lut_mem[106]=7'd48; lut_mem[107]=7'd48; lut_mem[108]=7'd49; lut_mem[109]=7'd49; lut_mem[110]=7'd49; lut_mem[111]=7'd49;
    lut_mem[112]=7'd50; lut_mem[113]=7'd50; lut_mem[114]=7'd50; lut_mem[115]=7'd50; lut_mem[116]=7'd51; lut_mem[117]=7'd51; lut_mem[118]=7'd51; lut_mem[119]=7'd51;
    lut_mem[120]=7'd52; lut_mem[121]=7'd52; lut_mem[122]=7'd52; lut_mem[123]=7'd52; lut_mem[124]=7'd53; lut_mem[125]=7'd53; lut_mem[126]=7'd53; lut_mem[127]=7'd53;
    lut_mem[128]=7'd54; lut_mem[129]=7'd54; lut_mem[130]=7'd54; lut_mem[131]=7'd54; lut_mem[132]=7'd55; lut_mem[133]=7'd55; lut_mem[134]=7'd55; lut_mem[135]=7'd55;
    lut_mem[136]=7'd56; lut_mem[137]=7'd56; lut_mem[138]=7'd56; lut_mem[139]=7'd56; lut_mem[140]=7'd57; lut_mem[141]=7'd57; lut_mem[142]=7'd57; lut_mem[143]=7'd57;
    lut_mem[144]=7'd58; lut_mem[145]=7'd58; lut_mem[146]=7'd58; lut_mem[147]=7'd58; lut_mem[148]=7'd59; lut_mem[149]=7'd59; lut_mem[150]=7'd59; lut_mem[151]=7'd59;
    lut_mem[152]=7'd60; lut_mem[153]=7'd60; lut_mem[154]=7'd60; lut_mem[155]=7'd60; lut_mem[156]=7'd61; lut_mem[157]=7'd61; lut_mem[158]=7'd61; lut_mem[159]=7'd61;
    lut_mem[160]=7'd62; lut_mem[161]=7'd62; lut_mem[162]=7'd62; lut_mem[163]=7'd62; lut_mem[164]=7'd63; lut_mem[165]=7'd63; lut_mem[166]=7'd63; lut_mem[167]=7'd64;
    lut_mem[168]=7'd64; lut_mem[169]=7'd64; lut_mem[170]=7'd64; lut_mem[171]=7'd65; lut_mem[172]=7'd65; lut_mem[173]=7'd65; lut_mem[174]=7'd65; lut_mem[175]=7'd66;
    lut_mem[176]=7'd66; lut_mem[177]=7'd66; lut_mem[178]=7'd67; lut_mem[179]=7'd67; lut_mem[180]=7'd67; lut_mem[181]=7'd68; lut_mem[182]=7'd68; lut_mem[183]=7'd68;
    lut_mem[184]=7'd68; lut_mem[185]=7'd69; lut_mem[186]=7'd69; lut_mem[187]=7'd69; lut_mem[188]=7'd70; lut_mem[189]=7'd70; lut_mem[190]=7'd70; lut_mem[191]=7'd71;
    lut_mem[192]=7'd71; lut_mem[193]=7'd71; lut_mem[194]=7'd72; lut_mem[195]=7'd72; lut_mem[196]=7'd72; lut_mem[197]=7'd73; lut_mem[198]=7'd73; lut_mem[199]=7'd73;
    lut_mem[200]=7'd74; lut_mem[201]=7'd74; lut_mem[202]=7'd74; lut_mem[203]=7'd75; lut_mem[204]=7'd75; lut_mem[205]=7'd76; lut_mem[206]=7'd76; lut_mem[207]=7'd76;
    lut_mem[208]=7'd77; lut_mem[209]=7'd77; lut_mem[210]=7'd78; lut_mem[211]=7'd78; lut_mem[212]=7'd78; lut_mem[213]=7'd79; lut_mem[214]=7'd79; lut_mem[215]=7'd80;
    lut_mem[216]=7'd80; lut_mem[217]=7'd81; lut_mem[218]=7'd81; lut_mem[219]=7'd82; lut_mem[220]=7'd82; lut_mem[221]=7'd83; lut_mem[222]=7'd83; lut_mem[223]=7'd84;
    lut_mem[224]=7'd84; lut_mem[225]=7'd85; lut_mem[226]=7'd86; lut_mem[227]=7'd86; lut_mem[228]=7'd87; lut_mem[229]=7'd88; lut_mem[230]=7'd88; lut_mem[231]=7'd89;
    lut_mem[232]=7'd90; lut_mem[233]=7'd90; lut_mem[234]=7'd91; lut_mem[235]=7'd92; lut_mem[236]=7'd93; lut_mem[237]=7'd94; lut_mem[238]=7'd95; lut_mem[239]=7'd95;
    lut_mem[240]=7'd95; lut_mem[241]=7'd95; lut_mem[242]=7'd95; lut_mem[243]=7'd95; lut_mem[244]=7'd95; lut_mem[245]=7'd95; lut_mem[246]=7'd95; lut_mem[247]=7'd95;
    lut_mem[248]=7'd95; lut_mem[249]=7'd95; lut_mem[250]=7'd95; lut_mem[251]=7'd95; lut_mem[252]=7'd95; lut_mem[253]=7'd95; lut_mem[254]=7'd95; lut_mem[255]=7'd95;
end
// ==== END AUTO-GEN: ALS_LUT ====

endmodule
