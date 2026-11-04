//=============================================================================
// 文件名   : color_bar.v
// 模块名   : color_bar
// 功能     : 【彩条图案发生器 / 纯测试用视频源】
//            自己产生一套完整的 VGA/HDMI 视频时序信号（hs / vs / de），
//            同时在有效显示区画出一条经典的 8 色彩条（白黄青绿紫红蓝黑）。
//
// 为什么要写这个模块 ?
//   FPGA 的视频通路（SDRAM 读 -> FIFO -> 时序 -> HDMI 编码）非常复杂，
//   如果一上来就接真实的 SD 卡图片，一旦屏幕不亮，你根本分不清是：
//       (a) 时序不对？  (b) SDRAM 没读出来？  (c) HDMI 编码错了？  (d) SD 卡没读到？
//   所以工程里通常先做一个"最简视频源"——不依赖任何外部存储，
//   只要 PLL 出时钟、这个模块就能出图。屏幕亮出彩条 = 时序和 HDMI 通路 OK，
//   这样就把问题范围缩小了一大半。这是调试视频系统最有效的"分段排查法"。
//
// 如何使用 ?
//   本模块是"独立视频源"，在 video_timing_data.v 里被例化。
//   默认编译宏是 `VIDEO_640_480`（见 video_define.v），即 640x480@60Hz。
//   RGB 输出接了空端口（.rgb_r() 悬空），只取 hs/vs/de 三个时序信号，
//   所以颜色数据实际不用——因为工程最终显示的是 SD 卡图片，彩条只用来产生时序。
//
// 为什么这样写 ?  （初学者重点看这几条）
//   1) 视频时序的核心思想："数格子"。用一个水平计数器 h_cnt 数像素，
//      一个垂直计数器 v_cnt 数行。数到不同区间就输出不同的信号：
//          [H_FP 后沿前肩] -> 拉 hs 同步脉冲（宽度 H_SYNC）
//          [H_BP 后肩之后 ] -> h_active=1，进入行有效显示
//      垂直方向同理，只是 v_cnt 每"一整行"才加 1。
//   2) 关于消隐区（blanking）：H_TOTAL = 有效 + 前肩 + 同步 + 后肩。
//      640x480 时 = 640 + 16 + 96 + 48 = 800，正好是标准 VGA 的 800 点一行。
//      这 160 个点的空隙就是 CRT 时代的"回扫时间"，现在数字屏也照样要留。
//   3) 为什么 hs/vs 要再打一拍（_d0）？因为信号经过组合逻辑输出会带毛刺，
//      打一拍既能去毛刺、又能让 hs/vs/de 三条信号对齐到同一时钟沿，
//      下游采样时不会出现"de 有效了但 hs 还没翻"的错位。
//=============================================================================

// 引入自定义头文件，里面有 `VIDEO_640_480` 等编译宏定义
`include "video_define.v"

module color_bar(
	input                 clk,           // pixel clock - 像素时钟（RGB 域时钟，640x480 时为 25MHz）
	input                 rst,           // reset signal high active - 复位，高电平有效
	output                hs,            // horizontal synchronization - 行同步信号
	output                vs,            // vertical synchronization - 场（帧）同步信号
	output                de,            // video valid - 数据有效标志，=1 时 rgb 才是有意义的像素
	output[7:0]           rgb_r,         // video red data - 红色分量 8bit
	output[7:0]           rgb_g,         // video green data - 绿色分量 8bit
	output[7:0]           rgb_b          // video blue data - 蓝色分量 8bit
);

//-----------------------------------------------------------------------------
// 一、视频时序参数表（按分辨率分组，靠编译宏 `ifdef 从里面选一组）
//     初学者只需记住：改分辨率 = 改这里的数字 + 改 PLL 时钟频率。
//     下面这些数字全部来自标准 VESA 时序表，直接抄即可，别自己乱改。
//-----------------------------------------------------------------------------

// 720P：1280x720 @ 74.25MHz
`ifdef  VIDEO_1280_720
parameter H_ACTIVE = 16'd1280;           //horizontal active time (pixels) 行有效像素数
parameter H_FP = 16'd110;                //horizontal front porch (pixels) 行前肩
parameter H_SYNC = 16'd40;               //horizontal sync time(pixels) 行同步脉冲宽度
parameter H_BP = 16'd220;                //horizontal back porch (pixels) 行后肩
parameter V_ACTIVE = 16'd720;            //vertical active Time (lines) 场有效行数
parameter V_FP  = 16'd5;                 //vertical front porch (lines) 场前肩
parameter V_SYNC  = 16'd5;               //vertical sync time (lines) 场同步脉冲宽度
parameter V_BP  = 16'd20;                //vertical back porch (lines) 场后肩
parameter HS_POL = 1'b1;                 //horizontal sync polarity, 1 : POSITIVE,0 : NEGATIVE; 行同步极性
parameter VS_POL = 1'b1;                 //vertical sync polarity, 1 : POSITIVE,0 : NEGATIVE; 场同步极性
`endif

//480x272 9Mhz —— 小尺寸 TFT 屏常用
`ifdef  VIDEO_480_272
parameter H_ACTIVE = 16'd480; 
parameter H_FP = 16'd2;       
parameter H_SYNC = 16'd41;    
parameter H_BP = 16'd2;       
parameter V_ACTIVE = 16'd272; 
parameter V_FP  = 16'd2;     
parameter V_SYNC  = 16'd10;   
parameter V_BP  = 16'd2;     
parameter HS_POL = 1'b0;     // 注意：低分辨率屏的极性经常是负的
parameter VS_POL = 1'b0;
`endif

//640x480 25.175Mhz —— ★本工程使用这一组★（标准 VGA，显示器/HDMI 都认）
`ifdef  VIDEO_640_480
parameter H_ACTIVE = 16'd640; 
parameter H_FP = 16'd16;      
parameter H_SYNC = 16'd96;    
parameter H_BP = 16'd48;      
parameter V_ACTIVE = 16'd480; 
parameter V_FP  = 16'd10;    
parameter V_SYNC  = 16'd2;    
parameter V_BP  = 16'd33;    
parameter HS_POL = 1'b0;     // 640x480@60Hz 标准极性为负
parameter VS_POL = 1'b0;
`endif

//800x480 33Mhz
`ifdef  VIDEO_800_480
parameter H_ACTIVE = 16'd800; 
parameter H_FP = 16'd40;      
parameter H_SYNC = 16'd128;   
parameter H_BP = 16'd88;      
parameter V_ACTIVE = 16'd480; 
parameter V_FP  = 16'd1;     
parameter V_SYNC  = 16'd3;    
parameter V_BP  = 16'd21;    
parameter HS_POL = 1'b0;
parameter VS_POL = 1'b0;
`endif

//800x600 40Mhz
`ifdef  VIDEO_800_600
parameter H_ACTIVE = 16'd800; 
parameter H_FP = 16'd40;      
parameter H_SYNC = 16'd128;   
parameter H_BP = 16'd88;      
parameter V_ACTIVE = 16'd600; 
parameter V_FP  = 16'd1;     
parameter V_SYNC  = 16'd4;    
parameter V_BP  = 16'd23;    
parameter HS_POL = 1'b1;
parameter VS_POL = 1'b1;
`endif

//1024x768 65Mhz
`ifdef  VIDEO_1024_768
parameter H_ACTIVE = 16'd1024;
parameter H_FP = 16'd24;      
parameter H_SYNC = 16'd136;   
parameter H_BP = 16'd160;     
parameter V_ACTIVE = 16'd768; 
parameter V_FP  = 16'd3;      
parameter V_SYNC  = 16'd6;    
parameter V_BP  = 16'd29;     
parameter HS_POL = 1'b0;
parameter VS_POL = 1'b0;
`endif

//1920x1080 148.5Mhz
`ifdef  VIDEO_1920_1080
parameter H_ACTIVE = 16'd1920;
parameter H_FP = 16'd88;
parameter H_SYNC = 16'd44;
parameter H_BP = 16'd148; 
parameter V_ACTIVE = 16'd1080;
parameter V_FP  = 16'd4;
parameter V_SYNC  = 16'd5;
parameter V_BP  = 16'd36;
parameter HS_POL = 1'b1;
parameter VS_POL = 1'b1;
`endif

// 一行总共多少个像素点 / 一帧总共多少行 —— 由上面参数自动算出来，不要手写死
parameter H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;//horizontal total time (pixels)
parameter V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;//vertical total time (lines)

//-----------------------------------------------------------------------------
// 二、8 种颜色的 RGB 常数值（8bit 分量，0x00~0xFF）
//     彩条标准顺序：白 黄 青 绿 紫 红 蓝 黑
//-----------------------------------------------------------------------------
parameter WHITE_R       = 8'hff;
parameter WHITE_G       = 8'hff;
parameter WHITE_B       = 8'hff;
parameter YELLOW_R      = 8'hff;
parameter YELLOW_G      = 8'hff;
parameter YELLOW_B      = 8'h00;                                
parameter CYAN_R        = 8'h00;
parameter CYAN_G        = 8'hff;
parameter CYAN_B        = 8'hff;                                
parameter GREEN_R       = 8'h00;
parameter GREEN_G       = 8'hff;
parameter GREEN_B       = 8'h00;
parameter MAGENTA_R     = 8'hff;
parameter MAGENTA_G     = 8'h00;
parameter MAGENTA_B     = 8'hff;
parameter RED_R         = 8'hff;
parameter RED_G         = 8'h00;
parameter RED_B         = 8'h00;
parameter BLUE_R        = 8'h00;
parameter BLUE_G        = 8'h00;
parameter BLUE_B        = 8'hff;
parameter BLACK_R       = 8'h00;
parameter BLACK_G       = 8'h00;
parameter BLACK_B       = 8'h00;

//-----------------------------------------------------------------------------
// 三、内部信号声明
//-----------------------------------------------------------------------------
reg hs_reg;                      //horizontal sync register - 行同步的中间寄存器
reg vs_reg;                      //vertical sync register - 场同步的中间寄存器
reg hs_reg_d0;                   //delay 1 clock of 'hs_reg' - 打一拍输出去毛刺
reg vs_reg_d0;                   //delay 1 clock of 'vs_reg'
reg[11:0] h_cnt;                 //horizontal counter - 水平计数器：数到 H_TOTAL-1 归零
reg[11:0] v_cnt;                 //vertical counter - 垂直计数器：每行加 1，数到 V_TOTAL-1 归零
reg[11:0] active_x;              //video x position - 当前像素在有效区内的 x 坐标 (0 ~ H_ACTIVE-1)
reg[11:0] active_y;              //video y position - 当前像素在有效区内的 y 坐标（本模块没直接用到）
reg[7:0] rgb_r_reg;              //video red data register - 红分量输出寄存器
reg[7:0] rgb_g_reg;              //video green data register
reg[7:0] rgb_b_reg;              //video blue data register
reg h_active;                    //horizontal video active - 行有效标志
reg v_active;                    //vertical video active - 场有效标志
wire video_active;               //video active(horizontal active and vertical active) - 两者都为 1 才真正有效
reg video_active_d0;             //delay 1 clock of video_active - 再打一拍给 de

assign hs = hs_reg_d0;           // 输出行同步
assign vs = vs_reg_d0;           // 输出场同步
assign video_active = h_active & v_active;   // 行、场同时有效 = 像素有效
assign de = video_active_d0;                 // 打一拍变 de
assign rgb_r = rgb_r_reg;        // 输出三色分量
assign rgb_g = rgb_g_reg;
assign rgb_b = rgb_b_reg;

//-----------------------------------------------------------------------------
// 四、输出寄存器打一拍
//     作用：① 去毛刺 ② 让 hs/vs/de 三者相位对齐
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		begin
			hs_reg_d0 <= 1'b0;
			vs_reg_d0 <= 1'b0;
			video_active_d0 <= 1'b0;
		end
	else
		begin
			hs_reg_d0 <= hs_reg;
			vs_reg_d0 <= vs_reg;
			video_active_d0 <= video_active;
		end
end

//-----------------------------------------------------------------------------
// 五、水平计数器：每个像素时钟 +1，数满一整行（H_TOTAL）就清零
//     这就是"数格子"的格子尺。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		h_cnt <= 12'd0;
	else if(h_cnt == H_TOTAL - 1)//horizontal counter maximum value - 数到最后一点，归零
		h_cnt <= 12'd0;
	else
		h_cnt <= h_cnt + 12'd1;
end

//-----------------------------------------------------------------------------
// 六、计算当前像素的 x 坐标
//     前面 H_FP+H_SYNC+H_BP 个点是消隐区，从第 (H_FP+H_SYNC+H_BP) 个点开始算第 0 个有效像素。
//     为什么减 1？因为 h_cnt 是从 0 开始数的，边界要差一位。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		active_x <= 12'd0;
	else if(h_cnt >= H_FP + H_SYNC + H_BP - 1)//horizontal video active - 进入有效显示区
		active_x <= h_cnt - (H_FP[11:0] + H_SYNC[11:0] + H_BP[11:0] - 12'd1);
	else
		active_x <= active_x;    // 消隐区保持上一值（反正不用）
end

//-----------------------------------------------------------------------------
// 七、垂直计数器：注意！不是每个时钟都加，而是"一整行结束"才加 1
//     这里选了 h_cnt == H_FP-1 这个时刻（行同步开始前）作为"换行点"，
//     换成别的时刻其实也行，只要 h_cnt/v_cnt 配合起来时序正确即可。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		v_cnt <= 12'd0;
	else if(h_cnt == H_FP  - 1)//horizontal sync time - 每行到这个点时
		if(v_cnt == V_TOTAL - 1)//vertical counter maximum value - 数满一帧就归零
			v_cnt <= 12'd0;
		else
			v_cnt <= v_cnt + 12'd1;
	else
		v_cnt <= v_cnt;
end

//-----------------------------------------------------------------------------
// 八、产生行同步脉冲 hs_reg
//     在 h_cnt = H_FP-1 时开始拉成 HS_POL，持续 H_SYNC 个点后翻转回去。
//     用 ~hs_reg 而不是直接取反 HS_POL，是为了同时兼容正/负极性。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		hs_reg <= 1'b0;
	else if(h_cnt == H_FP - 1)//horizontal sync begin - 行同步开始
		hs_reg <= HS_POL;
	else if(h_cnt == H_FP + H_SYNC - 1)//horizontal sync end - 行同步结束
		hs_reg <= ~hs_reg;
	else
		hs_reg <= hs_reg;
end

//-----------------------------------------------------------------------------
// 九、行有效标志 h_active：从后肩结束一直拉到行末
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		h_active <= 1'b0;
	else if(h_cnt == H_FP + H_SYNC + H_BP - 1)//horizontal active begin - 行有效开始
		h_active <= 1'b1;
	else if(h_cnt == H_TOTAL - 1)//horizontal active end - 行有效结束
		h_active <= 1'b0;
	else
		h_active <= h_active;
end

//-----------------------------------------------------------------------------
// 十、产生场同步脉冲 vs_reg
//     必须同时满足"垂直计数到点"和"水平计数到点"两个条件，
//     否则同一个 v_cnt 值会在整行里重复出现，vs 会抖动。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		vs_reg <= 1'd0;
	else if((v_cnt == V_FP - 1) && (h_cnt == H_FP - 1))//vertical sync begin - 场同步开始
		vs_reg <= HS_POL;               // 注意：源码写的是 HS_POL，一般应为 VS_POL；
	                                    // 因为本工程两者相同(都是0)所以不影响，修改极性时要留意
	else if((v_cnt == V_FP + V_SYNC - 1) && (h_cnt == H_FP - 1))//vertical sync end - 场同步结束
		vs_reg <= ~vs_reg;  
	else
		vs_reg <= vs_reg;
end

//-----------------------------------------------------------------------------
// 十一、场有效标志 v_active
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		v_active <= 1'd0;
	else if((v_cnt == V_FP + V_SYNC + V_BP - 1) && (h_cnt == H_FP - 1))//vertical active begin - 场有效开始
		v_active <= 1'b1;
	else if((v_cnt == V_TOTAL - 1) && (h_cnt == H_FP - 1)) //vertical active end - 场有效结束
		v_active <= 1'b0;   
	else
		v_active <= v_active;
end

//-----------------------------------------------------------------------------
// 十二、根据 active_x 在有效区内画 8 色彩条
//      做法：把一行 640 个点平均切成 8 份（每份 80 点），
//      在每一份的起点处把颜色寄存器改成对应颜色，之后保持不变直到下一份。
//      —— 这就是"遇到边界就换颜色"的写法，简单且不需要乘法器。
//-----------------------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		begin
			rgb_r_reg <= 8'h00;
			rgb_g_reg <= 8'h00;
			rgb_b_reg <= 8'h00;
		end
	else if(video_active)                      // 只在有效显示区改颜色
		if(active_x == 12'd0)                  // 第 0 份：白
			begin
				rgb_r_reg <= WHITE_R;
				rgb_g_reg <= WHITE_G;
				rgb_b_reg <= WHITE_B;
			end
		else if(active_x == (H_ACTIVE/8) * 1)  // 第 1 份：黄
			begin
				rgb_r_reg <= YELLOW_R;
				rgb_g_reg <= YELLOW_G;
				rgb_b_reg <= YELLOW_B;
			end         
		else if(active_x == (H_ACTIVE/8) * 2)  // 第 2 份：青
			begin
				rgb_r_reg <= CYAN_R;
				rgb_g_reg <= CYAN_G;
				rgb_b_reg <= CYAN_B;
			end
		else if(active_x == (H_ACTIVE/8) * 3)  // 第 3 份：绿
			begin
				rgb_r_reg <= GREEN_R;
				rgb_g_reg <= GREEN_G;
				rgb_b_reg <= GREEN_B;
			end
		else if(active_x == (H_ACTIVE/8) * 4)  // 第 4 份：紫
			begin
				rgb_r_reg <= MAGENTA_R;
				rgb_g_reg <= MAGENTA_G;
				rgb_b_reg <= MAGENTA_B;
			end
		else if(active_x == (H_ACTIVE/8) * 5)  // 第 5 份：红
			begin
				rgb_r_reg <= RED_R;
				rgb_g_reg <= RED_G;
				rgb_b_reg <= RED_B;
			end
		else if(active_x == (H_ACTIVE/8) * 6)  // 第 6 份：蓝
			begin
				rgb_r_reg <= BLUE_R;
				rgb_g_reg <= BLUE_G;
				rgb_b_reg <= BLUE_B;
			end 
		else if(active_x == (H_ACTIVE/8) * 7)  // 第 7 份：黑
			begin
				rgb_r_reg <= BLACK_R;
				rgb_g_reg <= BLACK_G;
				rgb_b_reg <= BLACK_B;
			end
		else                                   // 份内其它点：保持颜色不变
			begin
				rgb_r_reg <= rgb_r_reg;
				rgb_g_reg <= rgb_g_reg;
				rgb_b_reg <= rgb_b_reg;
			end         
	else                                       // 消隐区输出黑（全 0）
		begin
			rgb_r_reg <= 8'h00;
			rgb_g_reg <= 8'h00;
			rgb_b_reg <= 8'h00;
		end
end

endmodule
