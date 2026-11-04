//==============================================================================
//  fat_quota_gate.v —— FAT 客户（fat_dir_lister）在"装载器忙"期间的限流器
//==============================================================================
//  【为什么需要它】
//  原来的写法是**全屏蔽**：
//      fat_req_g = lst_sd_req & ~((lst_own == OWN_MM) & ld_busy);
//  —— 只要"match 扫描 + 装载器忙"同时成立，FAT 客户就一个扇区都拿不到。
//  而 lister 的上界 `sec_left <= MAX_SECS(64)` 是**按扇区递减**的：一个扇区都拿不到，
//  就没有任何上界 ⇒ `O_busy` 永久为 1 ⇒
//      · 相册永远重扫不了（album_src 等 `!I_lst_busy`）⇒ alb_ready 永久 0
//      · 之后再也发不出任何装载（hd_take/tl_take 都要 alb_ok）
//      · 画面停在最后一帧 = "卡死"
//  而"不打断图片装载"这个目标只需要**限速**，不需要**全停**。
//
//  【做法】配额：每放过 N 个 IMG 扇区，就允许 FAT 通过**一个**扇区（`pass` 保持到
//  该扇区真的结束，所以不会半路撤回请求 —— 中途撤 req 会让 lister 读到残余字节）。
//  装载器空闲时（i_block=0）完全透传，与原行为逐位相同。
//
//  【代价】1 个 4 位计数 + 1 个 pass 位 ≈ 6 LUT；不引入任何定时器。
//==============================================================================
module fat_quota_gate #(
    parameter integer N = 8            // 每 N 个 IMG 扇区放 1 个 FAT 扇区
) (
    input  wire clk,
    input  wire rst,
    input  wire i_block,               // 1 = 需要限流的窗口（MM 扫描 & ld_busy）
    input  wire i_req,                 // FAT 客户的扇区请求（电平保持到拿到数据）
    input  wire i_img_end,             // IMG 客户一次扇区结束（单拍）
    input  wire i_fat_end,             // FAT 客户一次扇区结束（单拍）
    output wire o_req                  // 给仲裁器的 FAT 请求
);

    // 用 $clog2 定宽，N=8 时正好 4 位（与 MAX_SECS 的计数风格一致）
    localparam integer W = (N <= 1) ? 1 : $clog2(N + 1);

    reg [W-1:0] cnt;                   // 本窗口内已放过的 IMG 扇区数
    reg         pass;                  // 1 = 已放行一个 FAT 扇区，等它结束

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt  <= {W{1'b0}};
            pass <= 1'b0;
        end else if (!i_block) begin
            // 装载器空闲：回到起点，透传
            cnt  <= {W{1'b0}};
            pass <= 1'b0;
        end else if (i_fat_end && pass) begin
            // 放行的那一个 FAT 扇区收尾 ⇒ 重新计数
            pass <= 1'b0;
            cnt  <= {W{1'b0}};
        end else if (i_img_end) begin
            if (cnt == (N - 1)) begin
                cnt  <= {W{1'b0}};
                pass <= 1'b1;          // 下一拍起允许 FAT 通过一个扇区
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

    // 不在限流窗口 ⇒ 完全透传；在窗口内 ⇒ 只有 pass 有效时才透传
    //   （pass 一直保持到该 FAT 扇区结束，所以请求不会中途被撤）
    assign o_req = i_req & (~i_block | pass);

endmodule
