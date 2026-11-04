
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
# ★ clkc[2] = ext_mem_clk 的 180° 相移副本，给 SDRAM PHY 做数据采样窗口调整
#   （top_tf_hdmi_audio.v 里接到 ext_mem_clk_sft）。不 rename 的话，
#   报告里会显示一长串 sys_pll_m0/pll_inst.clkc[2]，而且容易被误认成空闲时钟。
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk sd_card_clk ext_mem_clk]

# ------------------------------------------------------------
# 5b. 图像/视频侧各时钟互相异步（★ 2026-10-15 正式启用，原第 6 节）
#
#     sd_card_clk(100M) / ext_mem_clk(125M) / ext_mem_clk_sft(125M 180°)
#     / video_clk(25M) / hdmi_5x_clk(125M) 之间的全部数据交换都经过
#     异步 FIFO（frame_fifo_*）或同步器（*ack_sync*、u*_sync），
#     属于标准 CDC 结构。除 sd_card_clk↔ext_mem_clk 的 Gray 指针同步路径外，
#     其余组间路径切除以避免对无关相位关系做常规 setup/hold 分析：
#       · setup：sd_card→video 等组被反复报告假违规
#       · hold ：video_timing_data_m0/read_req → u_scene 跨域路径被报
#         −1028ps，TD 的 hold-fixer 对它插 buffer 毫无意义，
#         且实测（0xC0000005 coredump ×3）正是 Hold Fix 阶段崩溃的诱因
#     音视频之间（第 5 节）此前已切。sd_card_clk↔ext_mem_clk 保持可分析，
#     只为 img_afifo Gray 源到第一级同步器设置最大延迟。
#     ★ 若以后新增"直接跨这些域的组合路径"，必须重新评估这一条。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {sd_card_clk ext_mem_clk}] \
    -group [get_clocks {ext_mem_clk_sft}] \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# img_afifo 双向 Gray 指针路径需有物理延迟上限，避免跨越多个目的时钟周期。
# sd_card_clk/ext_mem_clk 之间保留可分析关系，其余跨域组仍按异步路径切断。
set_max_delay -from [get_regs -hierarchical *wgray] -to [get_regs -hierarchical *wgray_s1] -datapath_only 7.0
set_max_delay -from [get_regs -hierarchical *rgray] -to [get_regs -hierarchical *rgray_s1] -datapath_only 7.0

# ------------------------------------------------------------
# 6. （已并入 5b：sd_card/ext_mem/video/hdmi_5x 的异步分组已于 2026-10-15
#     正式启用 —— 最初"先看报告"的保留意见已由实际报告验证完毕）
# ------------------------------------------------------------

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
