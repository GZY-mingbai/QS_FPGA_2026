# ============================================================================
#  _fat_phy.tcl —— 用"带新文件的临时 .al"跑完整综合 + 布局布线，报告落到正式工程
# ============================================================================
#  【怎么加新文件而不改 .al】主线 .al 由 TD GUI 管理，不许手改（见 PLAN §13）。
#  实测 `add_files <绝对路径>` 可以在**内存里的工程**上追加源文件：
#      · 不会写回 .al（脚本里没有 save_project）
#      · 运行目录仍是正式工程的 HDMI1.4b_Transmitter_v1.0_Runs
#      ⇒ 报告直接落到 phy_1，"看资源只认 phy_1"的纪律不用破。
#  （试过"临时 .al 副本 + 改名"，TD 直接报 PRJ-8643：文件名必须和工程名一致，
#    而副本要么改名丢工程名、要么覆盖正式文件 —— 两条路都不通。）
#
#  【怎么跑】td_commands_prompt.exe < tools/_fat_phy.tcl
#  注意：这条命令不会自己退出，必须等日志出现 "@@@ DONE" 再手动结束进程。
# ============================================================================
set t0 [clock seconds]

set device_name  eagle_s20.db
set package_name EG4S20BG256
set top_model_name top
set area_option  -packarea

set ROOT "C:/Users/mingbai/Documents/Code/QS_FPGA/src"
set AL   "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0.al"
set OUT  "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0_Runs/syn_1"
set PHY  "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0_Runs/phy_1"

puts "@@@ STEP1 import_device"
import_device $device_name -package $package_name

puts "@@@ STEP2 open_project (.al)"
open_project $AL

# ---- 新文件（GUI 的 .al 里还没有，内存里补上；不 save_project）----
puts "@@@ STEP2a add_files"
add_files "$ROOT/user_source/hdl_source/SD/ph_writer.v"
add_files "$ROOT/user_source/hdl_source/APP/album_src.v"

puts "@@@ STEP2b set_param"
set_param flow thread 32
set_param rtl infer_fsm off
set_param rtl infer_gsr on

puts "@@@ STEP2c read_adc / read_sdc"
read_adc "$ROOT/user_source/constraints_source/pin.adc"
read_sdc "$ROOT/user_source/constraints_source/timing.sdc"

puts "@@@ STEP3 elaborate"
elaborate -top $top_model_name

puts "@@@ STEP4 optimize_rtl"
commit_param -step rtl
insert_debugger
optimize_rtl
report_area -file "$OUT/syn_rtl.area"

puts "@@@ STEP5 optimize_gate"
commit_param -step gate
optimize_gate $area_option "$OUT/syn_gate.area"
legalize_phy_inst
update_timing

puts "@@@ STEP6 place"
commit_param -step place
insert_debugger
place
update_timing -mode manhattan
report_timing_summary -file "$PHY/place.timing"

puts "@@@ STEP7 route"
commit_param -step route
route
update_timing -mode final
report_timing_summary -file "$PHY/route.timing"
report_area -io_info -file "$PHY/HDMI1.4b_Transmitter_v1.0_phy.area"

puts "@@@ DONE elapsed=[expr {[clock seconds]-$t0}]s"
exit
