# 用 .al 直接跑综合（绕开 prj 旧快照问题）
# 用法：cd 到 src/td_project 后，把本脚本喂给 td_commands_prompt.exe
set t0 [clock seconds]

set device_name  eagle_s20.db
set package_name EG4S20BG256
set top_model_name top
set area_option  -packarea

set ROOT "C:/Users/mingbai/Documents/Code/QS_FPGA/src"
set AL   "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0.al"

puts "@@@ STEP1 import_device"
import_device $device_name -package $package_name

puts "@@@ STEP2 open_project (.al)"
open_project $AL

puts "@@@ STEP2b set_param"
set_param flow thread 32
set_param rtl infer_fsm off
set_param rtl infer_gsr on

puts "@@@ STEP3 elaborate"
elaborate -top $top_model_name
puts "@@@ ELAB_DONE"

puts "@@@ DONE elapsed=[expr {[clock seconds]-$t0}]s"
exit
