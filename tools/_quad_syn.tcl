# P1 综合：用 .al 直接跑（绕开 prj 旧快照）
set t0 [clock seconds]

set device_name  eagle_s20.db
set package_name EG4S20BG256
set top_model_name top
set area_option  -packarea

set ROOT "C:/Users/mingbai/Documents/Code/QS_FPGA/src"
set AL   "$ROOT/td_project/_quad_syn.al"
set OUT  "$ROOT/td_project/_quad_Runs/syn_1"

puts "@@@ STEP1 import_device"
import_device $device_name -package $package_name

puts "@@@ STEP2 open_project (.al)"
open_project $AL

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

puts "@@@ DONE elapsed=[expr {[clock seconds]-$t0}]s"
exit
