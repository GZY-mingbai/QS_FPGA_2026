# 单独综合 fat_dir_lister，量它的 LUT
set t0 [clock seconds]
set ROOT "C:/Users/mingbai/Documents/Code/QS_FPGA/src"

import_device eagle_s20.db -package EG4S20BG256
open_project "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0.al"

set_param flow thread 32
set_param rtl infer_fsm off
set_param rtl infer_gsr on

read_verilog "$ROOT/user_source/hdl_source/SD/fat_dir_lister.v"

puts "@@@ elaborate lister"
elaborate -top fat_dir_lister

commit_param -step rtl
optimize_rtl

commit_param -step gate
optimize_gate -packarea "$ROOT/td_project/HDMI1.4b_Transmitter_v1.0_Runs/syn_1/lister_gate.area"

puts "@@@ DONE elapsed=[expr {[clock seconds]-$t0}]s"
exit
