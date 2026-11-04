set t0   [clock seconds]
set ROOT "C:/Users/mingbai/Documents/Code/QS_FPGA/src"
set OUT  "$ROOT/td_project/_m_Runs"
file mkdir $OUT

import_device eagle_s20.db -package EG4S20BG256
open_project "HDMI1.4b_Transmitter_v1.0.al"
puts "@@@ OPEN OK"

set_param flow thread 32
set_param rtl  infer_gsr on
set_param rtl  min_control_set 3
set_param rtl  min_ripple_len 1
set_param gate map_mode area_prior
set_param gate pack_effort high
set_param gate pack_seq_in_io on

read_adc "$ROOT/user_source/constraints_source/pin.adc"
read_sdc "$ROOT/user_source/constraints_source/timing.sdc"

elaborate -top top
puts "@@@ ELAB OK [expr {[clock seconds]-$t0}]"

commit_param -step rtl
insert_debugger
optimize_rtl
puts "@@@ RTL OK [expr {[clock seconds]-$t0}]"

commit_param -step gate
optimize_gate "$OUT/m_gate.area"
puts "@@@ GATE OK [expr {[clock seconds]-$t0}]"

commit_param -step place
set rc [catch { place } err]
puts "@@@ PLACE rc=$rc $err"
if { $rc == 0 } {
    report_area -io_info -file "$OUT/m_phy.area"
    report_timing_summary -file "$OUT/m_place.timing"
}
puts "@@@ DONE [expr {[clock seconds]-$t0}]"
exit
