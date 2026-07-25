set project_file [file normalize ./project/Loongson_Soc.xpr]
if {![file exists $project_file]} {
    puts stderr "Vivado project not found: $project_file"
    exit 2
}

set report_name impl_default
if {[info exists ::env(CPU5_REPORT_NAME)] && $::env(CPU5_REPORT_NAME) ne ""} {
    set report_name $::env(CPU5_REPORT_NAME)
}
set report_dir [file normalize "./reports/$report_name"]
file mkdir $report_dir

open_project $project_file
set synth_run [get_runs synth_1]
set synth_progress [get_property PROGRESS $synth_run]
set synth_status [get_property STATUS $synth_run]
if {$synth_progress ne "100%" || ![string match "*Complete*" $synth_status]} {
    puts stderr "Synthesis is not complete: $synth_status ($synth_progress)"
    close_project
    exit 1
}

set impl_run [get_runs impl_1]
if {[info exists ::env(CPU5_IMPL_STRATEGY)] &&
    $::env(CPU5_IMPL_STRATEGY) ne ""} {
    set_property strategy $::env(CPU5_IMPL_STRATEGY) $impl_run
}

puts "CPU5_IMPL_BEGIN: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts "CPU5_IMPL_STRATEGY: [get_property strategy $impl_run]"
reset_run impl_1
launch_runs impl_1 -to_step route_design -jobs 4
wait_on_run impl_1

set progress [get_property PROGRESS $impl_run]
set status [get_property STATUS $impl_run]
if {$progress ne "100%" || ![string match "*Complete*" $status]} {
    puts stderr "Implementation failed: $status (progress $progress)"
    close_project
    exit 1
}

open_run impl_1
report_utilization -file "$report_dir/utilization.rpt"
report_timing_summary -delay_type min_max -max_paths 20 \
    -file "$report_dir/timing_summary.rpt"
report_route_status -file "$report_dir/route_status.rpt"
report_clock_utilization -file "$report_dir/clock_utilization.rpt"
report_high_fanout_nets -timing -fanout_greater_than 100 -max_nets 100 \
    -file "$report_dir/high_fanout.rpt"
report_design_analysis -congestion -file "$report_dir/congestion.rpt"

set setup_paths [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
if {[llength $setup_paths] != 0} {
    puts "CPU5_IMPL_WNS: [get_property SLACK [lindex $setup_paths 0]]"
}
set hold_paths [get_timing_paths -delay_type min -max_paths 1 -nworst 1]
if {[llength $hold_paths] != 0} {
    puts "CPU5_IMPL_WHS: [get_property SLACK [lindex $hold_paths 0]]"
}
puts "CPU5_IMPL_END: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts "CPU5_REPORT_DIR: $report_dir"

close_project
exit 0
