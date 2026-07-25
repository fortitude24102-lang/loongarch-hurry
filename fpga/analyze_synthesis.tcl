set project_file [file normalize ./project/Loongson_Soc.xpr]
if {![file exists $project_file]} {
    puts stderr "Vivado project not found: $project_file"
    puts stderr "Run create_project.tcl first"
    exit 2
}

set report_name baseline
if {[info exists ::env(CPU5_REPORT_NAME)] && $::env(CPU5_REPORT_NAME) ne ""} {
    set report_name $::env(CPU5_REPORT_NAME)
}
set report_dir [file normalize "./reports/$report_name"]
file mkdir $report_dir

open_project $project_file
update_compile_order -fileset sources_1

set synth_run [get_runs synth_1]
if {[info exists ::env(CPU5_SYNTH_STRATEGY)] &&
    $::env(CPU5_SYNTH_STRATEGY) ne ""} {
    set_property strategy $::env(CPU5_SYNTH_STRATEGY) $synth_run
}

puts "CPU5_SYNTH_BEGIN: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts "CPU5_SYNTH_STRATEGY: [get_property strategy $synth_run]"
set skip_synth 0
if {[info exists ::env(CPU5_SKIP_SYNTH)] &&
    $::env(CPU5_SKIP_SYNTH) ne "" &&
    $::env(CPU5_SKIP_SYNTH) ne "0"} {
    set skip_synth 1
}
if {!$skip_synth} {
    reset_run synth_1
    launch_runs synth_1 -jobs 4
    wait_on_run synth_1
} else {
    puts "CPU5_SYNTH_REUSE: existing synth_1 result"
}

set progress [get_property PROGRESS $synth_run]
set status [get_property STATUS $synth_run]
if {$progress ne "100%" || ![string match "*Complete*" $status]} {
    puts stderr "Synthesis failed: $status (progress $progress)"
    close_project
    exit 1
}

open_run synth_1
report_utilization -hierarchical -file "$report_dir/utilization_hierarchical.rpt"
report_utilization -file "$report_dir/utilization.rpt"
report_timing_summary -delay_type max -max_paths 20 \
    -file "$report_dir/timing_summary.rpt"
report_high_fanout_nets -fanout_greater_than 100 -max_nets 100 \
    -file "$report_dir/high_fanout.rpt"
report_ram_utilization -file "$report_dir/ram_utilization.rpt"

set worst_paths [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
if {[llength $worst_paths] != 0} {
    puts "CPU5_SYNTH_WNS: [get_property SLACK [lindex $worst_paths 0]]"
}
puts "CPU5_SYNTH_END: [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts "CPU5_REPORT_DIR: $report_dir"

close_project
exit 0
