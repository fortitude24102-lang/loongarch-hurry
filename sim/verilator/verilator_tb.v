`timescale 1ns / 1ps
`include "config.h"

module verilator_tb (
    input         clk,
    input         reset,
    input         uart_rx,
    output        uart_tx,
    output        uart_display,
    output [7:0]  uart_data,
    output        ext_ram_write_fire,
    output [19:0] ext_ram_write_addr,
    output [31:0] ext_ram_write_data,
    output [3:0]  ext_ram_write_be_n,
    input  [19:0] ext_ram_dump_addr,
    output [31:0] ext_ram_dump_data,
    output [31:0] debug_wb_pc,
    output [31:0] debug_wb_inst,
    output [3:0]  debug_wb_rf_wen,
    output [4:0]  debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata,
    output        cpu_ar_fire,
    output [31:0] cpu_ar_addr,
    output        cpu_aw_fire,
    output [31:0] cpu_aw_addr,
    output        data_uncache_en,
    output        data_valid,
    output        data_op,
    output        data_addr_ok,
    output [7:0]  data_index,
    output [19:0] data_tag,
    output [3:0]  data_offset,
    output [3:0]  data_wstrb,
    output [31:0] data_wdata,
    output [31:0] data_rd_addr,
    output [31:0] data_vaddr,
    output        data_wr_req,
    output [31:0] data_wr_addr,
    output [127:0] data_wr_data,
    output [4:0]  dcache_main_state,
    output        dcache_cache_hit,
    output [1:0]  dcache_way_hit,
    output        dcache_write_full,
    output        dcache_req_op,
    output [7:0]  dcache_req_index,
    output [3:0]  dcache_req_offset,
    output [31:0] dcache_req_wdata,
    output [7:0]  dcache_write_index,
    output [3:0]  dcache_write_offset,
    output [31:0] dcache_write_wdata,
    output [1:0]  dcache_write_way,
    output        dcache_req_dcacop,
    output [1:0]  dcache_req_cacop_mode,
    output [1:0]  dcache_way_d,
    output [1:0]  dcache_replace_way,
    output        dcache_replace_d,
    output        dcache_replace_v,
    output [19:0] dcache_replace_tag,
    output        csr_da,
    output        csr_pg,
    output [31:0] csr_dmw1,

    // CPU5 simulation-only performance events.  The C++ driver accumulates
    // them strictly between the supervisor 0x06/0x07 UART markers.
    output        perf_instret,
    output        perf_branch,
    output        perf_branch_mispredict,
    output        perf_ex_valid,
    output        perf_icache_access,
    output        perf_icache_miss,
    output        perf_icache_miss_active,
    output        perf_dcache_access,
    output        perf_dcache_miss,
    output        perf_dcache_wait,
    output        perf_direct_wait,
    output        perf_dcache_complete,
    output        perf_load_use_stall,
    output        perf_store_stall,
    output        perf_mul_stall,
    output        perf_div_stall,
    output        perf_mem_structural_stall,
    output        perf_pipeline_stall,
    output        perf_dcache_writeback_active,
    output        perf_sram_read_beat,
    output        perf_sram_write_beat,
    output        perf_uncached_access,
    output        perf_store_buffer_full,
    output        perf_store_buffer_enqueue,
    output        perf_store_buffer_forward,
    output [2:0]  perf_store_buffer_count,
    output        perf_dcache_forced_bypass,
    output        perf_dcache_adaptive_bypass,
    output        perf_dcache_read_wait_store,

    // Stage-4 dependency probes.
    output        perf_id_load_ex1_stall,
    output        perf_id_load_mem_stall,
    output        perf_id_mul_ex1_stall,
    output        perf_id_mul_ex2_stall,
    output        perf_ex_load_ex2_stall,
    output        perf_ex_load_exmem_stall,
    output        perf_ex_load_mem1_stall,

    // Stage-5 branch-predictor probes.
    output        perf_branch_pred_taken,
    output        perf_branch_actual_taken,
    output        perf_branch_correct_taken,
    output        perf_branch_correct_taken_flush,
    output        perf_fetch_pred_taken_btb_miss
);

wire [2:0]  video_red;
wire [2:0]  video_green;
wire [1:0]  video_blue;
wire        video_hsync;
wire        video_vsync;
wire        video_clk;
wire        video_de;
wire [15:0] leds;
wire [7:0]  dpy0;
wire [7:0]  dpy1;

wire [19:0] base_ram_addr;
wire [3:0]  base_ram_be_n;
wire        base_ram_ce_n;
wire        base_ram_oe_n;
wire        base_ram_we_n;
wire [31:0] base_ram_data;

wire [19:0] ext_ram_addr;
wire [3:0]  ext_ram_be_n;
wire        ext_ram_ce_n;
wire        ext_ram_oe_n;
wire        ext_ram_we_n;
wire [31:0] ext_ram_data;

wire [3:0]  touch_btn = 4'h0;
wire [31:0] dip_sw    = 32'h0000_abcd;
wire        uart_rx_line;

assign uart_rx_line = uart_rx;

soc_top #(.SIMULATION(1'b1)) u_soc_top (
    .clk           (clk),
    .reset         (reset),
    .video_red     (video_red),
    .video_green   (video_green),
    .video_blue    (video_blue),
    .video_hsync   (video_hsync),
    .video_vsync   (video_vsync),
    .video_clk     (video_clk),
    .video_de      (video_de),
    .touch_btn     (touch_btn),
    .dip_sw        (dip_sw),
    .leds          (leds),
    .dpy0          (dpy0),
    .dpy1          (dpy1),
    .base_ram_data (base_ram_data),
    .base_ram_addr (base_ram_addr),
    .base_ram_be_n (base_ram_be_n),
    .base_ram_ce_n (base_ram_ce_n),
    .base_ram_oe_n (base_ram_oe_n),
    .base_ram_we_n (base_ram_we_n),
    .ext_ram_data  (ext_ram_data),
    .ext_ram_addr  (ext_ram_addr),
    .ext_ram_be_n  (ext_ram_be_n),
    .ext_ram_ce_n  (ext_ram_ce_n),
    .ext_ram_oe_n  (ext_ram_oe_n),
    .ext_ram_we_n  (ext_ram_we_n),
    .UART_RX       (uart_rx_line),
    .UART_TX       (uart_tx)
);

sram_sp #(
    .AW        (20),
    .Init_File ("none"),
    .Init_Plusarg("base_ram_mif=%s"))
base_sram_sp (
    .ram_addr (base_ram_addr),
    .ram_be_n (base_ram_be_n),
    .ram_ce_n (base_ram_ce_n),
    .ram_oe_n (base_ram_oe_n),
    .ram_we_n (base_ram_we_n),
    .ram_data (base_ram_data)
);

sram_sp #(
    .AW        (20),
    .Init_File ("none"),
    .Init_Plusarg("ext_ram_mif=%s"))
ext_sram_sp (
    .ram_addr (ext_ram_addr),
    .ram_be_n (ext_ram_be_n),
    .ram_ce_n (ext_ram_ce_n),
    .ram_oe_n (ext_ram_oe_n),
    .ram_we_n (ext_ram_we_n),
    .ram_data (ext_ram_data)
);

wire uart_wen = u_soc_top.u_axi_uart_controller.uart0.PSEL &&
                u_soc_top.u_axi_uart_controller.uart0.PENABLE &&
                u_soc_top.u_axi_uart_controller.uart0.PWRITE;

assign uart_display = uart_wen &&
                      (u_soc_top.u_axi_uart_controller.uart0.PADDR[7:0] == 8'h0) &&
                      !u_soc_top.u_axi_uart_controller.uart0.regs.lcr[7];
assign uart_data    = u_soc_top.u_axi_uart_controller.uart0.PWDATA[7:0];

assign ext_ram_write_fire = (ext_ram_ce_n === 1'b0) && (ext_ram_we_n === 1'b0);
assign ext_ram_write_addr = ext_ram_addr;
assign ext_ram_write_data = ext_ram_data;
assign ext_ram_write_be_n = ext_ram_be_n;
assign ext_ram_dump_data  = ext_sram_sp.BRAM[ext_ram_dump_addr[19:0]];
assign debug_wb_pc        = u_soc_top.debug_wb_pc;
assign debug_wb_inst      = u_soc_top.debug_wb_inst;
assign debug_wb_rf_wen    = u_soc_top.debug_wb_rf_wen;
assign debug_wb_rf_wnum   = u_soc_top.debug_wb_rf_wnum;
assign debug_wb_rf_wdata  = u_soc_top.debug_wb_rf_wdata;
assign cpu_ar_fire        = u_soc_top.cpu_arvalid && u_soc_top.cpu_arready;
assign cpu_ar_addr        = u_soc_top.cpu_araddr;
assign cpu_aw_fire        = u_soc_top.cpu_awvalid && u_soc_top.cpu_awready;
assign cpu_aw_addr        = u_soc_top.cpu_awaddr;

// Opt-in transaction latency probe for the adaptive single-beat D-cache read
// path.  It is silent in normal regressions and intentionally bounded so a
// CRYPTONIGHT run can expose fixed handshake latency without flooding logs.
integer cpu5_lat_cycle;
integer cpu5_lat_start;
integer cpu5_lat_samples;
reg     cpu5_lat_active;
reg     cpu5_lat_cpu_ar_seen;
reg     cpu5_lat_sync_ar_seen;
reg     cpu5_lat_ram_ar_seen;
reg     cpu5_lat_ram_r_seen;
reg     cpu5_lat_sync_r_seen;

always @(posedge clk) begin
    if (reset) begin
        cpu5_lat_cycle        <= 0;
        cpu5_lat_start        <= 0;
        cpu5_lat_samples      <= 0;
        cpu5_lat_active       <= 1'b0;
        cpu5_lat_cpu_ar_seen  <= 1'b0;
        cpu5_lat_sync_ar_seen <= 1'b0;
        cpu5_lat_ram_ar_seen  <= 1'b0;
        cpu5_lat_ram_r_seen   <= 1'b0;
        cpu5_lat_sync_r_seen  <= 1'b0;
    end else begin
        cpu5_lat_cycle <= cpu5_lat_cycle + 1;

        if ($test$plusargs("cpu5_axi_latency_trace") &&
            !cpu5_lat_active && cpu5_lat_samples < 8 &&
            (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b1011) &&
            u_soc_top.u_cpu.u_cpu.dcache_axi_req) begin
            cpu5_lat_start        <= cpu5_lat_cycle;
            cpu5_lat_active       <= 1'b1;
            cpu5_lat_cpu_ar_seen  <= 1'b0;
            cpu5_lat_sync_ar_seen <= 1'b0;
            cpu5_lat_ram_ar_seen  <= 1'b0;
            cpu5_lat_ram_r_seen   <= 1'b0;
            cpu5_lat_sync_r_seen  <= 1'b0;
            $display("[AXI-LAT] sample=%0d start cycle=%0d addr=%08x",
                     cpu5_lat_samples, cpu5_lat_cycle,
                     u_soc_top.u_cpu.u_cpu.dcache_axi_addr);
        end

        if (cpu5_lat_active) begin
            if (!cpu5_lat_cpu_ar_seen &&
                u_soc_top.cpu_arvalid && u_soc_top.cpu_arready) begin
                cpu5_lat_cpu_ar_seen <= 1'b1;
                $display("[AXI-LAT] sample=%0d cpu_ar +%0d",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start);
            end
            if (!cpu5_lat_sync_ar_seen &&
                u_soc_top.cpu_sync_arvalid && u_soc_top.cpu_sync_arready) begin
                cpu5_lat_sync_ar_seen <= 1'b1;
                $display("[AXI-LAT] sample=%0d sync_ar +%0d",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start);
            end
            if (!cpu5_lat_ram_ar_seen &&
                u_soc_top.ram_arvalid && u_soc_top.ram_arready) begin
                cpu5_lat_ram_ar_seen <= 1'b1;
                $display("[AXI-LAT] sample=%0d ram_ar +%0d",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start);
            end
            if (!cpu5_lat_ram_r_seen &&
                u_soc_top.ram_rvalid && u_soc_top.ram_rready) begin
                cpu5_lat_ram_r_seen <= 1'b1;
                $display("[AXI-LAT] sample=%0d ram_r +%0d",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start);
            end
            if (!cpu5_lat_sync_r_seen &&
                u_soc_top.cpu_sync_rvalid && u_soc_top.cpu_sync_rready) begin
                cpu5_lat_sync_r_seen <= 1'b1;
                $display("[AXI-LAT] sample=%0d sync_r +%0d",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start);
            end
            if (u_soc_top.u_cpu.u_cpu.u_dcache.axi_valid &&
                u_soc_top.u_cpu.u_cpu.u_dcache.axi_ready) begin
                $display("[AXI-LAT] sample=%0d done +%0d cpu_ready=%0d state=%0h",
                         cpu5_lat_samples, cpu5_lat_cycle - cpu5_lat_start,
                         u_soc_top.u_cpu.u_cpu.u_dcache.ready,
                         u_soc_top.u_cpu.u_cpu.u_dcache.state);
                cpu5_lat_active  <= 1'b0;
                cpu5_lat_samples <= cpu5_lat_samples + 1;
            end
        end
    end
end

// Performance-counter probes.  The C++ driver uses the supervisor's 0x06/0x07
// UART_DATA writes as the measured window, excluding boot and serialization.
wire cpu5_profile_commit =
    u_soc_top.u_cpu.u_cpu.mem_wb_valid;
wire cpu5_profile_branch =
    u_soc_top.u_cpu.u_cpu.ex1_valid &&
    u_soc_top.u_cpu.u_cpu.ex1_br_r &&
    !u_soc_top.u_cpu.u_cpu.ex_stall &&
    u_soc_top.u_cpu.u_cpu.ex_mem_ready;
wire cpu5_profile_branch_mispredict =
    cpu5_profile_branch && u_soc_top.u_cpu.u_cpu.br_flush_any;
wire cpu5_profile_icache_access =
    u_soc_top.u_cpu.u_cpu.icache_pair_valid &&
    !u_soc_top.u_cpu.u_cpu.icache_busy &&
    u_soc_top.u_cpu.u_cpu.if1_valid &&
    u_soc_top.u_cpu.u_cpu.if_id_ready;
wire cpu5_profile_icache_miss =
    (u_soc_top.u_cpu.u_cpu.u_icache.state == 2'b00) &&
    u_soc_top.u_cpu.u_cpu.u_icache.req &&
    !u_soc_top.u_cpu.u_cpu.u_icache.hit &&
    !u_soc_top.u_cpu.u_cpu.u_icache.cacop_req;
wire cpu5_profile_dcache_access =
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b0000) &&
    u_soc_top.u_cpu.u_cpu.u_dcache.req &&
    !u_soc_top.u_cpu.u_cpu.u_dcache.flush_active &&
    !u_soc_top.u_cpu.u_cpu.u_dcache.cacop_req;
wire cpu5_profile_dcache_miss =
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b0001) &&
    !u_soc_top.u_cpu.u_cpu.u_dcache.srd_hit &&
    !u_soc_top.u_cpu.u_cpu.u_dcache.cacop_req;
wire cpu5_profile_cached_complete =
    u_soc_top.u_cpu.u_cpu.mem1_valid &&
    (u_soc_top.u_cpu.u_cpu.mem1_mr ||
     u_soc_top.u_cpu.u_cpu.mem1_mw) &&
    !u_soc_top.u_cpu.u_cpu.bypass_dcache &&
    u_soc_top.u_cpu.u_cpu.dcache_ready;
wire cpu5_profile_load_stall =
    u_soc_top.u_cpu.u_cpu.load_use_stall ||
    u_soc_top.u_cpu.u_cpu.ex2_ld_hazard ||
    (u_soc_top.u_cpu.u_cpu.ex_stall &&
     !u_soc_top.u_cpu.u_cpu.mul_busy &&
     !u_soc_top.u_cpu.u_cpu.cacop_busy);

assign perf_instret          = cpu5_profile_commit;
assign perf_branch           = cpu5_profile_branch;
assign perf_branch_mispredict= cpu5_profile_branch_mispredict;
assign perf_ex_valid         = u_soc_top.u_cpu.u_cpu.ex_valid;
assign perf_icache_access    = cpu5_profile_icache_access;
assign perf_icache_miss      = cpu5_profile_icache_miss;
assign perf_icache_miss_active =
    (u_soc_top.u_cpu.u_cpu.u_icache.state == 2'b10);
assign perf_dcache_access    = cpu5_profile_dcache_access;
assign perf_dcache_miss      = cpu5_profile_dcache_miss;
assign perf_dcache_wait      =
    u_soc_top.u_cpu.u_cpu.mem1_valid &&
    (u_soc_top.u_cpu.u_cpu.mem1_mr ||
     u_soc_top.u_cpu.u_cpu.mem1_mw) &&
    !u_soc_top.u_cpu.u_cpu.bypass_dcache &&
    !u_soc_top.u_cpu.u_cpu.dcache_ready;
assign perf_direct_wait      =
    u_soc_top.u_cpu.u_cpu.mem1_valid &&
    (u_soc_top.u_cpu.u_cpu.mem1_mr ||
     u_soc_top.u_cpu.u_cpu.mem1_mw) &&
    u_soc_top.u_cpu.u_cpu.bypass_dcache &&
    !u_soc_top.u_cpu.u_cpu.dcache_ready;
assign perf_dcache_complete  = cpu5_profile_cached_complete;
assign perf_load_use_stall   = cpu5_profile_load_stall;
assign perf_store_stall      =
    u_soc_top.u_cpu.u_cpu.mem1_valid &&
    u_soc_top.u_cpu.u_cpu.mem1_mw &&
    !u_soc_top.u_cpu.u_cpu.dcache_ready;
assign perf_mul_stall        = u_soc_top.u_cpu.u_cpu.mul_busy;
assign perf_div_stall        = 1'b0;  // This CPU has no implemented divider.
assign perf_mem_structural_stall =
    u_soc_top.u_cpu.u_cpu.if1_valid &&
    !u_soc_top.u_cpu.u_cpu.icache_bus_grant;
assign perf_pipeline_stall = u_soc_top.u_cpu.u_cpu.pc_stall;
assign perf_dcache_writeback_active =
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b1001) ||
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b0100) ||
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b0011);
assign perf_sram_read_beat =
    u_soc_top.u_cpu.u_cpu.axi_rvalid &&
    u_soc_top.u_cpu.u_cpu.axi_rready;
assign perf_sram_write_beat =
    u_soc_top.u_cpu.u_cpu.axi_wvalid &&
    u_soc_top.u_cpu.u_cpu.axi_wready;
assign perf_uncached_access =
    u_soc_top.u_cpu.u_cpu.mem1_valid &&
    (u_soc_top.u_cpu.u_cpu.mem1_mr ||
     u_soc_top.u_cpu.u_cpu.mem1_mw) &&
    u_soc_top.u_cpu.u_cpu.bypass_dcache &&
    u_soc_top.u_cpu.u_cpu.dcache_ready;
assign perf_store_buffer_full =
    u_soc_top.u_cpu.u_cpu.u_dcache.store_buffer_full;
assign perf_store_buffer_enqueue =
    u_soc_top.u_cpu.u_cpu.u_dcache.store_buffer_enqueue;
assign perf_store_buffer_forward =
    u_soc_top.u_cpu.u_cpu.u_dcache.store_buffer_forward;
assign perf_store_buffer_count =
    u_soc_top.u_cpu.u_cpu.u_dcache.store_buffer_count;
assign perf_dcache_forced_bypass =
    u_soc_top.u_cpu.u_cpu.cpu5_force_dcache_bypass;
assign perf_dcache_adaptive_bypass =
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b1011) ||
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b1100);
assign perf_dcache_read_wait_store =
    (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'b1111);
assign perf_id_load_ex1_stall =
    u_soc_top.u_cpu.u_cpu.id_valid &&
    u_soc_top.u_cpu.u_cpu.u_id.load_in_ex1;
assign perf_id_load_mem_stall =
    u_soc_top.u_cpu.u_cpu.id_valid &&
    u_soc_top.u_cpu.u_cpu.u_id.load_in_mem;
assign perf_id_mul_ex1_stall =
    u_soc_top.u_cpu.u_cpu.id_valid &&
    u_soc_top.u_cpu.u_cpu.u_id.mul_in_ex1;
assign perf_id_mul_ex2_stall =
    u_soc_top.u_cpu.u_cpu.id_valid &&
    u_soc_top.u_cpu.u_cpu.u_id.mul_in_ex2;
assign perf_ex_load_ex2_stall =
    u_soc_top.u_cpu.u_cpu.ex2_ld_hazard;
assign perf_ex_load_exmem_stall =
    u_soc_top.u_cpu.u_cpu.ex_valid &&
    u_soc_top.u_cpu.u_cpu.mem_mem_read &&
    (u_soc_top.u_cpu.u_cpu.mem_rd != 5'd0) &&
    ((u_soc_top.u_cpu.u_cpu.mem_rd == u_soc_top.u_cpu.u_cpu.ex_rs1_addr) ||
     (u_soc_top.u_cpu.u_cpu.mem_rd == u_soc_top.u_cpu.u_cpu.ex_rs2_addr));
assign perf_ex_load_mem1_stall =
    u_soc_top.u_cpu.u_cpu.ex_valid &&
    u_soc_top.u_cpu.u_cpu.mem1_mr &&
    (u_soc_top.u_cpu.u_cpu.mem1_rd != 5'd0) &&
    ((u_soc_top.u_cpu.u_cpu.mem1_rd == u_soc_top.u_cpu.u_cpu.ex_rs1_addr) ||
     (u_soc_top.u_cpu.u_cpu.mem1_rd == u_soc_top.u_cpu.u_cpu.ex_rs2_addr));

assign perf_branch_pred_taken =
    cpu5_profile_branch && u_soc_top.u_cpu.u_cpu.ex1_br_pred;
assign perf_branch_actual_taken =
    cpu5_profile_branch && u_soc_top.u_cpu.u_cpu.ex_branch_taken;
assign perf_branch_correct_taken =
    cpu5_profile_branch &&
    u_soc_top.u_cpu.u_cpu.ex1_br_pred &&
    u_soc_top.u_cpu.u_cpu.ex_branch_taken;
assign perf_branch_correct_taken_flush =
    perf_branch_correct_taken &&
    (u_soc_top.u_cpu.u_cpu.if_flush || u_soc_top.u_cpu.u_cpu.id_flush);
assign perf_fetch_pred_taken_btb_miss =
    u_soc_top.u_cpu.u_cpu.if1_allowin &&
    u_soc_top.u_cpu.u_cpu.icache_bus_grant &&
    !u_soc_top.u_cpu.u_cpu.icache_busy &&
    u_soc_top.u_cpu.u_cpu.pred_taken &&
    !u_soc_top.u_cpu.u_cpu.u_bpred.btb_hit;

`ifdef MYCPU_OPENLA500_PROBES
assign data_uncache_en    = u_soc_top.u_cpu.data_uncache_en;
assign data_valid         = u_soc_top.u_cpu.data_valid;
assign data_op            = u_soc_top.u_cpu.data_op;
assign data_addr_ok       = u_soc_top.u_cpu.data_addr_ok;
assign data_index         = u_soc_top.u_cpu.data_index;
assign data_tag           = u_soc_top.u_cpu.data_tag;
assign data_offset        = u_soc_top.u_cpu.data_offset;
assign data_wstrb         = u_soc_top.u_cpu.data_wstrb;
assign data_wdata         = u_soc_top.u_cpu.data_wdata;
assign data_rd_addr       = u_soc_top.u_cpu.data_rd_addr;
assign data_vaddr         = u_soc_top.u_cpu.data_vaddr;
assign data_wr_req        = u_soc_top.u_cpu.data_wr_req;
assign data_wr_addr       = u_soc_top.u_cpu.data_wr_addr;
assign data_wr_data       = u_soc_top.u_cpu.data_wr_data;
assign dcache_main_state  = u_soc_top.u_cpu.dcache.main_state;
assign dcache_cache_hit   = u_soc_top.u_cpu.dcache.cache_hit;
assign dcache_way_hit     = u_soc_top.u_cpu.dcache.way_hit;
assign dcache_write_full  = u_soc_top.u_cpu.dcache.write_state_is_full;
assign dcache_req_op      = u_soc_top.u_cpu.dcache.request_buffer_op;
assign dcache_req_index   = u_soc_top.u_cpu.dcache.request_buffer_index;
assign dcache_req_offset  = u_soc_top.u_cpu.dcache.request_buffer_offset;
assign dcache_req_wdata   = u_soc_top.u_cpu.dcache.request_buffer_wdata;
assign dcache_write_index = u_soc_top.u_cpu.dcache.write_buffer_index;
assign dcache_write_offset= u_soc_top.u_cpu.dcache.write_buffer_offset;
assign dcache_write_wdata = u_soc_top.u_cpu.dcache.write_buffer_wdata;
assign dcache_write_way   = u_soc_top.u_cpu.dcache.write_buffer_way;
assign dcache_req_dcacop  = u_soc_top.u_cpu.dcache.request_buffer_dcacop;
assign dcache_req_cacop_mode = u_soc_top.u_cpu.dcache.request_buffer_cacop_op_mode;
assign dcache_way_d       = u_soc_top.u_cpu.dcache.way_d;
assign dcache_replace_way = u_soc_top.u_cpu.dcache.replace_way;
assign dcache_replace_d   = u_soc_top.u_cpu.dcache.replace_d;
assign dcache_replace_v   = u_soc_top.u_cpu.dcache.replace_v;
assign dcache_replace_tag = u_soc_top.u_cpu.dcache.replace_tag;
assign csr_da             = u_soc_top.u_cpu.csr_da;
assign csr_pg             = u_soc_top.u_cpu.csr_pg;
assign csr_dmw1           = u_soc_top.u_cpu.csr_dmw1;
`else
assign data_uncache_en    = 1'b0;
assign data_valid         = 1'b0;
assign data_op            = 1'b0;
assign data_addr_ok       = 1'b0;
assign data_index         = 8'b0;
assign data_tag           = 20'b0;
assign data_offset        = 4'b0;
assign data_wstrb         = 4'b0;
assign data_wdata         = 32'b0;
assign data_rd_addr       = 32'b0;
assign data_vaddr         = 32'b0;
assign data_wr_req        = 1'b0;
assign data_wr_addr       = 32'b0;
assign data_wr_data       = 128'b0;
assign dcache_main_state  = 5'b0;
assign dcache_cache_hit   = 1'b0;
assign dcache_way_hit     = 2'b0;
assign dcache_write_full  = 1'b0;
assign dcache_req_op      = 1'b0;
assign dcache_req_index   = 8'b0;
assign dcache_req_offset  = 4'b0;
assign dcache_req_wdata   = 32'b0;
assign dcache_write_index = 8'b0;
assign dcache_write_offset= 4'b0;
assign dcache_write_wdata = 32'b0;
assign dcache_write_way   = 2'b0;
assign dcache_req_dcacop  = 1'b0;
assign dcache_req_cacop_mode = 2'b0;
assign dcache_way_d       = 2'b0;
assign dcache_replace_way = 2'b0;
assign dcache_replace_d   = 1'b0;
assign dcache_replace_v   = 1'b0;
assign dcache_replace_tag = 20'b0;
assign csr_da             = 1'b0;
assign csr_pg             = 1'b0;
assign csr_dmw1           = 32'b0;
`endif

// Focused boot/CSR probe. Enable with +cpu5_csr_probe.
// Keep this in the testbench so diagnosis does not alter CPU behavior.
reg        cpu5_csr_probe_en;
reg [31:0] cpu5_csr_probe_cycle;
reg [31:0] cpu5_csr_probe_prev_pc;
reg        cpu5_csr_probe_prev_exception;
reg        cpu5_uart_probe_en;
reg [7:0]  cpu5_uart_probe_count;
reg        cpu5_ctrl_probe_en;
reg [7:0]  cpu5_ctrl_probe_count;
reg        cpu5_stream_probe_en;
reg [9:0]  cpu5_stream_probe_count;
reg        cpu5_dcache_probe_en;
reg [31:0] cpu5_dcache_probe_addr;
reg [7:0]  cpu5_dcache_probe_count;
reg        cpu5_stream_data_probe_en;
reg [31:0] cpu5_stream_data_probe_addr;
reg [7:0]  cpu5_stream_data_probe_count;
reg        cpu5_matrix_data_probe_en;
reg [31:0] cpu5_matrix_data_probe_addr;
reg [7:0]  cpu5_matrix_data_probe_count;

initial begin
    cpu5_csr_probe_en             = $test$plusargs("cpu5_csr_probe");
    cpu5_uart_probe_en            = $test$plusargs("cpu5_uart_probe");
    cpu5_ctrl_probe_en            = $test$plusargs("cpu5_ctrl_probe");
    cpu5_stream_probe_en          = $test$plusargs("cpu5_stream_probe");
    cpu5_dcache_probe_en          =
        $value$plusargs("cpu5_dcache_addr=%h", cpu5_dcache_probe_addr);
    cpu5_stream_data_probe_en     =
        $value$plusargs("cpu5_stream_data_addr=%h",
                        cpu5_stream_data_probe_addr);
    cpu5_matrix_data_probe_en     =
        $value$plusargs("cpu5_matrix_data_addr=%h",
                        cpu5_matrix_data_probe_addr);
    cpu5_csr_probe_cycle          = 32'b0;
    cpu5_csr_probe_prev_pc        = 32'hffff_ffff;
    cpu5_csr_probe_prev_exception = 1'b0;
    cpu5_uart_probe_count         = 8'b0;
    cpu5_ctrl_probe_count         = 8'b0;
    cpu5_stream_probe_count       = 10'b0;
    cpu5_dcache_probe_count       = 8'b0;
    cpu5_stream_data_probe_count  = 8'b0;
    cpu5_matrix_data_probe_count  = 8'b0;
end

always @(posedge clk) begin
    if (reset) begin
        cpu5_csr_probe_cycle          <= 32'b0;
        cpu5_csr_probe_prev_pc        <= 32'hffff_ffff;
        cpu5_csr_probe_prev_exception <= 1'b0;
        cpu5_uart_probe_count         <= 8'b0;
        cpu5_ctrl_probe_count         <= 8'b0;
        cpu5_stream_probe_count       <= 10'b0;
        cpu5_dcache_probe_count       <= 8'b0;
        cpu5_stream_data_probe_count  <= 8'b0;
        cpu5_matrix_data_probe_count  <= 8'b0;
    end
    else begin
        cpu5_csr_probe_cycle          <= cpu5_csr_probe_cycle + 1'b1;
        cpu5_csr_probe_prev_pc        <= u_soc_top.u_cpu.u_cpu.pc;
        cpu5_csr_probe_prev_exception <= u_soc_top.u_cpu.u_cpu.exception;

        if (cpu5_csr_probe_en) begin
            if (u_soc_top.u_cpu.u_cpu.id_valid &&
                ((u_soc_top.u_cpu.u_cpu.id_inst[31:24] == 8'h04) ||
                 u_soc_top.u_cpu.u_cpu.illegal_id)) begin
                $display("[CPU5-CSR-ID c=%0d] pc=%08h inst=%08h valid=%b illegal=%b rd=%b wr=%b xchg=%b rj=%0d rd_num=%0d",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.id_pc_out,
                         u_soc_top.u_cpu.u_cpu.id_inst,
                         u_soc_top.u_cpu.u_cpu.id_valid,
                         u_soc_top.u_cpu.u_cpu.illegal_id,
                         u_soc_top.u_cpu.u_cpu.id_csr_rd,
                         u_soc_top.u_cpu.u_cpu.id_csr_wr,
                         u_soc_top.u_cpu.u_cpu.id_csr_xchg,
                         u_soc_top.u_cpu.u_cpu.id_inst[9:5],
                         u_soc_top.u_cpu.u_cpu.id_inst[4:0]);
            end

            if (u_soc_top.u_cpu.u_cpu.exception &&
                !cpu5_csr_probe_prev_exception) begin
                $display("[CPU5-EXCEPTION c=%0d] pc=%08h ex_illegal=%b next_pc=%08h eentry=%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.pc,
                         u_soc_top.u_cpu.u_cpu.ex_ine,
                         u_soc_top.u_cpu.u_cpu.next_pc,
                         u_soc_top.u_cpu.u_cpu.eentry);
            end

            if ((u_soc_top.u_cpu.u_cpu.pc == 32'h1c00_0000) &&
                (cpu5_csr_probe_prev_pc != 32'h1c00_0000)) begin
                $display("[CPU5-BOOT-ENTRY c=%0d] pc returned to reset entry",
                         cpu5_csr_probe_cycle);
            end

            if (u_soc_top.u_cpu.u_cpu.mem_wb_valid &&
                (u_soc_top.u_cpu.u_cpu.wb_csr_rd ||
                 u_soc_top.u_cpu.u_cpu.wb_csr_wr ||
                 u_soc_top.u_cpu.u_cpu.wb_csr_xchg)) begin
                $display("[CPU5-CSR-WB c=%0d] addr=%04h rd=%b wr=%b xchg=%b wdata=%08h wmask=%08h rdata=%08h crmd=%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.wb_csr_addr,
                         u_soc_top.u_cpu.u_cpu.wb_csr_rd,
                         u_soc_top.u_cpu.u_cpu.wb_csr_wr,
                         u_soc_top.u_cpu.u_cpu.wb_csr_xchg,
                         u_soc_top.u_cpu.u_cpu.wb_csr_wdata,
                         u_soc_top.u_cpu.u_cpu.wb_csr_wmask,
                         u_soc_top.u_cpu.u_cpu.csr_rdata,
                         u_soc_top.u_cpu.u_cpu.crmd);
            end
        end

        // Print once per completed UART transaction, not once per stalled cycle.
        if (cpu5_uart_probe_en &&
            (cpu5_uart_probe_count < 8'd32) &&
            u_soc_top.u_cpu.u_cpu.direct_req &&
            (u_soc_top.u_cpu.u_cpu.direct_paddr[31:20] == 12'h1f0) &&
            (u_soc_top.u_cpu.u_cpu.direct_we ?
             u_soc_top.u_cpu.u_cpu.arb_direct_wr_ready :
             u_soc_top.u_cpu.u_cpu.arb_direct_rd_ready)) begin
            cpu5_uart_probe_count <= cpu5_uart_probe_count + 1'b1;
            $display("[CPU5-UART-DONE c=%0d] va=%08h pa=%08h wr=%b strb=%h wdata=%08h rdata=%08h",
                     cpu5_csr_probe_cycle,
                     u_soc_top.u_cpu.u_cpu.direct_addr,
                     u_soc_top.u_cpu.u_cpu.direct_paddr,
                     u_soc_top.u_cpu.u_cpu.direct_we,
                     u_soc_top.u_cpu.u_cpu.mem1_byte_we,
                     u_soc_top.u_cpu.u_cpu.dcache_wdata,
                     u_soc_top.u_cpu.u_cpu.arb_direct_rdata);
        end

        if (cpu5_ctrl_probe_en && (cpu5_ctrl_probe_count < 8'd192)) begin
            if (u_soc_top.u_cpu.u_cpu.cacop_fire &&
                (u_soc_top.u_cpu.u_cpu.ex1_cacop_code_r[2:0] == 3'b001)) begin
                cpu5_ctrl_probe_count <= cpu5_ctrl_probe_count + 1'b1;
                $display("[CPU5-CACOP-FIRE c=%0d] pc=%08h code=%02h addr=%08h dc_req=%b ic_req=%b",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.ex1_pc_r,
                         u_soc_top.u_cpu.u_cpu.ex1_cacop_code_r,
                         u_soc_top.u_cpu.u_cpu.ex_alu_result,
                         u_soc_top.u_cpu.u_cpu.dc_cacop_req,
                         u_soc_top.u_cpu.u_cpu.ic_cacop_req);
            end
            else if (u_soc_top.u_cpu.u_cpu.dc_cacop_done) begin
                cpu5_ctrl_probe_count <= cpu5_ctrl_probe_count + 1'b1;
                $display("[CPU5-CACOP-DONE c=%0d] last_pc=%08h dc_done=%b ic_done=%b dc_state=%0d",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.last_cacop_pc,
                         u_soc_top.u_cpu.u_cpu.dc_cacop_done,
                         u_soc_top.u_cpu.u_cpu.ic_cacop_done,
                         u_soc_top.u_cpu.u_cpu.dcache_dbg_state);
            end
            else if ((u_soc_top.u_cpu.u_cpu.pc[31:24] != 8'h1c) &&
                     (u_soc_top.u_cpu.u_cpu.pc != 32'h8000_0000)) begin
                cpu5_ctrl_probe_count <= cpu5_ctrl_probe_count + 1'b1;
                $display("[CPU5-PC-ESCAPE c=%0d] pc=%08h next=%08h ex_pc=%08h ex1_pc=%08h br=%b target=%08h pred=%b/%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.pc,
                         u_soc_top.u_cpu.u_cpu.next_pc,
                         u_soc_top.u_cpu.u_cpu.ex_pc,
                         u_soc_top.u_cpu.u_cpu.ex1_pc_r,
                         u_soc_top.u_cpu.u_cpu.ex_branch_taken,
                         u_soc_top.u_cpu.u_cpu.ex_branch_target,
                         u_soc_top.u_cpu.u_cpu.pred_taken,
                         u_soc_top.u_cpu.u_cpu.pred_target);
            end
        end

        // Current STREAM failure probe: capture the first pass through the
        // entry/setup/loop window and the complete ready/valid chain.
        if (cpu5_stream_probe_en &&
            (cpu5_stream_probe_count < 10'd400) &&
            (u_soc_top.u_cpu.u_cpu.pc >= 32'h1c00_2008) &&
            (u_soc_top.u_cpu.u_cpu.pc <= 32'h1c00_2038)) begin
            cpu5_stream_probe_count <= cpu5_stream_probe_count + 1'b1;
            $display("[CPU5-STREAM c=%0d] pc=%08h next=%08h pst=%b | if1=%b/%08h allow=%b pair=%b ifid_rdy=%b | id=%b/%08h/%08h rdy=%b | ex=%b/%08h ex1r=%b | ex1=%b/%08h exmr=%b | exm=%b/%08h m1r=%b | m1=%b/%08h rd=%b wr=%b mrdy=%b | dc=%b/%08h st=%0d ax=%b/%08h ar=%b/%08h rdp=%b | r4=%08h r5=%08h r6=%08h",
                     cpu5_csr_probe_cycle,
                     u_soc_top.u_cpu.u_cpu.pc,
                     u_soc_top.u_cpu.u_cpu.next_pc,
                     u_soc_top.u_cpu.u_cpu.pc_stall,
                     u_soc_top.u_cpu.u_cpu.if1_valid,
                     u_soc_top.u_cpu.u_cpu.if1_pc,
                     u_soc_top.u_cpu.u_cpu.if1_allowin,
                     u_soc_top.u_cpu.u_cpu.icache_pair_valid,
                     u_soc_top.u_cpu.u_cpu.if_id_ready,
                     u_soc_top.u_cpu.u_cpu.id_valid,
                     u_soc_top.u_cpu.u_cpu.id_pc_out,
                     u_soc_top.u_cpu.u_cpu.id_inst,
                     u_soc_top.u_cpu.u_cpu.id_ready,
                     u_soc_top.u_cpu.u_cpu.ex_valid,
                     u_soc_top.u_cpu.u_cpu.ex_pc,
                     u_soc_top.u_cpu.u_cpu.ex1_ready,
                     u_soc_top.u_cpu.u_cpu.ex1_valid,
                     u_soc_top.u_cpu.u_cpu.ex1_pc_r,
                     u_soc_top.u_cpu.u_cpu.ex_mem_ready,
                     u_soc_top.u_cpu.u_cpu.ex_mem_valid,
                     u_soc_top.u_cpu.u_cpu.mem_addr_in,
                     u_soc_top.u_cpu.u_cpu.mem1_ready,
                     u_soc_top.u_cpu.u_cpu.mem1_valid,
                     u_soc_top.u_cpu.u_cpu.mem1_alu,
                     u_soc_top.u_cpu.u_cpu.mem1_mr,
                     u_soc_top.u_cpu.u_cpu.mem1_mw,
                     u_soc_top.u_cpu.u_cpu.mem_ready_out,
                     u_soc_top.u_cpu.u_cpu.dcache_req,
                     u_soc_top.u_cpu.u_cpu.dcache_addr,
                     u_soc_top.u_cpu.u_cpu.dcache_dbg_state,
                     u_soc_top.u_cpu.u_cpu.dcache_axi_req,
                     u_soc_top.u_cpu.u_cpu.dcache_axi_addr,
                     u_soc_top.u_cpu.u_cpu.axi_arvalid,
                     u_soc_top.u_cpu.u_cpu.axi_araddr,
                     u_soc_top.u_cpu.u_cpu.u_axi_arbiter.rd_pending,
                     u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[4],
                     u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[5],
                     u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[6]);
        end

        if (cpu5_dcache_probe_en && (cpu5_dcache_probe_count < 8'd128)) begin
            if (u_soc_top.u_cpu.u_cpu.dcache_req &&
                (u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'd0) &&
                (u_soc_top.u_cpu.u_cpu.dcache_addr[31:2] ==
                 cpu5_dcache_probe_addr[31:2])) begin
                cpu5_dcache_probe_count <= cpu5_dcache_probe_count + 1'b1;
                $display("[CPU5-DC-REQ c=%0d] addr=%08h wr=%b data=%08h be=%b",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.dcache_addr,
                         u_soc_top.u_cpu.u_cpu.mem1_mw,
                         u_soc_top.u_cpu.u_cpu.dcache_wdata,
                         u_soc_top.u_cpu.u_cpu.mem1_byte_we);
            end
            else if (u_soc_top.u_cpu.u_cpu.u_dcache.bram_we &&
                     (u_soc_top.u_cpu.u_cpu.u_dcache.tag_r ==
                      cpu5_dcache_probe_addr[31:13]) &&
                     (u_soc_top.u_cpu.u_cpu.u_dcache.bram_wr_addr[10:3] ==
                      cpu5_dcache_probe_addr[12:5]) &&
                     (u_soc_top.u_cpu.u_cpu.u_dcache.bram_wr_addr[2:0] ==
                      cpu5_dcache_probe_addr[4:2])) begin
                cpu5_dcache_probe_count <= cpu5_dcache_probe_count + 1'b1;
                $display("[CPU5-DC-BRAM-W c=%0d] st=%0d way=%0d data=%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.u_dcache.state,
                         u_soc_top.u_cpu.u_cpu.u_dcache.bram_wr_addr[12:11],
                         u_soc_top.u_cpu.u_cpu.u_dcache.bram_wr_data);
            end
            else if ((u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'd9) &&
                     u_soc_top.u_cpu.u_cpu.u_dcache.wb_fill_cap &&
                     (u_soc_top.u_cpu.u_cpu.u_dcache.tag_r ==
                      cpu5_dcache_probe_addr[31:13])) begin
                cpu5_dcache_probe_count <= cpu5_dcache_probe_count + 1'b1;
                $display("[CPU5-DC-WB-CAP c=%0d] slot=%0d rdaddr=%04h data=%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.u_dcache.wb_buf_cnt,
                         u_soc_top.u_cpu.u_cpu.u_dcache.bram_rd_addr,
                         u_soc_top.u_cpu.u_cpu.u_dcache.bram_rd_data);
            end
            else if ((u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'd3) &&
                     u_soc_top.u_cpu.u_cpu.dcache_axi_wnext &&
                     (u_soc_top.u_cpu.u_cpu.dcache_axi_addr[31:5] ==
                      cpu5_dcache_probe_addr[31:5])) begin
                cpu5_dcache_probe_count <= cpu5_dcache_probe_count + 1'b1;
                $display("[CPU5-DC-WB-BEAT c=%0d] beat=%0d data=%08h",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.u_dcache.wb_cnt,
                         u_soc_top.u_cpu.u_cpu.dcache_axi_wdata);
            end
            else if ((u_soc_top.u_cpu.u_cpu.u_dcache.state == 4'd10) &&
                     (u_soc_top.u_cpu.u_cpu.u_dcache.cacop_idx ==
                      cpu5_dcache_probe_addr[12:5])) begin
                cpu5_dcache_probe_count <= cpu5_dcache_probe_count + 1'b1;
                $display("[CPU5-DC-CACOP c=%0d] code=%02h way=%0d wb_active=%b",
                         cpu5_csr_probe_cycle,
                         u_soc_top.u_cpu.u_cpu.u_dcache.cacop_code_r,
                         u_soc_top.u_cpu.u_cpu.u_dcache.cacop_way,
                         u_soc_top.u_cpu.u_cpu.u_dcache.cacop_wb_active);
            end
        end

        // STREAM load-to-store probe.  Gate it around one destination word so
        // the log shows the complete RAW-hazard timeline without flooding.
        if (cpu5_stream_data_probe_en &&
            (cpu5_stream_data_probe_count < 8'd160) &&
            ((u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[5] >=
              cpu5_stream_data_probe_addr - 32'd4) &&
             (u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[5] <=
              cpu5_stream_data_probe_addr + 32'd8)) &&
            ((u_soc_top.u_cpu.u_cpu.ex_valid &&
              ((u_soc_top.u_cpu.u_cpu.ex_pc == 32'h1c00_2018) ||
               (u_soc_top.u_cpu.u_cpu.ex_pc == 32'h1c00_201c))) ||
             (u_soc_top.u_cpu.u_cpu.ex1_valid &&
              ((u_soc_top.u_cpu.u_cpu.ex1_pc_r == 32'h1c00_2018) ||
               (u_soc_top.u_cpu.u_cpu.ex1_pc_r == 32'h1c00_201c))) ||
             u_soc_top.u_cpu.u_cpu.ex_mem_valid ||
             u_soc_top.u_cpu.u_cpu.mem1_valid ||
             u_soc_top.u_cpu.u_cpu.mem_valid_out)) begin
            cpu5_stream_data_probe_count <= cpu5_stream_data_probe_count + 1'b1;
            $display("[CPU5-STREAM-DATA c=%0d] ex=%b/%08h rs=%0d,%0d raw=%08h,%08h fwd=%0d,%0d src=%08h,%08h stl=%b ld2=%b | ex1=%b/%08h src=%08h,%08h rd=%0d mr=%b mw=%b | exm=%b addr=%08h data=%08h rd=%0d mr=%b mw=%b | m1=%b addr=%08h data=%08h rd=%0d mr=%b mw=%b rdy=%b dc=%08h/%b | m2v=%b rd=%0d rw=%b result=%08h | wb=%b rd=%0d data=%08h",
                     cpu5_csr_probe_cycle,
                     u_soc_top.u_cpu.u_cpu.ex_valid,
                     u_soc_top.u_cpu.u_cpu.ex_pc,
                     u_soc_top.u_cpu.u_cpu.ex_rs1_addr,
                     u_soc_top.u_cpu.u_cpu.ex_rs2_addr,
                     u_soc_top.u_cpu.u_cpu.ex_rs1,
                     u_soc_top.u_cpu.u_cpu.ex_rs2,
                     u_soc_top.u_cpu.u_cpu.forward_a,
                     u_soc_top.u_cpu.u_cpu.forward_b,
                     u_soc_top.u_cpu.u_cpu.ex1_src1,
                     u_soc_top.u_cpu.u_cpu.ex1_src2,
                     u_soc_top.u_cpu.u_cpu.ex_stall,
                     u_soc_top.u_cpu.u_cpu.ex2_ld_hazard,
                     u_soc_top.u_cpu.u_cpu.ex1_valid,
                     u_soc_top.u_cpu.u_cpu.ex1_pc_r,
                     u_soc_top.u_cpu.u_cpu.ex1_src1_r,
                     u_soc_top.u_cpu.u_cpu.ex1_src2_r,
                     u_soc_top.u_cpu.u_cpu.ex1_rd_r,
                     u_soc_top.u_cpu.u_cpu.ex1_mr_r,
                     u_soc_top.u_cpu.u_cpu.ex1_mw_r,
                     u_soc_top.u_cpu.u_cpu.ex_mem_valid,
                     u_soc_top.u_cpu.u_cpu.mem_addr_in,
                     u_soc_top.u_cpu.u_cpu.mem_wdata_in,
                     u_soc_top.u_cpu.u_cpu.mem_rd,
                     u_soc_top.u_cpu.u_cpu.mem_mem_read,
                     u_soc_top.u_cpu.u_cpu.mem_mem_write,
                     u_soc_top.u_cpu.u_cpu.mem1_valid,
                     u_soc_top.u_cpu.u_cpu.mem1_alu,
                     u_soc_top.u_cpu.u_cpu.mem1_rs2,
                     u_soc_top.u_cpu.u_cpu.mem1_rd,
                     u_soc_top.u_cpu.u_cpu.mem1_mr,
                     u_soc_top.u_cpu.u_cpu.mem1_mw,
                     u_soc_top.u_cpu.u_cpu.dcache_ready,
                     u_soc_top.u_cpu.u_cpu.dcache_rdata,
                     u_soc_top.u_cpu.u_cpu.dcache_req,
                     u_soc_top.u_cpu.u_cpu.mem_valid_out,
                     u_soc_top.u_cpu.u_cpu.mem_rd_reg,
                     u_soc_top.u_cpu.u_cpu.mem_reg_write_reg,
                     u_soc_top.u_cpu.u_cpu.mem_result,
                     u_soc_top.u_cpu.u_cpu.wb_reg_write,
                     u_soc_top.u_cpu.u_cpu.wb_rd,
                     u_soc_top.u_cpu.u_cpu.wb_data);
        end

        // MATRIX lane3 probe: follow mul/add/store retirement for one C word
        // across several outer-loop accumulations.
        if (cpu5_matrix_data_probe_en &&
            (cpu5_matrix_data_probe_count < 8'd200) &&
            (u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[8] >=
             cpu5_matrix_data_probe_addr) &&
            (u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[8] <=
             cpu5_matrix_data_probe_addr + 32'd16) &&
            ((u_soc_top.u_cpu.u_cpu.ex_valid &&
              (u_soc_top.u_cpu.u_cpu.ex_pc >= 32'h1c00_2098) &&
              (u_soc_top.u_cpu.u_cpu.ex_pc <= 32'h1c00_20c4)) ||
             (u_soc_top.u_cpu.u_cpu.ex1_valid &&
              (u_soc_top.u_cpu.u_cpu.ex1_pc_r >= 32'h1c00_2098) &&
              (u_soc_top.u_cpu.u_cpu.ex1_pc_r <= 32'h1c00_20c4)) ||
             (u_soc_top.u_cpu.u_cpu.ex_mem_valid &&
              ((u_soc_top.u_cpu.u_cpu.mem_rd == 5'd24) ||
               (u_soc_top.u_cpu.u_cpu.mem_rd == 5'd28))) ||
             (u_soc_top.u_cpu.u_cpu.mem_valid_out &&
              ((u_soc_top.u_cpu.u_cpu.mem_rd_reg == 5'd24) ||
               (u_soc_top.u_cpu.u_cpu.mem_rd_reg == 5'd28))) ||
             (u_soc_top.u_cpu.u_cpu.wb_reg_write &&
              ((u_soc_top.u_cpu.u_cpu.wb_rd == 5'd24) ||
               (u_soc_top.u_cpu.u_cpu.wb_rd == 5'd28))))) begin
            cpu5_matrix_data_probe_count <= cpu5_matrix_data_probe_count + 1'b1;
            $display("[CPU5-MATRIX-DATA c=%0d] ex=%b/%08h rd=%0d raw=%08h,%08h fwd=%0d,%0d src=%08h,%08h stl=%b | ex1=%b/%08h rd=%0d src=%08h,%08h mul=%b cnt=%0d p=%08h alu=%08h | exm=%b rd=%0d val=%08h data=%08h | m1=%b rd=%0d addr=%08h data=%08h mr=%b mw=%b rdy=%b dc=%08h | m2=%b rd=%0d rw=%b val=%08h | wb=%b rd=%0d val=%08h | r24=%08h r28=%08h",
                     cpu5_csr_probe_cycle,
                     u_soc_top.u_cpu.u_cpu.ex_valid,
                     u_soc_top.u_cpu.u_cpu.ex_pc,
                     u_soc_top.u_cpu.u_cpu.ex_rd,
                     u_soc_top.u_cpu.u_cpu.ex_rs1,
                     u_soc_top.u_cpu.u_cpu.ex_rs2,
                     u_soc_top.u_cpu.u_cpu.forward_a,
                     u_soc_top.u_cpu.u_cpu.forward_b,
                     u_soc_top.u_cpu.u_cpu.ex1_src1,
                     u_soc_top.u_cpu.u_cpu.ex1_src2,
                     u_soc_top.u_cpu.u_cpu.ex_stall,
                     u_soc_top.u_cpu.u_cpu.ex1_valid,
                     u_soc_top.u_cpu.u_cpu.ex1_pc_r,
                     u_soc_top.u_cpu.u_cpu.ex1_rd_r,
                     u_soc_top.u_cpu.u_cpu.ex1_src1_r,
                     u_soc_top.u_cpu.u_cpu.ex1_src2_r,
                     u_soc_top.u_cpu.u_cpu.ex1_is_mul_r,
                     u_soc_top.u_cpu.u_cpu.mul_cnt,
                     u_soc_top.u_cpu.u_cpu.mul_p[31:0],
                     u_soc_top.u_cpu.u_cpu.ex_alu_result,
                     u_soc_top.u_cpu.u_cpu.ex_mem_valid,
                     u_soc_top.u_cpu.u_cpu.mem_rd,
                     u_soc_top.u_cpu.u_cpu.mem_addr_in,
                     u_soc_top.u_cpu.u_cpu.mem_wdata_in,
                     u_soc_top.u_cpu.u_cpu.mem1_valid,
                     u_soc_top.u_cpu.u_cpu.mem1_rd,
                     u_soc_top.u_cpu.u_cpu.mem1_alu,
                     u_soc_top.u_cpu.u_cpu.mem1_rs2,
                     u_soc_top.u_cpu.u_cpu.mem1_mr,
                     u_soc_top.u_cpu.u_cpu.mem1_mw,
                     u_soc_top.u_cpu.u_cpu.dcache_ready,
                     u_soc_top.u_cpu.u_cpu.dcache_rdata,
                     u_soc_top.u_cpu.u_cpu.mem_valid_out,
                     u_soc_top.u_cpu.u_cpu.mem_rd_reg,
                     u_soc_top.u_cpu.u_cpu.mem_reg_write_reg,
                     u_soc_top.u_cpu.u_cpu.mem_result,
                     u_soc_top.u_cpu.u_cpu.wb_reg_write,
                     u_soc_top.u_cpu.u_cpu.wb_rd,
                     u_soc_top.u_cpu.u_cpu.wb_data,
                     u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[24],
                     u_soc_top.u_cpu.u_cpu.u_id.u_regfile.rf[28]);
        end
    end
end

endmodule
