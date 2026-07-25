// ============================================================================
// core_top — CPU-5 封装（nscscc-solo-la-soc 官方契约）
// ============================================================================
module core_top #(
    parameter TLBNUM = 32
)(
    input           aclk,
    input           aresetn,
    input  [7:0]    intrpt,

    output [3:0]    arid,
    output [31:0]   araddr,
    output [7:0]    arlen,
    output [2:0]    arsize,
    output [1:0]    arburst,
    output [1:0]    arlock,
    output [3:0]    arcache,
    output [2:0]    arprot,
    output          arvalid,
    input           arready,
    input  [3:0]    rid,
    input  [31:0]   rdata,
    input  [1:0]    rresp,
    input           rlast,
    input           rvalid,
    output          rready,

    output [3:0]    awid,
    output [31:0]   awaddr,
    output [7:0]    awlen,
    output [2:0]    awsize,
    output [1:0]    awburst,
    output [1:0]    awlock,
    output [3:0]    awcache,
    output [2:0]    awprot,
    output          awvalid,
    input           awready,
    output [3:0]    wid,
    output [31:0]   wdata,
    output [3:0]    wstrb,
    output          wlast,
    output          wvalid,
    input           wready,
    input  [3:0]    bid,
    input  [1:0]    bresp,
    input           bvalid,
    output          bready,

    input           break_point,
    input           infor_flag,
    input  [4:0]    reg_num,
    output          ws_valid,
    output [31:0]   rf_rdata,

    output [31:0]   debug0_wb_pc,
    output [3:0]    debug0_wb_rf_wen,
    output [4:0]    debug0_wb_rf_wnum,
    output [31:0]   debug0_wb_rf_wdata,
    output [31:0]   debug0_wb_inst
);

    wire cpu_reset = ~aresetn;
    wire [7:0] cpu_arlen, cpu_awlen;
    wire       cpu_wlast;

    assign arid    = 4'h0;
    assign arlen   = cpu_arlen;
    assign arsize  = (araddr[31:20] == 12'h1F0) ? 3'b000 : 3'b010;
    assign arburst = 2'b01;
    assign arlock  = 2'b00;
    assign arcache = 4'h0;
    assign awid    = 4'h0;
    assign awlen   = cpu_awlen;
    assign awsize  = 3'b010;
    assign awburst = 2'b01;
    assign awlock  = 2'b00;
    assign awcache = 4'h0;
    assign wid     = 4'h0;
    assign wlast   = cpu_wlast;

    assign ws_valid = 1'b0;
    assign rf_rdata = 32'b0;

    wire        dbg_wb_we;
    wire [4:0]  dbg_wb_rd;
    wire [31:0] dbg_wb_data;
    wire [3:0]  dbg_dc_state;
    assign debug0_wb_pc       = 32'b0;
    assign debug0_wb_inst     = 32'b0;
    assign debug0_wb_rf_wen   = {4{dbg_wb_we}};
    assign debug0_wb_rf_wnum  = dbg_wb_rd;
    assign debug0_wb_rf_wdata = dbg_wb_data;

    cpu_core u_cpu(
        .clk(aclk), .reset(cpu_reset), .hw_int(intrpt),
        .axi_arvalid(arvalid), .axi_arready(arready),
        .axi_araddr(araddr),   .axi_arprot(arprot),
        .axi_arlen(cpu_arlen),
        .axi_rvalid(rvalid),   .axi_rready(rready),
        .axi_rdata(rdata),     .axi_rresp(rresp),
        .axi_rlast(rlast),
        .axi_awvalid(awvalid), .axi_awready(awready),
        .axi_awaddr(awaddr),   .axi_awprot(awprot),
        .axi_awlen(cpu_awlen),
        .axi_wvalid(wvalid),   .axi_wready(wready),
        .axi_wdata(wdata),     .axi_wstrb(wstrb),
        .axi_wlast(cpu_wlast),
        .axi_bvalid(bvalid),   .axi_bready(bready),
        .axi_bresp(bresp),
        .debug_pc(),
        .debug_wb_we(dbg_wb_we), .debug_wb_rd(dbg_wb_rd), .debug_wb_data(dbg_wb_data),
        .debug_dc_state(dbg_dc_state)
    );

    // ------------------------------------------------------------------
    // 仿真探针（有界心跳，不刷屏）
    // ------------------------------------------------------------------
    // synthesis translate_off
    reg [31:0] ct_cycle;  integer ct_ar_total, ct_aw_total;
    reg [31:0] ct_last_ar, ct_last_aw;  reg ct_rst_seen;
    integer ct_dc_ar, ct_dc_aw;   // D-cache AXI 事务计数
    integer ct_dc_ar_stuck;       // dcache AR 停滞检测
    reg [3:0] ct_dc_state_d1;
    integer ct_r_total;           // R 握手计数，与 AR 比较可检测挂起的 AR
    reg [31:0] ct_ar_inflight;    // 最近一次 AR 地址（用于定位挂起事务）
    reg [31:0] ct_aw_stuck;       // AW 首次未完成握手的时刻（0=已完成或从未发）
    reg [31:0] ct_aw_stuck_addr;  // 卡住的 AW 地址
    integer ct_ar_7f;             // 到 0x1C7Fxxxx (ExtRAM 边界外) 的 AR 计数
    reg ct_trace_en;
    initial begin
        ct_cycle=0; ct_ar_total=0; ct_aw_total=0;
        ct_last_ar=0; ct_last_aw=0; ct_rst_seen=0;
        ct_dc_ar=0; ct_dc_aw=0; ct_dc_ar_stuck=0; ct_dc_state_d1=0;
        ct_r_total=0; ct_ar_inflight=0;
        ct_aw_stuck=0; ct_aw_stuck_addr=0;
        ct_ar_7f=0;
        ct_trace_en=$test$plusargs("cpu5_cache_stats");
    end
    always @(posedge aclk) begin
        ct_cycle <= ct_cycle + 1;
        if (!ct_rst_seen && aresetn === 1'b1) begin
            ct_rst_seen <= 1'b1;
            if (ct_trace_en)
                $display("[CT-RST c=%0d] aresetn released", ct_cycle);
        end
        if (arvalid && arready) begin
            ct_last_ar <= araddr; ct_ar_total = ct_ar_total + 1;
            ct_ar_inflight <= araddr;   // 记录最近 AR 地址
            // 推断 D-cache AR：地址在 ExtRAM 范围 (0x1C400000-0x1C7FFFFF)
            if (araddr[31:22] == 10'h071) ct_dc_ar = ct_dc_ar + 1;
            // 陷阱：坏地址 AR（未映射区域），一次性打印
            if (araddr < 32'h00001000 || (araddr > 32'h1FFFFFFF && araddr < 32'h80000000)) begin
                $display("[CT-BAD-AR c=%0d] addr=%h ar_total=%0d dc_ar=%0d",
                         ct_cycle, araddr, ct_ar_total, ct_dc_ar);
            end
            // ExtRAM 边界外计数 (0x1C7Fxxxx，超出 MIF 覆盖)
            if (araddr[31:12] == 20'h1C7F0) ct_ar_7f = ct_ar_7f + 1;
        end
        // AW 探针：区分"没发"vs"发了没回"
        if (awvalid && !awready && ct_aw_stuck == 0) begin
            ct_aw_stuck <= ct_cycle;   // 记录 AW 第一次未完成握手的时刻
            ct_aw_stuck_addr <= awaddr;
        end
        if (awvalid && awready) begin
            ct_last_aw <= awaddr; ct_aw_total = ct_aw_total + 1;
            ct_aw_stuck <= 0;           // 握手成功，清除
            if (awaddr[31:22] == 10'h071) ct_dc_aw = ct_dc_aw + 1;
        end
        if (rvalid && rready) begin
            ct_r_total = ct_r_total + 1;
        end
        // D-cache 停滞检测：连续 256 拍在同一非 IDLE 状态
        ct_dc_state_d1 <= dbg_dc_state;
        if (dbg_dc_state != 4'b0000 && dbg_dc_state == ct_dc_state_d1)
            ct_dc_ar_stuck = ct_dc_ar_stuck + 1;
        else
            ct_dc_ar_stuck = 0;
        if (ct_trace_en && ct_cycle[15:0] == 16'hFFFF)
            $display("[CT-HB c=%0d] ar=%0d r=%0d aw=%0d | dc_ar=%0d dc_aw=%0d dc_st=%d stuck=%0d | inflight=%h ar_7f=%0d",
                     ct_cycle, ct_ar_total, ct_r_total, ct_aw_total,
                     ct_dc_ar, ct_dc_aw, dbg_dc_state, ct_dc_ar_stuck,
                     ct_ar_inflight, ct_ar_7f);
    end
    // synthesis translate_on

endmodule
