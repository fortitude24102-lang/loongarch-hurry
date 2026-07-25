// ============================================================================
// axi_arbiter — 3-source AXI arbitration with burst support
//
// Sources:
//   1. I-cache fill (read only, lowest priority)
//   2. D-cache miss/writeback (read + write)
//   3. Direct bypass (UART/ExtRAM, read + write, highest priority)
//
// Direct bypass is always single-beat (len=0).
// I-cache and D-cache use 8-beat bursts (len=7) for line fills/writebacks.
//
// AXI channels: AR, R, AW, W, B (5 independent channels)
// ============================================================================
module axi_arbiter(
    input  clk,
    input  reset,

    // ---- I-cache AXI-like (source 1, lowest priority) ----
    input        ic_axi_req,
    input  [31:0] ic_axi_addr,
    input  [7:0]  ic_axi_len,       // burst 长度
    output [31:0] ic_axi_rdata,
    output        ic_axi_valid,

    // ---- D-cache AXI-like (source 2) ----
    input        dc_axi_req,
    input  [31:0] dc_axi_addr,
    input        dc_axi_we,
    input  [31:0] dc_axi_wdata,
    input  [3:0]  dc_axi_wstrb,
    input  [7:0]  dc_axi_len,       // burst 长度
    output [31:0] dc_axi_rdata,
    output        dc_axi_valid,
    output        dc_axi_wr_done,   // B 响应脉冲（写 burst 完成）
    output        dc_axi_wnext,     // 写 burst 下一字请求（每 W beat 一脉冲）

    // ---- Independent D-cache store-buffer writer ----
    input         sb_axi_req,
    input  [31:0] sb_axi_addr,
    input  [31:0] sb_axi_wdata,
    input  [3:0]  sb_axi_wstrb,
    output        sb_axi_wr_done,

    // ---- Direct bypass (source 3, highest priority) ----
    input        direct_req,
    input  [31:0] direct_addr,
    input        direct_we,
    input  [31:0] direct_wdata,
    input  [3:0]  direct_wstrb,
    output [31:0] direct_rdata,
    output        direct_ready,
    output        direct_rd_ready,
    output        direct_wr_ready,

    // ---- AXI Master Outputs ----
    // AR channel
    output reg         axi_arvalid,
    input              axi_arready,
    output reg  [31:0] axi_araddr,
    output reg  [7:0]  axi_arlen,
    output      [2:0]  axi_arprot,

    // R channel
    input              axi_rvalid,
    output             axi_rready,
    input       [31:0] axi_rdata,
    input       [1:0]  axi_rresp,
    input              axi_rlast,

    // AW channel
    output reg         axi_awvalid,
    input              axi_awready,
    output reg  [31:0] axi_awaddr,
    output reg  [7:0]  axi_awlen,
    output      [2:0]  axi_awprot,

    // W channel (wdata/wstrb 用 wire 组合直通，避免 dcache burst 1 拍延迟)
    output reg         axi_wvalid,
    input              axi_wready,
    output      [31:0] axi_wdata,
    output      [3:0]  axi_wstrb,
    output reg         axi_wlast,

    // B channel
    input              axi_bvalid,
    output             axi_bready,
    input       [1:0]  axi_bresp
);

    // ================================================================
    // Source tracking
    // ================================================================
    localparam SRC_NONE   = 2'b00;
    localparam SRC_ICACHE = 2'b01;
    localparam SRC_DCACHE = 2'b10;
    localparam SRC_DIRECT = 2'b11;

    wire direct_rd = direct_req && !direct_we;
    wire direct_wr = direct_req &&  direct_we;
    wire dc_rd     = dc_axi_req && !dc_axi_we;
    wire dc_wr     = dc_axi_req &&  dc_axi_we;
    wire sb_wr     = sb_axi_req;

    // 读通道仲裁：direct 读 > dcache 读 > icache 读
    wire rd_sel_direct = direct_rd;
    wire rd_sel_dc     = dc_rd && !direct_rd;
    wire rd_sel_ic     = ic_axi_req && !direct_rd && !dc_rd;
    wire read_req      = rd_sel_direct || rd_sel_dc || rd_sel_ic;

    // 写通道仲裁：direct 写 > dcache 写
    wire wr_sel_direct = direct_wr;
    wire wr_sel_dc     = dc_wr && !direct_wr;
    wire wr_sel_sb     = sb_wr && !direct_wr && !dc_wr;
    wire write_req     = wr_sel_direct || wr_sel_dc || wr_sel_sb;

    // ================================================================
    // Read path: AR + R (burst-aware)
    // ================================================================
    reg [1:0]  rd_src;
    reg        rd_pending;
    reg [7:0]  rd_beat_cnt;
    reg [7:0]  rd_total_len;

    assign axi_rready  = axi_rvalid;
    assign axi_arprot  = 3'b000;

    always @(posedge clk) begin
        if (reset) begin
            axi_arvalid  <= 1'b0;
            axi_araddr   <= 32'h0;
            axi_arlen    <= 8'h00;
            rd_src       <= SRC_NONE;
            rd_pending   <= 1'b0;
            rd_beat_cnt  <= 8'h00;
            rd_total_len <= 8'h00;
        end else begin
            // Issue AR
            if (!rd_pending && read_req) begin
                axi_arvalid <= 1'b1;
                axi_araddr  <= rd_sel_direct ? direct_addr :
                               rd_sel_dc     ? dc_axi_addr : ic_axi_addr;
                axi_arlen   <= rd_sel_direct ? 8'h00 :
                               rd_sel_dc     ? dc_axi_len : ic_axi_len;
                rd_src      <= rd_sel_direct ? SRC_DIRECT :
                               rd_sel_dc     ? SRC_DCACHE : SRC_ICACHE;
                rd_total_len <= rd_sel_direct ? 8'h00 :
                                rd_sel_dc     ? dc_axi_len : ic_axi_len;
                rd_pending   <= 1'b1;
                rd_beat_cnt  <= 8'h00;
            end else if (axi_arvalid && axi_arready) begin
                axi_arvalid <= 1'b0;
            end

            // R beats: count until rlast
            if (axi_rvalid && axi_rready) begin
                if (axi_rlast || rd_beat_cnt == rd_total_len) begin
                    rd_pending  <= 1'b0;
                    rd_beat_cnt <= 8'h00;
                end else begin
                    rd_beat_cnt <= rd_beat_cnt + 8'h01;
                end
            end
        end
    end

    // Read data routing
    wire rdata_valid = axi_rvalid && axi_rready;

    assign ic_axi_rdata = axi_rdata;
    assign ic_axi_valid = rdata_valid && (rd_src == SRC_ICACHE);

    assign dc_axi_rdata = axi_rdata;
    assign dc_axi_valid = rdata_valid && (rd_src == SRC_DCACHE);

    assign direct_rdata = axi_rdata;
    wire direct_rd_done = rdata_valid && (rd_src == SRC_DIRECT);

    // ================================================================
    // Write path: AW + W (burst-aware) + B
    // ================================================================
    reg [1:0]  wr_src;
    reg        wr_pending;
    reg [7:0]  wr_beat_cnt;
    reg [7:0]  wr_total_len;
    reg        wr_is_sb;
    reg        direct_wr_hold;
    reg [31:0] direct_wr_addr;
    reg        direct_wr_hold_d1;

    // 锁存 direct 单拍写数据；dcache burst 用组合直通
    reg [31:0] direct_wdata_latched;
    reg [3:0]  direct_wstrb_latched;

    // W channel data mux：dcache burst 组合直通（避 1 拍延迟），direct 用锁存值
    wire wr_dcache_active =
        (wr_src == SRC_DCACHE) && wr_pending && !wr_is_sb;
    wire wr_sb_active =
        (wr_src == SRC_DCACHE) && wr_pending && wr_is_sb;
    assign axi_wdata = wr_dcache_active ? dc_axi_wdata :
                       wr_sb_active     ? sb_axi_wdata :
                                          direct_wdata_latched;
    assign axi_wstrb = wr_dcache_active ? dc_axi_wstrb :
                       wr_sb_active     ? sb_axi_wstrb :
                                          direct_wstrb_latched;

    assign axi_bready  = 1'b1;
    assign axi_awprot  = 3'b000;

    wire block_direct_wr = direct_wr_hold && (direct_addr == direct_wr_addr);

    // dc_axi_wnext：每 W beat 握手时发脉冲，通知 dcache 推进 wdata
    assign dc_axi_wnext = wr_dcache_active && axi_wvalid && axi_wready;

    always @(posedge clk) begin
        if (reset) begin
            axi_awvalid  <= 1'b0; axi_awaddr <= 32'h0; axi_awlen <= 8'h00;
            axi_wvalid   <= 1'b0; axi_wlast  <= 1'b0;
            wr_src       <= SRC_NONE;
            wr_pending   <= 1'b0;
            wr_beat_cnt  <= 8'h00;
            wr_total_len <= 8'h00;
            wr_is_sb     <= 1'b0;
            direct_wr_hold <= 1'b0;
            direct_wr_addr <= 32'h0;
            direct_wr_hold_d1 <= 1'b0;
            direct_wdata_latched <= 32'h0;
            direct_wstrb_latched <= 4'h0;
        end else begin
            direct_wr_hold_d1 <= direct_wr_hold;

            if (!(direct_req && direct_we) ||
                (direct_wr_hold_d1 && direct_addr != direct_wr_addr))
                direct_wr_hold <= 1'b0;

            // ---- Start new write burst ----
            if (!wr_pending && write_req && !block_direct_wr) begin
                axi_awvalid <= 1'b1;
                axi_awaddr  <= wr_sel_direct ? direct_addr :
                               wr_sel_dc     ? dc_axi_addr : sb_axi_addr;
                axi_awlen   <= (wr_sel_direct || wr_sel_sb) ?
                               8'h00 : dc_axi_len;
                wr_src      <= wr_sel_direct ? SRC_DIRECT : SRC_DCACHE;
                wr_is_sb    <= wr_sel_sb;
                wr_total_len <= (wr_sel_direct || wr_sel_sb) ?
                                8'h00 : dc_axi_len;
                wr_pending   <= 1'b1;
                wr_beat_cnt  <= 8'h00;
                // 第一拍 W：锁存 direct 数据（dcache 用组合直通）
                if (wr_sel_direct) begin
                    direct_wdata_latched <= direct_wdata;
                    direct_wstrb_latched <= direct_wstrb;
                end
                axi_wvalid <= 1'b1;
                // 不能用 wr_total_len（NBA：同拍赋值，读旧值），直接用源表达式
                axi_wlast  <= wr_sel_direct || wr_sel_sb ||
                              (dc_axi_len == 8'h00);
            end else begin
                // ---- AW handshake ----
                if (axi_awvalid && axi_awready)
                    axi_awvalid <= 1'b0;

                // ---- W channel: beat-by-beat ----
                if (axi_wvalid && axi_wready) begin
                    axi_wvalid <= 1'b0;
                    if (wr_beat_cnt == wr_total_len) begin
                        // 最后一拍完成，等 B 响应
                    end else begin
                        // 推进到下一拍
                        wr_beat_cnt <= wr_beat_cnt + 8'h01;
                        axi_wvalid <= 1'b1;
                        axi_wlast  <= (wr_beat_cnt + 8'h01 == wr_total_len);
                    end
                end
            end

            // ---- B response ----
            if (axi_bvalid && axi_bready) begin
                wr_pending  <= 1'b0;
                wr_beat_cnt <= 8'h00;
                if (wr_src == SRC_DIRECT) begin
                    direct_wr_hold <= 1'b1;
                    direct_wr_addr <= direct_addr;
                end
            end
        end
    end

    wire direct_wr_done = axi_bvalid && axi_bready && (wr_src == SRC_DIRECT);

    // ================================================================
    // Completion signals
    // ================================================================
    assign direct_rd_ready = direct_rd_done;
    assign direct_wr_ready = direct_wr_done;
    assign direct_ready    = direct_rd_done || direct_wr_done;
    assign dc_axi_wr_done  =
        axi_bvalid && axi_bready && (wr_src == SRC_DCACHE) && !wr_is_sb;
    assign sb_axi_wr_done  =
        axi_bvalid && axi_bready && (wr_src == SRC_DCACHE) && wr_is_sb;

endmodule
