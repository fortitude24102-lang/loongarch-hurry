// ============================================================================
// dcache — 4路组相联数据缓存 (256 sets × 4 ways × 8 words = 32KB)
//   写回（write-back），写分配（write-allocate），阻塞式填充
//   伪 LRU 替换（3bit/set），BRAM 数据 + 分布式 RAM tag/valid/dirty/LRU
// ============================================================================
module dcache(
    input  clk,
    input  reset,

    input         req,
    input  [31:0] addr,
    input  [31:0] wdata,
    input         we,
    output [31:0] rdata,
    output        ready,
    input  [3:0]  byte_we,

    output reg         axi_req,
    output reg  [31:0] axi_addr,
    output reg  [31:0] axi_wdata,
    output reg  [3:0]  axi_wstrb,
    output reg         axi_we,
    output             sb_axi_req,
    output [31:0]      sb_axi_addr,
    output [31:0]      sb_axi_wdata,
    output [3:0]       sb_axi_wstrb,
    input              sb_wr_done,
    output reg  [7:0]  axi_len,      // burst 长度：8'h07=cache line, 8'h00=单拍
    input       [31:0] axi_rdata,
    input              axi_valid,
    input              axi_ready,
    input              wr_done,      // B 响应脉冲（写 burst 完成）
    input              axi_wnext,    // 写 burst 下一字请求（每 W beat 一脉冲）

    // ---- cacop 缓存操作接口 ----
    input              cacop_req,
    input  [4:0]       cacop_code,
    input  [31:0]      cacop_addr,
    output reg         cacop_done,

    output      [3:0]  dbg_state,    // 诊断：当前 FSM 状态
    output             store_buffer_full,
    output             store_buffer_enqueue,
    output             store_buffer_forward,
    output      [2:0]  store_buffer_count
);

    parameter NUM_SETS    = 256;
    parameter NUM_WAYS    = 4;
    parameter LINE_SIZE   = 8;
    parameter INDEX_BITS  = 8;
    parameter OFFSET_BITS = 5;
    parameter TAG_BITS    = 19;   // 32 - 8 - 5 = 19
    parameter LRU_BITS    = 3;    // 4-way 伪 LRU
    parameter ADAPTIVE_BYPASS = 1;

    // ---- BRAM 数据阵列：XPM SDPRAM (8192×32 = 32KB) ----
    reg [12:0] bram_rd_addr;   // {way(2), idx(8), word(3)} = 13 bits for 8192 entries
    wire [12:0] bram_rd_port_addr;
    reg [12:0] bram_wr_addr;
    reg [31:0]              bram_wr_data;
    reg                     bram_we;
    wire [31:0]             bram_rd_data;  // XPM doutb 驱动，必须 wire
    reg [31:0]              wb_data_latched; // WB 预取数据寄存器

    // WB line buffer：写回前预读全部 8 字到寄存器，避免 XPM 实时竞态
    reg [31:0] wb_buf [0:7];
    reg [2:0]  wb_buf_cnt;    // 已填充字数 (0..7)，=8 时表示满
    reg        wb_fill_cap;   // 0=等 XPM, 1=本拍捕获 bram_rd_data

    // ---- cacop 操作状态 ----
    reg [1:0]  cacop_way;     // 当前操作的 way (0..3)
    reg [INDEX_BITS-1:0] cacop_idx;   // 操作的 set index
    reg [4:0]  cacop_code_r;  // 捕获的 cacop_code（防流水线推进覆盖）

    reg [TAG_BITS-1:0] cacop_tag;
    reg        cacop_wb_active;

    xpm_memory_sdpram #(
        .ADDR_WIDTH_A       (13),
        .ADDR_WIDTH_B       (13),
        .BYTE_WRITE_WIDTH_A (8),
        .CLOCKING_MODE      ("common_clock"),
        .MEMORY_PRIMITIVE   ("block"),
        .MEMORY_SIZE        (262144),        // 8192 × 32 = 32KB
        .READ_DATA_WIDTH_B  (32),
        .READ_LATENCY_B     (1),
        .WRITE_DATA_WIDTH_A (32),
        .WRITE_MODE_B       ("READ_FIRST")
    ) u_bram (
        .clka   (clk),
        .ena    (1'b1),
        .wea    ({4{bram_we}}),
        .addra  (bram_wr_addr),
        .dina   (bram_wr_data),
        .sleep  (1'b0),
        .clkb   (clk),
        .enb    (1'b1),
        .regceb (1'b1),
        .addrb  (bram_rd_port_addr),
        .doutb  (bram_rd_data)
    );

    // ---- 分布式 RAM: tag / valid / dirty ----
    // Vivado 2019.2 cannot infer the former [way][set] 3-D shape and expands
    // it into registers plus large mux trees.  Flatten {way,set} into one RAM
    // address so each metadata array has a supported single-memory shape.
    localparam META_ENTRIES = NUM_WAYS * NUM_SETS;
    (* ram_style = "distributed" *) reg [TAG_BITS-1:0] cache_tag [0:META_ENTRIES-1];
    // Keep the four status bits of a set together.  A flat 1024x1 array makes
    // every dynamic update decode {way,set} across all flops; the packed form
    // only decodes the set and selects one of four nearby bits.
    reg [NUM_WAYS-1:0] cache_valid [0:NUM_SETS-1];
    reg [NUM_WAYS-1:0] cache_dirty [0:NUM_SETS-1];

    function [INDEX_BITS+1:0] meta_addr;
        input [1:0] way;
        input [INDEX_BITS-1:0] set_idx;
        begin
            meta_addr = {way, set_idx};
        end
    endfunction

    // ---- 伪 LRU（3-bit 树，每 set 独立）----
    // lru[0]: 0=左子树最近(Way0/Way1), 1=右子树最近(Way2/Way3)
    // lru[1]: 0=Way0最近, 1=Way1最近
    // lru[2]: 0=Way2最近, 1=Way3最近
    (* ram_style = "distributed" *) reg [LRU_BITS-1:0] plru [0:NUM_SETS-1];
    // Small global miss-pressure predictor.  It only selects the miss path;
    // every tag hit still uses the cache.
    reg [5:0] global_miss_pressure;
`ifdef VERILATOR
    reg cpu5_disable_adaptive_bypass;
    reg cpu5_adaptive_eager;
    initial cpu5_disable_adaptive_bypass =
        $test$plusargs("cpu5_dcache_no_adaptive");
    initial cpu5_adaptive_eager =
        $test$plusargs("cpu5_dcache_adaptive_eager");
    wire adaptive_bypass_enabled =
        ADAPTIVE_BYPASS && !cpu5_disable_adaptive_bypass;
`else
    wire cpu5_adaptive_eager = 1'b0;
    wire adaptive_bypass_enabled = ADAPTIVE_BYPASS;
`endif

    // ---- 地址锁存 ----
    reg [INDEX_BITS-1:0] idx_r;
    reg [TAG_BITS-1:0]   tag_r;
    reg [2:0]            word_r;
    reg                  we_r;
    reg [31:0]           wdata_r;
    reg [3:0]            be_r;

    // ---- 替换信息（组合逻辑）：优先 invalid way，其次用 plru ---
    wire [1:0] victim_way;
    wire       victim_dirty;
    // 找到第一个 invalid way（0-3），全 valid 才用 plru
    wire       any_invalid = !cache_valid[idx_r][2'd0] ||
                             !cache_valid[idx_r][2'd1] ||
                             !cache_valid[idx_r][2'd2] ||
                             !cache_valid[idx_r][2'd3];
    wire [1:0] first_invalid = !cache_valid[idx_r][2'd0] ? 2'd0 :
                               !cache_valid[idx_r][2'd1] ? 2'd1 :
                               !cache_valid[idx_r][2'd2] ? 2'd2 : 2'd3;
    assign victim_way   = any_invalid ? first_invalid : plru_get_victim(plru[idx_r]);
    assign victim_dirty =
        any_invalid ? 1'b0 : cache_dirty[idx_r][victim_way];

    // ---- 字节合并 ----
    function [31:0] merge_wdata;
        input [31:0] old_data;
        input [31:0] new_data;
        input [3:0]  byte_en;
        begin
            merge_wdata[31:24] = byte_en[3] ? new_data[31:24] : old_data[31:24];
            merge_wdata[23:16] = byte_en[2] ? new_data[23:16] : old_data[23:16];
            merge_wdata[15:8]  = byte_en[1] ? new_data[15:8]  : old_data[15:8];
            merge_wdata[7:0]   = byte_en[0] ? new_data[7:0]   : old_data[7:0];
        end
    endfunction

    // ---- FSM (4-bit: 8状态不够，XPM BRAM 需额外 WB_START) ----
    localparam IDLE     = 4'b0000,
               SRD      = 4'b0001,
               CACHE    = 4'b0010,
               HIT_DATA = 4'b0111,
               WB_WAIT  = 4'b1001,  // XPM 读延迟等待（bram_rd_addr 稳定后空转一拍）
               WB_START = 4'b0100,  // XPM BRAM 预取：bram_rd_data→wb_data_latched
               WB       = 4'b0011,
               WB_TURN  = 4'b0110,  // 未使用
               REFILL   = 4'b0101,
               DONE     = 4'b1000,
               CACOP    = 4'b1010,
               BYPASS_RD = 4'b1011,
               BYPASS_WR = 4'b1100,
               SB_FULL   = 4'b1101,
               MISS_WAIT = 4'b1110,
               BYPASS_RD_WAIT = 4'b1111;

    reg [3:0] state;
    wire [INDEX_BITS-1:0] req_idx =
        addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    wire [TAG_BITS-1:0] req_tag =
        addr[31:OFFSET_BITS+INDEX_BITS];
    wire [3:0] req_hit_vec;
    assign req_hit_vec[0] = cache_valid[req_idx][2'd0] &&
        cache_tag[meta_addr(2'd0, req_idx)] == req_tag;
    assign req_hit_vec[1] = cache_valid[req_idx][2'd1] &&
        cache_tag[meta_addr(2'd1, req_idx)] == req_tag;
    assign req_hit_vec[2] = cache_valid[req_idx][2'd2] &&
        cache_tag[meta_addr(2'd2, req_idx)] == req_tag;
    assign req_hit_vec[3] = cache_valid[req_idx][2'd3] &&
        cache_tag[meta_addr(2'd3, req_idx)] == req_tag;
    wire req_hit = |req_hit_vec;
    wire [1:0] req_hit_way = req_hit_vec[0] ? 2'd0 :
                             req_hit_vec[1] ? 2'd1 :
                             req_hit_vec[2] ? 2'd2 : 2'd3;
    reg       srd_hit;
    reg [1:0] srd_hit_way;
    // MEM1 holds an IDLE request stable for the entire cycle.  Select the
    // matching way directly into the synchronous BRAM address before the
    // capture edge, so its data is available throughout SRD.
    assign bram_rd_port_addr =
        (state == IDLE && req) ?
            {req_hit_way, req_idx, addr[OFFSET_BITS-1:2]} :
            bram_rd_addr;
    reg [31:0] dcache_addr_dbg;
    integer dc_dbg_cycle;
    reg [3:0] fill_cnt;
    reg [3:0] wb_cnt;
    reg [12:0] wb_base;   // {victim_way(2), idx(8), word(3)} = 13 bits

    // ---- 全 cache flush 状态 ----
    reg        flush_active;
    reg [7:0]  flush_set;   // 当前扫描的 set (0..NUM_SETS-1)
    reg [1:0]  flush_way;   // 当前扫描的 way (0..NUM_WAYS-1)
    reg [15:0] idle_cnt;    // 空闲计数器：超时自动 flush
    reg        flush_done;   // flush 完成标志（探针用）
    reg [31:0]             wb_rdata;
    reg [1:0]              victim_way_r; // 锁存 victim_way，防 REFILL 期间 plru 更新导致变化

    // ---- 4-entry store buffer for adaptive no-allocate stores ----
    // Entries remain valid through the AXI B response, so a following load
    // can overlay pending bytes even while the oldest write is draining.
    localparam SB_ENTRIES = 4;
    // Sequential store streams should not fetch an entire destination line.
    // Random stores remain write-allocate/write-back.
    reg [31:0] last_store_addr;
    reg [15:0] store_seq_count;
    reg [4:0]  recent_load_age;
    wire sequential_store_mode =
        store_seq_count >= 16'h8000 && recent_load_age < 5'd16;
    reg                  sb_valid [0:SB_ENTRIES-1];
    reg [31:0]           sb_addr  [0:SB_ENTRIES-1];
    reg [31:0]           sb_data  [0:SB_ENTRIES-1];
    reg [3:0]            sb_strb  [0:SB_ENTRIES-1];
    reg [1:0]            sb_head;
    reg [1:0]            sb_tail;
    reg [2:0]            sb_count;
    reg                  sb_drain_active;

    // ---- Write Buffer：4 项 store→load 前递 ----
    localparam WB_ENTRIES = 4;
    reg                  wb_valid  [0:WB_ENTRIES-1];
    reg [TAG_BITS-1:0]   wb_tag    [0:WB_ENTRIES-1];
    reg [INDEX_BITS-1:0] wb_idx    [0:WB_ENTRIES-1];
    reg [2:0]            wb_word   [0:WB_ENTRIES-1];
    reg [31:0]           wb_data   [0:WB_ENTRIES-1];
    integer wb_i, wb_j;

    // 组合并行查找：Load 地址命中 Write Buffer？
    wire [WB_ENTRIES-1:0] wb_hit_vec;
    wire                  wb_rd_hit;
    wire [31:0]           wb_rd_data;
    generate
        genvar gi;
        for (gi = 0; gi < WB_ENTRIES; gi = gi + 1) begin : wb_gen
            assign wb_hit_vec[gi] = wb_valid[gi] &&
                (wb_tag[gi] == tag_r) && (wb_idx[gi] == idx_r) && (wb_word[gi] == word_r);
        end
    endgenerate
    assign wb_rd_hit  = |wb_hit_vec;
    assign wb_rd_data = wb_hit_vec[0] ? wb_data[0] :
                        wb_hit_vec[1] ? wb_data[1] :
                        wb_hit_vec[2] ? wb_data[2] : wb_data[3];

    wire [31:0] sb_req_addr = {tag_r, idx_r, word_r, 2'b00};
    wire        sb_pop = sb_drain_active && sb_wr_done;
    assign sb_axi_req   = sb_drain_active;
    assign sb_axi_addr  = sb_addr[sb_head];
    assign sb_axi_wdata = sb_data[sb_head];
    assign sb_axi_wstrb = sb_strb[sb_head];
    wire [SB_ENTRIES-1:0] sb_match_vec;
    generate
        genvar sbi;
        for (sbi = 0; sbi < SB_ENTRIES; sbi = sbi + 1) begin : sb_gen
            assign sb_match_vec[sbi] =
                sb_valid[sbi] &&
                !(sb_pop && sb_head == sbi[1:0]) &&
                sb_addr[sbi] == sb_req_addr;
        end
    endgenerate
    wire sb_match = |sb_match_vec;
    wire [31:0] sb_forward_data =
        sb_match_vec[0] ? sb_data[0] :
        sb_match_vec[1] ? sb_data[1] :
        sb_match_vec[2] ? sb_data[2] : sb_data[3];
    wire [3:0] sb_forward_strb =
        sb_match_vec[0] ? sb_strb[0] :
        sb_match_vec[1] ? sb_strb[1] :
        sb_match_vec[2] ? sb_strb[2] : sb_strb[3];
    wire sb_can_accept = sb_match || sb_count < SB_ENTRIES || sb_pop;
    wire adaptive_miss_selected =
        adaptive_bypass_enabled &&
        global_miss_pressure >= 6'd48;
    wire sb_enqueue =
        ((sequential_store_mode && state == SRD && !srd_hit && we_r) ||
         (((state == SRD) || (state == CACHE)) && !srd_hit && we_r &&
          adaptive_miss_selected) ||
         state == SB_FULL) &&
        sb_can_accept;
    wire sb_enqueue_new = sb_enqueue && !sb_match;
    assign store_buffer_full = (state == SB_FULL);
    assign store_buffer_enqueue = sb_enqueue;
    assign store_buffer_forward =
        (state == BYPASS_RD) && axi_valid && axi_ready && sb_match;
    assign store_buffer_count = sb_count;

    integer sb_rst_i;
    always @(posedge clk) begin
        if (reset) begin
            sb_head <= 2'd0;
            sb_tail <= 2'd0;
            sb_count <= 3'd0;
            for (sb_rst_i = 0; sb_rst_i < SB_ENTRIES;
                 sb_rst_i = sb_rst_i + 1)
                sb_valid[sb_rst_i] <= 1'b0;
        end else begin
            if (sb_pop) begin
                sb_valid[sb_head] <= 1'b0;
                sb_head <= sb_head + 2'd1;
            end
            if (sb_enqueue) begin
                if (sb_match) begin
                    if (sb_match_vec[0]) begin
                        sb_data[0] <= merge_wdata(
                            sb_data[0], wdata_r, be_r);
                        sb_strb[0] <= sb_strb[0] | be_r;
                    end else if (sb_match_vec[1]) begin
                        sb_data[1] <= merge_wdata(
                            sb_data[1], wdata_r, be_r);
                        sb_strb[1] <= sb_strb[1] | be_r;
                    end else if (sb_match_vec[2]) begin
                        sb_data[2] <= merge_wdata(
                            sb_data[2], wdata_r, be_r);
                        sb_strb[2] <= sb_strb[2] | be_r;
                    end else begin
                        sb_data[3] <= merge_wdata(
                            sb_data[3], wdata_r, be_r);
                        sb_strb[3] <= sb_strb[3] | be_r;
                    end
                end else begin
                    sb_valid[sb_tail] <= 1'b1;
                    sb_addr[sb_tail] <= sb_req_addr;
                    sb_data[sb_tail] <= merge_wdata(
                        32'h0, wdata_r, be_r);
                    sb_strb[sb_tail] <= be_r;
                    sb_tail <= sb_tail + 2'd1;
                end
            end
            case ({sb_enqueue_new, sb_pop})
                2'b10: sb_count <= sb_count + 3'd1;
                2'b01: sb_count <= sb_count - 3'd1;
                default: sb_count <= sb_count;
            endcase
        end
    end

    // ---- 伪 LRU 更新函数（内联逻辑）----
    // 访问 way 时更新 plru[index]：将被访问路标记为最近使用
    function [LRU_BITS-1:0] plru_update;
        input [LRU_BITS-1:0] old_lru;
        input [1:0] way;
        begin
            plru_update = old_lru;
            case (way)
                2'd0: begin plru_update[0] = 1'b0; plru_update[1] = 1'b0; end
                2'd1: begin plru_update[0] = 1'b0; plru_update[1] = 1'b1; end
                2'd2: begin plru_update[0] = 1'b1; plru_update[2] = 1'b0; end
                2'd3: begin plru_update[0] = 1'b1; plru_update[2] = 1'b1; end
            endcase
        end
    endfunction

    // ---- 伪 LRU 获取 victim way ----
    function [1:0] plru_get_victim;
        input [LRU_BITS-1:0] lru_val;
        begin
            // plru_update records the most recently used subtree/leaf.
            // Replacement must therefore walk the opposite subtree and leaf.
            if (lru_val[0] == 1'b0)          // 左子树最近，替换右子树
                plru_get_victim = lru_val[2] ? 2'd2 : 2'd3;
            else                              // 右子树最近，替换左子树
                plru_get_victim = lru_val[1] ? 2'd0 : 2'd1;
        end
    endfunction

    // ---- Write Buffer 管理（独立 always）----
    reg        wb_do_write;     // 本拍有 store hit，写入 WB
    reg        wb_do_inval;     // 本拍有 evict/fill，清除 WB 对应行
    reg [TAG_BITS-1:0]   wb_inval_tag;
    reg [INDEX_BITS-1:0] wb_inval_idx;

    integer wb_wi, wb_wj;
    reg      wb_matched, wb_allocated;
    always @(posedge clk) begin
        if (reset) begin
            for (wb_wi = 0; wb_wi < WB_ENTRIES; wb_wi = wb_wi + 1)
                wb_valid[wb_wi] <= 1'b0;
        end else begin
            // 清除：evict 或 fill 完成后，该 {tag, index} 的全部 WB 条目失效
            if (wb_do_inval) begin
                for (wb_wi = 0; wb_wi < WB_ENTRIES; wb_wi = wb_wi + 1)
                    if (wb_valid[wb_wi] && wb_tag[wb_wi] == wb_inval_tag && wb_idx[wb_wi] == wb_inval_idx)
                        wb_valid[wb_wi] <= 1'b0;
            end
            // 写入：store hit → 合并或新增
            if (wb_do_write) begin
                wb_matched = 1'b0;
                // 查找是否已有同 {tag, idx, word} 的条目 → 合并
                for (wb_wi = 0; wb_wi < WB_ENTRIES; wb_wi = wb_wi + 1) begin
                    if (!wb_matched && wb_valid[wb_wi] && wb_tag[wb_wi] == tag_r && wb_idx[wb_wi] == idx_r && wb_word[wb_wi] == word_r) begin
                        wb_data[wb_wi] <= merge_wdata(wb_data[wb_wi], wdata_r, be_r);
                        wb_matched = 1'b1;
                    end
                end
                // 未命中 → 找空位新增
                if (!wb_matched) begin
                    wb_allocated = 1'b0;
                    for (wb_wj = 0; wb_wj < WB_ENTRIES; wb_wj = wb_wj + 1) begin
                        if (!wb_allocated && !wb_valid[wb_wj]) begin
                            wb_valid[wb_wj] <= 1'b1;
                            wb_tag[wb_wj]   <= tag_r;
                            wb_idx[wb_wj]   <= idx_r;
                            wb_word[wb_wj]  <= word_r;
                            wb_data[wb_wj]  <= wdata_r;
                            wb_allocated = 1'b1;
                        end
                    end
                end
            end
        end
    end

    // Single write port for tag RAM.  Keeping tag updates in the main FSM
    // makes Vivado see several possible write ports and dissolve the 19-Kbit
    // array into registers.
    wire tag_refill_write =
        (state == REFILL) && !cacop_req && axi_valid && axi_ready &&
        (fill_cnt == LINE_SIZE - 1);
    wire tag_cacop_clear =
        (state == CACOP) && (cacop_code_r[4:3] == 2'b00);
    wire tag_write = tag_refill_write || tag_cacop_clear;
    wire [INDEX_BITS+1:0] tag_write_addr =
        tag_refill_write ? meta_addr(victim_way_r, idx_r) :
                           meta_addr(cacop_way, cacop_idx);
    wire [TAG_BITS-1:0] tag_write_data =
        tag_refill_write ? tag_r : {TAG_BITS{1'b0}};
    always @(posedge clk) begin
        if (tag_write)
            cache_tag[tag_write_addr] <= tag_write_data;
    end

    integer rst_i, rst_j;
    always @(posedge clk) begin
        if (reset) begin
            state   <= IDLE;
            srd_hit <= 1'b0;
            srd_hit_way <= 2'd0;
            axi_req <= 1'b0;
            axi_addr <= 32'h0;
            axi_wstrb <= 4'h0;
            axi_len  <= 8'h00;
            bram_we <= 1'b0;
            victim_way_r <= 2'd0;
            wb_buf_cnt <= 0;
            wb_fill_cap <= 1'b0;
            global_miss_pressure <=
                cpu5_adaptive_eager ? 6'd63 : 6'd0;
            cacop_done <= 1'b0;
            cacop_wb_active <= 1'b0;
            sb_drain_active <= 1'b0;
            last_store_addr <= 32'h0;
            store_seq_count <= 16'd0;
            recent_load_age <= 5'h1f;
            for (wb_i = 0; wb_i < WB_ENTRIES; wb_i = wb_i + 1) wb_valid[wb_i] = 1'b0;
            for (rst_j = 0; rst_j < NUM_SETS; rst_j = rst_j + 1) begin
                cache_valid[rst_j] = {NUM_WAYS{1'b0}};
                cache_dirty[rst_j] = {NUM_WAYS{1'b0}};
            end
            // plru 必须复位为 0，否则 victim_way=X → FILL 写到 X way
            flush_done <= 1'b0;
            idle_cnt <= 16'd0;
            for (rst_i = 0; rst_i < NUM_SETS; rst_i = rst_i + 1)
                plru[rst_i] = 3'b0;
            flush_active <= 1'b0;
        end else begin
            bram_we <= 1'b0;
            wb_do_write <= 1'b0;
            wb_do_inval <= 1'b0;
            cacop_done <= 1'b0;

            if (state == SRD && we_r) begin
                if (dcache_addr_dbg == last_store_addr + 32'd4) begin
                    if (store_seq_count != 16'hffff)
                        store_seq_count <= store_seq_count + 16'd1;
                end else begin
                    store_seq_count <= 16'd0;
                end
                last_store_addr <= dcache_addr_dbg;
            end
            if (state == SRD) begin
                if (!we_r)
                    recent_load_age <= 5'd0;
                else if (recent_load_age != 5'h1f)
                    recent_load_age <= recent_load_age + 5'd1;
            end

            // The store buffer has an independent AXI write source, leaving
            // the foreground D-cache port free to launch a random read.
            if (!sb_drain_active && sb_count != 0) begin
                sb_drain_active <= 1'b1;
            end
            if (sb_pop) begin
                sb_drain_active <= 1'b0;
            end

            // ---- 空闲计数器：req/cacop_req=0 时递增 ----
            if (state == IDLE && !req && !cacop_req && !flush_active)
                idle_cnt <= idle_cnt + 16'd1;
            else if (req || cacop_req)
                idle_cnt <= 16'd0;

            // ---- 全 cache flush：空闲超时或显式触发，逐 set/way 扫描 dirty line 并写回 ----
            // Whole-cache writeback is requested explicitly with CACOP 0x09.
            // UART polling must not trigger a scan merely because D-cache is idle.
            if (flush_active && state == IDLE && !req) begin
                // 跳过非 dirty 的 {way, set}
                if (!cache_valid[flush_set][flush_way] ||
                    !cache_dirty[flush_set][flush_way]) begin
                    if (flush_way == NUM_WAYS-1) begin
                        flush_way <= 2'd0;
                        if (flush_set == NUM_SETS-1) begin
                            // 全部扫描完成
                            flush_active <= 1'b0;
                            flush_done   <= 1'b1;
                        end else begin
                            flush_set <= flush_set + 8'd1;
                        end
                    end else begin
                        flush_way <= flush_way + 2'd1;
                    end
                end else begin
                    // 此 {way, set} dirty：发起写回
                    // synthesis translate_off
                    $display("[DC-FP c=%0d] flush w=%0d s=%0d dirty→WB_WAIT", dc_dbg_cycle, flush_way, flush_set);
                    // synthesis translate_on
                    idx_r   <= flush_set;
                    tag_r   <= cache_tag[meta_addr(flush_way, flush_set)];
                    wb_base <= {flush_way, flush_set, 3'b000};
                    wb_cnt  <= 0;
                    wb_buf_cnt <= 0;
                    wb_fill_cap <= 1'b0;
                    axi_len <= 8'h07;
                    axi_addr <= {cache_tag[meta_addr(flush_way, flush_set)],
                                 flush_set, {OFFSET_BITS{1'b0}}};
                    victim_way_r <= flush_way;
                    bram_rd_addr <= {flush_way, flush_set, 3'b000};
                    state <= WB_WAIT;
                    // synthesis translate_off
                    $display("[DC-WB0 c=%0d] flush w=%0d s=%0d → WB_WAIT",
                             dc_dbg_cycle, flush_way, flush_set);
                    // synthesis translate_on
                    // 写回完成后清除 dirty
                    cache_dirty[flush_set][flush_way] <= 1'b0;
                end
            end

            case (state)
                IDLE: begin
                    if (cacop_req) begin
                        // Buffered normal-memory stores must reach SRAM before
                        // a cache maintenance operation can complete.
                        if (sb_count == 0 && !sb_drain_active) begin
                            cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                            cacop_way <= cacop_addr[1:0];
                            cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                            cacop_code_r <= cacop_code;
                            state <= CACOP;
                        end
                    end else if (req && !flush_active) begin
                        dcache_addr_dbg <= addr;
                        idx_r   <= addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        tag_r   <= addr[31:OFFSET_BITS+INDEX_BITS];
                        word_r  <= addr[OFFSET_BITS-1:2];
                        srd_hit <= req_hit;
                        srd_hit_way <= req_hit_way;
                        we_r    <= we;
                        wdata_r <= wdata;
                        be_r    <= byte_we;
                        bram_rd_addr <= {2'b00, addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS], addr[OFFSET_BITS-1:2]};
                        state   <= SRD;
                    end
                end

                SRD: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else if (srd_hit) begin
                        // IDLE already selected the hit way on the BRAM port,
                        // so every way can complete in SRD.
                        bram_rd_addr <= {srd_hit_way, idx_r, word_r};
                        if (global_miss_pressure != 6'd0)
                            global_miss_pressure <=
                                global_miss_pressure - 6'd1;
                        plru[idx_r] <= plru_update(plru[idx_r], srd_hit_way);
                        if (we_r) begin
                            bram_we      <= 1'b1;
                            bram_wr_addr <= {srd_hit_way, idx_r, word_r};
                            bram_wr_data <=
                                merge_wdata(bram_rd_data, wdata_r, be_r);
                            cache_dirty[idx_r][srd_hit_way] <= 1'b1;
                            wb_do_write <= 1'b1;
                        end
                        state <= IDLE;
                    end else begin
                        // Start miss service immediately instead of spending
                        // an empty lookup cycle in CACHE.
                        // A no-write-allocate store miss is intentional and
                        // must not train the random-load bypass detector.
                        if (sequential_store_mode && we_r &&
                            global_miss_pressure != 6'd0)
                            global_miss_pressure <=
                                global_miss_pressure - 6'd1;
                        else if (sequential_store_mode && we_r)
                            global_miss_pressure <= 6'd0;
                        else if (global_miss_pressure >= 6'd62)
                            global_miss_pressure <= 6'd63;
                        else
                            global_miss_pressure <=
                                global_miss_pressure + 6'd2;
                        victim_way_r <= victim_way;
                        if (sequential_store_mode && we_r) begin
                            state <= sb_can_accept ? DONE : SB_FULL;
                        end else if (adaptive_miss_selected) begin
                            if (we_r) begin
                                state <= sb_can_accept ? DONE : SB_FULL;
                            end else begin
                                axi_req   <= 1'b1;
                                axi_addr  <= sb_req_addr;
                                axi_we    <= 1'b0;
                                axi_len   <= 8'h00;
                                state     <= BYPASS_RD;
                            end
                        end else if (sb_count != 0 || sb_drain_active) begin
                            state <= MISS_WAIT;
                        end else if (victim_dirty) begin
                            wb_base <= {victim_way, idx_r, 3'b000};
                            wb_cnt  <= 0;
                            wb_buf_cnt <= 0;
                            wb_fill_cap <= 1'b0;
                            axi_addr <= {
                                cache_tag[meta_addr(victim_way, idx_r)],
                                idx_r, {OFFSET_BITS{1'b0}}};
                            axi_len <= 8'h07;
                            bram_rd_addr <= {
                                victim_way, idx_r, 3'b000};
                            state <= WB_WAIT;
                        end else begin
                            axi_req  <= 1'b1;
                            axi_addr <= {tag_r, idx_r,
                                         {OFFSET_BITS{1'b0}}};
                            axi_we   <= 1'b0;
                            axi_len  <= 8'h07;
                            fill_cnt <= 0;
                            state    <= REFILL;
                        end
                    end
                end

                CACHE: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else if (srd_hit) begin
                        // Non-zero way was selected in SRD; BRAM data is now
                        // valid, so complete without the old HIT_DATA hop.
                        if (we_r) begin
                            bram_we      <= 1'b1;
                            bram_wr_addr <= {srd_hit_way, idx_r, word_r};
                            bram_wr_data <=
                                merge_wdata(bram_rd_data, wdata_r, be_r);
                            cache_dirty[idx_r][srd_hit_way] <= 1'b1;
                            wb_do_write <= 1'b1;
                        end
                        state <= IDLE;
                    end else begin
                        // Metadata is stable for this blocking request.  If a
                        // maintenance request changed it unexpectedly, retry.
                        state <= SRD;
                    end
                end

                HIT_DATA: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else if (we_r) begin
                        bram_we     <= 1'b1;
                        bram_wr_addr <= {srd_hit_way, idx_r, word_r};
                        bram_wr_data <= merge_wdata(bram_rd_data, wdata_r, be_r);
                        cache_dirty[idx_r][srd_hit_way] <= 1'b1;
                        wb_do_write <= 1'b1;
                    end else begin
                        wb_rdata <= bram_rd_data;
                    end
                    plru[idx_r] <= plru_update(plru[idx_r], srd_hit_way);
                    state <= IDLE;
                    // fast-path disabled
                end

                WB_WAIT: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else begin
                    // Pipeline the synchronous BRAM read: after one startup
                    // cycle, capture one word and present the following address
                    // every cycle.  The previous implementation inserted an
                    // unnecessary wait cycle between all eight captures.
                    if (wb_fill_cap) begin
                        wb_buf[wb_buf_cnt] <= bram_rd_data;
                        if (wb_buf_cnt == 3'd7) begin
                            state <= WB_START;
                        end else begin
                            bram_rd_addr <= wb_base + wb_buf_cnt + 3'd2;
                            wb_buf_cnt <= wb_buf_cnt + 3'd1;
                        end
                    end else begin
                        // word 0 is sampled by the XPM at this edge; prepare
                        // word 1 while it appears at bram_rd_data.
                        bram_rd_addr <= wb_base + 3'd1;
                        wb_fill_cap <= 1'b1;
                    end
                    end  // else begin (cacop bypass)
                end

                WB_START: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else begin
                    // line buffer 已就绪，直接发 AXI 写 burst
                    axi_req <= 1'b1;
                    axi_we  <= 1'b1;
                    axi_wstrb <= 4'b1111;
                    axi_wdata <= wb_buf[0];       // word 0
                    wb_data_latched <= wb_buf[0];  // 同步（供 WB 第一拍用）
                    wb_cnt <= 0;                   // beat 计数复位
                    state <= WB;
                    end  // else begin
                end

                WB: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else begin
                    // 从 wb_buf 流式输出
                    if (axi_wnext) begin
                        wb_cnt <= wb_cnt + 4'd1;
                        if (wb_cnt < 4'd7)
                            axi_wdata <= wb_buf[wb_cnt + 4'd1];  // 下一字
                    end else if (wb_cnt == 0) begin
                        axi_wdata <= wb_buf[0];  // 首拍保持
                    end
                    if (wr_done) begin
                        wb_do_inval   <= 1'b1;
                        wb_inval_tag  <= cache_tag[meta_addr(victim_way_r, idx_r)];
                        wb_inval_idx  <= idx_r;
                        if (cacop_wb_active) begin
                            axi_req <= 1'b0;
                            axi_we  <= 1'b0;
                            cache_valid[idx_r][victim_way_r] <= 1'b0;
                            cache_dirty[idx_r][victim_way_r] <= 1'b0;
                            cacop_wb_active <= 1'b0;
                            cacop_done <= 1'b1;
                            state <= IDLE;
                        end else if (flush_active) begin
                            axi_req <= 1'b0;
                            axi_we  <= 1'b0;
                            if (flush_way == NUM_WAYS-1) begin
                                flush_way <= 2'd0;
                                if (flush_set == NUM_SETS-1) begin
                                    flush_active <= 1'b0;
                                    flush_done   <= 1'b1;
                                end else begin
                                    flush_set <= flush_set + 8'd1;
                                end
                            end else begin
                                flush_way <= flush_way + 2'd1;
                            end
                            state <= IDLE;
                        end else begin
                            axi_req  <= 1'b1;
                            axi_addr <= {tag_r, idx_r, {OFFSET_BITS{1'b0}}};
                            axi_we   <= 1'b0;
                            axi_len  <= 8'h07;
                            fill_cnt <= 0;
                            state    <= REFILL;
                        end
                    end
                    end  // else begin
                end

                REFILL: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else if (fill_cnt == LINE_SIZE) begin
                        bram_we     <= 1'b1;
                        bram_wr_addr <= {victim_way_r, idx_r, word_r};
                        bram_wr_data <= merge_wdata(bram_rd_data, wdata_r, be_r);
                        state <= DONE;
                    end else if (axi_valid && axi_ready) begin
                        bram_we     <= 1'b1;
                        bram_wr_addr <= {victim_way_r, idx_r, fill_cnt[2:0]};
                        bram_wr_data <= axi_rdata;

                        if (!we_r && fill_cnt[2:0] == word_r)
                            wb_rdata <= axi_rdata;

                        fill_cnt <= fill_cnt + 1;
                        // burst 模式无需递增地址——arbiter 内部处理

                        if (fill_cnt == LINE_SIZE - 1) begin
                            cache_valid[idx_r][victim_way_r] <= 1'b1;
                            cache_dirty[idx_r][victim_way_r] <= we_r;
                            plru[idx_r] <= plru_update(plru[idx_r], victim_way_r);
                            axi_req <= 1'b0;

                            if (we_r) begin
                                if (fill_cnt[2:0] == word_r) begin
                                    bram_we     <= 1'b1;
                                    bram_wr_addr <= {victim_way_r, idx_r, word_r};
                                    bram_wr_data <= merge_wdata(axi_rdata, wdata_r, be_r);
                                    state <= DONE;
                                end else begin
                                    bram_rd_addr <= {victim_way_r, idx_r, word_r};
                                end
                            end else begin
                                state <= DONE;
                            end
                        end
                    end
                end

                BYPASS_RD: begin
                    if (axi_valid && axi_ready) begin
                        wb_rdata <= sb_match ?
                            merge_wdata(axi_rdata, sb_forward_data,
                                        sb_forward_strb) :
                            axi_rdata;
                        axi_req <= 1'b0;
                        state <= IDLE;
                    end
                end

                BYPASS_WR: begin
                    if (wr_done) begin
                        axi_req <= 1'b0;
                        axi_we <= 1'b0;
                        state <= DONE;
                    end
                end

                SB_FULL: begin
                    if (sb_can_accept)
                        state <= DONE;
                end

                BYPASS_RD_WAIT: begin
                    if (!sb_drain_active) begin
                        axi_req <= 1'b1;
                        axi_addr <= sb_req_addr;
                        axi_we <= 1'b0;
                        axi_len <= 8'h00;
                        state <= BYPASS_RD;
                    end
                end

                MISS_WAIT: begin
                    if (sb_count == 0 && !sb_drain_active) begin
                        victim_way_r <= victim_way;
                        if (victim_dirty) begin
                            wb_base <= {victim_way, idx_r, 3'b000};
                            wb_cnt <= 0;
                            wb_buf_cnt <= 0;
                            wb_fill_cap <= 1'b0;
                            axi_addr <= {
                                cache_tag[meta_addr(victim_way, idx_r)], idx_r,
                                {OFFSET_BITS{1'b0}}};
                            axi_len <= 8'h07;
                            bram_rd_addr <= {
                                victim_way, idx_r, 3'b000};
                            state <= WB_WAIT;
                        end else begin
                            axi_req <= 1'b1;
                            axi_addr <= {
                                tag_r, idx_r,
                                {OFFSET_BITS{1'b0}}};
                            axi_we <= 1'b0;
                            axi_len <= 8'h07;
                            fill_cnt <= 0;
                            state <= REFILL;
                        end
                    end
                end

                DONE: begin
                    if (cacop_req) begin
                        cacop_idx <= cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
                        cacop_way <= cacop_addr[1:0]; cacop_tag <= cacop_addr[31:OFFSET_BITS+INDEX_BITS];
                        cacop_code_r <= cacop_code; axi_req <= 1'b0; state <= CACOP;
                    end else begin
                    // 新行填入完成 → WB 中同一 {tag, idx} 的旧条目失效
                    wb_do_inval  <= 1'b1;
                    wb_inval_tag <= tag_r;
                    wb_inval_idx <= idx_r;
                    state <= IDLE;
                    end  // else begin
                end

                CACOP: begin
                    // code[4:3]=00: index init/invalidate selected way
                    // code[4:3]=01: index writeback-invalidate selected way
                    // code[4:3]=10: hit invalidate across all ways
                    if (cacop_code_r[4:3] == 2'b10) begin
                        for (rst_i = 0; rst_i < NUM_WAYS; rst_i = rst_i + 1)
                            if (cache_valid[cacop_idx][rst_i] &&
                                cache_tag[meta_addr(rst_i, cacop_idx)] == cacop_tag) begin
                                cache_valid[cacop_idx][rst_i] <= 1'b0;
                                cache_dirty[cacop_idx][rst_i] <= 1'b0;
                            end
                        cacop_done <= 1'b1;
                        state <= IDLE;
                    end else if (cacop_code_r[4:3] == 2'b01 &&
                                 cache_valid[cacop_idx][cacop_way] &&
                                 cache_dirty[cacop_idx][cacop_way]) begin
                        // Dirty data remains valid until the AXI B response.
                        idx_r   <= cacop_idx;
                        tag_r   <= cache_tag[meta_addr(cacop_way, cacop_idx)];
                        wb_base <= {cacop_way, cacop_idx, 3'b000};
                        wb_cnt  <= 0;
                        wb_buf_cnt <= 0;
                        wb_fill_cap <= 1'b0;
                        axi_len <= 8'h07;
                        axi_addr <= {cache_tag[meta_addr(cacop_way, cacop_idx)],
                                    cacop_idx, {OFFSET_BITS{1'b0}}};
                        victim_way_r <= cacop_way;
                        bram_rd_addr <= {cacop_way, cacop_idx, 3'b000};
                        cacop_wb_active <= 1'b1;
                        state <= WB_WAIT;
                    end else begin
                        cache_valid[cacop_idx][cacop_way] <= 1'b0;
                        cache_dirty[cacop_idx][cacop_way] <= 1'b0;
                        cacop_done <= 1'b1;
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // IDLE presents the selected hit way, so all hits are ready in SRD.
    assign rdata = (state == BYPASS_RD && axi_valid && axi_ready) ?
                       (sb_match ?
                            merge_wdata(axi_rdata, sb_forward_data,
                                        sb_forward_strb) :
                            axi_rdata) :
                   ((state == SRD && srd_hit) && !we_r) ? bram_rd_data :
                   ((state == CACHE && srd_hit) && !we_r) ? bram_rd_data :
                   ((state == HIT_DATA) && !we_r) ? bram_rd_data :
                   wb_rdata;
    assign ready = (state == SRD && srd_hit) ||
                   (state == CACHE && srd_hit) ||
                   (state == HIT_DATA) ||
                   (state == BYPASS_RD && axi_valid && axi_ready) ||
                   (state == DONE);
    assign dbg_state = state;

    // ========================================================================
    // 全量诊断探针：自动检测数据完整性
    // ========================================================================
    // synthesis translate_off
    integer dc_hit_cnt, dc_miss_cnt, dc_fill_cnt, dc_wb_cnt;
    reg [3:0] dc_prev_state;
    reg       dc_prev_flush_active;
    reg       dc_trace_en;

    initial begin
        dc_dbg_cycle = 0; dc_hit_cnt = 0; dc_miss_cnt = 0;
        dc_fill_cnt = 0; dc_wb_cnt = 0;
        dc_prev_state = IDLE;
        dc_prev_flush_active = 0;
        dc_trace_en = $test$plusargs("cpu5_cache_stats");
    end

    always @(posedge clk) begin
        dc_dbg_cycle <= dc_dbg_cycle + 1;
        dc_prev_state <= state;
        dc_prev_flush_active <= flush_active;

        // 首次确认 dcache 已加载
        if (dc_trace_en && dc_dbg_cycle == 1000)
            $display("[DC-INIT] 4way-32KB cache: sets=%0d ways=%0d line=%0d idx_bits=%0d",
                     NUM_SETS, NUM_WAYS, LINE_SIZE, INDEX_BITS);

        // flush 开始/完成通知
        if (dc_trace_en && flush_active && !dc_prev_flush_active)
            $display("[DC-FLUSH c=%0d] flush started (auto-idle timeout)", dc_dbg_cycle);
        if (dc_trace_en && !flush_active && dc_prev_flush_active)
            $display("[DC-FLUSH c=%0d] flush completed", dc_dbg_cycle);

        if (!reset && dc_dbg_cycle > 500) begin
            // === 1. 状态机卡死检测 ===
            if (dc_trace_en && state != IDLE && state == dc_prev_state &&
                dc_dbg_cycle[12:0] == 13'h1FFF)
                $display("[DC-WARN c=%0d] stuck in state=%0d for >8192 cycles", dc_dbg_cycle, state);

            // === 2. 统计 ===
            if ((state == SRD && srd_hit) ||
                (state == CACHE && srd_hit) ||
                state == HIT_DATA)
                dc_hit_cnt = dc_hit_cnt + 1;
            if (state == SRD && !srd_hit) dc_miss_cnt = dc_miss_cnt + 1;
            if (state == REFILL && axi_valid && axi_ready) dc_fill_cnt = dc_fill_cnt + 1;
            if (state == REFILL && dc_prev_state == WB) dc_wb_cnt = dc_wb_cnt + 1;

            // === 3. 心跳摘要（每 10M 拍）===
            if (dc_trace_en && dc_dbg_cycle[22:0] == 23'h0) begin
                $display("[DC-SUM c=%0d] hit=%0d miss=%0d fill=%0d wb=%0d | st=%0d",
                         dc_dbg_cycle, dc_hit_cnt, dc_miss_cnt, dc_fill_cnt, dc_wb_cnt, state);
            end

            // === 3. 异常 ===
            if (dc_trace_en && srd_hit &&
                ((cache_valid[idx_r][2'd0] &&
                  cache_tag[meta_addr(2'd0, idx_r)] == tag_r) +
                 (cache_valid[idx_r][2'd1] &&
                  cache_tag[meta_addr(2'd1, idx_r)] == tag_r) +
                 (cache_valid[idx_r][2'd2] &&
                  cache_tag[meta_addr(2'd2, idx_r)] == tag_r) +
                 (cache_valid[idx_r][2'd3] &&
                  cache_tag[meta_addr(2'd3, idx_r)] == tag_r) > 1))
                $display("[DC-ERR c=%0d] MULTI-HIT idx=%0d tag=%h", dc_dbg_cycle, idx_r, tag_r);

            if (dc_trace_en && state == SRD && !srd_hit &&
                cache_valid[idx_r][2'd0] &&
                cache_valid[idx_r][2'd1] &&
                cache_valid[idx_r][2'd2] &&
                cache_valid[idx_r][2'd3] &&
                cache_dirty[idx_r][2'd0] &&
                cache_dirty[idx_r][2'd1] &&
                cache_dirty[idx_r][2'd2] &&
                cache_dirty[idx_r][2'd3])
                $display("[DC-INFO c=%0d] dirty victim writeback idx=%0d", dc_dbg_cycle, idx_r);

            if (srd_hit && tag_r === 18'hX)
                $display("[DC-ERR c=%0d] HIT-WITH-X-TAG idx=%0d", dc_dbg_cycle, idx_r);
        end
    end
    // synthesis translate_on

endmodule
