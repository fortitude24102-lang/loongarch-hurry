// ============================================================================
// icache — 4KB 直接映射指令缓存 (128 sets × 8 words)
//   行为级数组 + (* ram_style = "block" *) 推断 BRAM
//   已验证综合成功：44K LUT, 0 error
// ============================================================================
module icache(
    input  clk,
    input  reset,

    input         req,
    input  [31:0] addr,
    output [31:0] inst,         // 组合逻辑输出（hit 时），与 addr 同步变化
    output reg    ready,
    output        busy,
    output        pair_valid,   // 本拍 inst 与 addr 严格配对有效（if_id 锁存依据）

    output reg         axi_req,
    output reg  [31:0] axi_addr,
    output reg  [7:0]  axi_len,      // burst 长度：8'h07=8-beat line fill
    input       [31:0] axi_rdata,
    input              axi_valid,
    input              axi_ready,

    // ---- cacop 缓存操作接口 ----
    input              cacop_req,
    input  [4:0]       cacop_code,
    input  [31:0]      cacop_addr,
    output reg         cacop_done
);

    parameter NUM_SETS    = 128;
    parameter LINE_SIZE   = 8;
    parameter INDEX_BITS  = 7;
    parameter OFFSET_BITS = 5;
    parameter TAG_BITS    = 20;

    localparam IDLE       = 2'b00,
               CACHE_READ = 2'b01,
               CACOP_ST   = 2'b11,
               MISS_FETCH = 2'b10;

    (* ram_style = "block" *) reg [31:0] icache_data [0:NUM_SETS*LINE_SIZE-1];

    (* ram_style = "distributed" *) reg [TAG_BITS-1:0] cache_tag [0:NUM_SETS-1];
    (* ram_style = "distributed" *) reg                cache_valid [0:NUM_SETS-1];

    wire [INDEX_BITS-1:0]  index       = addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    wire [TAG_BITS-1:0]    addr_tag    = addr[31:OFFSET_BITS+INDEX_BITS];
    wire [2:0]             word_offset = addr[OFFSET_BITS-1:2];
    wire [INDEX_BITS+2:0]  way_addr    = {index, word_offset};
    wire                   hit         = cache_valid[index] && (cache_tag[index] == addr_tag);
    reg  [1:0]             state;

    // 组合读取：addr 变化 → inst 立即变化，消除 1 周期 PC/inst 错位
    wire [31:0] comb_inst = icache_data[way_addr];
    reg  [31:0] inst_latched;
    reg  [31:0] ready_addr;     // ready/inst_latched 对应的取指地址
    wire        serve_hit = (state == CACHE_READ || state == MISS_FETCH) && req && hit;
    assign      inst      = serve_hit ? comb_inst : inst_latched;

    // 配对有效：正在组合服务当前 addr，或已完成的取指对（ready+inst_latched）恰好
    //   属于当前 addr（"已完成但未消费"，如 store 抢总线前一拍刚命中的指令）。
    //   反例即幽灵注入：ready 残留为 1 但 ready_addr≠addr（if1 已推进）→ 必须拦。
    //   单看 ready 无法区分这两种情况——ready 不携带"属于哪个 pc"的信息。
    assign      pair_valid = serve_hit || (ready && ready_addr == addr);

    // busy 必须在整个 miss 处理期间连续为高，否则 PC/if1 会在状态机
    //   CACHE_READ(miss)→IDLE→MISS_FETCH 的 IDLE 空档里前进一拍，
    //   使 if1_pc 越过触发 miss 的那条指令（如 0x20 的 ld.w 被跳过）。
    //   补上 IDLE 且请求未命中这一拍（纯增量，不改原有两项行为）。
    assign busy = (state == MISS_FETCH) || ((state == CACHE_READ) && !hit)
                  || ((state == IDLE) && req && !hit);

    reg [31:0]           miss_addr;
    reg [INDEX_BITS-1:0] miss_idx;
    reg [TAG_BITS-1:0]   miss_tag;
    reg [3:0]            fill_counter;

    // Keep the tag array on one synchronous write port so Vivado can infer
    // distributed RAM instead of expanding it into registers and muxes.
    wire tag_cacop_clear =
        cacop_req && (cacop_code[4:3] == 2'b00) && (state != CACOP_ST);
    wire tag_refill_write =
        (state == MISS_FETCH) && !cacop_req && axi_valid && axi_ready &&
        (fill_counter == LINE_SIZE - 1);
    wire tag_write = tag_cacop_clear || tag_refill_write;
    wire [INDEX_BITS-1:0] tag_write_addr =
        tag_cacop_clear ?
        cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS] : miss_idx;
    wire [TAG_BITS-1:0] tag_write_data =
        tag_cacop_clear ? {TAG_BITS{1'b0}} : miss_tag;
    always @(posedge clk) begin
        if (tag_write)
            cache_tag[tag_write_addr] <= tag_write_data;
    end

    // 锁存请求地址——避免组合逻辑 addr 变化导致读错 word_offset
    integer ci;
    always @(posedge clk) begin
        if (reset) begin
            state        <= IDLE;
            ready        <= 1'b0;
            ready_addr   <= 32'h0;
            inst_latched <= 32'h0;   // 防 X：未初始化的锁存值在 MISS_FETCH 前暴露
            axi_req      <= 1'b0;
            axi_len      <= 8'h00;
            fill_counter <= 0;
            cacop_done   <= 1'b0;
            for (ci = 0; ci < NUM_SETS; ci = ci + 1)
                cache_valid[ci] = 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (cacop_req) begin
                        // cacop: 清除 indexed set 的 valid（I-cache 无脏数据）
                        //   cacop_code[4:3]: 0=StoreTag(也清tag), 1/2=Invalidate
                        cache_valid[cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS]] <= 1'b0;
                        cacop_done <= 1'b1;
                        state <= CACOP_ST;
                    end else if (req) begin
                        if (hit) begin
                            inst_latched <= icache_data[way_addr];
                            ready <= 1'b1;
                            ready_addr <= addr;
                            state <= CACHE_READ;
                        end else begin
                            miss_addr    <= addr;
                            miss_idx     <= index;
                            miss_tag     <= addr_tag;
                            axi_req      <= 1'b1;
                            axi_addr     <= {addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                            axi_len      <= 8'h07;  // 8-beat 读 burst
                            ready        <= 1'b0;
                            fill_counter <= 0;
                            state        <= MISS_FETCH;
                        end
                    end else begin
                        ready <= 1'b0;
                    end
                end

                CACHE_READ: begin
                    if (cacop_req) begin
                        cache_valid[cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS]] <= 1'b0;
                        axi_req <= 1'b0;   // 丢弃未决的 AXI 请求
                        cacop_done <= 1'b1;
                        state <= CACOP_ST;
                    end else if (req && hit) begin
                        inst_latched <= icache_data[way_addr];
                        ready <= 1'b1;
                        ready_addr <= addr;
                    end else begin
                        ready <= 1'b0;
                        state <= IDLE;
                    end
                end

                MISS_FETCH: begin
                    if (cacop_req) begin
                        cache_valid[cacop_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS]] <= 1'b0;
                        axi_req <= 1'b0;   // 放弃正在进行的 refill
                        cacop_done <= 1'b1;
                        state <= CACOP_ST;
                    end else begin
                    // 不因 req=0 取消填充——D-cache 请求会暂时抢占总线
                    if (axi_valid && axi_ready) begin
                        icache_data[{miss_idx, fill_counter[2:0]}] <= axi_rdata;
                        fill_counter <= fill_counter + 1;
                        if (fill_counter == LINE_SIZE - 1) begin
                            cache_valid[miss_idx] <= 1'b1;

                            inst_latched <= (miss_addr[4:2] == fill_counter[2:0]) ? axi_rdata :
                                    icache_data[{miss_idx, miss_addr[4:2]}];
                            ready   <= 1'b1;
                            ready_addr <= miss_addr;
                            axi_req <= 1'b0;
                            state   <= CACHE_READ;
                        end
                        // burst 模式无需递增地址——arbiter 内部处理
                    end
                    // 填充期间仍可响应其他已缓存 line 的 hit
                    else if (req && hit) begin
                        inst_latched <= icache_data[way_addr];
                        ready <= 1'b1;
                        ready_addr <= addr;
                    end
                    end  // else begin
                end  // MISS_FETCH

                CACOP_ST: begin
                    cacop_done <= 1'b0;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
