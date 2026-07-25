// ============================================================================
// gshare_predictor — Gshare 分支预测器
//   GHR(8-bit) XOR PC[7:0] → BHT(256×2-bit saturating counters) → pred_taken
//   BTB(64-entry, PC[7:2] index, PC[31:8] tag) → pred_target
//   BTB 存 is_uncond 标志：无条件分支命中时强制 pred_taken=1，根治死循环
//   更新在 EX2（分支决议），不用等 WB——无例外时结果已确认
//
//   所有时序逻辑合并在一个 always 块内，用单一 integer 复位，
//   避免 XSim 多 always 块 + 多 integer 并行复位时的调度未定义行为。
// ============================================================================
module gshare_predictor (
    input  clk,
    input  rst,

    // ---- 预测接口 (IF1) ----
    input  [31:0] pc,
    output        pred_taken,
    output [31:0] pred_target,
    output [7:0]  pred_ghr,

    // ---- 更新接口 (EX2) ----
    input         update_en,
    input  [31:0] update_pc,
    input         update_taken,
    input  [31:0] update_target,
    input  [7:0]  update_ghr_snap,
    input         update_indirect,
    input         update_uncond,    // 无条件分支→BTB 存 is_uncond 标志

    // ---- 恢复接口 (EX2) ----
    input         flush,
    input  [7:0]  flush_ghr
);

    // ========================================================================
    // GHR: 8-bit 全局历史
    // ========================================================================
    reg [7:0] ghr;

    // ========================================================================
    // BHT: 256 × 2-bit 饱和计数器
    // ========================================================================
    reg [1:0] bht [0:255];

    // ---- 预测读 (组合逻辑) ----
    wire [7:0] bht_index = pc[7:0] ^ ghr;
    wire [1:0] bht_counter = bht[bht_index];
    wire [7:0] bht_upd_idx = update_pc[7:0] ^ update_ghr_snap;

    // ========================================================================
    // BTB: 64 项
    // ========================================================================
    reg [23:0] btb_tag    [0:63];
    reg [31:0] btb_target [0:63];
    reg        btb_valid  [0:63];
    reg        btb_indirect [0:63];
    reg        btb_uncond [0:63];   // 1=无条件分支，命中时强制 pred_taken

    wire [5:0]  btb_idx = pc[7:2];
    wire        btb_hit = btb_valid[btb_idx] && (btb_tag[btb_idx] == pc[31:8]);
    wire [5:0]  btb_upd_idx = update_pc[7:2];

    // ---- 预测输出 (X-safe: 用 == 而非 >=, 避免 bht_counter 含 X 时传播) ----
    // 只有 BTB 命中且目标稳定时才允许 taken；条件分支使用标准两位计数器，
    // 无条件直接跳转由 BTB 标志强制 taken。JIRL 在目标校验/RAS 完成前不预测。
    wire bht_taken = (bht_counter == 2'b10) || (bht_counter == 2'b11);
    assign pred_taken  = btb_hit && !btb_indirect[btb_idx] &&
                         (btb_uncond[btb_idx] || bht_taken);
    assign pred_target = btb_hit ? btb_target[btb_idx] : (pc + 32'd4);
    assign pred_ghr    = ghr;

    // ========================================================================
    // 全部时序逻辑合并在单 always 块 —— 消除 XSim 多 always 并行复位调度冲突
    // ========================================================================
    integer bi, bv;
    always @(posedge clk) begin
        if (rst) begin
            ghr <= 8'b0;
            for (bi = 0; bi < 256; bi = bi + 1)
                bht[bi] = 2'b01;
            for (bv = 0; bv < 64; bv = bv + 1) begin
                btb_valid[bv]  = 1'b0;
                btb_indirect[bv] = 1'b0;
                btb_uncond[bv] = 1'b0;
            end
        end else begin
            // ---- GHR 更新 ----
            if (flush)
                ghr <= flush_ghr;
            else if (update_en)
                ghr <= {ghr[6:0], update_taken};

            // ---- BHT 更新 ----
            if (update_en) begin
                case (bht[bht_upd_idx])
                    2'b00: bht[bht_upd_idx] <= update_taken ? 2'b01 : 2'b00;
                    2'b01: bht[bht_upd_idx] <= update_taken ? 2'b10 : 2'b00;
                    2'b10: bht[bht_upd_idx] <= update_taken ? 2'b11 : 2'b01;
                    2'b11: bht[bht_upd_idx] <= update_taken ? 2'b11 : 2'b10;
                endcase
            end

            // ---- BTB 更新 ----
            if (update_en && update_taken) begin
                btb_tag[btb_upd_idx]    <= update_pc[31:8];
                btb_target[btb_upd_idx] <= update_target;
                btb_valid[btb_upd_idx]  <= 1'b1;
                btb_indirect[btb_upd_idx] <= update_indirect;
                btb_uncond[btb_upd_idx] <= update_uncond;
            end
        end
    end

endmodule
