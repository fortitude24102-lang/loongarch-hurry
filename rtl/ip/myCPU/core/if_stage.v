// ============================================================================
// if_stage — 纯取指阶段（无流水线控制，仅数据通路）
// ============================================================================
module if_stage(
    input  [31:0] icache_inst,
    input  [31:0] pc,
    output [31:0] inst_out,
    output [31:0] pc_out
);
    assign inst_out = icache_inst;
    assign pc_out   = pc;
endmodule
