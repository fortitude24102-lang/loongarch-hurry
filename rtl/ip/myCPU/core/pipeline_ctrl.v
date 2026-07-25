// ============================================================================
// pipeline_ctrl — 流水线控制器（分支 + 例外 + 预测错误恢复）
// ============================================================================
module pipeline_ctrl(
    input  clk,
    input  reset,
    input  [31:0] pc,
    input         branch_taken,
    input  [31:0] branch_target,
    input         misp_flush,
    input         exception,
    input         ertn,
    input  [31:0] era,
    input  [31:0] eentry,
    input         if_id_valid,
    input         id_ex_valid,
    input         ex_mem_valid,
    input         mem_wb_valid,
    output reg    if_flush,
    output reg    id_flush,
    output reg    ex_flush
);

    reg branch_taken_d1;
    reg misp_flush_d1;
    always @(posedge clk) begin
        if (reset) begin
            branch_taken_d1 <= 1'b0;
            misp_flush_d1   <= 1'b0;
        end else begin
            branch_taken_d1 <= branch_taken;
            misp_flush_d1   <= misp_flush;
        end
    end
    wire branch_rising = branch_taken && !branch_taken_d1;
    wire misp_rising   = misp_flush   && !misp_flush_d1;

    always @(*) begin
        if_flush = 1'b0;
        id_flush = 1'b0;
        ex_flush = 1'b0;

        if (exception || ertn) begin
            if_flush = 1'b1;
            id_flush = 1'b1;
            ex_flush = 1'b1;
        end
        else if (branch_rising) begin
            if_flush = 1'b1;
            id_flush = 1'b1;
            ex_flush = 1'b0;
        end
        else if (misp_rising) begin
            if_flush = 1'b1;
            id_flush = 1'b1;
            ex_flush = 1'b0;
        end
    end

endmodule
