// ============================================================================
// next_pc_gen — 下一条 PC 生成器
//   优先级：分支跳转 > 例外/ERTN > 预测错误恢复 > 预测 > PC+4
//   branch_taken 最高——真实分支不可被预测或 stall 阻塞
// ============================================================================
module next_pc_gen(
    input  [31:0] pc,
    input         branch_taken,
    input  [31:0] branch_target,
    input         exception,
    input         eret,
    input  [31:0] era,
    input  [31:0] eentry,
    input         misp_flush,
    input  [31:0] misp_target,
    input         pred_taken,
    input  [31:0] pred_target,
    output [31:0] next_pc
);

    wire [31:0] pc_plus_4 = pc + 32'd4;

    wire exception_or_eret = exception || eret;
    wire [31:0] exception_pc = eret ? era : eentry;

    assign next_pc = branch_taken      ? branch_target :
                     exception_or_eret ? exception_pc :
                     misp_flush        ? misp_target :
                     pred_taken        ? pred_target   : pc_plus_4;

endmodule
