// ============================================================================
// branch_unit — 分支判断单元
//   根据分支类型信号比较操作数 a 和 b，输出跳转决策。
//   无条件跳转 (B/BL/JIRL)：所有条件信号为 0 时成立。
// ============================================================================
module branch_unit(
    input  [31:0] a,          // 源操作数1 (rj 或 PC)
    input  [31:0] b,          // 源操作数2 (rd 或 imm 或 0)
    input  [31:0] pc,         // 当前指令 PC（保留，未使用）
    input         branch,     // 是否为分支/跳转指令
    input         branch_ne,  // 条件分支：不等 (BNE)
    input         branch_eq,  // 条件分支：相等 (BEQ)
    input         branch_lt,  // 条件分支：有符号小于 (BLT)
    input         branch_ge,  // 条件分支：有符号大于等于 (BGE)
    input         branch_ltu, // 条件分支：无符号小于 (BLTU)
    input         branch_geu, // 条件分支：无符号大于等于 (BGEU)
    output        taken       // 跳转成立
);

    // ---- 比较结果 ----------------------------------------------------------
    wire cmp_eq  = (a == b);
    wire cmp_lt  = $signed(a) < $signed(b);
    wire cmp_ge  = $signed(a) >= $signed(b);
    wire cmp_ltu = a < b;
    wire cmp_geu = a >= b;

    // ---- 无条件跳转 --------------------------------------------------------
    // 当 branch=1 且所有条件分支标志为 0 时，为无条件跳转
    wire is_uncond = branch && !branch_ne && !branch_eq && !branch_lt && !branch_ge && !branch_ltu && !branch_geu;

    // ---- 条件跳转判断 -------------------------------------------------------
    wire cond_taken = (branch_eq  &&  cmp_eq) ||  // BEQ
                      (branch_ne  && !cmp_eq) ||  // BNE
                      (branch_lt  &&  cmp_lt) ||  // BLT
                      (branch_ge  &&  cmp_ge) ||  // BGE
                      (branch_ltu &&  cmp_ltu) || // BLTU
                      (branch_geu &&  cmp_geu);   // BGEU

    assign taken = branch && (is_uncond || cond_taken);

endmodule
