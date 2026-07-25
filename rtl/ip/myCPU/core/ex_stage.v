// ============================================================================
// ex_stage — EX2: ALU + 分支 (src1/src2 已由 EX1 前递)
// ============================================================================
module ex_stage(
    // EX1 寄存器输出 (已前递)
    input  [31:0] src1_in,
    input  [31:0] src2_in,
    input  [31:0] load_addr_in,   // AGU 预计算的 load/store 地址 (EX1 算好)
    input  [31:0] pc_in,
    input  [31:0] imm_in,
    input  [4:0]  rd_in,
    input  [3:0]  alu_op_in,
    input         alu_src_in,
    input         mem_read_in,
    input         mem_write_in,
    input         mem_to_reg_in,
    input         reg_write_in,
    input         branch_in,
    input         branch_ne_in, branch_eq_in,
    input         branch_lt_in,
    input         branch_ge_in,
    input         branch_ltu_in,
    input         branch_geu_in,
    input         ertn_in,
    input         syscall_in,
    input         break_in,
    input  [1:0]  mem_width_in,
    input         mem_signed_in,
    input         csr_rd_in,
    input         csr_wr_in,
    input         csr_xchg_in,
    input  [13:0] csr_addr_in,
    input         is_jirl_in,
    input         is_pcaddu12i_in,
    input         illegal_in,
    input         ex1_valid,    // EX1 级 valid，门控分支输出

    // 乘法结果（停顿式：结果已就绪时由 cpu_core 注入）
    input         is_mul_in,
    input  [1:0]  mul_op_in,
    input  [31:0] mul_p_low_in,    // mul_p[31:0]，在 ex_stage 内部做 op mux
    input  [31:0] mul_p_high_in,   // mul_p[63:32]

    // 输出到 ex_mem_reg (组合逻辑)
    output [31:0] alu_result,
    output [31:0] rs2_forwarded,
    output [4:0]  rd_out,
    output        mem_read_out,
    output        mem_write_out,
    output        mem_to_reg_out,
    output        reg_write_out,
    output        branch_taken,
    output [31:0] branch_target,
    output        csr_rd_out,
    output        csr_wr_out,
    output        csr_xchg_out,
    output [13:0] csr_addr_out,
    output [31:0] csr_wdata_out,
    output [31:0] csr_wmask_out,
    output        ine_out,
    output [1:0]  mem_width_out,   // 直通 ex1_reg，对齐其他控制信号
    output        mem_signed_out
);

    // ---- ALU ----
    wire [31:0] alu_b = alu_src_in ? imm_in : src2_in;
    wire [31:0] alu_a = is_pcaddu12i_in ? pc_in : src1_in;
    wire [31:0] alu_raw;
    alu u_alu(.a(alu_a), .b(alu_b), .op(alu_op_in), .y(alu_raw));

    // 访存指令用 EX1 预计算的 AGU 地址，跳过 ALU
    wire is_mem_access = mem_read_in || mem_write_in;
    wire [31:0] alu_result_comb = is_mem_access ? load_addr_in : alu_raw;

    // ---- 分支 ----
    wire is_uncond_branch = branch_in && !branch_ne_in && !branch_eq_in &&
                            !branch_lt_in && !branch_ge_in && !branch_ltu_in && !branch_geu_in;
    wire is_pc_relative_cond = branch_in && (branch_ne_in || branch_eq_in ||
                                             branch_lt_in || branch_ge_in ||
                                             branch_ltu_in || branch_geu_in);
    // JIRL 是间接跳转，目标 = rj+offset（ALU 结果），不是 PC 相对
    wire is_pc_relative = (is_uncond_branch || is_pc_relative_cond) && !is_jirl_in;
    wire [31:0] br_target = is_pc_relative ? (pc_in + imm_in) : alu_result_comb;

    wire br_taken;
    branch_unit u_branch(
        .a(is_uncond_branch ? pc_in : src1_in),
        .b(src2_in), .pc(pc_in),
        .branch(branch_in), .branch_ne(branch_ne_in),
        .branch_eq(branch_eq_in),
        .branch_lt(branch_lt_in), .branch_ge(branch_ge_in),
        .branch_ltu(branch_ltu_in), .branch_geu(branch_geu_in),
        .taken(br_taken)
    );

    wire need_link = branch_in && reg_write_in;
    wire [31:0] alu_final = need_link ? (pc_in + 32'd4) : alu_result_comb;
    wire is_illegal = illegal_in && ex1_valid;

    wire [31:0] mul_result_w = (mul_op_in == 2'b00) ? mul_p_low_in : mul_p_high_in;
    assign alu_result      = is_illegal  ? 32'h0 :
                              is_mul_in   ? mul_result_w : alu_final;

    assign rs2_forwarded   = is_illegal ? 32'h0          : src2_in;
    assign rd_out          = is_illegal ? 5'h0           : rd_in;
    assign mem_read_out    = is_illegal ? 1'b0           : mem_read_in;
    assign mem_write_out   = is_illegal ? 1'b0           : mem_write_in;
    assign mem_to_reg_out  = is_illegal ? 1'b0           : mem_to_reg_in;
    assign reg_write_out   = is_illegal ? 1'b0           : reg_write_in;
    assign branch_taken    = ex1_valid && branch_in ? br_taken : 1'b0;
    assign branch_target   = br_target;
    assign csr_rd_out      = is_illegal ? 1'b0           : csr_rd_in;
    assign csr_wr_out      = is_illegal ? 1'b0           : csr_wr_in;
    assign csr_xchg_out    = is_illegal ? 1'b0           : csr_xchg_in;
    assign csr_addr_out    = is_illegal ? 14'h0          : csr_addr_in;
    assign csr_wdata_out   = is_illegal ? 32'h0          : src1_in;
    assign csr_wmask_out   = is_illegal ? 32'h0          :
                             csr_wr_in   ? 32'hffff_ffff  :
                             csr_xchg_in ? src2_in        : 32'h0;
    assign ine_out         = is_illegal;
    assign mem_width_out   = is_illegal ? 2'b10 : mem_width_in;
    assign mem_signed_out  = is_illegal ? 1'b1  : mem_signed_in;

endmodule
