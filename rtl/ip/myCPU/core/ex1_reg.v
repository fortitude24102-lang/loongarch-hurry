// ============================================================================
// ex1_reg �? EX1→EX2 流水线寄存器
//   EX1 完成前�?? MUX 选择并锁存操作数，EX2 �? ALU + 分支
// ============================================================================
module ex1_reg(
    input  clk,
    input  reset,
    input  flush,
    input  stall,     // Partial Bubble: 冻结 EX1→EX2

    input         valid_in,
    input         ready_in,
    output        ready_out,

    // 操作�? (前�?? MUX 输出)
    input  [31:0] src1_in,
    input  [31:0] src2_in,
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
    input         branch_ne_in, input branch_eq_in,
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
    input         br_pred_in,  input [7:0] br_ghr_in,
    input         is_ll_in,    input        is_sc_in,
    input         is_mul_in,   input [1:0]  mul_op_in,
    input [4:0]   cacop_code_in,  input        cacop_valid_in,
    input         cpu_cfg_in,     input        idle_valid_in,

    output reg         valid_out,
    output reg [31:0]  src1_out,
    output reg [31:0]  load_addr_out,
    output reg [31:0]  src2_out,
    output reg [31:0]  pc_out,
    output reg [31:0]  imm_out,
    output reg [4:0]   rd_out,
    output reg [3:0]   alu_op_out,
    output reg         alu_src_out,
    output reg         mem_read_out,
    output reg         mem_write_out,
    output reg         mem_to_reg_out,
    output reg         reg_write_out,
    output reg         branch_out,
    output reg         branch_ne_out, output reg branch_eq_out,
    output reg         branch_lt_out,
    output reg         branch_ge_out,
    output reg         branch_ltu_out,
    output reg         branch_geu_out,
    output reg         ertn_out,
    output reg         syscall_out,
    output reg         break_out,
    output reg [1:0]   mem_width_out,
    output reg         mem_signed_out,
    output reg         csr_rd_out,
    output reg         csr_wr_out,
    output reg         csr_xchg_out,
    output reg [13:0]  csr_addr_out,
    output reg         is_jirl_out,
    output reg         is_pcaddu12i_out,
    output reg         illegal_out,
    output reg         br_pred_out, output reg [7:0] br_ghr_out,
    output reg         is_ll_out,   output reg        is_sc_out,
    output reg         is_mul_out,  output reg [1:0]   mul_op_out,
    output reg [4:0]   cacop_code_out, output reg cacop_valid_out,
    output reg         cpu_cfg_out,    output reg idle_valid_out
);

    // 防止 self-forwarding replay�?
    // 同一指令�? EX1 �? valid 时又�? EX_in 呈现（src 已被 forwarding 更新）→ �?
    reg ready_out_prev;

    assign ready_out = stall ? 1'b0 : (!valid_out || ready_in);

    always @(posedge clk) begin
        ready_out_prev <= ready_out;

        if (reset || flush) begin
            valid_out   <= 1'b0;
            src1_out    <= 32'h0;
            src2_out    <= 32'h0;
            load_addr_out <= 32'h0;
            pc_out      <= 32'h0;
            imm_out     <= 32'h0;
            rd_out      <= 5'h0;
            alu_op_out  <= 4'h0;
            alu_src_out <= 1'b0;
            mem_read_out <= 1'b0; mem_write_out <= 1'b0; mem_to_reg_out <= 1'b0;
            reg_write_out <= 1'b0;
            branch_out  <= 1'b0; branch_ne_out <= 1'b0;
            branch_eq_out <= 1'b0;
            branch_lt_out <= 1'b0; branch_ge_out <= 1'b0;
            branch_ltu_out <= 1'b0; branch_geu_out <= 1'b0;
            ertn_out <= 1'b0; syscall_out <= 1'b0; break_out <= 1'b0;
            mem_width_out <= 2'b10; mem_signed_out <= 1'b1;
            csr_rd_out <= 1'b0; csr_wr_out <= 1'b0; csr_xchg_out <= 1'b0;
            csr_addr_out <= 14'h0;
            is_jirl_out <= 1'b0; is_pcaddu12i_out <= 1'b0;
            illegal_out <= 1'b0;
            br_pred_out <= 1'b0; br_ghr_out <= 8'h0;
            is_ll_out   <= 1'b0; is_sc_out   <= 1'b0;
            is_mul_out  <= 1'b0; mul_op_out  <= 2'b00;
            cacop_code_out <= 5'd0; cacop_valid_out <= 1'b0;
            cpu_cfg_out    <= 1'b0; idle_valid_out   <= 1'b0;
        end else if (ready_out) begin
            // 当前 valid_out=1 且输入仍是同�? PC �? self-forwarding replay，杀
            if (valid_out && valid_in && pc_in == pc_out) begin
                valid_out     <= 1'b0;
                mem_read_out  <= 1'b0;
                mem_write_out <= 1'b0;
                reg_write_out <= 1'b0;
                branch_out    <= 1'b0;
                is_mul_out    <= 1'b0;
                cacop_valid_out <= 1'b0;
                cacop_code_out <= 5'd0;
                cpu_cfg_out   <= 1'b0;
            end else begin
                valid_out <= valid_in;
                if (valid_in) begin
                    src1_out    <= src1_in;    src2_out    <= src2_in;
                    load_addr_out <= src1_in + imm_in;   // 内部计算，避免外�? wire �? XSim Z 问题
                    pc_out      <= pc_in;      imm_out     <= imm_in;
                    rd_out      <= rd_in;      alu_op_out  <= alu_op_in;
                    alu_src_out <= alu_src_in;
                    mem_read_out <= mem_read_in; mem_write_out <= mem_write_in;
                    mem_to_reg_out <= mem_to_reg_in; reg_write_out <= reg_write_in;
                    branch_out  <= branch_in;  branch_ne_out <= branch_ne_in;
                    branch_eq_out <= branch_eq_in;
                    branch_lt_out <= branch_lt_in; branch_ge_out <= branch_ge_in;
                    branch_ltu_out <= branch_ltu_in; branch_geu_out <= branch_geu_in;
                    ertn_out    <= ertn_in;    syscall_out <= syscall_in;
                    break_out   <= break_in;
                    mem_width_out <= mem_width_in; mem_signed_out <= mem_signed_in;
                    csr_rd_out  <= csr_rd_in;  csr_wr_out <= csr_wr_in;
                    csr_xchg_out <= csr_xchg_in; csr_addr_out <= csr_addr_in;
                    is_jirl_out <= is_jirl_in; is_pcaddu12i_out <= is_pcaddu12i_in;
                    illegal_out <= illegal_in;
                    br_pred_out <= br_pred_in; br_ghr_out <= br_ghr_in;
                    is_ll_out   <= is_ll_in;   is_sc_out   <= is_sc_in;
                    is_mul_out  <= is_mul_in;  mul_op_out  <= mul_op_in;
                    cacop_code_out <= cacop_code_in; cacop_valid_out <= cacop_valid_in;
                    cpu_cfg_out    <= cpu_cfg_in;    idle_valid_out   <= idle_valid_in;
                end else begin
                    mem_read_out  <= 1'b0;
                    mem_write_out <= 1'b0;
                    reg_write_out <= 1'b0;
                    branch_out    <= 1'b0;
                    is_mul_out    <= 1'b0;
                cacop_valid_out <= 1'b0;
                cacop_code_out <= 5'd0;
                cpu_cfg_out   <= 1'b0;
                end
            end
        end
    end

endmodule
