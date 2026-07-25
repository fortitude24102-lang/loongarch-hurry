// ============================================================================
// id_ex_reg �?? ID/EX 流水线寄存器（valid/ready + stall + flush�??
// ============================================================================
module id_ex_reg(
    input  clk, input  reset,
    input         valid_in, input  ready_in,
    input         flush,    input  stall,
    input  [31:0] stalled_rs1_in, input [31:0] stalled_rs2_in,
    input  [31:0] pc_in,    input [31:0] rs1_in,    input [31:0] rs2_in,
    input  [31:0] imm_in,   input [4:0]  rd_in,     input [4:0] rs1_addr_in,
    input  [4:0]  rs2_addr_in,  input [3:0] alu_op_in,  input alu_src_in,
    input  mem_read_in,   input mem_write_in,  input mem_to_reg_in,
    input  reg_write_in,  input branch_in,     input branch_ne_in, input branch_eq_in,
    input  branch_lt_in,  input branch_ge_in,  input branch_ltu_in,
    input  branch_geu_in, input ertn_in,       input syscall_in,
    input  break_in,      input [1:0] mem_width_in, input mem_signed_in,
    input  csr_rd_in,     input csr_wr_in,     input csr_xchg_in,
    input  [13:0] csr_addr_in, input is_jirl_in, input is_pcaddu12i_in,
    input  illegal_in,
    input         br_pred_in,  input [7:0] br_ghr_in,
    input         is_ll_in,    input        is_sc_in,
    input         is_mul_in,   input [1:0]  mul_op_in,
    input [4:0]   cacop_code_in,  input        cacop_valid_in,
    input         cpu_cfg_in,     input        idle_valid_in,

    output        ready_out,
    output reg    valid_out,  output reg [31:0] pc_out,
    output reg [31:0] rs1_out,    output reg [31:0] rs2_out,
    output reg [31:0] imm_out,    output reg [4:0]  rd_out,
    output reg [4:0]  rs1_addr_out,output reg [4:0]  rs2_addr_out,
    output reg [3:0]  alu_op_out, output reg alu_src_out,
    output reg mem_read_out,  output reg mem_write_out,
    output reg mem_to_reg_out,output reg reg_write_out,
    output reg branch_out,    output reg branch_ne_out, output reg branch_eq_out,
    output reg branch_lt_out, output reg branch_ge_out,
    output reg branch_ltu_out,output reg branch_geu_out,
    output reg ertn_out,      output reg syscall_out, output reg break_out,
    output reg [1:0] mem_width_out, output reg mem_signed_out,
    output reg csr_rd_out,    output reg csr_wr_out, output reg csr_xchg_out,
    output reg [13:0] csr_addr_out, output reg is_jirl_out,
    output reg is_pcaddu12i_out, output reg illegal_out,
    output reg br_pred_out, output reg [7:0] br_ghr_out,
    output reg is_ll_out,   output reg is_sc_out,
    output reg is_mul_out,  output reg [1:0] mul_op_out,
    output reg [4:0]  cacop_code_out, output reg cacop_valid_out,
    output reg        cpu_cfg_out,    output reg idle_valid_out
);

    assign ready_out = stall ? 1'b0 : (!valid_out || ready_in);

    always @(posedge clk) begin
        if (reset || flush) begin
            valid_out <= 0; pc_out <= 0; rs1_out <= 0; rs2_out <= 0;
            imm_out <= 0; rd_out <= 0; rs1_addr_out <= 0; rs2_addr_out <= 0;
            alu_op_out <= 0; alu_src_out <= 0;
            mem_read_out <= 0; mem_write_out <= 0; mem_to_reg_out <= 0;
            reg_write_out <= 0; branch_out <= 0; branch_ne_out <= 0;
            branch_eq_out <= 0;
            branch_lt_out <= 0; branch_ge_out <= 0;
            branch_ltu_out <= 0; branch_geu_out <= 0;
            ertn_out <= 0; syscall_out <= 0; break_out <= 0;
            mem_width_out <= 2'b10; mem_signed_out <= 1'b1;
            csr_rd_out <= 0; csr_wr_out <= 0; csr_xchg_out <= 0;
            csr_addr_out <= 0; is_jirl_out <= 0; is_pcaddu12i_out <= 0;
            illegal_out <= 0;
            br_pred_out <= 1'b0; br_ghr_out <= 8'h0;
            is_ll_out   <= 1'b0; is_sc_out   <= 1'b0;
            is_mul_out  <= 1'b0; mul_op_out  <= 2'b00;
            cacop_code_out <= 5'd0; cacop_valid_out <= 1'b0;
            cpu_cfg_out    <= 1'b0; idle_valid_out   <= 1'b0;
        end else if (stall || (valid_out && !ready_in)) begin
            // Forwarding sources may retire while this instruction waits
            // behind an older load or normal downstream backpressure.
            // Preserve forwarded operands as they become available instead
            // of falling back to stale RF values after the forwarding pulse.
            rs1_out <= stalled_rs1_in;
            rs2_out <= stalled_rs2_in;
        end else if (ready_out) begin
            valid_out <= valid_in;
            if (valid_in) begin
                pc_out <= pc_in; rs1_out <= rs1_in; rs2_out <= rs2_in;
                imm_out <= imm_in; rd_out <= rd_in;
                rs1_addr_out <= rs1_addr_in; rs2_addr_out <= rs2_addr_in;
                alu_op_out <= alu_op_in; alu_src_out <= alu_src_in;
                mem_read_out <= mem_read_in; mem_write_out <= mem_write_in;
                mem_to_reg_out <= mem_to_reg_in; reg_write_out <= reg_write_in;
                branch_out <= branch_in; branch_ne_out <= branch_ne_in;
                branch_eq_out <= branch_eq_in;
                branch_lt_out <= branch_lt_in; branch_ge_out <= branch_ge_in;
                branch_ltu_out <= branch_ltu_in; branch_geu_out <= branch_geu_in;
                ertn_out <= ertn_in; syscall_out <= syscall_in; break_out <= break_in;
                mem_width_out <= mem_width_in; mem_signed_out <= mem_signed_in;
                csr_rd_out <= csr_rd_in; csr_wr_out <= csr_wr_in;
                csr_xchg_out <= csr_xchg_in; csr_addr_out <= csr_addr_in;
                is_jirl_out <= is_jirl_in; is_pcaddu12i_out <= is_pcaddu12i_in;
                illegal_out <= illegal_in;
                br_pred_out <= br_pred_in; br_ghr_out <= br_ghr_in;
                is_ll_out   <= is_ll_in;   is_sc_out   <= is_sc_in;
                is_mul_out  <= is_mul_in;  mul_op_out  <= mul_op_in;
                cacop_code_out <= cacop_code_in; cacop_valid_out <= cacop_valid_in;
                cpu_cfg_out    <= cpu_cfg_in;    idle_valid_out   <= idle_valid_in;
            end else begin
                // bubble: 清零关键控制信号
                mem_read_out  <= 1'b0;
                mem_write_out <= 1'b0;
                reg_write_out <= 1'b0;
                branch_out    <= 1'b0;
                is_mul_out    <= 1'b0;
                cacop_code_out <= 5'b0; cacop_valid_out <= 1'b0;
                cpu_cfg_out    <= 1'b0; idle_valid_out   <= 1'b0;
            end
        end
    end

endmodule
