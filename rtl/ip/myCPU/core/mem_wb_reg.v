// ============================================================================
// mem_wb_reg — MEM/WB 流水线寄存器（valid/ready + stall + flush）
// ============================================================================
module mem_wb_reg(
    input  clk, input  reset,
    input         valid_in,  input  ready_in,
    input         flush,     input  stall,
    input  [31:0] mem_result_in, input [4:0] rd_in,
    input  mem_to_reg_in,   input reg_write_in,
    input  csr_rd_in,       input csr_wr_in,     input csr_xchg_in,
    input  [13:0] csr_addr_in,  input [31:0] csr_wdata_in,
    input  [31:0] csr_wmask_in,

    output        ready_out,
    output reg    valid_out,      output reg [31:0] mem_result_out,
    output reg [4:0]  rd_out,     output reg mem_to_reg_out,
    output reg reg_write_out,     output reg csr_rd_out,
    output reg csr_wr_out,        output reg csr_xchg_out,
    output reg [13:0] csr_addr_out, output reg [31:0] csr_wdata_out,
    output reg [31:0] csr_wmask_out
);

    assign ready_out = stall ? 1'b0 : (!valid_out || ready_in);

    always @(posedge clk) begin
        if (reset || flush) begin
            valid_out <= 0; mem_result_out <= 0; rd_out <= 5'h0;
            mem_to_reg_out <= 0; reg_write_out <= 0;
            csr_rd_out <= 0; csr_wr_out <= 0; csr_xchg_out <= 0;
            csr_addr_out <= 0; csr_wdata_out <= 0; csr_wmask_out <= 0;
        end else if (stall) begin
        end else if (ready_out) begin
            valid_out <= valid_in;
            if (valid_in) begin
                mem_result_out <= mem_result_in; rd_out <= rd_in;
                mem_to_reg_out <= mem_to_reg_in; reg_write_out <= reg_write_in;
                csr_rd_out <= csr_rd_in; csr_wr_out <= csr_wr_in;
                csr_xchg_out <= csr_xchg_in; csr_addr_out <= csr_addr_in;
                csr_wdata_out <= csr_wdata_in;
                csr_wmask_out <= csr_wmask_in;
            end else begin
                mem_to_reg_out <= 1'b0;
                reg_write_out  <= 1'b0;
                csr_rd_out     <= 1'b0;
                csr_wr_out     <= 1'b0;
                csr_xchg_out   <= 1'b0;
            end
        end
    end

endmodule
