// ============================================================================
// mem1_reg — MEM1→MEM2 流水线寄存器
//   MEM1 发出 dcache 请求，mem1_reg 锁存控制信号，
//   MEM2 在 dcache CACHE 周期接收数据并完成对齐
// ============================================================================
module mem1_reg(
    input  clk,
    input  reset,

    input         valid_in,     // ex_mem_valid
    input         ready_in,     // mem2_ready (MEM2 可以接收)
    input         flush,        // 例外刷新
    input         stall,        // 停顿

    // 控制信号
    input  [31:0] alu_result_in,
    input  [31:0] rs2_in,
    input  [4:0]  rd_in,
    input         mem_read_in,
    input         mem_write_in,
    input         mem_to_reg_in,
    input         reg_write_in,
    input  [1:0]  mem_width_in,
    input         mem_signed_in,
    input         is_uart_in,
    input         csr_rd_in,
    input         csr_wr_in,
    input         csr_xchg_in,
    input  [13:0] csr_addr_in,
    input  [31:0] csr_wdata_in,
    input  [31:0] csr_wmask_in,

    output        ready_out,
    output reg    valid_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] rs2_out,
    output reg [4:0]  rd_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg mem_to_reg_out,
    output reg reg_write_out,
    output reg [1:0] mem_width_out,
    output reg mem_signed_out,
    output reg is_uart_out,
    output reg csr_rd_out,
    output reg csr_wr_out,
    output reg csr_xchg_out,
    output reg [13:0] csr_addr_out,
    output reg [31:0] csr_wdata_out,
    output reg [31:0] csr_wmask_out
);

    assign ready_out = stall ? 1'b0 : (!valid_out || ready_in);

    always @(posedge clk) begin
        if (reset || flush) begin
            valid_out      <= 1'b0;
            alu_result_out <= 32'h0;
            rs2_out        <= 32'h0;
            rd_out         <= 5'h0;
            mem_read_out   <= 1'b0;
            mem_write_out  <= 1'b0;
            mem_to_reg_out <= 1'b0;
            reg_write_out  <= 1'b0;
            mem_width_out  <= 2'b10;
            mem_signed_out <= 1'b1;
            is_uart_out    <= 1'b0;
            csr_rd_out      <= 1'b0;
            csr_wr_out      <= 1'b0;
            csr_xchg_out    <= 1'b0;
            csr_addr_out    <= 14'b0;
            csr_wdata_out   <= 32'b0;
            csr_wmask_out   <= 32'b0;
        end else if (stall) begin
            alu_result_out <= alu_result_out;   // 显式保持
        end else if (ready_out) begin
            valid_out <= valid_in;
            if (valid_in) begin
                alu_result_out <= alu_result_in;
                rs2_out        <= rs2_in;
                rd_out         <= rd_in;
                mem_read_out   <= mem_read_in;
                mem_write_out  <= mem_write_in;
                mem_to_reg_out <= mem_to_reg_in;
                reg_write_out  <= reg_write_in;
                mem_width_out  <= mem_width_in;
                mem_signed_out <= mem_signed_in;
                is_uart_out    <= is_uart_in;
                csr_rd_out      <= csr_rd_in;
                csr_wr_out      <= csr_wr_in;
                csr_xchg_out    <= csr_xchg_in;
                csr_addr_out    <= csr_addr_in;
                csr_wdata_out   <= csr_wdata_in;
                csr_wmask_out   <= csr_wmask_in;
            end else begin
                mem_read_out  <= 1'b0;
                mem_write_out <= 1'b0;
                reg_write_out <= 1'b0;
                csr_rd_out    <= 1'b0;
                csr_wr_out    <= 1'b0;
                csr_xchg_out  <= 1'b0;
                alu_result_out <= alu_result_out;
            end
        end
    end

endmodule
