// ============================================================================
// mem_stage �? MEM2: dcache 数据接收 + 字节对齐 + 符号扩展
//   MEM1 (cpu_core) 已发�? dcache 请求，本模块等待 ready 后处理数�?
// ============================================================================
module mem_stage(
    input clk,
    input reset,

    input  valid_in,          // mem1_reg.valid_out
    output ready_out,

    input [31:0] alu_result_in,
    input [4:0]  rd_in,
    input        mem_read_in,
    input        mem_write_in,
    input        mem_to_reg_in,
    input        reg_write_in,
    input [1:0]  mem_width_in,
    input        mem_signed_in,
    input        is_uart,
    input        csr_rd_in,
    input        csr_wr_in,
    input        csr_xchg_in,
    input [13:0] csr_addr_in,
    input [31:0] csr_wdata_in,
    input [31:0] csr_wmask_in,

    input  [31:0] dcache_rdata,
    input         dcache_ready,

    output reg [31:0] mem_result,
    output reg        csr_rd_out,
    output reg        csr_wr_out,
    output reg        csr_xchg_out,
    output reg [13:0] csr_addr_out,
    output reg [31:0] csr_wdata_out,
    output reg [31:0] csr_wmask_out,
    output reg [4:0]  mem_rd_out,          // 同步寄存
    output reg        mem_reg_write_out,    // 同步寄存
    output reg        mem_to_reg_out,       // 同步寄存（对�? mem_result�?
    output reg        valid_out             // 同步寄存（对�? mem_result�?
);

    // ---- 读数据对�? --------------------------------------------------------
    reg [31:0] mem_read_data;
    always @(*) begin
        case (mem_width_in)
            2'b00: begin  // byte
                case (alu_result_in[1:0])
                    2'b00: mem_read_data = {24'b0, dcache_rdata[7:0]};
                    2'b01: mem_read_data = {24'b0, dcache_rdata[15:8]};
                    2'b10: mem_read_data = {24'b0, dcache_rdata[23:16]};
                    2'b11: mem_read_data = {24'b0, dcache_rdata[31:24]};
                    default: mem_read_data = {24'b0, dcache_rdata[7:0]};
                endcase
                if (mem_signed_in && mem_read_data[7])
                    mem_read_data = {24'hFF_FFFF, mem_read_data[7:0]};
            end
            2'b01: begin  // half
                if (alu_result_in[1])
                    mem_read_data = {16'b0, dcache_rdata[31:16]};
                else
                    mem_read_data = {16'b0, dcache_rdata[15:0]};
                if (mem_signed_in && mem_read_data[15])
                    mem_read_data = {16'hFFFF, mem_read_data[15:0]};
            end
            default: mem_read_data = dcache_rdata;  // word
        endcase
    end

    // UART 写不再走快�?�完成路径：�? SoC �? UART �? AXI CDC+Crossbar 后面�?
    //   写必须等 B 响应—�?�否则背靠背 UART 写时前一个写还在 wr_pending�?
    //   仲裁器无法捕获后�?个写的数据（已经流出 MEM1），字节静默丢失�?
    //   �? thinpad uart_controller 有本�? FIFO 缓冲，快速完成才安全�?
    // A UART store is complete only after the direct AXI path receives B.
    // Until then mem1_reg keeps address/data/strb stable for the arbiter.
    wire mem_access_done = !(mem_read_in || mem_write_in) || dcache_ready;
    wire [31:0] mem_result_w = mem_to_reg_in ? mem_read_data : alu_result_in;
    assign ready_out = !valid_in || mem_access_done;

    always @(posedge clk) begin
        if (reset) begin
            mem_result        <= 32'h0;
            mem_rd_out        <= 5'h0;
            mem_reg_write_out <= 1'b0;
            mem_to_reg_out    <= 1'b0;
            valid_out         <= 1'b0;
            csr_rd_out        <= 1'b0;
            csr_wr_out        <= 1'b0;
            csr_xchg_out      <= 1'b0;
            csr_addr_out      <= 14'b0;
            csr_wdata_out     <= 32'b0;
            csr_wmask_out     <= 32'b0;
        end else begin
            if (valid_in && mem_access_done) begin
                mem_result        <= mem_result_w;
                mem_rd_out        <= rd_in;
                mem_reg_write_out <= reg_write_in && !mem_write_in;
                mem_to_reg_out    <= mem_to_reg_in;
                csr_rd_out        <= csr_rd_in;
                csr_wr_out        <= csr_wr_in;
                csr_xchg_out      <= csr_xchg_in;
                csr_addr_out      <= csr_addr_in;
                csr_wdata_out     <= csr_wdata_in;
                csr_wmask_out     <= csr_wmask_in;
            end else begin
                mem_reg_write_out <= 1'b0;
                csr_rd_out        <= 1'b0;
                csr_wr_out        <= 1'b0;
                csr_xchg_out      <= 1'b0;
            end
            valid_out <= valid_in && mem_access_done;
        end
    end

endmodule
