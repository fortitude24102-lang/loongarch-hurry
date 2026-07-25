// ============================================================================
// forwarding_unit — 并行比较 + 优先编码，减少组合逻辑级数
//   forward code: 2'b10=EX, 2'b01=MEM, 2'b11=WB, 2'b00=RF
// ============================================================================
module forwarding_unit(
    input  [4:0]  rs1,  input  [4:0]  rs2,
    // EX 组合前推（load 时 ALU 结果是地址，不可前递）
    input  [4:0]  ex_comb_rd,     input ex_comb_reg_write,   input [31:0] ex_comb_result,
    input         ex_comb_is_load,
    // EX_MEM 寄存器前推（load 时 ALU 结果是地址，不可前递）
    input  [4:0]  ex_mem_rd,      input ex_mem_reg_write,    input [31:0] ex_mem_result,
    input         ex_mem_is_load,
    // MEM 组合/寄存器前推
    input  [4:0]  mem_comb_rd,    input mem_comb_reg_write,  input [31:0] mem_comb_result,
    input         mem_comb_is_load,
    input  [4:0]  mem_rd,         input mem_reg_write,       input [31:0] mem_result,
    input         mem_valid,      // mem_result 完成脉冲对齐信号——mem_* 保持不清零，无此限定会前递陈值
    // WB 前推
    input  [4:0]  wb_rd,          input wb_reg_write,        input [31:0] wb_data,
    output reg [1:0]  forward_a,       output reg [1:0]  forward_b,
    output [31:0] ex_forward_a,
    output [31:0] ex_forward_b,
    output [31:0] mem_forward_data
);

    // ---- 并行比较：所有条件同时计算（1-2 级 LUT） ----
    // rs1  （ex_mem/mem_comb 对 load 无效——ALU 结果是地址不是数据）
    wire ex_comb_a  = ex_comb_reg_write  && (ex_comb_rd  != 5'd0) && (ex_comb_rd  == rs1) && !ex_comb_is_load;
    wire ex_mem_a   = ex_mem_reg_write   && (ex_mem_rd   != 5'd0) && (ex_mem_rd   == rs1) && !ex_mem_is_load;
    wire mem_comb_a = mem_comb_reg_write && (mem_comb_rd != 5'd0) && (mem_comb_rd == rs1) && !mem_comb_is_load;
    wire mem_a      = mem_reg_write && mem_valid && (mem_rd != 5'd0) && (mem_rd == rs1);
    wire wb_a       = wb_reg_write       && (wb_rd       != 5'd0) && (wb_rd       == rs1);

    // rs2
    wire ex_comb_b  = ex_comb_reg_write  && (ex_comb_rd  != 5'd0) && (ex_comb_rd  == rs2) && !ex_comb_is_load;
    wire ex_mem_b   = ex_mem_reg_write   && (ex_mem_rd   != 5'd0) && (ex_mem_rd   == rs2) && !ex_mem_is_load;
    wire mem_comb_b = mem_comb_reg_write && (mem_comb_rd != 5'd0) && (mem_comb_rd == rs2) && !mem_comb_is_load;
    wire mem_b      = mem_reg_write && mem_valid && (mem_rd != 5'd0) && (mem_rd == rs2);
    wire wb_b       = wb_reg_write       && (wb_rd       != 5'd0) && (wb_rd       == rs2);

    // ---- 优先编码 ----
    always @(*) begin
        // 优先级 = 流水线年龄序：MEM2 比 WB 年轻，年轻定义优先。
        //   MEM 源已用 mem_valid 限定（mem_result 完成后保持不清零，
        //   无限定会把退休陈值当前递源——这曾被"WB 优先于 MEM"倒置掩盖，
        //   但倒置在 WAW 在飞时前递更老的 WB 值：pcaddu12i r12; ld.w r12; jirl r12
        //   释放拍 MEM=load 新值/WB=旧值 → jirl 跳错，官方 kernel 入口实测）。
        if (ex_comb_a)       forward_a = 2'b10;
        else if (ex_mem_a)   forward_a = 2'b10;
        else if (mem_comb_a) forward_a = 2'b10;
        else if (mem_a)      forward_a = 2'b01;
        else if (wb_a)       forward_a = 2'b11;
        else                 forward_a = 2'b00;

        if (ex_comb_b)       forward_b = 2'b10;
        else if (ex_mem_b)   forward_b = 2'b10;
        else if (mem_comb_b) forward_b = 2'b10;
        else if (mem_b)      forward_b = 2'b01;
        else if (wb_b)       forward_b = 2'b11;
        else                 forward_b = 2'b00;

        // 诊断：前递活跃或有寄存器待写时打印（默认关闭——几乎每拍触发会淹没数据；
        //   需要看前递细节时把 1'b0 改回下面的条件即可）
        if (1'b0 &&
            (ex_comb_reg_write || ex_mem_reg_write || mem_comb_reg_write || mem_reg_write || wb_reg_write ||
             forward_a != 2'b00 || forward_b != 2'b00)) begin
            $display("[FWD_ALL] rs1=%d rs2=%d | exC: rd=%d rw=%b L=%b | exM: rd=%d rw=%b L=%b | mC: rd=%d rw=%b L=%b | m: rd=%d rw=%b | wb: rd=%d rw=%b | => fA=%d fB=%d",
                rs1, rs2,
                ex_comb_rd, ex_comb_reg_write, ex_comb_is_load,
                ex_mem_rd, ex_mem_reg_write, ex_mem_is_load,
                mem_comb_rd, mem_comb_reg_write, mem_comb_is_load,
                mem_rd, mem_reg_write,
                wb_rd, wb_reg_write,
                forward_a, forward_b);
        end
    end

    // ---- 前递数据 ----
    assign ex_forward_a = ex_comb_a ? ex_comb_result :
                          ex_mem_a  ? ex_mem_result   : mem_comb_result;
    assign ex_forward_b = ex_comb_b ? ex_comb_result :
                          ex_mem_b  ? ex_mem_result   : mem_comb_result;
    assign mem_forward_data = mem_result;

endmodule
