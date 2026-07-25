// ============================================================================
// alu — 32 位算术逻辑单元
//   运算：ADD/SUB/AND/OR/XOR/NOR/SLT/SLTU/SLL/SRL/SRA
//   移位器采用 5 级对数桶形结构，每级由移位量对应位控制 2:1 MUX，
//   可被 Vivado 精确映射为 5 层 LUT，是主频的关键路径。
//   加法/减法加 DSP48 属性，映射到硬核以提速。
// ============================================================================
module alu(
    input  [31:0] a,
    input  [31:0] b,
    input  [3:0]  op,
    output reg [31:0] y
);

    // ---- 1. 加法 / 减法 ----------------------------------------------------
    (* use_dsp = "yes" *) wire [31:0] result_add;
    (* use_dsp = "yes" *) wire [31:0] result_sub;
    assign result_add = a + b;
    assign result_sub = a - b;

    // ---- 2. 逻辑运算 -------------------------------------------------------
    wire [31:0] result_and = a & b;
    wire [31:0] result_or  = a | b;
    wire [31:0] result_xor = a ^ b;
    wire [31:0] result_nor = ~result_or;

    // ---- 3. 比较运算（利用减法器结果） -------------------------------------
    // SLT (有符号小于) / SLTU (无符号小于)
    wire [31:0] result_slt  = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
    wire [31:0] result_sltu = (a < b)                     ? 32'd1 : 32'd0;

    // ---- 4. 移位运算：5 级对数桶形移位器 -----------------------------------
    // SLL (逻辑左移)
    wire [31:0] sll_1  = b[0] ? {a[30:0], 1'b0}      : a;
    wire [31:0] sll_2  = b[1] ? {sll_1[29:0], 2'b0}  : sll_1;
    wire [31:0] sll_4  = b[2] ? {sll_2[27:0], 4'b0}  : sll_2;
    wire [31:0] sll_8  = b[3] ? {sll_4[23:0], 8'b0}  : sll_4;
    wire [31:0] sll_16 = b[4] ? {sll_8[15:0], 16'b0} : sll_8;

    // SRL (逻辑右移)
    wire [31:0] srl_1  = b[0] ? {1'b0, a[31:1]}       : a;
    wire [31:0] srl_2  = b[1] ? {2'b0, srl_1[31:2]}   : srl_1;
    wire [31:0] srl_4  = b[2] ? {4'b0, srl_2[31:4]}   : srl_2;
    wire [31:0] srl_8  = b[3] ? {8'b0, srl_4[31:8]}   : srl_4;
    wire [31:0] srl_16 = b[4] ? {16'b0, srl_8[31:16]} : srl_8;

    // SRA (算术右移)
    wire [31:0] sra_1  = b[0] ? {{ 1{a[31]}}, a[31:1]}        : a;
    wire [31:0] sra_2  = b[1] ? {{ 2{sra_1[31]}}, sra_1[31:2]} : sra_1;
    wire [31:0] sra_4  = b[2] ? {{ 4{sra_2[31]}}, sra_2[31:4]} : sra_2;
    wire [31:0] sra_8  = b[3] ? {{ 8{sra_4[31]}}, sra_4[31:8]} : sra_4;
    wire [31:0] sra_16 = b[4] ? {{16{sra_8[31]}}, sra_8[31:16]} : sra_8;

    // ---- 5. 结果选择 -------------------------------------------------------
    always @(*) begin
        case (op)
            4'b0000: y = result_add;    // ADD / ADDI / LU12I / 地址计算
            4'b0001: y = result_sub;    // SUB
            4'b0010: y = result_and;    // AND / ANDI
            4'b0011: y = result_or;     // OR  / ORI
            4'b0100: y = result_xor;    // XOR / XORI
            4'b0101: y = result_slt;    // SLT / SLTI
            4'b0110: y = result_sltu;   // SLTU / SLTUI
            4'b0111: y = sll_16;        // SLL.W / SLLI.W
            4'b1000: y = srl_16;        // SRL.W / SRLI.W
            4'b1001: y = sra_16;        // SRA.W / SRAI.W
            4'b1010: y = result_nor;       // NOR
            default: y = 32'd0;
        endcase
    end

endmodule
