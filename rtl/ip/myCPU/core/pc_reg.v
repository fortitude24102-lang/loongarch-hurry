// ============================================================================
// pc_reg — 程序计数器寄存器
//   复位后从 0x1C000000 开始取指（新版 SDK supervisor 复位入口，VA==PA）。
//   旧 0x80000000 链接的测试程序仍可跑：入口代码位置无关（PC 相对），
//   绝对地址 0x80xxxxxx 经 v2p 别名到同一物理。
//   stall=1 时 PC 不变，但 br_flush 或实际分支跳转必须绕过停顿。
// ============================================================================
module pc_reg(
    input         clk,
    input         reset,
    input         stall,
    input         br_flush,         // 预测错误纠正
    input         branch_taken,     // 实际分支跳转信号
    input  [31:0] next_pc,
    output reg [31:0] pc
);

    always @(posedge clk) begin
        if (reset)
            pc <= 32'h1c000000;
        else if (!stall || br_flush || branch_taken)
            pc <= next_pc;
    end

endmodule
