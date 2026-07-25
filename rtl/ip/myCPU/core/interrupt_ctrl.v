// ============================================================================
// interrupt_ctrl — 中断控制器
//   检测外部硬件中断。当任意 hw_int 位为 1 且 CRMD[2] (IE)=1 时产生中断请求。
// ============================================================================
module interrupt_ctrl(
    input  clk,
    input  [7:0]  hw_int,        // 外部硬件中断（8 位）
    input  [31:0] CRMD,          // CRMD 控制寄存器
    output         interrupt_req  // 中断请求
);

    // CRMD[2] = IE（全局中断使能位）
    assign interrupt_req = (|hw_int) && CRMD[2];

endmodule
