// ============================================================================
// exception_unit — 例外汇总单元
//   收集各例外源，输出统一例外信号到流水线控制器和 CSR 文件。
//
//   例外类型及优先级（从高到低）：
//     1. 中断（外部硬件中断）
//     2. INE — 非法指令例外（ecode = 0x0D）
//     3. 系统调用 SYSCALL（ecode = 0x0B）
//     4. 断点 BREAK（ecode = 0x0C）
//     5. TLB 重填（ecode = 0x3F）—— 暂不产生
//     6. 页错误 PAGEFAULT（ecode = 0x3F）—— 暂不产生
//     7. TLB 修改（ecode = 0x3F）—— 暂不产生
// ============================================================================
module exception_unit(
    input  clk,
    // 例外源输入
    input  interrupt,          // 外部中断
    input  syscall,            // SYSCALL 指令
    input  break_inst,         // BREAK 指令
    input  illegal_inst,       // 非法指令 (INE)
    input  page_fault,         // 页错误（来自 MMU）
    input  tlb_modify,         // TLB 修改例外（来自 MMU）
    input  tlb_refill,         // TLB 重填例外（来自 MMU）

    // 例外输出
    output exception,          // 任意例外发生
    output is_interrupt,       // 中断标志
    output is_syscall,         // 系统调用标志
    output is_break,           // 断点标志
    output is_ine,             // 非法指令标志
    output is_page_fault,      // 页错误标志
    output is_tlb_modify,      // TLB 修改标志
    output is_tlb_refill       // TLB 重填标志
);

    // 各例外类型标志
    assign is_interrupt   = interrupt;
    assign is_syscall     = syscall && !interrupt;
    assign is_break       = break_inst && !interrupt && !syscall;
    assign is_ine         = illegal_inst && !interrupt && !syscall && !break_inst;
    assign is_page_fault  = page_fault && !interrupt && !syscall && !break_inst && !illegal_inst;
    assign is_tlb_modify  = tlb_modify && !interrupt && !syscall && !break_inst && !illegal_inst && !page_fault;
    assign is_tlb_refill  = tlb_refill && !interrupt && !syscall && !break_inst && !illegal_inst && !page_fault && !tlb_modify;

    // 任意例外发生
    assign exception = interrupt || syscall || break_inst || illegal_inst ||
                       page_fault || tlb_modify || tlb_refill;


endmodule
