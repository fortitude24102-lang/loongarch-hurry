// ============================================================================
// MMU — 内存管理单元
//   DA 模式（CRMD[3]=1）：虚拟地址即物理地址，恒等映射。
//   PG 模式（CRMD[3]=0）：DMW 直接映射窗口匹配。
//     DMW0/DMW1 窗口：VA[31:29] 匹配 VSEG → PA={PSEG, VA[28:0]}
//     缓存属性由 MAT[0] 决定（1=cacheable, 0=uncached）。
//     无 DMW 命中：清除 VA bit31（映射到低 2GB 物理空间）。
// ============================================================================
module mmu(
    input  clk,
    input  reset,
    input  [31:0] vaddr,        // 虚拟地址
    input         is_fetch,     // 取指访问
    input         is_write,     // 写访问
    input  [31:0] crmd,         // CRMD 控制寄存器
    input  [31:0] dmw0,         // DMW0 窗口（CSR 0x180）
    input  [31:0] dmw1,         // DMW1 窗口（CSR 0x181）
    output [31:0] paddr,        // 物理地址
    output        is_cacheable, // 缓存属性：1=allow cache, 0=bypass
    output        page_fault,   // 页错误
    output        tlb_modify,   // TLB 修改例外（页只读但尝试写）
    output        tlb_refill    // TLB 重填例外（TLB 未命中）
);

    // DA 位：CRMD[3]，1=直接地址模式，0=分页模式
    wire da_mode = crmd[3];

    // DMW 窗口匹配（PG 模式有效）：VA[31:29] 匹配 VSEG 且 PLV0 使能
    wire dmw0_hit = !da_mode && (vaddr[31:29] == dmw0[31:29]) && dmw0[0];
    wire dmw1_hit = !da_mode && (vaddr[31:29] == dmw1[31:29]) && dmw1[0];

    // 物理地址：DMW 命中 → {PSEG, VA[28:0]}，无命中 → 清 bit31
    assign paddr = da_mode ? vaddr :
                   dmw0_hit ? {dmw0[27:25], vaddr[28:0]} :
                   dmw1_hit ? {dmw1[27:25], vaddr[28:0]} :
                   {1'b0, vaddr[30:0]};

    // 缓存属性：MAT[0]=1 → cacheable
    wire [1:0] mmu_mat = dmw0_hit ? dmw0[5:4] :
                         dmw1_hit ? dmw1[5:4] : 2'b00;
    assign is_cacheable = (mmu_mat[0] == 1'b1);

    // 例外：当前实现不产生
    assign page_fault  = 1'b0;
    assign tlb_modify  = 1'b0;
    assign tlb_refill  = 1'b0;

    // synthesis translate_off
    wire dmw_trans = dmw0_hit || dmw1_hit;
    reg  mmu_trace_en;
    initial mmu_trace_en = $test$plusargs("cpu5_mmu_trace");
    always @(posedge clk) begin
        if (mmu_trace_en && !da_mode && dmw_trans)
            $display("[MMU c=%0d] PG mode: va=%h pa=%h dmw0_h=%b dmw1_h=%b cache=%b",
                     $time, vaddr, paddr, dmw0_hit, dmw1_hit, is_cacheable);
    end
    // synthesis translate_on

endmodule
