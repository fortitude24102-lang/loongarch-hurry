module cpu_core(
    input clk, input reset, input [7:0] hw_int,

    // AXI Master (burst-capable)
    output        axi_arvalid, input        axi_arready,
    output [31:0] axi_araddr,  output [2:0] axi_arprot,
    output [7:0]  axi_arlen,
    input         axi_rvalid,  output       axi_rready,
    input  [31:0] axi_rdata,   input  [1:0] axi_rresp,
    input         axi_rlast,
    output        axi_awvalid, input        axi_awready,
    output [31:0] axi_awaddr,  output [2:0] axi_awprot,
    output [7:0]  axi_awlen,
    output        axi_wvalid,  input        axi_wready,
    output [31:0] axi_wdata,   output [3:0] axi_wstrb,
    output        axi_wlast,
    input         axi_bvalid,  output       axi_bready,
    input  [1:0]  axi_bresp,

    output [15:0] debug_pc,

    // WB 级调试（core_top → debug0_wb_*）
    output        debug_wb_we,
    output [4:0]  debug_wb_rd,
    output [31:0] debug_wb_data,
    output [3:0]  debug_dc_state  // D-cache FSM 状态（诊断用）
);
    wire [31:0] pc, next_pc; wire pc_stall;
    wire if_flush, id_flush, ex_flush, br_misp_flush;
    wire [31:0] if_inst_raw; wire if_allowin, icache_ready, icache_busy;
    wire [31:0] if_pc, id_pc_out, id_inst, id_rs1, id_rs2, id_imm;
    wire [4:0] id_rd, id_rs1_addr, id_rs2_addr; wire [3:0] id_alu_op;
    wire id_alu_src, id_mem_read, id_mem_write, id_mem_to_reg, id_reg_write, id_branch, id_branch_ne, id_branch_eq, id_ertn;
    wire id_branch_lt, id_branch_ge, id_branch_ltu, id_branch_geu, id_syscall, id_break, id_is_jirl, id_is_pcaddu12i;
    wire [1:0] id_mem_width; wire id_mem_signed;
    wire id_csr_rd, id_csr_wr, id_csr_xchg; wire [13:0] id_csr_addr;
    wire load_use_stall; wire id_valid, id_ready; wire id_ex_ready; wire illegal_id;
    wire id_is_mul; wire [1:0] id_mul_op;
    wire id_cpu_cfg; wire [4:0] id_cacop_code; wire id_cacop_valid;
    wire id_valid_gated = id_valid && !(if_flush || id_flush) && !load_use_stall;
    wire [31:0] ex_pc, ex_rs1, ex_rs2, ex_imm; wire [4:0] ex_rd, ex_rs1_addr, ex_rs2_addr; wire [3:0] ex_alu_op;
    wire [31:0] ex1_src1, ex1_src2;
    wire ex_alu_src, ex_mem_read, ex_mem_write, ex_mem_to_reg, ex_reg_write, ex_branch, ex_branch_ne, ex_branch_eq, ex_ertn;
    wire ex_branch_lt, ex_branch_ge, ex_branch_ltu, ex_branch_geu, ex_syscall, ex_break, ex_is_jirl, ex_is_pcaddu12i;
    wire [1:0] ex_mem_width; wire ex_mem_signed;
    wire ex_csr_rd, ex_csr_wr, ex_csr_xchg; wire [13:0] ex_csr_addr;
    wire ex_valid; wire illegal_ex;
    wire ex_is_mul; wire [1:0] ex_mul_op;
    wire ex_cpu_cfg; wire [4:0] ex_cacop_code; wire ex_cacop_valid;
    wire dc_cacop_req, dc_cacop_done, ic_cacop_req, ic_cacop_done;
    reg  cacop_busy;
    // EX1→EX2 冒险检测（全部预声明，避免隐式 1-bit）
    wire ex1_valid, ex1_rw_r, ex1_rw; wire [4:0] ex1_rd_r;
    wire ex1_is_mul_r; wire [1:0] ex1_mul_op_r;
    wire ex1_is_ll_r, ex1_is_sc_r;
    wire ex1_ready;
    wire [31:0] ex1_src1_r, ex1_src2_r, ex1_pc_r, ex1_imm_r;
    wire [3:0]  ex1_aluop_r;
    wire [31:0] ex1_mem_addr;
    wire ex1_alusrc_r, ex1_mr_r, ex1_mw_r, ex1_mtr_r;
    wire ex1_br_r, ex1_brne_r, ex1_breq_r, ex1_brlt_r, ex1_brge_r, ex1_brltu_r, ex1_brgeu_r;
    wire ex1_ertn_r, ex1_sys_r, ex1_brk_r, ex1_msgn_r;
    wire [1:0]  ex1_mwdt_r; wire ex1_csrrd_r, ex1_csrwr_r, ex1_csrx_r;
    wire [13:0] ex1_csraddr_r; wire ex1_jirl_r, ex1_pcad_r, ex1_ill_r;
    wire ex1_cpu_cfg_r; wire [4:0] ex1_cacop_code_r; wire ex1_cacop_valid_r;
    wire ex_stall;
    wire ex2_ld_hazard;   // 预声明：390 行 id_ex_reg 先于 592 行定义使用，隐式 wire 会变 Z→stall=X 冻死流水线（实测）
    wire [31:0] ex_alu_result_raw; wire [31:0] ex_alu_result, ex_rs2_fwd; wire [4:0] ex_rd_out;
    wire [31:0] cpucfg_data;  // csr_file cpucfg 组合读输出
    wire [31:0] dmw0, dmw1;    // CSR DMW 窗口值 → MMU
    wire        mmu_data_cacheable;  // MMU 输出：当前数据访问是否可缓存
    wire ex_mem_read_out, ex_mem_write_out, ex_mem_to_reg_out, ex_reg_write_out;
    wire ex_branch_taken; wire [31:0] ex_branch_target; wire ex_ine;
    wire ex_csr_rd_out, ex_csr_wr_out, ex_csr_xchg_out; wire [13:0] ex_csr_addr_out; wire [31:0] ex_csr_wdata, ex_csr_wmask;
    wire [1:0] ex_mem_width_stage; wire ex_mem_signed_stage;  // ex_stage 直通输出，对齐控制信号
    wire [1:0] forward_a, forward_b; wire [31:0] ex_forward_a, ex_forward_b, mem_forward_data;
    wire ex_mem_valid, ex_mem_ready;
    wire ex_mem_csr_rd, ex_mem_csr_wr, ex_mem_csr_xchg; wire [13:0] ex_mem_csr_addr; wire [31:0] ex_mem_csr_wdata, ex_mem_csr_wmask;
    wire [31:0] mem_addr_in, mem_wdata_in; wire [4:0] mem_rd;
    wire mem_mem_read, mem_mem_write, mem_mem_to_reg, mem_reg_write;
    wire [1:0] mem_mem_width; wire mem_mem_signed;
    wire [31:0] mem_result;
    wire mem_csr_rd, mem_csr_wr, mem_csr_xchg;
    wire [13:0] mem_csr_addr;
    wire [31:0] mem_csr_wdata, mem_csr_wmask;
    wire mem_reg_write_reg;     // mem_stage 同步 reg_write → mem_wb_reg + 前递
    wire [4:0] mem_rd_reg;      // mem_stage 同步 rd 输出——必须显式声明 5-bit，隐式 1-bit 截断致 wb_rd=Z
    wire mem_valid_out;         // mem_stage 同步 valid_out → mem_wb_reg.valid_in
    wire mem1_ready, mem1_valid;
    wire [31:0] mem1_alu, mem1_rs2;
    wire [4:0]  mem1_rd;
    wire mem1_mr, mem1_mw, mem1_mtr, mem1_rw;
    wire [1:0]  mem1_mwdt; wire mem1_msgn;
    wire mem1_uart;
    wire mem1_csr_rd, mem1_csr_wr, mem1_csr_xchg;
    wire [13:0] mem1_csr_addr;
    wire [31:0] mem1_csr_wdata, mem1_csr_wmask;
    reg  [3:0]  mem1_byte_we;
    reg  [31:0] mem1_aligned_wdata;
    // CPUCFG 索引 = rj 寄存器值（src1 前递结果在 EX1），组合驱动 csr_file 查表
    // cpucfg: 覆写 ALU 结果为 csr_file 查表值
    assign ex_alu_result = ex1_cpu_cfg_r ? cpucfg_data : ex_alu_result_raw;
    wire [3:0] dcache_byte_we; wire dcache_ready;
    wire [31:0] dcache_addr, dcache_wdata, dcache_rdata; wire dcache_req; wire mem_ready_out;
    wire [3:0] dcache_dbg_state;   // D-cache FSM 状态诊断
    wire [31:0] wb_data; wire [4:0] wb_rd; wire wb_reg_write, wb_we, wb_mem_to_reg;
    wire wb_csr_rd, wb_csr_wr, wb_csr_xchg; wire [13:0] wb_csr_addr; wire [31:0] wb_csr_wdata, wb_csr_wmask;
    wire [31:0] csr_rdata, wb_mem_result;
    wire [31:0] crmd; wire interrupt; wire [31:0] era, eentry;
    wire icache_axi_req, dcache_axi_req; wire [31:0] icache_axi_addr, dcache_axi_addr;
    wire dcache_axi_we; wire [31:0] dcache_axi_wdata;
    wire [3:0] dcache_axi_wstrb;
    wire sb_axi_req; wire [31:0] sb_axi_addr, sb_axi_wdata;
    wire [3:0] sb_axi_wstrb; wire sb_axi_wr_done;
    wire [7:0]  icache_axi_len;     // icache burst 长度
    wire [7:0]  dcache_axi_len;     // dcache burst 长度
    wire        dcache_axi_wnext;   // arbiter→dcache: 写 burst 下一字
    wire mem_mem_to_reg_out;    // mem→wb 寄存器间控制信号，显式声明
    wire mem_we; assign mem_we = mem_mem_write;  // MMU is_write 端口，EX/MEM 级写信号
    wire id_ex_valid; assign id_ex_valid = ex_valid;  // pipeline_ctrl 用(=id_ex_reg.valid_out)
    wire if1_allowin, if1_valid, if_id_ready;
    wire [31:0] if1_pc;
    wire [31:0] arb_direct_rdata;
    wire arb_direct_ready, arb_direct_rd_ready, arb_direct_wr_ready;
    reg  is_uncond_ex;
    reg  [65:0] mul_p;
    wire mul_busy;
    // ===== 虚→物地址转换 ====
    function [31:0] v2p(input [31:0] a);
        v2p = (a[31:24] == 8'h80) ? {8'h1c, a[23:0]} : a;
    endfunction
    function is_uart_addr(input [31:0] a);
        is_uart_addr = (a[31:20] == 12'h1F0) || (a[31:4] == 28'hBFD003F);
    endfunction
    wire [31:0] dcache_mmu_paddr;
    wire [31:0] dcache_paddr = v2p(dcache_mmu_paddr);
    wire is_uart    = is_uart_addr(dcache_paddr);
    wire is_extram  = (dcache_paddr[31:22] == 10'h071);
    wire is_baseram = (dcache_paddr[31:22] == 10'h070);
    // 4路组相联 32KB write-back + 边界旁路
    // DA 模式：仅 ExtRAM 可缓存；PG 模式：MMU DMW MAT 决定
    wire is_cacheable = crmd[3] ? (is_extram && dcache_paddr < 32'h1C7F0000)
                                : mmu_data_cacheable;
    // Simulation-only A/B switch for stage-2 cache evaluation.  The synthesis
    // build ties this low, so the experiment cannot change the FPGA design.
`ifdef VERILATOR
    reg cpu5_force_dcache_bypass;
    initial cpu5_force_dcache_bypass =
        $test$plusargs("cpu5_dcache_bypass");
`else
    wire cpu5_force_dcache_bypass = 1'b0;
`endif
    wire bypass_dcache = !is_cacheable || cpu5_force_dcache_bypass;
    // EX1 AGU 已算好地址，比 EX2 早一周期检测 UART 读
    wire [31:0] ex1_mmu_paddr;
    wire [31:0] ex1_paddr = v2p(ex1_mmu_paddr);
    wire uart_read_early = ex1_mr_r && ex1_valid && is_uart_addr(ex1_paddr);
    reg  uart_read_pending;
    reg  [31:0] uart_read_addr;   // 锁存 UART 读地址：pending 提前触发时 mem1_alu 还没对齐，
                                  //   必须用检测当拍就正确的 ex1_mem_addr，否则 direct 读会用到旧地址
    always @(posedge clk) begin
        if (reset)              uart_read_pending <= 1'b0;
        else if (uart_read_early) begin
            uart_read_pending <= 1'b1;
            uart_read_addr    <= ex1_mem_addr;
        end
        else if (arb_direct_rd_ready) uart_read_pending <= 1'b0;
    end
    wire direct_req = (dcache_req && bypass_dcache) || uart_read_pending;
    // direct_we 不能用 dcache_byte_we（UART 时被清 0），直接用原始 byte_we
    wire direct_we  = direct_req && mem1_valid && mem1_mw && (|mem1_byte_we);
    // direct 地址：写事务(direct_we)必须用 mem1 对齐的 store 地址——
    //   store 在 MEM1 发写时下一条 UART 轮询 ld.b 已进 EX1 触发 pending，
    //   AW 会被读地址(+5)污染 → 字节写到错误偏移被 accept-ignore 静默吞掉
    //   （16550 冒烟实测：11 字符只有 icache miss 恰好延迟下轮询的 'D''K' 存活）。
    //   仅读事务才用 pending 锁存的提前地址。
    wire [31:0] direct_addr = (uart_read_pending && !direct_we) ? uart_read_addr : dcache_addr;

    wire [31:0] inst_mmu_paddr;
    wire [31:0] icache_axi_paddr = v2p(inst_mmu_paddr);
    mmu u_mmu_inst(
        .clk(clk),
        .reset(reset),
        .vaddr(icache_axi_addr),
        .is_fetch(1'b1),
        .is_write(1'b0),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),  // I-cache 不走 MMU cacheable（始终用 icache）
        .paddr(inst_mmu_paddr)
    );
    wire mmu_page_fault, mmu_tlb_modify, mmu_tlb_refill;
    mmu u_mmu_data(
        .clk(clk),
        .reset(reset),
        .vaddr(dcache_addr),
        .is_fetch(1'b0),
        .is_write(mem_we),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(mmu_data_cacheable),
        .paddr(dcache_mmu_paddr),
        .page_fault(mmu_page_fault),
        .tlb_modify(mmu_tlb_modify),
        .tlb_refill(mmu_tlb_refill)
    );

    // The early UART read path observes EX1, before the MEM1 data address.
    // Translate that address independently so it cannot feed back into
    // bypass_dcache/direct_req.
    mmu u_mmu_ex1_data(
        .clk(clk),
        .reset(reset),
        .vaddr(ex1_mem_addr),
        .is_fetch(1'b0),
        .is_write(1'b0),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),
        .paddr(ex1_mmu_paddr)
    );

    // mem1_reg must capture the UART attribute for its own input address,
    // rather than the previous transaction currently at its output.
    wire [31:0] mem_addr_mmu_paddr;
    wire [31:0] mem_addr_paddr = v2p(mem_addr_mmu_paddr);
    wire mem_addr_is_uart = is_uart_addr(mem_addr_paddr);
    mmu u_mmu_mem1_input(
        .clk(clk),
        .reset(reset),
        .vaddr(mem_addr_in),
        .is_fetch(1'b0),
        .is_write(mem_mem_write),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),
        .paddr(mem_addr_mmu_paddr)
    );

    wire exception;
    exception_unit u_exception(
        .clk(clk),
        .interrupt(interrupt),
        .syscall(ex_syscall),
        .break_inst(ex_break),
        .illegal_inst(ex_ine),
        .page_fault(mmu_page_fault),
        .tlb_modify(mmu_tlb_modify),
        .tlb_refill(mmu_tlb_refill),
        .exception(exception)
    );

    // ===== Gshare 分支预测 =====
    wire        pred_taken;
    wire [31:0] pred_target;
    wire [7:0]  pred_ghr;
    wire        pred_update_en;
    wire [7:0]  pred_update_ghr;
    wire        pred_flush;
    wire [7:0]  pred_flush_ghr;

    // 预测信息流水线：IF1 → IF2 → ID → EX1 → EX2
    wire        if1_pred_taken;   // if1_reg 输出
    wire [7:0]  if1_pred_ghr;
    wire        id_br_pred;       // if_id_reg 输出
    wire        id_is_ll;         // id_stage 输出：LL.W
    wire        id_is_sc;         // id_stage 输出：SC.W
    wire [7:0]  id_br_ghr;
    wire        ex_br_pred;       // id_ex_reg 输出
    wire [7:0]  ex_br_ghr;
    wire        ex_is_ll;         // id_ex_reg 输出
    wire        ex_is_sc;
    wire        ex1_br_pred;      // ex1_reg 输出
    wire        ex1_is_ll;        // ex1_reg 输出
    wire        ex1_is_sc;
    wire [7:0]  ex1_br_ghr;

    gshare_predictor u_bpred (
        .clk(clk), .rst(reset),
        .pc(pc),
        .pred_taken(pred_taken),
        .pred_target(pred_target),
        .pred_ghr(pred_ghr),
        .update_en(pred_update_en),
        .update_pc(ex1_pc_r),
        .update_taken(ex_branch_taken),
        .update_target(ex_branch_target),
        .update_ghr_snap(pred_update_ghr),
        .update_indirect(ex1_jirl_r),
        .update_uncond(is_uncond_ex),
        .flush(pred_flush),
        .flush_ghr(pred_flush_ghr)
    );

    // 预测错误检测（EX2）
    //   无条件跳转永不触发"预测跳转但实际没跳"的错误（misp_flush），
    //   但 BTB 冷启动时仍通过 br_ctrl_flush 正常冲刷→训练 BTB。
    // 使用 ex1_reg 输出（与 EX2 指令对齐），而非 id_ex_reg（EX1，差 1 拍）
    always @(*) begin
        is_uncond_ex = ex1_br_r && !ex1_brne_r && !ex1_breq_r && !ex1_brlt_r &&
                       !ex1_brge_r && !ex1_brltu_r && !ex1_brgeu_r;
    end
    wire branch_resolve_fire =
        ex1_valid && ex1_br_r && !ex_stall && ex_mem_ready;
    reg  br_mispredict;
    always @(*) begin
        br_mispredict =
            ex1_valid && ex1_br_r && ex1_br_pred && !ex_branch_taken;
    end
    assign br_misp_flush = br_mispredict;

    // 分支信号门控：实际跳转但未预测到 → 刷流水线
    wire br_ctrl_flush =
        ex1_valid && ex1_br_r && ex_branch_taken && !ex1_br_pred;

    // 预测更新 + 恢复（EX2）
    assign pred_update_en  = branch_resolve_fire && !exception;
    assign pred_update_ghr = ex1_br_ghr;
    assign pred_flush      = br_mispredict || (ex_flush && ex_branch);
    assign pred_flush_ghr  = {ex1_br_ghr[6:0], ex_branch_taken};

    wire br_flush_any = br_ctrl_flush || br_misp_flush;

    pc_reg u_pc_reg(
        .clk(clk),
        .reset(reset),
        .stall(pc_stall),
        .br_flush(br_flush_any),
        .branch_taken(br_ctrl_flush),
        .next_pc(next_pc),
        .pc(pc)
    );

    // ===== ICache =====
    wire [31:0] icache_inst;
    wire icache_pair_valid;     // icache 输出 inst 与 if1_pc 严格配对有效
    // AXI read/write channels are independent.  A background D-cache store
    // buffer drain must not suppress instruction reads.
    wire icache_bus_grant =
        !direct_req && !(dcache_axi_req && !dcache_axi_we);

    // AXI arbiter → icache
    wire [31:0] arb_ic_rdata;
    wire        arb_ic_valid;

    icache u_icache(
        .clk(clk),
        .reset(reset),
        .req((if_id_ready || !icache_ready) && icache_bus_grant && !br_flush_any),
        .addr(if1_pc),     // if1_pc 与 I-cache 1 周期延迟对齐，复位=0x80000000
        .inst(icache_inst),
        .ready(icache_ready),
        .busy(icache_busy),
        .pair_valid(icache_pair_valid),
        .axi_req(icache_axi_req),
        .axi_addr(icache_axi_addr),
        .axi_len(icache_axi_len),
        .axi_rdata(arb_ic_rdata),
        .axi_valid(arb_ic_valid),
        .axi_ready(arb_ic_valid),
        .cacop_req(ic_cacop_req),
        .cacop_code(ex1_cacop_code_r),
        .cacop_addr(ex1_mem_addr),
        .cacop_done(ic_cacop_done)
    );
    assign if_inst_raw = icache_inst;

    // ===== IF1: 请求锁存 → if1_reg =====
    if1_reg u_if1_reg(
        .clk(clk),
        .reset(reset),
        .valid_i(if1_allowin && icache_bus_grant && !icache_busy),
        // ready_i = "本拍取指对已交付 if_id"。不能用裸 icache_ready：ready 不携带
        //   "属于哪个 pc"，if1 持有未交付的 pc 时（如 store 抢总线压制取指），
        //   裸 ready=1 会让 if1 误判已消费而冲掉 valid → 该 pc 永久丢失
        //   （C3 实测：0x30 的 addi.w r15 被跳过，PC 已前进到 0x34 无人回补）。
        .ready_i(icache_pair_valid && if_id_ready),
        .flush_i(if_flush),
        .stall_i(load_use_stall),
        .pc_i(pc),
        .pred_taken_i(pred_taken),
        .pred_ghr_i(pred_ghr),
        .ready_o(if1_allowin),
        .valid_o(if1_valid),
        .pc_o(if1_pc),
        .pred_taken_o(if1_pred_taken),
        .pred_ghr_o(if1_pred_ghr)
    );

    // ===== IF2: ICache 返回 + if1_reg PC → if_id_reg =====
    wire [31:0] if2_inst_out, if2_pc_out;
    if_stage u_if_stage(
        .icache_inst(if_inst_raw),
        .pc(if1_pc),
        .inst_out(if2_inst_out),
        .pc_out(if2_pc_out)
    );

    // ===== IF2/ID register =====
    if_id_reg u_if_id_reg(
        .clk(clk),
        .reset(reset),
        // valid_i 用 icache_pair_valid（严格配对）而非裸 icache_ready：
        //   ready 是寄存器残留信号、不携带"属于哪个 pc"的信息，两类事故均源于此——
        //   (1) 命中→缺失切换/store 抢总线那拍，ready 残留 1 而 inst 陈旧、if1_pc 已
        //       前进 → 锁存 {新 pc, 旧 inst} = 幽灵指令（同址重复 store 死锁的根因）；
        //   (2) 反向一刀切门控（如 bus_grant）又会扔掉"已完成未消费"的合法取指对
        //       （0x1c 的 or r14 被丢，STORE 数据变 0）。
        //   pair_valid = 正在组合服务当前 pc || (ready && ready_addr==if1_pc)，
        //   两种事故同时杜绝。保留 !icache_busy 与 if1_reg 门控对称。
        // 还必须 if1_valid（架构存活）：分支冲刷清 if1.valid 但 if1_pc 残留旧值；
        //   若冲刷时该 pc 的缺失填充在飞（不可取消），填充完成后 ready_addr 恰与
        //   陈旧 if1_pc 配对 → 已冲刷路径的指令被当有效注入（C3 T20 实测：jirl
        //   影子 0x120 跨行缺失，BAD 在冲刷后被回灌执行）。if1 现经 ready_i 交付制
        //   在缺失期间保持 v=1，仅冲刷/交付后清零，故此门控不会拦截正常交付。
        .valid_i(icache_pair_valid && !icache_busy && if1_valid),
        .ready_i(id_ready),
        .flush_i(if_flush||id_flush),
        .stall_i(load_use_stall),
        .pc_i(if2_pc_out),
        .inst_i(if2_inst_out),
        .br_pred_i(if1_pred_taken),
        .br_ghr_i(if1_pred_ghr),
        .ready_o(if_id_ready),
        .valid_o(id_valid),
        .pc_o(if_pc),
        .inst_o(id_inst),
        .br_pred_o(id_br_pred),
        .br_ghr_o(id_br_ghr)
    );
    assign if_allowin = if1_allowin;  // IF1 控��� PC 停顿（替代旧 if_id_ready）
    assign debug_pc = pc[15:0];
    assign debug_wb_we   = wb_we;
    assign debug_wb_rd   = wb_rd;
    assign debug_wb_data = wb_data;
    assign debug_dc_state = dcache_dbg_state;

    // ===== ID stage =====
    id_stage u_id(
        .clk(clk),
        .reset(reset),
        .valid_in(id_valid_gated),
        .ready_out(id_ready),
        .ready_in(id_ex_ready),
        .pc_in(if_pc),
        .inst_in(id_inst),
        .wb_rd(wb_rd),
        .wb_data(wb_data),
        .wb_we(wb_we),
        .id_ex_mem_read(ex_mem_read),
        .id_ex_rd(ex_rd),
        .id_ex_is_mul(ex_is_mul),
        .ex1_is_mul(ex1_is_mul_r),
        .ex1_valid(ex1_valid),
        .ex1_rd(ex1_rd_r),
        .ex_mem_mem_read(mem_mem_read),   .ex_mem_rd(mem_rd),
        .mem_mem_read(mem1_mr),           .mem_rd(mem1_rd),
        .wb_load(wb_mem_to_reg),
        .stall_out(load_use_stall),
        .pc_out(id_pc_out),
        .rs1_val(id_rs1),
        .rs2_val(id_rs2),
        .imm_out(id_imm),
        .rd_out(id_rd),
        .rs1_addr(id_rs1_addr),
        .rs2_addr(id_rs2_addr),
        .alu_op(id_alu_op),
        .alu_src(id_alu_src),
        .mem_read(id_mem_read),
        .mem_write(id_mem_write),
        .mem_to_reg(id_mem_to_reg),
        .reg_write(id_reg_write),
        .branch(id_branch),
        .branch_ne(id_branch_ne),
        .branch_eq(id_branch_eq),
        .branch_lt(id_branch_lt),
        .branch_ge(id_branch_ge),
        .branch_ltu(id_branch_ltu),
        .branch_geu(id_branch_geu),
        .ertn(id_ertn),
        .syscall(id_syscall),
        .break_inst(id_break),
        .mem_width(id_mem_width),
        .mem_signed(id_mem_signed),
        .csr_rd(id_csr_rd),
        .csr_wr(id_csr_wr),
        .csr_xchg(id_csr_xchg),
        .csr_addr(id_csr_addr),
        .is_jirl(id_is_jirl),
        .is_pcaddu12i(id_is_pcaddu12i),
        .is_ll(id_is_ll),
        .is_sc(id_is_sc),
        .illegal(illegal_id),
        .is_mul(id_is_mul),
        .mul_op(id_mul_op),
        .cpu_cfg(id_cpu_cfg),
        .cacop_code(id_cacop_code),
        .cacop_valid(id_cacop_valid)
    );
    // 总线被数据访问占用时取指无法进行，PC 必须同步停顿
    wire bus_busy = direct_req || dcache_axi_req;

    // ===== ID/EX register =====
    id_ex_reg u_id_ex(
        .clk(clk),
        .reset(reset),
        .valid_in(id_valid_gated),
        .ready_in(ex1_ready),
        .flush(ex_flush || id_flush),   // branch 时 id_flush=1 杀 EX1 投机指令，不伤 MEM
        .stall(ex_stall || ex2_ld_hazard),  // load-use + EX2-load hazard: 消费者留在 EX1
        .stalled_rs1_in(ex1_src1),
        .stalled_rs2_in(ex1_src2),
        .pc_in(id_pc_out),
        .rs1_in(id_rs1),
        .rs2_in(id_rs2),
        .imm_in(id_imm),
        .rd_in(id_rd),
        .rs1_addr_in(id_rs1_addr),
        .rs2_addr_in(id_rs2_addr),
        .alu_op_in(id_alu_op),
        .alu_src_in(id_alu_src),
        .mem_read_in(id_mem_read),
        .mem_write_in(id_mem_write),
        .mem_to_reg_in(id_mem_to_reg),
        .reg_write_in(id_reg_write),
        .branch_in(id_branch),
        .branch_ne_in(id_branch_ne),
        .branch_eq_in(id_branch_eq),
        .branch_lt_in(id_branch_lt),
        .branch_ge_in(id_branch_ge),
        .branch_ltu_in(id_branch_ltu),
        .branch_geu_in(id_branch_geu),
        .ertn_in(id_ertn),
        .syscall_in(id_syscall),
        .break_in(id_break),
        .mem_width_in(id_mem_width),
        .mem_signed_in(id_mem_signed),
        .csr_rd_in(id_csr_rd),
        .csr_wr_in(id_csr_wr),
        .csr_xchg_in(id_csr_xchg),
        .csr_addr_in(id_csr_addr),
        .is_jirl_in(id_is_jirl),
        .is_pcaddu12i_in(id_is_pcaddu12i),
        .br_pred_in(id_br_pred),
        .br_ghr_in(id_br_ghr),
        .is_ll_in(id_is_ll),
        .is_sc_in(id_is_sc),
        .is_mul_in(id_is_mul),
        .mul_op_in(id_mul_op),
        .cacop_code_in(id_cacop_code),
        .cacop_valid_in(id_cacop_valid),
        .cpu_cfg_in(id_cpu_cfg),
        .idle_valid_in(1'b0),
        .ready_out(id_ex_ready),
        .pc_out(ex_pc),
        .rs1_out(ex_rs1),
        .rs2_out(ex_rs2),
        .imm_out(ex_imm),
        .rd_out(ex_rd),
        .rs1_addr_out(ex_rs1_addr),
        .rs2_addr_out(ex_rs2_addr),
        .alu_op_out(ex_alu_op),
        .alu_src_out(ex_alu_src),
        .mem_read_out(ex_mem_read),
        .mem_write_out(ex_mem_write),
        .mem_to_reg_out(ex_mem_to_reg),
        .reg_write_out(ex_reg_write),
        .branch_out(ex_branch),
        .branch_ne_out(ex_branch_ne),
        .branch_eq_out(ex_branch_eq),
        .branch_lt_out(ex_branch_lt),
        .branch_ge_out(ex_branch_ge),
        .branch_ltu_out(ex_branch_ltu),
        .branch_geu_out(ex_branch_geu),
        .ertn_out(ex_ertn),
        .syscall_out(ex_syscall),
        .break_out(ex_break),
        .mem_width_out(ex_mem_width),
        .mem_signed_out(ex_mem_signed),
        .csr_rd_out(ex_csr_rd),
        .csr_wr_out(ex_csr_wr),
        .csr_xchg_out(ex_csr_xchg),
        .csr_addr_out(ex_csr_addr),
        .is_jirl_out(ex_is_jirl),
        .is_pcaddu12i_out(ex_is_pcaddu12i),
        .br_pred_out(ex_br_pred),
        .br_ghr_out(ex_br_ghr),
        .is_ll_out(ex_is_ll),
        .is_sc_out(ex_is_sc),
        .is_mul_out(ex_is_mul),
        .mul_op_out(ex_mul_op),
        .cacop_code_out(ex_cacop_code),
        .cacop_valid_out(ex_cacop_valid),
        .cpu_cfg_out(ex_cpu_cfg),
        .idle_valid_out(),
        .valid_out(ex_valid),
        .illegal_in(illegal_id),
        .illegal_out(illegal_ex)
    );

    // ===== Forwarding (split A/B + EX comb) =====
    forwarding_unit u_fwd(
        .rs1(ex_rs1_addr),
        .rs2(ex_rs2_addr),
        .ex_comb_rd(ex1_rd_r),          // EX2 前递 (ex1_reg.rd)
        .ex_comb_reg_write(ex1_rw_r && ex1_valid),
        .ex_comb_result(ex_alu_result),  // EX2 ALU 组合输出
        .ex_comb_is_load(ex_mem_read_out), // load 的 ALU 结果是地址，不可前递
        .ex_mem_rd(mem_rd),
        .ex_mem_reg_write(mem_reg_write),
        .ex_mem_result(mem_addr_in),
        .ex_mem_is_load(mem_mem_read),   // load 的 ALU 结果是地址，不可前递
        .mem_comb_rd(mem1_rd),
        .mem_comb_reg_write(mem1_rw),
        .mem_comb_result(mem1_alu),
        .mem_comb_is_load(mem1_mr),      // load 的 ALU 结果是地址，不可前递
        .mem_rd(mem_rd_reg),           // mem_stage 同步寄存器输出（与 mem_result 对齐）
        .mem_reg_write(mem_reg_write_reg),
        .mem_result(mem_result),
        .mem_valid(mem_valid_out),     // 完成脉冲——防 mem_result 保持陈值被当前递源
        .wb_rd(wb_rd),
        .wb_reg_write(wb_reg_write),
        .wb_data(wb_data),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .ex_forward_a(ex_forward_a),
        .ex_forward_b(ex_forward_b),
        .mem_forward_data(mem_forward_data)
    );

    // ===== EX1: 前递 MUX (组合逻辑) =====
    //   直接使用前递条件信号，避免跨模块 NBA 时序问题
    wire fwd_ex_comb_a  = ex1_rw_r && ex1_valid && (ex1_rd_r == ex_rs1_addr) && (ex1_rd_r != 5'd0) && !ex_mem_read_out;
    wire fwd_ex_mem_a   = mem_reg_write && (mem_rd == ex_rs1_addr) && (mem_rd != 5'd0) && !mem_mem_read;
    wire fwd_mem_comb_a = mem1_rw && (mem1_rd == ex_rs1_addr) && (mem1_rd != 5'd0) && !mem1_mr;
    wire fwd_mem_a      = mem_reg_write_reg && mem_valid_out && (mem_rd_reg == ex_rs1_addr) && (mem_rd_reg != 5'd0);
    wire fwd_wb_a       = wb_reg_write && (wb_rd == ex_rs1_addr) && (wb_rd != 5'd0);

    wire fwd_ex_comb_b  = ex1_rw_r && ex1_valid && (ex1_rd_r == ex_rs2_addr) && (ex1_rd_r != 5'd0) && !ex_mem_read_out;
    wire fwd_ex_mem_b   = mem_reg_write && (mem_rd == ex_rs2_addr) && (mem_rd != 5'd0) && !mem_mem_read;
    wire fwd_mem_comb_b = mem1_rw && (mem1_rd == ex_rs2_addr) && (mem1_rd != 5'd0) && !mem1_mr;
    wire fwd_mem_b      = mem_reg_write_reg && mem_valid_out && (mem_rd_reg == ex_rs2_addr) && (mem_rd_reg != 5'd0);
    wire fwd_wb_b       = wb_reg_write && (wb_rd == ex_rs2_addr) && (wb_rd != 5'd0);

    // 优先级 = 流水线年龄序：MEM2(mem_*_reg) 一定比 WB 年轻，年轻定义优先。
    //   MEM 源必须用 mem_valid_out 限定——mem_result/mem_rd_reg 完成后保持不清零，
    //   无 valid 限定会把早已退休的陈值当前递源（历史事故 r12 陈值即此因，
    //   当时用"WB 优先于 MEM"倒置补丁掩盖；但倒置在 WAW 在飞时前递更老的 WB 值：
    //   pcaddu12i r12; ld.w r12; jirl r12 —— 释放拍 MEM=load 新值/WB=pcaddu12i 旧值，
    //   WB 优先 → jirl 跳错（官方 kernel 入口 stub 实测）。valid 限定 + 自然序两者兼治。）
    // 转发 mux：直接使用 forwarding_unit 的 if/else 输出（X-safe），
    //   而非本地嵌套三元（X 在三元中会传播，在 if/else 中走 else 分支→00）
    assign ex1_src1 = (forward_a == 2'b10) ? ex_forward_a :
                      (forward_a == 2'b01) ? mem_forward_data :
                      (forward_a == 2'b11) ? wb_data : ex_rs1;
    assign ex1_src2 = (forward_b == 2'b10) ? ex_forward_b :
                      (forward_b == 2'b01) ? mem_forward_data :
                      (forward_b == 2'b11) ? wb_data : ex_rs2;

    // ===== EX1→EX2 register =====
    ex1_reg u_ex1_reg(
        .clk(clk), .reset(reset),
        // 分支 flush(id_flush) 不得在 EX2 指令滞留时清除它：
        //   taken 分支因下游反压(前一条访存占 MEM)滞留在 EX2 的同拍，自己触发的
        //   id_flush 会把滞留的分支连 link 写一起蒸发（官方 kernel .OP_D 实测：
        //   jirl/bl 前一条是访存时 ra link 丢失 → READSERIAL 用两代前的 ra 返回）。
        //   滞留拍跳过无害：ex1_reg 未 ready 收不进投机指令，id_ex 已被 flush 成气泡。
        //   异常 ex_flush 语义不变。
        .flush(ex_flush || (id_flush && !(ex1_valid && !ex_mem_ready))),
        .stall(ex_stall),
        .valid_in(ex_valid && !ex2_ld_hazard), .ready_in(ex_mem_ready),
        .ready_out(ex1_ready),
        .src1_in(ex1_src1), .src2_in(ex1_src2),
        .pc_in(ex_pc), .imm_in(ex_imm), .rd_in(ex_rd),
        .alu_op_in(ex_alu_op), .alu_src_in(ex_alu_src),
        .mem_read_in(ex_mem_read), .mem_write_in(ex_mem_write),
        .mem_to_reg_in(ex_mem_to_reg), .reg_write_in(ex_reg_write),
        .branch_in(ex_branch), .branch_ne_in(ex_branch_ne),
        .branch_eq_in(ex_branch_eq),
        .branch_lt_in(ex_branch_lt), .branch_ge_in(ex_branch_ge),
        .branch_ltu_in(ex_branch_ltu), .branch_geu_in(ex_branch_geu),
        .ertn_in(ex_ertn), .syscall_in(ex_syscall), .break_in(ex_break),
        .mem_width_in(ex_mem_width), .mem_signed_in(ex_mem_signed),
        .csr_rd_in(ex_csr_rd), .csr_wr_in(ex_csr_wr), .csr_xchg_in(ex_csr_xchg),
        .csr_addr_in(ex_csr_addr), .is_jirl_in(ex_is_jirl),
        .is_pcaddu12i_in(ex_is_pcaddu12i), .illegal_in(illegal_ex),
        .br_pred_in(ex_br_pred), .br_ghr_in(ex_br_ghr),
        .is_ll_in(ex_is_ll), .is_sc_in(ex_is_sc),
        .is_mul_in(ex_is_mul), .mul_op_in(ex_mul_op),
        .cacop_code_in(ex_cacop_code), .cacop_valid_in(ex_cacop_valid),
        .cpu_cfg_in(ex_cpu_cfg), .idle_valid_in(1'b0),
        .valid_out(ex1_valid), .src1_out(ex1_src1_r), .src2_out(ex1_src2_r),
        .load_addr_out(ex1_mem_addr),
        .pc_out(ex1_pc_r), .imm_out(ex1_imm_r), .rd_out(ex1_rd_r),
        .alu_op_out(ex1_aluop_r), .alu_src_out(ex1_alusrc_r),
        .mem_read_out(ex1_mr_r), .mem_write_out(ex1_mw_r),
        .mem_to_reg_out(ex1_mtr_r), .reg_write_out(ex1_rw_r),
        .branch_out(ex1_br_r), .branch_ne_out(ex1_brne_r), .branch_eq_out(ex1_breq_r),
        .branch_lt_out(ex1_brlt_r), .branch_ge_out(ex1_brge_r),
        .branch_ltu_out(ex1_brltu_r), .branch_geu_out(ex1_brgeu_r),
        .ertn_out(ex1_ertn_r), .syscall_out(ex1_sys_r), .break_out(ex1_brk_r),
        .mem_width_out(ex1_mwdt_r), .mem_signed_out(ex1_msgn_r),
        .csr_rd_out(ex1_csrrd_r), .csr_wr_out(ex1_csrwr_r),
        .csr_xchg_out(ex1_csrx_r), .csr_addr_out(ex1_csraddr_r),
        .is_jirl_out(ex1_jirl_r), .is_pcaddu12i_out(ex1_pcad_r),
        .illegal_out(ex1_ill_r),
        .br_pred_out(ex1_br_pred), .br_ghr_out(ex1_br_ghr),
        .is_ll_out(ex1_is_ll), .is_sc_out(ex1_is_sc),
        .is_mul_out(ex1_is_mul_r), .mul_op_out(ex1_mul_op_r),
        .cacop_code_out(ex1_cacop_code_r), .cacop_valid_out(ex1_cacop_valid_r),
        .cpu_cfg_out(ex1_cpu_cfg_r), .idle_valid_out()
    );

    // EX2-load 冒险：load 在 EX2、消费者在 EX1 —— LU 只查 EX1、ex_stall 只查 EXMEM/MEM1，
    //   这正是保护空窗（fwd_ex_comb 用 !ex_mem_read_out 排除了 EX2 load 前递，却无配套 stall）。
    //   icache 行命中的紧凑流水会踩中：pcaddu12i;ld.w;jirl 的 jirl 带 pcaddu12i 旧值跑掉
    //   （官方 kernel .OP_D 实测：jirl 比 load 的 AR 早 2 拍 → 跳进 GOT 数据区 → 非法指令重启）。
    //   不能加进 ex_stall（会冻结 ex1_reg 把 load 自己冻死在 EX2 → 死锁）：
    //   只扣 id_ex（消费者留在 EX1）+ ex1_reg 灌气泡，load 正常流出 EX2，
    //   下一拍 load 进 EXMEM 由 ex_stall 现有项接管。
    assign ex2_ld_hazard = ex_valid && ex1_valid && ex1_mr_r && (ex1_rd_r != 5'd0) &&
                           (ex1_rd_r == ex_rs1_addr || ex1_rd_r == ex_rs2_addr);

    // ===== cacop: 缓存操作请求生成 + 忙标志 =====
    //    cacop_code[2:0]: 0=I-cache, 1=D-cache
    //    边沿检测：只在 cacop 指令首次进入 EX1 时发射一次（0→1 跳变）。
    //    避免电平敏感方案下同一指令在 EX1 停留期间反复重发。
    wire any_cacop_done = dc_cacop_done || ic_cacop_done;
    reg  ex1_cacop_valid_d1;
    always @(posedge clk) begin
        if (reset) ex1_cacop_valid_d1 <= 1'b0;
        else ex1_cacop_valid_d1 <= ex1_cacop_valid_r;
    end
    wire cacop_fire = ex1_cacop_valid_r && !ex1_cacop_valid_d1 && ex1_valid && !cacop_busy;
    assign dc_cacop_req = cacop_fire && (ex1_cacop_code_r[2:0] == 3'b001);
    assign ic_cacop_req = cacop_fire && (ex1_cacop_code_r[2:0] == 3'b000);

    always @(posedge clk) begin
        if (reset) cacop_busy <= 1'b0;
        else if (any_cacop_done) cacop_busy <= 1'b0;   // done 优先
        else if (cacop_fire) cacop_busy <= 1'b1;
    end

    // synthesis translate_off
    reg [31:0] last_cacop_pc;
    reg        core_trace_en;
    initial begin
        last_cacop_pc = 32'b0;
        core_trace_en = $test$plusargs("cpu5_core_trace");
    end
    always @(posedge clk) if (cacop_fire) last_cacop_pc <= ex1_pc_r;
    // 检测是否长时间没有 cacop 且没有 AR 活动——打印 PC
    reg [31:0] stall_cnt;
    always @(posedge clk) begin
        if (reset) stall_cnt <= 0;
        else if (cacop_fire || (axi_arvalid && axi_arready)) stall_cnt <= 0;
        else if (stall_cnt < 32'hFFFFFFFF) stall_cnt <= stall_cnt + 1;
    end
    always @(posedge clk) begin
        if (core_trace_en && stall_cnt == 32'd5000)
            $display("[STUCK c=%0d] stuck, last_cacop_pc=%h DA=%b PG=%b cur_pc=%h | ic_bsy=%b ic_rdy=%b dc_st=%d cacop_bsy=%b ex_stl=%b arv=%b arr=%b",
                     $time, last_cacop_pc, crmd[3], crmd[4], pc,
                     icache_busy, icache_ready, dcache_dbg_state, cacop_busy, ex_stall,
                     axi_arvalid, axi_arready);
        // 每 10000 拍印一次当前 PC
        if (core_trace_en && stall_cnt > 32'd5000 && stall_cnt[13:0] == 14'h0)
            $display("[TRACE c=%0d] cur_pc=%h ic_bsy=%b dc_st=%d ex_stl=%b arv=%b arr=%b",
                     $time, pc, icache_busy, dcache_dbg_state, ex_stall,
                     axi_arvalid, axi_arready);
    end
    // synthesis translate_on

    assign ex_stall = mul_busy || cacop_busy || (ex_valid && (
        // EX/MEM 有 load → 停顿（load 数据在 MEM 才有）
        (mem_mem_read     && mem_rd    != 0 && (mem_rd    == ex_rs1_addr || mem_rd    == ex_rs2_addr)) ||
        // MEM1 有 load → 停顿
        (mem1_mr          && mem1_rd   != 0 && (mem1_rd   == ex_rs1_addr || mem1_rd   == ex_rs2_addr))
        // 注意：不要在此加 MEM2(mem_reg_write_reg) 匹配项。
        //   该情形是"非 load 的 RAW 相关"，MEM2 结果已可前递（fwd_mem_a/fwd_wb_a 覆盖），
        //   本就不应阻塞。若加上，会与"ex_stall 冻结 ex1_reg、但 ex_mem 不冻结"配合，
        //   把被冻结的 EX2 指令每拍重复灌进 MEM/WB，令 mem_rd_reg 永久锁定 → 死锁。
    ));

    // ===== EX2: ALU + 分支 =====
    ex_stage u_ex(
        .src1_in(ex1_src1_r), .src2_in(ex1_src2_r),
        .load_addr_in(ex1_mem_addr),
        .pc_in(ex1_pc_r), .imm_in(ex1_imm_r), .rd_in(ex1_rd_r),
        .alu_op_in(ex1_aluop_r), .alu_src_in(ex1_alusrc_r),
        .mem_read_in(ex1_mr_r), .mem_write_in(ex1_mw_r),
        .mem_to_reg_in(ex1_mtr_r), .reg_write_in(ex1_rw_r),
        .branch_in(ex1_br_r), .branch_ne_in(ex1_brne_r),
        .branch_eq_in(ex1_breq_r),
        .branch_lt_in(ex1_brlt_r), .branch_ge_in(ex1_brge_r),
        .branch_ltu_in(ex1_brltu_r), .branch_geu_in(ex1_brgeu_r),
        .ertn_in(ex1_ertn_r), .syscall_in(ex1_sys_r), .break_in(ex1_brk_r),
        .mem_width_in(ex1_mwdt_r), .mem_signed_in(ex1_msgn_r),
        .csr_rd_in(ex1_csrrd_r), .csr_wr_in(ex1_csrwr_r),
        .csr_xchg_in(ex1_csrx_r), .csr_addr_in(ex1_csraddr_r),
        .is_jirl_in(ex1_jirl_r), .is_pcaddu12i_in(ex1_pcad_r),
        .illegal_in(ex1_ill_r),
        .ex1_valid(ex1_valid),
        .is_mul_in(ex1_is_mul_r), .mul_op_in(ex1_mul_op_r),
        .mul_p_low_in(mul_p[31:0]), .mul_p_high_in(mul_p[63:32]),
        .alu_result(ex_alu_result_raw), .rs2_forwarded(ex_rs2_fwd),
        .rd_out(ex_rd_out), .mem_read_out(ex_mem_read_out),
        .mem_write_out(ex_mem_write_out), .mem_to_reg_out(ex_mem_to_reg_out),
        .reg_write_out(ex_reg_write_out),
        .branch_taken(ex_branch_taken), .branch_target(ex_branch_target),
        .csr_rd_out(ex_csr_rd_out), .csr_wr_out(ex_csr_wr_out),
        .csr_xchg_out(ex_csr_xchg_out), .csr_addr_out(ex_csr_addr_out),
        .csr_wdata_out(ex_csr_wdata), .csr_wmask_out(ex_csr_wmask),
        .ine_out(ex_ine),
        .mem_width_out(ex_mem_width_stage), .mem_signed_out(ex_mem_signed_stage)
    );

    // ===== 乘法器（inline 行为级，1 级流水） =====
    //   综合时替换为 mult_gen_0 IP 并将 mul_busy/cnt 阈值 +1。
    wire       mul_signed = (ex1_mul_op_r != 2'b10);
    wire [32:0] mul_a = {mul_signed & ex1_src1_r[31], ex1_src1_r};
    wire [32:0] mul_b = {mul_signed & ex1_src2_r[31], ex1_src2_r};
    wire       mul_in_ex2 = ex1_is_mul_r && ex1_valid;
    reg  [1:0] mul_cnt;
    // synthesis translate_off
    reg  [4:0] mul_dbg_cnt;
    initial mul_dbg_cnt = 0;
    // synthesis translate_on

    // EX2 本拍捕获新指令（与 u_ex1_reg 捕获条件同源：ready_out && valid_in）。
    //   mul→mul 背靠背无缝交接时 mul_in_ex2 电平不落，旧 FSM（只看 !mul_in_ex2）不重启：
    //   cnt 卡在 2、mul_ce=0（新操作数没锁进乘法器）、mul_busy=0（当拍被判完成）→
    //   第二条 mul 偷走前一条的 mul_p（MATRIX 4 路展开 lane3 全错 2304 词实测：
    //   前 3 条 mul 之间有 load 排空/取指气泡幸免，唯 mul3→mul4 在 EX1 排队无缝进入）。
    wire ex2_capture = ex1_ready && ex_valid && !ex2_ld_hazard;

    always @(posedge clk) begin
        if (reset)                mul_cnt <= 2'd0;
        else if (ex2_capture)     mul_cnt <= 2'd0;   // 新指令进入 EX2：重启（含 mul→mul 背靠背）
        else if (!mul_in_ex2)     mul_cnt <= 2'd0;
        else if (mul_cnt < 2'd2)  mul_cnt <= mul_cnt + 2'd1;
    end

    wire mul_ce   = mul_in_ex2 && (mul_cnt < 2'd2);
    assign mul_busy = mul_in_ex2 && (mul_cnt < 2'd2);

    // Inline 1-stage registered signed multiplier (cnt=0 latch inputs, cnt=1 result ready)
    reg [32:0] mul_a1, mul_b1;
    always @(posedge clk) begin
        if (reset) begin
            mul_a1 <= 33'd0;
            mul_b1 <= 33'd0;
            mul_p  <= 66'd0;
        end else if (mul_ce) begin
            mul_a1 <= mul_a;
            mul_b1 <= mul_b;
            // 用组合输入 mul_a/mul_b 而非锁存值 mul_a1/mul_b1：
            //   NBA 在本次 always 中 mul_a1 还是旧值，$signed(mul_a1)*$signed(mul_b1)
            //   算的是上一拍的乘积 → 第一拍 mul_p 是陈值（MATRIX dcache 数据错误根因）。
            mul_p  <= $signed(mul_a) * $signed(mul_b);
            // synthesis translate_off
            if (core_trace_en && mul_dbg_cnt < 5'd16) begin
                $display("[MUL c=%0t] pc=%08h cnt=%0d a=%08h b=%08h p=%08h",
                         $time, ex1_pc_r, mul_cnt, mul_a[31:0], mul_b[31:0],
                         $signed(mul_a) * $signed(mul_b));
                mul_dbg_cnt <= mul_dbg_cnt + 5'd1;
            end
            // synthesis translate_on
        end
    end

    // synthesis translate_off
    // C[0..7] store probe: track first 8 stores to C matrix
    reg [2:0] cst_dbg_cnt;
    initial cst_dbg_cnt = 0;
    always @(posedge clk) begin
        if (core_trace_en && !reset && mem1_valid && mem1_mw &&
            dcache_ready && cst_dbg_cnt < 4 &&
            dcache_addr >= 32'h1C420000 && dcache_addr < 32'h1C420020) begin
            $display("[C-ST c=%0t] addr=%08h wdata=%08h be=%b",
                     $time, dcache_addr, dcache_wdata, mem1_byte_we);
            cst_dbg_cnt = cst_dbg_cnt + 3'd1;
        end
    end
    // synthesis translate_on

    // Advance PC exactly when IF1 accepts a new fetch PC.  All downstream
    // backpressure already reaches if1_allowin through the ready chain.
    // Stalling PC again on a later data-bus transaction can otherwise replay
    // an instruction that IF1/ID have already consumed.
    wire if1_pc_accept = if1_allowin && icache_bus_grant && !icache_busy;
    assign pc_stall = !if1_pc_accept;

    // ===== EX2/MEM register =====
    //   ex_stall 冻结 ex1_reg（EX2 保持），但本级 stall=1'b0 会继续前进。
    //   若不 gate，被冻结的 EX2 指令会每拍重复灌入 MEM/WB（寄存器重复写、
    //   store 重复执行，且令 mem_rd_reg 永久锁定）。停顿时插入气泡即可。
    ex_mem_reg u_ex_mem(
        .clk(clk),
        .reset(reset),
        .valid_in(ex1_valid && !ex_stall),
        .ready_in(mem1_ready),      // 反压来自 mem1_reg（而非 MEM2）
        .flush(ex_flush),
        .stall(1'b0),
        .alu_result_in(ex_alu_result),
        .rs2_in(ex_rs2_fwd),
        .rd_in(ex_rd_out),
        .mem_read_in(ex_mem_read_out),
        .mem_write_in(ex_mem_write_out),
        .mem_to_reg_in(ex_mem_to_reg_out),
        .reg_write_in(ex_reg_write_out),
        .mem_width_in(ex_mem_width_stage),   // 来自 ex_stage，与其他控制信号对齐
        .mem_signed_in(ex_mem_signed_stage), // 来自 ex_stage
        .csr_rd_in(ex_csr_rd_out),
        .csr_wr_in(ex_csr_wr_out),
        .csr_xchg_in(ex_csr_xchg_out),
        .csr_addr_in(ex_csr_addr_out),
        .csr_wdata_in(ex_csr_wdata),
        .csr_wmask_in(ex_csr_wmask),
        .ready_out(ex_mem_ready),
        .csr_rd_out(ex_mem_csr_rd),
        .csr_wr_out(ex_mem_csr_wr),
        .csr_xchg_out(ex_mem_csr_xchg),
        .csr_addr_out(ex_mem_csr_addr),
        .csr_wdata_out(ex_mem_csr_wdata),
        .csr_wmask_out(ex_mem_csr_wmask),
        .alu_result_out(mem_addr_in),
        .rs2_out(mem_wdata_in),
        .rd_out(mem_rd),
        .mem_read_out(mem_mem_read),
        .mem_write_out(mem_mem_write),
        .mem_to_reg_out(mem_mem_to_reg),
        .reg_write_out(mem_reg_write),
        .mem_width_out(mem_mem_width),
        .mem_signed_out(mem_mem_signed),
        .valid_out(ex_mem_valid)
    );

    // ===== MEM1: dcache 请求生成 (组合逻辑，来自 mem1_reg 以保持请求稳定) =====
    always @(*) begin
        case (mem1_mwdt)
            2'b00: begin  // ST.B
                case (mem1_alu[1:0])
                    2'b00: mem1_byte_we = 4'b0001;
                    2'b01: mem1_byte_we = 4'b0010;
                    2'b10: mem1_byte_we = 4'b0100;
                    2'b11: mem1_byte_we = 4'b1000;
                    default: mem1_byte_we = 4'b0001;
                endcase
                case (mem1_alu[1:0])
                    2'b00: mem1_aligned_wdata = {24'b0, mem1_rs2[7:0]};
                    2'b01: mem1_aligned_wdata = {16'b0, mem1_rs2[7:0], 8'b0};
                    2'b10: mem1_aligned_wdata = {8'b0, mem1_rs2[7:0], 16'b0};
                    2'b11: mem1_aligned_wdata = {mem1_rs2[7:0], 24'b0};
                    default: mem1_aligned_wdata = {24'b0, mem1_rs2[7:0]};
                endcase
            end
            2'b01: begin  // ST.H
                if (mem1_alu[1]) begin
                    mem1_byte_we = 4'b1100;
                    mem1_aligned_wdata = {mem1_rs2[15:0], 16'b0};
                end else begin
                    mem1_byte_we = 4'b0011;
                    mem1_aligned_wdata = {16'b0, mem1_rs2[15:0]};
                end
            end
            default: begin  // ST.W
                mem1_byte_we = 4'b1111;
                mem1_aligned_wdata = mem1_rs2;
            end
        endcase
    end

    // dcache 请求从 mem1_reg 输出保持，对齐 MEM1→MEM2 两周期访问
    wire mem1_dcache_req = mem1_valid && (mem1_mr || mem1_mw);
    assign dcache_req   = mem1_dcache_req;
    assign dcache_addr  = mem1_alu;
    assign dcache_wdata = mem1_aligned_wdata;
    assign dcache_byte_we = mem1_valid && mem1_mw && !bypass_dcache ? mem1_byte_we : 4'b0000;

    // ===== MEM1→MEM2 register =====
    mem1_reg u_mem1_reg(
        .clk(clk), .reset(reset),
        .valid_in(ex_mem_valid),
        .ready_in(mem_ready_out),       // MEM2 ready
        .flush(ex_flush),
        .stall(1'b0),
        .alu_result_in(mem_addr_in), .rs2_in(mem_wdata_in), .rd_in(mem_rd),
        .mem_read_in(mem_mem_read), .mem_write_in(mem_mem_write),
        .mem_to_reg_in(mem_mem_to_reg), .reg_write_in(mem_reg_write),
        .mem_width_in(mem_mem_width), .mem_signed_in(mem_mem_signed),
        .is_uart_in(mem_addr_is_uart),
        .csr_rd_in(ex_mem_csr_rd), .csr_wr_in(ex_mem_csr_wr),
        .csr_xchg_in(ex_mem_csr_xchg), .csr_addr_in(ex_mem_csr_addr),
        .csr_wdata_in(ex_mem_csr_wdata), .csr_wmask_in(ex_mem_csr_wmask),
        .ready_out(mem1_ready),
        .valid_out(mem1_valid), .alu_result_out(mem1_alu), .rs2_out(mem1_rs2),
        .rd_out(mem1_rd), .mem_read_out(mem1_mr), .mem_write_out(mem1_mw),
        .mem_to_reg_out(mem1_mtr), .reg_write_out(mem1_rw),
        .mem_width_out(mem1_mwdt), .mem_signed_out(mem1_msgn),
        .is_uart_out(mem1_uart),
        .csr_rd_out(mem1_csr_rd), .csr_wr_out(mem1_csr_wr),
        .csr_xchg_out(mem1_csr_xchg), .csr_addr_out(mem1_csr_addr),
        .csr_wdata_out(mem1_csr_wdata), .csr_wmask_out(mem1_csr_wmask)
    );

    // ===== MEM2: dcache 数据接收 + 对齐 =====
    mem_stage u_mem(
        .clk(clk), .reset(reset),
        .valid_in(mem1_valid),
        .ready_out(mem_ready_out),
        .alu_result_in(mem1_alu),
        .rd_in(mem1_rd),
        .mem_read_in(mem1_mr), .mem_write_in(mem1_mw),
        .mem_to_reg_in(mem1_mtr), .reg_write_in(mem1_rw),
        .mem_width_in(mem1_mwdt), .mem_signed_in(mem1_msgn),
        .is_uart(mem1_uart),
        .csr_rd_in(mem1_csr_rd), .csr_wr_in(mem1_csr_wr),
        .csr_xchg_in(mem1_csr_xchg), .csr_addr_in(mem1_csr_addr),
        .csr_wdata_in(mem1_csr_wdata), .csr_wmask_in(mem1_csr_wmask),
        .dcache_rdata(dcache_rdata), .dcache_ready(dcache_ready),
        .mem_result(mem_result),
        .mem_rd_out(mem_rd_reg),
        .mem_reg_write_out(mem_reg_write_reg),
        .mem_to_reg_out(mem_mem_to_reg_out),
        .valid_out(mem_valid_out),
        .csr_rd_out(mem_csr_rd), .csr_wr_out(mem_csr_wr),
        .csr_xchg_out(mem_csr_xchg), .csr_addr_out(mem_csr_addr),
        .csr_wdata_out(mem_csr_wdata), .csr_wmask_out(mem_csr_wmask)
    );

    // ===== DCache =====
    wire [31:0] dcache_rdata_int;
    wire dcache_ready_int;

    // AXI arbiter → dcache
    wire [31:0] arb_dc_rdata;
    wire        arb_dc_valid;
    wire        arb_dc_wr_done;

    dcache u_dcache(
        .clk(clk),
        .reset(reset),
        .req(dcache_req&&!bypass_dcache),
        .addr(dcache_addr),
        .wdata(dcache_wdata),
        .we(mem1_mw&&!bypass_dcache),
        .rdata(dcache_rdata_int),
        .ready(dcache_ready_int),
        .axi_req(dcache_axi_req),
        .axi_addr(dcache_axi_addr),
        .axi_wdata(dcache_axi_wdata),
        .axi_wstrb(dcache_axi_wstrb),
        .axi_we(dcache_axi_we),
        .sb_axi_req(sb_axi_req),
        .sb_axi_addr(sb_axi_addr),
        .sb_axi_wdata(sb_axi_wdata),
        .sb_axi_wstrb(sb_axi_wstrb),
        .sb_wr_done(sb_axi_wr_done),
        .axi_len(dcache_axi_len),
        .axi_rdata(arb_dc_rdata),
        .axi_valid(arb_dc_valid),
        .axi_ready(arb_dc_valid),
        .wr_done(arb_dc_wr_done),
        .axi_wnext(dcache_axi_wnext),
        .byte_we(dcache_byte_we),
        .dbg_state(dcache_dbg_state),
        .store_buffer_full(),
        .store_buffer_enqueue(),
        .store_buffer_forward(),
        .store_buffer_count(),
        .cacop_req(dc_cacop_req),
        .cacop_code(ex1_cacop_code_r),
        .cacop_addr(ex1_mem_addr),
        .cacop_done(dc_cacop_done)
    );

    // AXI arbiter → direct bypass
    // Bug#8: 官方 axi2apb 按 wstrb 逐字节连写——恒 1111 会把 st.b 溢写到邻寄存器
    //   （LCR 被清→DLAB 早失效→DLL 落进 THR→除数 0 发送器僵死→LSR 恒 0 死轮询，实测）。
    //   mem1_byte_we 本就是按宽度/地址算好的 lane 选通，直接用。
    wire [3:0]  direct_wstrb = direct_we ? mem1_byte_we : 4'b0000;

    assign dcache_rdata = bypass_dcache ? arb_direct_rdata : dcache_rdata_int;
    // 读完成用读数据有效、写完成用写响应，按当前 mem 操作(mem1_mr/mem1_mw)选择——
    //   否则 direct 写的 B 响应会把紧跟的 direct 读误判为完成、捕获旧数据。
    assign dcache_ready = bypass_dcache ? (mem1_mw ? arb_direct_wr_ready : arb_direct_rd_ready)
                                        : dcache_ready_int;

    // ===== AXI 地址虚→物转换（总线边界）=====
    // Each request source is translated from its own stable address.  In
    // particular, cacheability no longer depends on the arbiter selection,
    // which removes the direct_req <-> MMU combinational loop.
    wire [31:0] dcache_axi_mmu_paddr;
    wire [31:0] sb_axi_mmu_paddr;
    wire [31:0] direct_mmu_paddr;
    wire [31:0] dcache_axi_paddr = v2p(dcache_axi_mmu_paddr);
    wire [31:0] sb_axi_paddr = v2p(sb_axi_mmu_paddr);
    wire [31:0] direct_paddr = v2p(direct_mmu_paddr);

    mmu u_mmu_dcache_axi(
        .clk(clk),
        .reset(reset),
        .vaddr(dcache_axi_addr),
        .is_fetch(1'b0),
        .is_write(dcache_axi_we),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),
        .paddr(dcache_axi_mmu_paddr)
    );

    mmu u_mmu_direct_axi(
        .clk(clk),
        .reset(reset),
        .vaddr(direct_addr),
        .is_fetch(1'b0),
        .is_write(direct_we),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),
        .paddr(direct_mmu_paddr)
    );

    mmu u_mmu_dcache_store_buffer(
        .clk(clk),
        .reset(reset),
        .vaddr(sb_axi_addr),
        .is_fetch(1'b0),
        .is_write(1'b1),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .is_cacheable(),
        .paddr(sb_axi_mmu_paddr)
    );

    // ===== AXI Arbiter =====
    axi_arbiter u_axi_arbiter(
        .clk(clk),
        .reset(reset),
        // I-cache
        .ic_axi_req(icache_axi_req),
        .ic_axi_addr(icache_axi_paddr),
        .ic_axi_len(icache_axi_len),
        .ic_axi_rdata(arb_ic_rdata),
        .ic_axi_valid(arb_ic_valid),
        // D-cache
        .dc_axi_req(dcache_axi_req),
        .dc_axi_addr(dcache_axi_paddr),
        .dc_axi_we(dcache_axi_we),
        .dc_axi_wdata(dcache_axi_wdata),
        .dc_axi_wstrb(dcache_axi_wstrb),
        .dc_axi_len(dcache_axi_len),
        .dc_axi_rdata(arb_dc_rdata),
        .dc_axi_valid(arb_dc_valid),
        .dc_axi_wr_done(arb_dc_wr_done),
        .dc_axi_wnext(dcache_axi_wnext),
        .sb_axi_req(sb_axi_req),
        .sb_axi_addr(sb_axi_paddr),
        .sb_axi_wdata(sb_axi_wdata),
        .sb_axi_wstrb(sb_axi_wstrb),
        .sb_axi_wr_done(sb_axi_wr_done),
        // Direct bypass
        .direct_req(direct_req),
        .direct_addr(direct_paddr),
        .direct_we(direct_we),
        .direct_wdata(dcache_wdata),
        .direct_wstrb(direct_wstrb),
        .direct_rdata(arb_direct_rdata),
        .direct_ready(arb_direct_ready),
        .direct_rd_ready(arb_direct_rd_ready),
        .direct_wr_ready(arb_direct_wr_ready),
        // AXI master ports
        .axi_arvalid(axi_arvalid), .axi_arready(axi_arready),
        .axi_araddr(axi_araddr),   .axi_arprot(axi_arprot),
        .axi_arlen(axi_arlen),
        .axi_rvalid(axi_rvalid),   .axi_rready(axi_rready),
        .axi_rdata(axi_rdata),     .axi_rresp(axi_rresp),
        .axi_rlast(axi_rlast),
        .axi_awvalid(axi_awvalid), .axi_awready(axi_awready),
        .axi_awaddr(axi_awaddr),   .axi_awprot(axi_awprot),
        .axi_awlen(axi_awlen),
        .axi_wvalid(axi_wvalid),   .axi_wready(axi_wready),
        .axi_wdata(axi_wdata),     .axi_wstrb(axi_wstrb),
        .axi_wlast(axi_wlast),
        .axi_bvalid(axi_bvalid),   .axi_bready(axi_bready),
        .axi_bresp(axi_bresp)
    );

    // ===== MEM/WB register =====
    wire mem_wb_valid;
    mem_wb_reg u_mem_wb(
        .clk(clk),
        .reset(reset),
        .valid_in(mem_valid_out),       // mem_stage 同步 valid，与 mem_result 对齐
        .ready_in(1'b1),
        .flush(1'b0),
        .stall(1'b0),
        .mem_result_in(mem_result),
        .rd_in(mem_rd_reg),            // mem_stage 同步寄存器输出（与 mem_result 对齐）
        .mem_to_reg_in(mem_mem_to_reg_out),  // mem_stage 同步输出（与 mem_result 对齐）
        .reg_write_in(mem_reg_write_reg),  // mem_stage 同步寄存器输出（与 mem_result 对齐）
        .csr_rd_in(mem_csr_rd),
        .csr_wr_in(mem_csr_wr),
        .csr_xchg_in(mem_csr_xchg),
        .csr_addr_in(mem_csr_addr),
        .csr_wdata_in(mem_csr_wdata),
        .csr_wmask_in(mem_csr_wmask),
        .ready_out(),
        .mem_result_out(wb_mem_result),
        .rd_out(wb_rd),
        .mem_to_reg_out(wb_mem_to_reg),
        .reg_write_out(wb_reg_write),
        .csr_rd_out(wb_csr_rd),
        .csr_wr_out(wb_csr_wr),
        .csr_xchg_out(wb_csr_xchg),
        .csr_addr_out(wb_csr_addr),
        .csr_wdata_out(wb_csr_wdata),
        .csr_wmask_out(wb_csr_wmask),
        .valid_out(mem_wb_valid)
    );
    // wb_we 必须用 valid 门控：mem_wb_reg 的数据寄存器在 valid_in=0 的空拍保持旧值，
    //   不门控则同一条指令的写在停顿期间每拍重复提交（R1W 实测 47 拍连写同值——
    //   当前同值幂等无害，但它掩盖时序问题且浪费写端口语义）。valid_out 每拍精确
    //   反映"本拍有指令完成"，门控后每条指令恰好写一拍。
    assign wb_we = wb_reg_write && mem_wb_valid;
    wire is_csr_wb = wb_csr_rd || wb_csr_wr || wb_csr_xchg;
    assign wb_data = is_csr_wb ? csr_rdata : wb_mem_result;
    wire [31:0] csr_wdata_masked =
        (csr_rdata & ~wb_csr_wmask) | (wb_csr_wdata & wb_csr_wmask);

    // ===== Exception / CSR =====
    wire interrupt_req;
    interrupt_ctrl u_int(
        .clk(clk),
        .hw_int(hw_int),
        .CRMD(crmd),
        .interrupt_req(interrupt_req)
    );
    assign interrupt = interrupt_req;
    csr_file u_csr(
        .clk(clk),
        .reset(reset),
        .csr_we(mem_wb_valid && (wb_csr_wr || wb_csr_xchg)),
        .csr_addr(wb_csr_addr),
        .csr_wdata(csr_wdata_masked),
        .csr_rdata(csr_rdata),
        .exception(exception),
        .era_in(ex1_pc_r),     // EX2 检测异常，PC 取 ex1_reg（对齐异常指令）
        .ertn(ex_ertn),
        .interrupt(interrupt),
        .syscall(ex_syscall),
        .break_inst(ex_break),
        .illegal_inst(ex_ine),
        .page_fault(mmu_page_fault),
        .tlb_modify(mmu_tlb_modify),
        .tlb_refill(mmu_tlb_refill),
        .pc(ex_pc),
        .era(era),
        .eentry(eentry),
        .crmd(crmd),
        .dmw0(dmw0), .dmw1(dmw1),
        .cpucfg_idx(ex1_src1_r[13:0]),
        .cpucfg_data(cpucfg_data)
    );

    // ===== Next PC =====
    wire [31:0] misp_target = ex1_pc_r + 32'd4;

    next_pc_gen u_next_pc(
        .pc(pc),
        .branch_taken(br_ctrl_flush),
        .branch_target(ex_branch_target),
        .exception(exception),
        .eret(ex_ertn),
        .era(era),
        .eentry(eentry),
        .misp_flush(br_misp_flush),
        .misp_target(misp_target),
        .pred_taken(pred_taken),
        .pred_target(pred_target),
        .next_pc(next_pc)
    );

    // ===== Pipeline Ctrl =====
    pipeline_ctrl u_pipeline_ctrl(
        .clk(clk),
        .reset(reset),
        .pc(pc),
        .branch_taken(br_ctrl_flush),
        .branch_target(ex_branch_target),
        .misp_flush(br_misp_flush),
        .exception(exception),
        .ertn(ex_ertn),
        .era(era),
        .eentry(eentry),
        .if_id_valid(id_valid),
        .id_ex_valid(id_ex_valid),
        .ex_mem_valid(ex_mem_valid),
        .mem_wb_valid(mem_wb_valid),
        .if_flush(if_flush),
        .id_flush(id_flush),
        .ex_flush(ex_flush)
    );
endmodule
