// ============================================================================
// csr_file — LoongArch 控制状态寄存器文件
//   实现以下 CSR：
//     CRMD (0x0)  — 当前模式控制
//     PRMD (0x1)  — 先前模式
//     ESTAT(0x5)  — 例外状态（含 ECODE）
//     ERA  (0x6)  — 例外返回地址
//     EENTRY(0xC) — 例外入口地址
//     SAVE0~3, TID, TCFG, TVAL, TICLR, LLBCTL, ECFG, BADV, CPUID, TLBRENTRY
// ============================================================================
module csr_file(
    input  clk,
    input  reset,
    input  csr_we,              // CSR 写使能
    input  [13:0] csr_addr,     // CSR 地址
    input  [31:0] csr_wdata,    // CSR 写数据
    output reg [31:0] csr_rdata,// CSR 读数据

    // 例外输入
    input         exception,    // 任意例外发生
    input  [31:0] era_in,       // 例外返回地址（当前 PC）
    input         ertn,         // ERTN 指令（例外返回）
    input         interrupt,    // 外部中断
    input         syscall,      // SYSCALL 指令
    input         break_inst,   // BREAK 指令
    input         illegal_inst, // 非法指令 (INE)
    input         page_fault,   // 页错误
    input         tlb_modify,   // TLB 修改例外
    input         tlb_refill,   // TLB 重填例外
    input  [31:0] pc,           // 当前指令 PC

    // CSR 输出
    output [31:0] era,          // 例外返回地址
    output [31:0] eentry,       // 例外入口地址
    output [31:0] crmd,          // 当前模式控制
    output [31:0] dmw0,         // 直接映射窗口 0
    output [31:0] dmw1,         // 直接映射窗口 1
    input  [13:0] cpucfg_idx,   // CPUCFG 配置字索引（rj）
    output [31:0] cpucfg_data   // CPUCFG 读数据
);

    // ---- CSR 寄存器 --------------------------------------------------------
    reg [31:0] CRMD, PRMD, ESTAT, ERA, EENTRY;
    reg [31:0] SAVE0, SAVE1, SAVE2, SAVE3;
    reg [31:0] TID, TCFG, TVAL, TICLR, LLBCTL;
    reg [31:0] ECFG, BADV, CPUID, TLBRENTRY;
    reg [31:0] DMW0, DMW1;

    // ---- 例外码（ECODE）生成 -----------------------------------------------
    wire [5:0] ecode;
    assign ecode = interrupt    ? 6'h00 :   // 中断
                   syscall      ? 6'h0B :   // 系统调用
                   break_inst   ? 6'h0C :   // 断点
                   illegal_inst ? 6'h0D :   // 非法指令
                   page_fault   ? 6'h01 :   // 页错误（PIL：取指 / PIS：存贮 / PIF：取指）
                   tlb_modify   ? 6'h03 :   // TLB 修改例外
                   tlb_refill   ? 6'h02 :   // TLB 重填例外
                   exception    ? 6'h3F :   // 未知例外
                   6'h00;

    // ---- 输出连线 ----------------------------------------------------------
    assign era    = ERA;
    assign eentry = EENTRY;
    assign dmw0   = DMW0;
    assign dmw1   = DMW1;

    // CPUCFG 配置字表（组合逻辑）
    assign cpucfg_data =
        (cpucfg_idx == 14'h1)  ? 32'h0001f1f4 :  // ARCH=0,PGMMU=1,PALEN=31,VALEN=31
        (cpucfg_idx == 14'h2)  ? 32'h00000000 :  // 无浮点
        (cpucfg_idx == 14'h10) ? 32'h00000005 :  // bit0=I-cache, bit2=D-cache 存在→kernel 走 cache 路线
        (cpucfg_idx == 14'h11) ? 32'h05080000 :  // I:off=5(32B),idx=8(256sets),max_way=0
        (cpucfg_idx == 14'h12) ? 32'h05080003 :  // D:off=5(32B),idx=8(256sets),max_way=3
        32'h00000000;
    assign crmd   = CRMD;

    // ---- CSR 读（组合逻辑）--------------------------------------------------
    always @(*) begin
        case (csr_addr)
            14'h0:  csr_rdata = CRMD;
            14'h1:  csr_rdata = PRMD;
            14'h4:  csr_rdata = ECFG;
            14'h5:  csr_rdata = {ESTAT[31:22], ecode, ESTAT[15:0]};
            14'h6:  csr_rdata = ERA;
            14'h7:  csr_rdata = BADV;
            14'hc:  csr_rdata = EENTRY;
            14'h20: csr_rdata = CPUID;
            14'h30: csr_rdata = SAVE0;
            14'h31: csr_rdata = SAVE1;
            14'h32: csr_rdata = SAVE2;
            14'h33: csr_rdata = SAVE3;
            14'h40: csr_rdata = TID;
            14'h41: csr_rdata = TCFG;
            14'h42: csr_rdata = TVAL;
            14'h44: csr_rdata = TICLR;
            14'h60: csr_rdata = LLBCTL;
            14'h88: csr_rdata = TLBRENTRY;
            14'h180: csr_rdata = DMW0;
            14'h181: csr_rdata = DMW1;
            default: csr_rdata = 32'h0;
        endcase
    end

    // ---- CSR 写 + 例外处理 -------------------------------------------------
    // 例外触发信号（任意一种例外或中断）
    wire exception_trigger = syscall || break_inst || illegal_inst ||
                             page_fault || tlb_modify || tlb_refill || interrupt;

    always @(posedge clk) begin
        if (reset) begin
            CRMD   <= 32'h00000008;  // DA=1 (直接地址模式), PLV=0 (内核), IE=0
            PRMD   <= 32'h0;
            ESTAT  <= 32'h0;
            ERA    <= 32'h0;
            EENTRY <= 32'h1c000000;
            SAVE0  <= 32'h0; SAVE1 <= 32'h0; SAVE2 <= 32'h0; SAVE3 <= 32'h0;
            TID    <= 32'h0; TCFG  <= 32'h0; TVAL  <= 32'h0; TICLR <= 32'h0;
            LLBCTL <= 32'h0;
            ECFG   <= 32'h0; BADV   <= 32'h0; CPUID <= 32'h0; TLBRENTRY <= 32'h0;
            DMW0   <= 32'h0; DMW1   <= 32'h0;
        end else begin
            // ---- ERTN：例外返回 --------------------------------------------
            if (ertn) begin
                CRMD[1:0]    <= PRMD[1:0];      // 恢复特权级别
                CRMD[2]      <= PRMD[2];         // 恢复全局中断使能
                ESTAT[21:16] <= 6'h00;           // 清除 ECODE
                if (LLBCTL[2]) LLBCTL[1] <= 1'b1;
            end
            // ---- 例外/中断触发 ---------------------------------------------
            else if (exception_trigger) begin
                ERA        <= era_in;            // 保存返回地址
                PRMD[1:0]  <= CRMD[1:0];         // 保存当前特权级别
                PRMD[2]    <= CRMD[2];           // 保存中断使能
                // 根据例外类型设置 ECODE
                if (interrupt)                 ESTAT[21:16] <= 6'h00;
                else if (syscall)              ESTAT[21:16] <= 6'h0B;
                else if (break_inst)           ESTAT[21:16] <= 6'h0C;
                else if (illegal_inst)         ESTAT[21:16] <= 6'h0D;
                else if (page_fault)           ESTAT[21:16] <= 6'h01;
                else if (tlb_modify)           ESTAT[21:16] <= 6'h03;
                else if (tlb_refill)           ESTAT[21:16] <= 6'h02;
                // 进入内核模式，关闭中断
                CRMD[1:0]  <= 2'b00;
                CRMD[2]    <= 1'b0;
            end
            // ---- CSR 写 ---------------------------------------------------
            else if (csr_we) begin
                case (csr_addr)
                    14'h0:  CRMD      <= csr_wdata;
                    14'h1:  PRMD      <= csr_wdata;
                    14'h4:  ECFG      <= csr_wdata;
                    14'h5:  ESTAT     <= csr_wdata;
                    14'h6:  ERA       <= csr_wdata;
                    14'h7:  BADV      <= csr_wdata;
                    14'hc:  EENTRY    <= {csr_wdata[31:6], 6'b0};
                    14'h20: CPUID     <= csr_wdata;
                    14'h30: SAVE0     <= csr_wdata;
                    14'h31: SAVE1     <= csr_wdata;
                    14'h32: SAVE2     <= csr_wdata;
                    14'h33: SAVE3     <= csr_wdata;
                    14'h40: TID       <= csr_wdata;
                    14'h41: TCFG      <= csr_wdata;
                    14'h42: TVAL      <= csr_wdata;
                    14'h44: TICLR     <= csr_wdata;
                    14'h60: LLBCTL    <= csr_wdata;
                    14'h88: TLBRENTRY <= csr_wdata;
                    14'h180: DMW0      <= csr_wdata;
                    14'h181: DMW1      <= csr_wdata;
                endcase
            end
        end
    end

endmodule
