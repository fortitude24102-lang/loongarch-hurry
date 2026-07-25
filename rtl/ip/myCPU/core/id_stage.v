// ============================================================================
// id_stage �? ID stage (combinational decode + register read)
//
//   纯组合�?�辑，无流水线控制�?�valid/ready �? if_id_reg �? id_ex_reg 中处理�??
//   load_use_hazard �?测和 stall_out 是唯�?控制信号�?
// ============================================================================
module id_stage(
    input clk, input reset,
    input  valid_in, output ready_out, input  ready_in,
    input [31:0] pc_in, input [31:0] inst_in,
    input [4:0]  wb_rd, input [31:0] wb_data, input wb_we,
    input id_ex_mem_read, input [4:0] id_ex_rd,
    // MUL �? EX1/EX2 �? �? consumer �? ID 停顿直到 mul 离开 EX2
    input        id_ex_is_mul,     input        ex1_is_mul,
    input        ex1_valid,        input [4:0]  ex1_rd,
    // MEM 管线中是否有 load �? 持续停顿直到 WB
    input        ex_mem_mem_read,  input [4:0] ex_mem_rd,
    input        mem_mem_read,     input [4:0] mem_rd,
    input        wb_load,                                // WB 阶段�? load 指令
    output stall_out,
    output [31:0] pc_out, output [31:0] rs1_val, output [31:0] rs2_val,
    output [31:0] imm_out, output [4:0] rd_out,
    output [4:0] rs1_addr, output [4:0] rs2_addr,
    output [3:0] alu_op, output alu_src,
    output mem_read, output mem_write, output mem_to_reg, output reg_write,
    output branch, output branch_ne, output branch_eq, output branch_lt, output branch_ge,
    output branch_ltu, output branch_geu,
    output ertn, output syscall, output break_inst,
    output [1:0] mem_width, output mem_signed,
    output csr_rd, output csr_wr, output csr_xchg, output [13:0] csr_addr,
    output is_jirl, output is_pcaddu12i, output is_ll, output is_sc, output illegal,
    output is_mul, output [1:0] mul_op,
    output cpu_cfg, output [4:0] cacop_code, output cacop_valid
);

    function [31:0] imm_gen;
        input [31:0] inst;
        reg [11:0] si12; reg [15:0] offs16; reg [19:0] si20; reg [13:0] si14;
        begin
            si12=inst[21:10]; si20=inst[24:5]; offs16=inst[25:10]; si14=inst[24:11];
            case(inst[31:26])
                // 分支/跳转：offs16 是指令数偏移，CPU �? <<2 转字节地�?
                6'b010100,6'b010101: imm_gen={{4{inst[9]}},inst[9:0],inst[25:10],2'b00};
                6'b010110,6'b010111,6'b011000,6'b011001,6'b011010,6'b011011: imm_gen={{14{offs16[15]}},offs16,2'b00};
                6'b010011: imm_gen={{14{offs16[15]}},offs16,2'b00};
                default: begin
                    if(inst[31:25]==7'b0001010||inst[31:25]==7'b0001110) imm_gen={si20,12'b0};
                    else if(inst[31:25]==7'b0010000) imm_gen={{16{si14[13]}},si14,2'b00}; // LL.W/SC.W
                    else if(inst[31:22]==10'b0000001101||inst[31:22]==10'b0000001110||inst[31:22]==10'b0000001111) imm_gen={20'b0,si12};
                    else if(inst[31:22]==10'b0000000001) imm_gen={27'b0,inst[14:10]};
                    else imm_gen={{20{si12[11]}},si12};
                end
            endcase
        end
    endfunction

    wire [4:0] rd=inst_in[4:0], rj=inst_in[9:5], rk=inst_in[14:10];
    wire [31:0] imm=imm_gen(inst_in);

    wire [3:0] alu_op_w; wire alu_src_w,reg_write_w,mem_read_w,mem_write_w,mem_to_reg_w;
    wire branch_w,branch_ne_w,branch_eq_w,ertn_w;
    wire branch_lt_w,branch_ge_w,branch_ltu_w,branch_geu_w,syscall_w,break_w;
    wire [1:0] mem_width_w; wire mem_signed_w;
    wire [4:0] rd_out_w,rs1_addr_w,rs2_addr_w;
    wire csr_rd_w,csr_wr_w,csr_xchg_w,is_jirl_w,is_pcaddu12i_w,is_ll_w,is_sc_w,illegal_w;
    wire is_mul_w; wire [1:0] mul_op_w;
    wire cpu_cfg_w; wire [4:0] cacop_code_w; wire cacop_valid_w;

    decoder u_decoder(
        .inst(inst_in),.alu_op(alu_op_w),.alu_src(alu_src_w),
        .reg_write(reg_write_w),.mem_read(mem_read_w),.mem_write(mem_write_w),
        .mem_to_reg(mem_to_reg_w),.branch(branch_w),.branch_ne(branch_ne_w),
        .branch_eq(branch_eq_w),
        .branch_lt(branch_lt_w),.branch_ge(branch_ge_w),
        .branch_ltu(branch_ltu_w),.branch_geu(branch_geu_w),
        .ertn(ertn_w),.syscall(syscall_w),.break_inst(break_w),
        .mem_width(mem_width_w),.mem_signed(mem_signed_w),
        .rd_out(rd_out_w),.rs1_out(rs1_addr_w),.rs2_out(rs2_addr_w),
        .csr_rd(csr_rd_w),.csr_wr(csr_wr_w),.csr_xchg(csr_xchg_w),
        .is_jirl(is_jirl_w),.is_pcaddu12i(is_pcaddu12i_w),.is_ll(is_ll_w),.is_sc(is_sc_w),
        .is_mul(is_mul_w),.mul_op(mul_op_w),
        .cpu_cfg(cpu_cfg_w),.cacop_code(cacop_code_w),.cacop_valid(cacop_valid_w),
        .illegal(illegal_w)
    );

    wire [31:0] rdata1,rdata2;
    regfile u_regfile(.clk(clk),.raddr1(rs1_addr_w),.raddr2(rs2_addr_w),
        .waddr(wb_rd),.we(wb_we),.wdata(wb_data),.rdata1(rdata1),.rdata2(rdata2));

    // WB→ID 旁路：同�?周期 WB 写寄存器、ID 读同寄存器时，组合用 wb_data�?
    //   写发生在 posedge，读是组合�?�辑，不加旁路会读到旧�?��??
    wire [31:0] rs1_val_w = (wb_we && (wb_rd == rs1_addr_w) && (wb_rd != 5'd0)) ? wb_data : rdata1;
    wire [31:0] rs2_val_w = (wb_we && (wb_rd == rs2_addr_w) && (wb_rd != 5'd0)) ? wb_data : rdata2;

    // load-use 停顿�?
    //   相邻 load-use（如 ld.w rX; bne ...,rX）里，consumer �? ID �? load 正好�? EX1�?
    //   必须�? ID 就把 consumer 停住（晚�?拍进 EX1）；否则 consumer 会与 EX2 里的
    //   load 撞在�?起�?��?��?? ex_stall 无法处理"load �? EX2"（会�? load �?起冻死）�?
    //   consumer �? ID 多停�?拍后，load 已滑�? EX/MEM，落�? ex_stall 既有�?测，链条完整�?
    wire load_in_ex1 = id_ex_mem_read && ((id_ex_rd == rs1_addr_w) || (id_ex_rd == rs2_addr_w)) && (id_ex_rd != 5'd0);
    wire load_in_mem = mem_mem_read && ((mem_rd == rs1_addr_w) || (mem_rd == rs2_addr_w)) && (mem_rd != 5'd0);

    // mul→consumer 停顿：mul �? EX1/EX2 时，ID 中的 consumer 必须等到 mul 离开 EX2�?
    //   否则 consumer �? EX1 �? mul_busy 冻结期间，前递�?�不正确�?
    //   - mul stall �? 1 �? mul_p 还是 0，前递给 consumer 的是 0
    //   - mul stall �? 2 �? mul_p 算好�? ex1_reg 冻结不捕�?
    //   �? mul �? ex_mem/mem1 �? consumer 才进 EX1，此时前递链上结果已正确�?
    wire mul_in_ex1 = id_ex_is_mul && ((id_ex_rd == rs1_addr_w) || (id_ex_rd == rs2_addr_w)) && (id_ex_rd != 5'd0);
    wire mul_in_ex2 = ex1_is_mul && ex1_valid && ((ex1_rd == rs1_addr_w) || (ex1_rd == rs2_addr_w)) && (ex1_rd != 5'd0);
    assign stall_out = load_in_ex1 || load_in_mem || mul_in_ex1 || mul_in_ex2;
    assign ready_out = !valid_in || (ready_in && !stall_out);

    // 全部组合逻辑输出
    assign pc_out      = pc_in;
    assign rs1_val     = rs1_val_w;
    assign rs2_val     = rs2_val_w;
    assign imm_out     = imm;
    assign rd_out      = rd_out_w;
    assign rs1_addr    = rs1_addr_w;
    assign rs2_addr    = rs2_addr_w;
    assign alu_op      = alu_op_w;
    assign alu_src     = alu_src_w;
    assign mem_read    = mem_read_w;
    assign mem_write   = mem_write_w;
    assign mem_to_reg  = mem_to_reg_w;
    assign reg_write   = reg_write_w;
    assign branch      = branch_w;
    assign branch_ne   = branch_ne_w;
    assign branch_eq   = branch_eq_w;
    assign branch_lt   = branch_lt_w;
    assign branch_ge   = branch_ge_w;
    assign branch_ltu  = branch_ltu_w;
    assign branch_geu  = branch_geu_w;
    assign ertn        = ertn_w;
    assign syscall     = syscall_w;
    assign break_inst  = break_w;
    assign mem_width   = mem_width_w;
    assign mem_signed  = mem_signed_w;
    assign csr_rd      = csr_rd_w;
    assign csr_wr      = csr_wr_w;
    assign csr_xchg    = csr_xchg_w;
    assign csr_addr    = inst_in[23:10];
    assign is_jirl     = is_jirl_w;
    assign is_pcaddu12i = is_pcaddu12i_w;
    assign is_ll       = is_ll_w;
    assign is_sc       = is_sc_w;
    assign illegal     = illegal_w;
    assign is_mul      = is_mul_w;
    assign cpu_cfg     = cpu_cfg_w;
    assign cacop_code  = cacop_code_w;
    assign cacop_valid = cacop_valid_w;
    assign mul_op      = mul_op_w;

endmodule
