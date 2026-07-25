module decoder(
    input [31:0] inst,
    output reg [3:0] alu_op,
    output reg alu_src,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg mem_to_reg,
    output reg branch,
    output reg branch_ne,
    output reg branch_eq,
    output reg branch_lt,
    output reg branch_ge,
    output reg branch_ltu,
    output reg branch_geu,
    output reg ertn,
    output reg syscall,
    output reg break_inst,
    output reg [1:0] mem_width,
    output reg mem_signed,
    output reg [4:0] rd_out,
    output reg [4:0] rs1_out,
    output reg [4:0] rs2_out,
    output reg csr_rd,
    output reg csr_wr,
    output reg csr_xchg,
    output reg is_jirl,
    output reg is_pcaddu12i,
    output reg is_ll,
    output reg is_sc,
    output reg is_mul,
    output reg [1:0] mul_op,
    output reg cpu_cfg,
    output reg [4:0] cacop_code,
    output reg cacop_valid,
    output reg illegal
);

    wire [5:0]  opcode   = inst[31:26];
    wire [9:0]  opcode_i = inst[31:22];
    wire [16:0] opcode_r = inst[31:15];
    wire [4:0]  rd       = inst[4:0];
    wire [4:0]  rj       = inst[9:5];
    wire [4:0]  rk       = inst[14:10];

    reg is_decoded;

    always @(*) begin
        alu_op     = 4'b0000;
        alu_src    = 1'b0;
        reg_write  = 1'b0;
        mem_read   = 1'b0;
        mem_write  = 1'b0;
        mem_to_reg = 1'b0;
        branch     = 1'b0;
        branch_ne  = 1'b0;
        branch_eq  = 1'b0;
        branch_lt  = 1'b0;
        branch_ge  = 1'b0;
        branch_ltu = 1'b0;
        branch_geu = 1'b0;
        ertn       = 1'b0;
        syscall    = 1'b0;
        break_inst = 1'b0;
        mem_width  = 2'b10;
        mem_signed = 1'b1;
        rd_out     = rd;
        rs1_out    = rj;
        rs2_out    = rk;
        csr_rd     = 1'b0;
        csr_wr     = 1'b0;
        csr_xchg   = 1'b0;
        is_jirl       = 1'b0;
        is_pcaddu12i  = 1'b0;
        is_ll         = 1'b0;
        is_sc         = 1'b0;
        is_mul        = 1'b0;
        mul_op        = 2'b00;
        cpu_cfg       = 1'b0;
        cacop_code    = 5'b0;
        cacop_valid   = 1'b0;
        is_decoded    = 1'b0;
        illegal    = 1'b0;

        casez (opcode_r)
            17'b00000000000100000: begin alu_op = 4'b0000; reg_write = 1; is_decoded = 1; end
            17'b00000000000100010: begin alu_op = 4'b0001; reg_write = 1; is_decoded = 1; end
            17'b00000000000100100: begin alu_op = 4'b0101; reg_write = 1; is_decoded = 1; end
            17'b00000000000100101: begin alu_op = 4'b0110; reg_write = 1; is_decoded = 1; end
            17'b00000000000101001: begin alu_op = 4'b0010; reg_write = 1; is_decoded = 1; end
            17'b00000000000101010: begin alu_op = 4'b0011; reg_write = 1; is_decoded = 1; end
            17'b00000000000101011: begin alu_op = 4'b0100; reg_write = 1; is_decoded = 1; end
            17'b00000000000101000: begin alu_op = 4'b1010; reg_write = 1; is_decoded = 1; end
            17'b00000000000101110: begin alu_op = 4'b0111; reg_write = 1; is_decoded = 1; end
            17'b00000000000101111: begin alu_op = 4'b1000; reg_write = 1; is_decoded = 1; end
            17'b00000000000110000: begin alu_op = 4'b1001; reg_write = 1; is_decoded = 1; end
            17'b00000000000111000: begin reg_write = 1; is_mul = 1; mul_op = 2'b00; is_decoded = 1; end
            17'b00000000000111001: begin reg_write = 1; is_mul = 1; mul_op = 2'b01; is_decoded = 1; end
            17'b00000000000111010: begin reg_write = 1; is_mul = 1; mul_op = 2'b10; is_decoded = 1; end
            17'b00000000010000000: begin illegal = 1; end
            17'b00000000010000001: begin illegal = 1; end
            17'b00000000010000010: begin illegal = 1; end
            17'b00000000010000011: begin illegal = 1; end
        endcase

        if (!is_decoded && inst[31:10] == 22'h001B) begin
            cpu_cfg = 1; reg_write = 1; is_decoded = 1;
        end

        if (!is_decoded) begin
            casez (opcode_i)
                10'b0000001010: begin alu_op = 4'b0000; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000001000: begin alu_op = 4'b0101; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000001001: begin alu_op = 4'b0110; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000001101: begin alu_op = 4'b0010; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000001110: begin alu_op = 4'b0011; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000001111: begin alu_op = 4'b0100; alu_src = 1; reg_write = 1; is_decoded = 1; end
                10'b0000000001: begin
                    if (inst[17:15] != 3'b001) begin illegal = 1;
                    end else begin alu_src = 1; reg_write = 1; is_decoded = 1;
                        if (inst[19:18] == 2'b00) alu_op = 4'b0111;
                        else if (inst[19:18] == 2'b01) alu_op = 4'b1000;
                        else if (inst[19:18] == 2'b10) alu_op = 4'b1001;
                        else illegal = 1;
                    end
                end
                10'b0001010???: begin alu_op = 4'b0000; alu_src = 1; reg_write = 1; rs1_out = 5'd0; is_decoded = 1; end
                10'b0001110???: begin alu_op = 4'b0000; alu_src = 1; reg_write = 1; rs1_out = 5'd0; is_pcaddu12i = 1; is_decoded = 1; end
            endcase
        end

        if (!is_decoded) begin
            casez (opcode_i)
                10'b0010100000: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b00; mem_signed = 1; is_decoded = 1; end
                10'b0010100001: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b01; mem_signed = 1; is_decoded = 1; end
                10'b0010100010: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b10; is_decoded = 1; end
                10'b0010101000: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b00; mem_signed = 0; is_decoded = 1; end
                10'b0010101001: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b01; mem_signed = 0; is_decoded = 1; end
                10'b0010100100: begin alu_op = 4'b0000; alu_src = 1; mem_write = 1; mem_width = 2'b00; rs2_out = rd; is_decoded = 1; end
                10'b0010100101: begin alu_op = 4'b0000; alu_src = 1; mem_write = 1; mem_width = 2'b01; rs2_out = rd; is_decoded = 1; end
                10'b0010100110: begin alu_op = 4'b0000; alu_src = 1; mem_write = 1; mem_width = 2'b10; rs2_out = rd; is_decoded = 1; end
            endcase
        end

        if (!is_decoded) begin
            case (opcode)
                6'b010110: begin branch = 1; branch_eq = 1; rs2_out = rd; is_decoded = 1; end
                6'b010111: begin branch = 1; branch_ne = 1; rs2_out = rd; is_decoded = 1; end
                6'b011000: begin branch = 1; branch_lt = 1; rs2_out = rd; is_decoded = 1; end
                6'b011001: begin branch = 1; branch_ge = 1; rs2_out = rd; is_decoded = 1; end
                6'b011010: begin branch = 1; branch_ltu = 1; rs2_out = rd; is_decoded = 1; end
                6'b011011: begin branch = 1; branch_geu = 1; rs2_out = rd; is_decoded = 1; end
                6'b010101: begin branch = 1; alu_src = 1; reg_write = 1; rd_out = 5'd1; rs1_out = 5'd0; rs2_out = 5'd0; is_decoded = 1; end
                6'b010100: begin branch = 1; alu_src = 1; rs1_out = 5'd0; rs2_out = 5'd0; is_decoded = 1; end
                6'b010011: begin branch = 1; alu_src = 1; reg_write = 1; rd_out = rd; rs2_out = 5'd0; is_jirl = 1; is_decoded = 1; end
            endcase
        end

        if (!is_decoded) begin
            case (inst[31:25])
                7'b0010010: begin alu_op = 4'b0000; alu_src = 1; mem_read = 1; mem_to_reg = 1; reg_write = 1; mem_width = 2'b10; is_ll = 1; is_decoded = 1; end
                7'b0010011: begin alu_op = 4'b0000; alu_src = 1; mem_write = 1; mem_width = 2'b10; rs2_out = 5'd0; reg_write = 1; is_sc = 1; is_decoded = 1; end
            endcase
        end

        if (!is_decoded) begin
            case (inst[31:25])
                7'b0000101: begin is_decoded = 1; end
                7'b0000110: begin alu_op = 4'b1111; reg_write = 1; is_decoded = 1; end
            endcase
        end

        if (!is_decoded) begin
            if (inst[31:25] == 7'b0000000 && inst[19:10] == 10'h000) begin
                case (inst[9:0])
                    10'b0001100010: begin is_decoded = 1; end
                    10'b0001100011: begin is_decoded = 1; end
                endcase
            end
            if (!is_decoded) begin
                casez (opcode_i)
                    10'b0000000000: begin
                        case (inst[24:15])
                            10'b0001010110: begin syscall = 1; is_decoded = 1; end
                            10'b0001010100: begin break_inst = 1; is_decoded = 1; end
                            10'b0000001001: begin ertn = 1; is_decoded = 1; end
                        endcase
                    end
                    10'b0000011000: begin  // cacop: inst[31:25]=0000011, inst[24:22]=000
                        cacop_code  = inst[4:0];
                        cacop_valid = 1;
                        is_decoded  = 1;
                    end
                endcase
            end
        end

        // CSR instructions share opcode 0x04 in inst[31:24].  The rj field
        // selects the operation: 0=CSRRD, 1=CSRWR, otherwise CSRXCHG.
        // CSRWR/CSRXCHG take the write value from rd; CSRXCHG takes its mask
        // from rj.
        if (!is_decoded && inst[31:24] == 8'h04) begin
            reg_write = 1'b1;
            if (rj == 5'd0) begin
                csr_rd  = 1'b1;
                rs1_out = 5'd0;
                rs2_out = 5'd0;
            end else if (rj == 5'd1) begin
                csr_wr  = 1'b1;
                rs1_out = rd;
                rs2_out = 5'd0;
            end else begin
                csr_xchg = 1'b1;
                rs1_out  = rd;
                rs2_out  = rj;
            end
            is_decoded = 1'b1;
        end

        if (is_decoded && opcode_i == 10'b0000000001 && (inst[19:18] == 2'b00 || inst[19:18] == 2'b01 || inst[19:18] == 2'b10))
            illegal = 1'b0;
        if (!is_decoded) illegal = 1;
    end

endmodule
