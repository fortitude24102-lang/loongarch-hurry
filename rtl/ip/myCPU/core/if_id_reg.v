module if_id_reg(
    input  clk, input  reset,
    input         valid_i,  input  ready_i,
    input         flush_i,  input  stall_i,
    input  [31:0] pc_i,     input  [31:0] inst_i,
    input         br_pred_i, input  [7:0] br_ghr_i,
    output        ready_o,
    output reg    valid_o,  output reg [31:0] pc_o,
    output reg [31:0] inst_o,
    output reg        br_pred_o,
    output reg [7:0]  br_ghr_o
);

    assign ready_o = stall_i ? 1'b0 : ready_i;

    always @(posedge clk) begin
        if (reset || flush_i) begin
            valid_o   <= 1'b0; pc_o <= 32'h0; inst_o <= 32'h0;
            br_pred_o <= 1'b0; br_ghr_o <= 8'h0;
        end else if (!stall_i && ready_o) begin
            valid_o <= valid_i;
            if (valid_i) begin
                pc_o      <= pc_i;    inst_o    <= inst_i;
                br_pred_o <= br_pred_i; br_ghr_o <= br_ghr_i;
            end
        end
    end

endmodule
