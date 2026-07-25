// ============================================================================
// if1_reg — IF1→IF2 流水线寄存器
//   锁存请求 PC + 分支预测 GHR 快照
// ============================================================================
module if1_reg(
    input  clk,
    input  reset,

    input         valid_i,
    input         ready_i,
    input         flush_i,
    input         stall_i,

    input  [31:0] pc_i,
    input         pred_taken_i,
    input  [7:0]  pred_ghr_i,

    output        ready_o,
    output reg    valid_o,
    output reg [31:0] pc_o,
    output reg        pred_taken_o,
    output reg [7:0]  pred_ghr_o
);

    assign ready_o = stall_i ? 1'b0 : (!valid_o || ready_i);

    always @(posedge clk) begin
        if (reset) begin
            valid_o      <= 1'b0;
            pc_o         <= 32'h1c000000;
            pred_taken_o <= 1'b0;
            pred_ghr_o   <= 8'h0;
        end else if (flush_i) begin
            // 分支冲刷：valid 清 0，pc_o 保持旧值 (等待下一周期捕获新 PC)
            //   不能复位到 0x80000000，否则 I-cache 会查错误的地址
            valid_o      <= 1'b0;
            pred_taken_o <= 1'b0;
            pred_ghr_o   <= 8'h0;
            // pc_o 保持——下周期 ready_o 时会捕获 pc_i (= 分支目标)
        end else if (stall_i) begin
            // hold
        end else if (ready_o) begin
            if (valid_o && valid_i && pc_i == pc_o) begin
                valid_o <= 1'b0;
            end else begin
                valid_o <= valid_i;
                if (valid_i) begin
                    pc_o         <= pc_i;
                    pred_taken_o <= pred_taken_i;
                    pred_ghr_o   <= pred_ghr_i;
                end
            end
        end
    end

endmodule
