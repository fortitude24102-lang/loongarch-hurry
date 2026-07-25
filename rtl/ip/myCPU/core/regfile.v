// ============================================================================
// regfile — 32×32 位通用寄存器文件
//   2 读端口（组合逻辑），1 写端口（时序逻辑）。
//   r0 硬连线为 0，写入 r0 无效。
// ============================================================================
module regfile(
    input         clk,
    input  [4:0]  raddr1,     // 读地址1
    input  [4:0]  raddr2,     // 读地址2
    input  [4:0]  waddr,      // 写地址
    input         we,         // 写使能
    input  [31:0] wdata,      // 写数据
    output [31:0] rdata1,     // 读数据1
    output [31:0] rdata2      // 读数据2
);

    reg [31:0] rf [0:31];

    // 仿真初始化：清零全部寄存器（FPGA 上电即为 0，仿真中避免 'X）
    integer _ri_;
    initial begin
        for (_ri_ = 0; _ri_ < 32; _ri_ = _ri_ + 1)
            rf[_ri_] = 32'h0;
    end

    // r0 硬连线为 0，写忽略；同周期写旁路（write-first）
    //   用 === case equality 防 X——waddr 若未初始化为 Z，Z===raddr→1'b0，
    //   不会像 == 那样产生 X（Z == 12 = X）。
    wire bypass1 = we && (waddr === raddr1);
    wire bypass2 = we && (waddr === raddr2);
    assign rdata1 = (raddr1 == 5'd0) ? 32'd0 : (bypass1 ? wdata : rf[raddr1]);
    assign rdata2 = (raddr2 == 5'd0) ? 32'd0 : (bypass2 ? wdata : rf[raddr2]);

    always @(posedge clk) begin
        if (we && waddr != 5'd0)
            rf[waddr] <= wdata;
    end

endmodule
