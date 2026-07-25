// Minimal behavioral model for the subset of xpm_memory_sdpram used by myCPU.
// This file is compiled only by the Verilator runner; Vivado uses the real XPM.
module xpm_memory_sdpram #(
    parameter integer ADDR_WIDTH_A       = 6,
    parameter integer ADDR_WIDTH_B       = 6,
    parameter integer BYTE_WRITE_WIDTH_A = 8,
    parameter         CLOCKING_MODE      = "common_clock",
    parameter         MEMORY_PRIMITIVE   = "auto",
    parameter integer MEMORY_SIZE        = 2048,
    parameter integer READ_DATA_WIDTH_B  = 32,
    parameter integer READ_LATENCY_B     = 1,
    parameter integer WRITE_DATA_WIDTH_A = 32,
    parameter         WRITE_MODE_B       = "READ_FIRST"
) (
    input                                      clka,
    input                                      ena,
    input  [WRITE_DATA_WIDTH_A/BYTE_WRITE_WIDTH_A-1:0] wea,
    input  [ADDR_WIDTH_A-1:0]                  addra,
    input  [WRITE_DATA_WIDTH_A-1:0]            dina,
    input                                      sleep,
    input                                      clkb,
    input                                      enb,
    input                                      regceb,
    input  [ADDR_WIDTH_B-1:0]                  addrb,
    output reg [READ_DATA_WIDTH_B-1:0]         doutb
);

    localparam integer DEPTH =
        MEMORY_SIZE / WRITE_DATA_WIDTH_A;
    localparam integer WRITE_LANES =
        WRITE_DATA_WIDTH_A / BYTE_WRITE_WIDTH_A;

    reg [WRITE_DATA_WIDTH_A-1:0] mem [0:DEPTH-1];
    integer lane;

    always @(posedge clka) begin
        if (ena && !sleep) begin
            for (lane = 0; lane < WRITE_LANES; lane = lane + 1) begin
                if (wea[lane]) begin
                    mem[addra][lane*BYTE_WRITE_WIDTH_A +: BYTE_WRITE_WIDTH_A]
                        <= dina[lane*BYTE_WRITE_WIDTH_A +: BYTE_WRITE_WIDTH_A];
                end
            end
        end
    end

    // READ_LATENCY_B=1 and WRITE_MODE_B="READ_FIRST" in the current D-cache.
    // Nonblocking writes above naturally preserve read-first behavior when the
    // common clock accesses the same location through both ports.
    always @(posedge clkb) begin
        if (enb && regceb && !sleep)
            doutb <= mem[addrb];
    end

endmodule
