`timescale 1ns/1ps

module dcache_store_buffer_tb;
    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    reg         req = 1'b0;
    reg [31:0]  addr = 32'h0;
    reg [31:0]  wdata = 32'h0;
    reg         we = 1'b0;
    reg [3:0]   byte_we = 4'h0;
    wire [31:0] rdata;
    wire        ready;

    wire        axi_req;
    wire [31:0] axi_addr;
    wire [31:0] axi_wdata;
    wire [3:0]  axi_wstrb;
    wire        axi_we;
    wire [7:0]  axi_len;
    reg [31:0]  axi_rdata = 32'h0;
    reg         axi_valid = 1'b0;
    wire        axi_ready = axi_valid;
    reg         wr_done = 1'b0;
    reg         axi_wnext = 1'b0;

    wire        cacop_done;
    wire [3:0]  dbg_state;
    wire        store_buffer_full;
    wire        store_buffer_enqueue;
    wire        store_buffer_forward;
    wire [2:0]  store_buffer_count;

    dcache dut (
        .clk(clk),
        .reset(reset),
        .req(req),
        .addr(addr),
        .wdata(wdata),
        .we(we),
        .rdata(rdata),
        .ready(ready),
        .byte_we(byte_we),
        .axi_req(axi_req),
        .axi_addr(axi_addr),
        .axi_wdata(axi_wdata),
        .axi_wstrb(axi_wstrb),
        .axi_we(axi_we),
        .axi_len(axi_len),
        .axi_rdata(axi_rdata),
        .axi_valid(axi_valid),
        .axi_ready(axi_ready),
        .wr_done(wr_done),
        .axi_wnext(axi_wnext),
        .cacop_req(1'b0),
        .cacop_code(5'h0),
        .cacop_addr(32'h0),
        .cacop_done(cacop_done),
        .dbg_state(dbg_state),
        .store_buffer_full(store_buffer_full),
        .store_buffer_enqueue(store_buffer_enqueue),
        .store_buffer_forward(store_buffer_forward),
        .store_buffer_count(store_buffer_count)
    );

    reg [31:0] mem_a = 32'h1111_1111;
    reg [31:0] mem_b = 32'h2222_2222;
    reg        write_hold = 1'b0;
    reg        write_pending = 1'b0;
    integer    write_delay = 0;
    reg [31:0] write_addr = 32'h0;
    reg [31:0] write_data = 32'h0;
    reg [3:0]  write_strb = 4'h0;
    reg        read_hold = 1'b0;
    reg        read_pending = 1'b0;
    reg [31:0] read_addr = 32'h0;
    reg        forward_seen = 1'b0;

    function automatic [31:0] merge_bytes;
        input [31:0] old_data;
        input [31:0] new_data;
        input [3:0] strb;
        begin
            merge_bytes[7:0] = strb[0] ? new_data[7:0] : old_data[7:0];
            merge_bytes[15:8] = strb[1] ? new_data[15:8] : old_data[15:8];
            merge_bytes[23:16] = strb[2] ? new_data[23:16] : old_data[23:16];
            merge_bytes[31:24] = strb[3] ? new_data[31:24] : old_data[31:24];
        end
    endfunction

    always @(posedge clk) begin
        axi_valid <= 1'b0;
        wr_done <= 1'b0;
        axi_wnext <= 1'b0;
        if (store_buffer_forward)
            forward_seen <= 1'b1;

        if (!axi_req)
            write_hold <= 1'b0;
        if (!write_hold && axi_req && axi_we) begin
            write_hold <= 1'b1;
            write_pending <= 1'b1;
            write_delay <= 40;
            write_addr <= axi_addr;
            write_data <= axi_wdata;
            write_strb <= axi_wstrb;
            axi_wnext <= 1'b1;
        end else if (write_pending) begin
            if (write_delay == 0) begin
                if (write_addr == 32'h0000_1000)
                    mem_a <= merge_bytes(mem_a, write_data, write_strb);
                else if (write_addr == 32'h0000_1004)
                    mem_b <= merge_bytes(mem_b, write_data, write_strb);
                write_pending <= 1'b0;
                wr_done <= 1'b1;
            end else begin
                write_delay <= write_delay - 1;
            end
        end

        if (!axi_req)
            read_hold <= 1'b0;
        if (!read_hold && axi_req && !axi_we) begin
            read_hold <= 1'b1;
            read_pending <= 1'b1;
            read_addr <= axi_addr;
        end else if (read_pending) begin
            axi_rdata <=
                (read_addr == 32'h0000_1000) ? mem_a : mem_b;
            axi_valid <= 1'b1;
            read_pending <= 1'b0;
        end
    end

    task automatic issue_store;
        input [31:0] task_addr;
        input [31:0] task_data;
        input [3:0] task_strb;
        begin
            @(negedge clk);
            addr = task_addr;
            wdata = task_data;
            byte_we = task_strb;
            we = 1'b1;
            req = 1'b1;
            wait (ready);
            @(negedge clk);
            req = 1'b0;
            we = 1'b0;
            byte_we = 4'h0;
        end
    endtask

    task automatic issue_load;
        input [31:0] task_addr;
        input [31:0] expected;
        begin
            @(negedge clk);
            addr = task_addr;
            byte_we = 4'h0;
            we = 1'b0;
            req = 1'b1;
            wait (ready);
            if (rdata !== expected)
                $fatal(1, "load mismatch: expected=%08x actual=%08x",
                       expected, rdata);
            @(negedge clk);
            req = 1'b0;
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        reset = 1'b0;

        // A occupies the AXI write channel.  B remains queued, so the load of
        // B must overlay the still-old SRAM word with buffered bytes.
        issue_store(32'h0000_1000, 32'haaaa_aaaa, 4'b1111);
        issue_store(32'h0000_1004, 32'hbbbb_cccc, 4'b0011);
        issue_load(32'h0000_1004, 32'h2222_cccc);

        if (!forward_seen)
            $fatal(1, "store-buffer forwarding event was not observed");
        $display("[SB-UNIT] PASS: partial-byte store-to-load forwarding");
        $finish;
    end

    initial begin
        #20000;
        $fatal(1, "store-buffer unit test timed out");
    end
endmodule
