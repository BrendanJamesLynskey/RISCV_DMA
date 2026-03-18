// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_fifo;
    import dma_pkg::*;

    // Parameters
    localparam DATA_W     = 32;
    localparam DEPTH      = 16;
    localparam CLK_PERIOD = 10;

    // Signals
    logic                    clk;
    logic                    srst;
    logic                    wr_en;
    logic [DATA_W-1:0]      wr_data;
    logic                    full;
    logic                    rd_en;
    logic [DATA_W-1:0]      rd_data;
    logic                    empty;
    logic [$clog2(DEPTH):0] count;

    // DUT
    dma_fifo #(.DATA_W(DATA_W), .DEPTH(DEPTH)) u_dut (
        .clk     (clk),
        .srst    (srst),
        .wr_en   (wr_en),
        .wr_data (wr_data),
        .full    (full),
        .rd_en   (rd_en),
        .rd_data (rd_data),
        .empty   (empty),
        .count   (count)
    );

    // Clock
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Test infrastructure
    int pass_count = 0;
    int fail_count = 0;
    int test_count = 0;

    task automatic check(input string name, input logic [DATA_W-1:0] got, expected);
        test_count++;
        if (got === expected) begin
            $display("[PASS] %s: got 0x%08h", name, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s: got 0x%08h, expected 0x%08h", name, got, expected);
            fail_count++;
        end
    endtask

    task automatic check_1b(input string name, input logic got, expected);
        test_count++;
        if (got === expected) begin
            $display("[PASS] %s: got %0b", name, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s: got %0b, expected %0b", name, got, expected);
            fail_count++;
        end
    endtask

    task automatic check_count(input string name, input logic [$clog2(DEPTH):0] got, expected);
        test_count++;
        if (got === expected) begin
            $display("[PASS] %s: got %0d", name, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s: got %0d, expected %0d", name, got, expected);
            fail_count++;
        end
    endtask

    task automatic reset_dut();
        wr_en   = 0;
        rd_en   = 0;
        wr_data = '0;
        srst    = 1;
        repeat (3) @(posedge clk);
        srst = 0;
        @(posedge clk);
    endtask

    task automatic write_word(input logic [DATA_W-1:0] data);
        // Drive signals just after the clock edge so they're stable for one full cycle
        @(negedge clk);
        wr_en   = 1;
        wr_data = data;
        @(negedge clk);
        wr_en   = 0;
        wr_data = '0;
    endtask

    task automatic read_word(output logic [DATA_W-1:0] data);
        @(negedge clk);
        data  = rd_data; // combinational read from rd_ptr
        rd_en = 1;
        @(negedge clk);
        rd_en = 0;
    endtask

    // ======== Tests ========
    initial begin
        $dumpfile("tb_dma_fifo.vcd");
        $dumpvars(0, tb_dma_fifo);

        // ---- Test 1: Reset clears FIFO ----
        $display("\n--- Test 1: Reset clears FIFO ---");
        reset_dut();
        check_1b("empty after reset", empty, 1'b1);
        check_1b("full after reset",  full,  1'b0);
        check_count("count after reset", count, 0);

        // ---- Test 2: Single write/read ----
        $display("\n--- Test 2: Single write/read ---");
        reset_dut();
        write_word(32'hCAFE_BABE);
        check_1b("not empty after write", empty, 1'b0);
        check_count("count after 1 write", count, 1);
        begin
            logic [DATA_W-1:0] rdata;
            read_word(rdata);
            check("single read data", rdata, 32'hCAFE_BABE);
        end
        check_1b("empty after read", empty, 1'b1);
        check_count("count after read", count, 0);

        // ---- Test 3: Fill to full ----
        $display("\n--- Test 3: Fill to full ---");
        reset_dut();
        for (int i = 0; i < DEPTH; i++) begin
            write_word(32'hA000_0000 + i);
        end
        check_1b("full after filling", full, 1'b1);
        check_count("count when full", count, DEPTH[$clog2(DEPTH):0]);

        // ---- Test 4: Full write rejected ----
        $display("\n--- Test 4: Full write rejected ---");
        // FIFO is still full from test 3
        write_word(32'hDEAD_DEAD);
        check_count("count unchanged after full write", count, DEPTH[$clog2(DEPTH):0]);
        check_1b("still full", full, 1'b1);

        // ---- Test 5: Drain from full ----
        $display("\n--- Test 5: Drain from full ---");
        // FIFO is full with data from test 3
        for (int i = 0; i < DEPTH; i++) begin
            begin
                logic [DATA_W-1:0] rdata;
                read_word(rdata);
                check($sformatf("drain word %0d", i), rdata, 32'hA000_0000 + i);
            end
        end
        check_1b("empty after drain", empty, 1'b1);
        check_count("count after drain", count, 0);

        // ---- Test 6: Empty read ----
        $display("\n--- Test 6: Empty read ---");
        // FIFO is empty from test 5
        @(negedge clk);
        rd_en = 1;
        @(negedge clk);
        rd_en = 0;
        check_1b("still empty after empty read", empty, 1'b1);
        check_count("count unchanged after empty read", count, 0);

        // ---- Test 7: Concurrent write/read ----
        $display("\n--- Test 7: Concurrent write/read ---");
        reset_dut();
        // First put one word in
        write_word(32'h1111_1111);
        check_count("count = 1 before concurrent", count, 1);
        // Simultaneous write and read — drive on negedge so signals are stable for one posedge
        @(negedge clk);
        wr_en   = 1;
        wr_data = 32'h2222_2222;
        rd_en   = 1;
        @(negedge clk);
        wr_en   = 0;
        rd_en   = 0;
        check_count("count unchanged after concurrent wr/rd", count, 1);
        // Read out the remaining word — should be 0x2222_2222
        begin
            logic [DATA_W-1:0] rdata;
            read_word(rdata);
            check("concurrent: remaining word", rdata, 32'h2222_2222);
        end

        // ---- Test 8: FIFO data integrity ----
        $display("\n--- Test 8: FIFO data integrity ---");
        reset_dut();
        for (int i = 0; i < DEPTH; i++) begin
            write_word(32'hDEAD_0000 + i);
        end
        for (int i = 0; i < DEPTH; i++) begin
            begin
                logic [DATA_W-1:0] rdata;
                read_word(rdata);
                check($sformatf("integrity word %0d", i), rdata, 32'hDEAD_0000 + i);
            end
        end

        // ---- Summary ----
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
