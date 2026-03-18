// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_arbiter;
    import dma_pkg::*;

    localparam int NUM_CH     = 4;
    localparam int CLK_PERIOD = 10;

    // Clock and reset
    logic clk;
    logic srst;

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // Round-robin DUT signals
    // ----------------------------------------------------------------
    logic [NUM_CH-1:0]   rr_req;
    logic [NUM_CH*2-1:0] rr_req_type_flat;
    logic [NUM_CH*32-1:0] rr_req_addr_flat;
    logic [NUM_CH*8-1:0] rr_req_len_flat;
    logic [NUM_CH-1:0]   rr_grant;
    logic                rr_grant_valid;
    arb_req_type_t       rr_axi_req_type;
    logic [31:0]         rr_axi_req_addr;
    logic [7:0]          rr_axi_req_len;
    logic                rr_axi_req_valid;
    logic                rr_axi_req_done;

    dma_arbiter #(.NUM_CH(NUM_CH), .ARB_MODE("ROUND_ROBIN")) u_rr (
        .clk           (clk),
        .srst          (srst),
        .req           (rr_req),
        .req_type_flat (rr_req_type_flat),
        .req_addr_flat (rr_req_addr_flat),
        .req_len_flat  (rr_req_len_flat),
        .grant         (rr_grant),
        .grant_valid   (rr_grant_valid),
        .axi_req_type  (rr_axi_req_type),
        .axi_req_addr  (rr_axi_req_addr),
        .axi_req_len   (rr_axi_req_len),
        .axi_req_valid (rr_axi_req_valid),
        .axi_req_done  (rr_axi_req_done)
    );

    // ----------------------------------------------------------------
    // Fixed-priority DUT signals
    // ----------------------------------------------------------------
    logic [NUM_CH-1:0]   fp_req;
    logic [NUM_CH*2-1:0] fp_req_type_flat;
    logic [NUM_CH*32-1:0] fp_req_addr_flat;
    logic [NUM_CH*8-1:0] fp_req_len_flat;
    logic [NUM_CH-1:0]   fp_grant;
    logic                fp_grant_valid;
    arb_req_type_t       fp_axi_req_type;
    logic [31:0]         fp_axi_req_addr;
    logic [7:0]          fp_axi_req_len;
    logic                fp_axi_req_valid;
    logic                fp_axi_req_done;

    dma_arbiter #(.NUM_CH(NUM_CH), .ARB_MODE("FIXED_PRIORITY")) u_fp (
        .clk           (clk),
        .srst          (srst),
        .req           (fp_req),
        .req_type_flat (fp_req_type_flat),
        .req_addr_flat (fp_req_addr_flat),
        .req_len_flat  (fp_req_len_flat),
        .grant         (fp_grant),
        .grant_valid   (fp_grant_valid),
        .axi_req_type  (fp_axi_req_type),
        .axi_req_addr  (fp_axi_req_addr),
        .axi_req_len   (fp_axi_req_len),
        .axi_req_valid (fp_axi_req_valid),
        .axi_req_done  (fp_axi_req_done)
    );

    // ----------------------------------------------------------------
    // Test infrastructure
    // ----------------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;
    int test_count = 0;

    task automatic check(input string name, input logic [31:0] got, expected);
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

    // ----------------------------------------------------------------
    // Helper: reset both DUTs
    // ----------------------------------------------------------------
    task automatic reset_duts();
        rr_req          = '0;
        rr_axi_req_done = 0;
        fp_req          = '0;
        fp_axi_req_done = 0;
        rr_req_type_flat = '0;
        rr_req_addr_flat = '0;
        rr_req_len_flat  = '0;
        fp_req_type_flat = '0;
        fp_req_addr_flat = '0;
        fp_req_len_flat  = '0;
        srst = 1;
        repeat (3) @(posedge clk);
        srst = 0;
        @(posedge clk);
    endtask

    // ----------------------------------------------------------------
    // Helper: pulse axi_req_done for one cycle on RR arbiter
    // ----------------------------------------------------------------
    task automatic rr_done_pulse();
        @(negedge clk);
        rr_axi_req_done = 1;
        @(negedge clk);
        rr_axi_req_done = 0;
    endtask

    // ----------------------------------------------------------------
    // Helper: pulse axi_req_done for one cycle on FP arbiter
    // ----------------------------------------------------------------
    task automatic fp_done_pulse();
        @(negedge clk);
        fp_axi_req_done = 1;
        @(negedge clk);
        fp_axi_req_done = 0;
    endtask

    // ================================================================
    // Main test sequence
    // ================================================================
    initial begin
        $dumpfile("tb_dma_arbiter.vcd");
        $dumpvars(0, tb_dma_arbiter);

        // ---- Test 1: No requests ----
        $display("\n--- Test 1: No requests ---");
        reset_duts();
        // Leave rr_req = 0, wait a couple of cycles
        repeat (3) @(posedge clk);
        check_1b("grant_valid=0 with no requests", rr_grant_valid, 1'b0);

        // ---- Test 2: Single request ----
        $display("\n--- Test 2: Single request ---");
        reset_duts();
        // Drive request on negedge so it is stable for the upcoming posedge
        @(negedge clk);
        rr_req          = 4'b0001;
        rr_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_READ);
        rr_req_addr_flat[0*32 +: 32] = 32'hDEAD_0000;
        rr_req_len_flat[0*8 +: 8]   = 8'd15;
        // Next posedge: arbiter in IDLE sees winner_found -> registers grant, enters BUSY
        @(posedge clk);
        // Outputs are registered, so sample just after the edge (at negedge)
        @(negedge clk);
        check_1b("grant[0] asserted",  rr_grant[0],     1'b1);
        check_1b("grant_valid=1",      rr_grant_valid,  1'b1);
        check_1b("axi_req_valid pulse", rr_axi_req_valid, 1'b1);
        // Wait one more posedge — axi_req_valid should drop (single-cycle pulse)
        @(posedge clk);
        @(negedge clk);
        check_1b("axi_req_valid deasserted", rr_axi_req_valid, 1'b0);
        // Clean up: pulse done to release
        rr_done_pulse();
        rr_req = '0;
        @(posedge clk);

        // ---- Test 3: Two simultaneous (RR) ----
        $display("\n--- Test 3: Two simultaneous requests (RR) ---");
        reset_duts();
        @(negedge clk);
        rr_req          = 4'b0011; // ch0 and ch1
        rr_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_READ);
        rr_req_addr_flat[0*32 +: 32] = 32'hAAAA_0000;
        rr_req_len_flat[0*8 +: 8]   = 8'd4;
        rr_req_type_flat[1*2 +: 2] = 2'(ARB_REQ_WRITE);
        rr_req_addr_flat[1*32 +: 32] = 32'hBBBB_0000;
        rr_req_len_flat[1*8 +: 8]   = 8'd8;
        // Next posedge: IDLE -> BUSY, grant ch0
        @(posedge clk);
        @(negedge clk);
        check_1b("ch0 granted first",  rr_grant[0], 1'b1);
        check_1b("ch1 not granted yet", rr_grant[1], 1'b0);
        // Pulse done to release ch0
        rr_done_pulse();
        // done_pulse returns at negedge. The posedge during the pulse moved state to IDLE.
        // Next posedge: IDLE sees ch1 pending (ptr advanced past ch0) -> grants ch1
        @(posedge clk);
        @(negedge clk);
        check_1b("ch1 granted second",  rr_grant[1], 1'b1);
        check_1b("ch0 not granted now", rr_grant[0], 1'b0);
        // Clean up
        rr_done_pulse();
        rr_req = '0;
        @(posedge clk);

        // ---- Test 4: Round-robin fairness ----
        $display("\n--- Test 4: Round-robin fairness ---");
        reset_duts();
        @(negedge clk);
        rr_req = 4'b1111; // all 4 channels request
        for (int i = 0; i < NUM_CH; i++) begin
            rr_req_type_flat[i*2 +: 2] = 2'(ARB_REQ_READ);
            rr_req_addr_flat[i*32 +: 32] = 32'h0000_1000 + (i * 32'h100);
            rr_req_len_flat[i*8 +: 8]   = i[7:0] + 8'd1;
        end
        // First grant: posedge sees IDLE + winner_found -> BUSY + grant
        @(posedge clk);
        @(negedge clk);
        check_1b("RR cycle 0: ch0 granted", rr_grant[0], 1'b1);
        // Subsequent cycles: done_pulse -> 1 posedge -> new grant
        for (int cycle = 1; cycle < 8; cycle++) begin
            integer exp_ch;
            exp_ch = cycle % NUM_CH;
            rr_done_pulse();
            @(posedge clk);
            @(negedge clk);
            check_1b($sformatf("RR cycle %0d: ch%0d granted", cycle, exp_ch),
                      rr_grant[exp_ch], 1'b1);
        end
        rr_done_pulse();
        rr_req = '0;
        @(posedge clk);

        // ---- Test 5: Hold-until-done ----
        $display("\n--- Test 5: Hold-until-done ---");
        reset_duts();
        @(negedge clk);
        rr_req          = 4'b0001;
        rr_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_WRITE);
        rr_req_addr_flat[0*32 +: 32] = 32'hF000_0000;
        rr_req_len_flat[0*8 +: 8]   = 8'd10;
        @(posedge clk);
        @(negedge clk);
        check_1b("grant held cycle 0", rr_grant[0], 1'b1);
        // Wait 5 more cycles without asserting done — grant must stay
        repeat (5) begin
            @(posedge clk);
            @(negedge clk);
            check_1b("grant still held", rr_grant[0], 1'b1);
        end
        // Now release
        rr_done_pulse();
        rr_req = '0;
        @(posedge clk);

        // ---- Test 6: Fixed-priority mode ----
        $display("\n--- Test 6: Fixed-priority mode ---");
        reset_duts();
        @(negedge clk);
        fp_req          = 4'b0101; // ch0 and ch2
        fp_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_READ);
        fp_req_addr_flat[0*32 +: 32] = 32'h1000_0000;
        fp_req_len_flat[0*8 +: 8]   = 8'd2;
        fp_req_type_flat[2*2 +: 2] = 2'(ARB_REQ_WRITE);
        fp_req_addr_flat[2*32 +: 32] = 32'h2000_0000;
        fp_req_len_flat[2*8 +: 8]   = 8'd4;
        // Ch0 should always win (lower index = higher priority)
        @(posedge clk);
        @(negedge clk);
        check_1b("FP: ch0 wins over ch2 (first)",  fp_grant[0], 1'b1);
        check_1b("FP: ch2 not granted (first)",    fp_grant[2], 1'b0);
        fp_done_pulse();
        @(posedge clk);
        @(negedge clk);
        check_1b("FP: ch0 wins over ch2 (second)", fp_grant[0], 1'b1);
        check_1b("FP: ch2 not granted (second)",   fp_grant[2], 1'b0);
        // Clean up
        fp_done_pulse();
        fp_req = '0;
        @(posedge clk);

        // ---- Test 7: Priority pointer update ----
        $display("\n--- Test 7: Priority pointer update ---");
        reset_duts();
        // First grant ch1 by requesting only ch1
        @(negedge clk);
        rr_req          = 4'b0010; // ch1 only
        rr_req_type_flat[1*2 +: 2] = 2'(ARB_REQ_READ);
        rr_req_addr_flat[1*32 +: 32] = 32'hCC00_0000;
        rr_req_len_flat[1*8 +: 8]   = 8'd3;
        @(posedge clk);
        @(negedge clk);
        check_1b("ch1 granted", rr_grant[1], 1'b1);
        // Clear ch1 request before done so arbiter doesn't re-grant ch1
        @(negedge clk);
        rr_req = '0;
        // Complete ch1 — priority_ptr advances to 2
        rr_done_pulse();
        // Now request ch0 and ch2 — ch2 should win (scanning from ptr=2)
        @(negedge clk);
        rr_req          = 4'b0101; // ch0 and ch2
        rr_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_READ);
        rr_req_addr_flat[0*32 +: 32] = 32'hDD00_0000;
        rr_req_len_flat[0*8 +: 8]   = 8'd1;
        rr_req_type_flat[2*2 +: 2] = 2'(ARB_REQ_WRITE);
        rr_req_addr_flat[2*32 +: 32] = 32'hEE00_0000;
        rr_req_len_flat[2*8 +: 8]   = 8'd2;
        @(posedge clk); // arbiter sees req in IDLE, transitions to BUSY
        @(negedge clk); // grant is now registered
        @(posedge clk); // ensure grant is stable
        @(negedge clk);
        check_1b("after ch1, ch2 wins over ch0", rr_grant[2], 1'b1);
        check_1b("ch0 not granted yet",          rr_grant[0], 1'b0);
        // Clean up
        rr_done_pulse();
        rr_req = '0;
        @(posedge clk);

        // ---- Test 8: Grant de-assertion ----
        $display("\n--- Test 8: Grant de-assertion ---");
        reset_duts();
        @(negedge clk);
        rr_req          = 4'b0001;
        rr_req_type_flat[0*2 +: 2] = 2'(ARB_REQ_DESC);
        rr_req_addr_flat[0*32 +: 32] = 32'h0000_0400;
        rr_req_len_flat[0*8 +: 8]   = 8'd5;
        @(posedge clk);
        @(negedge clk);
        check_1b("grant_valid=1 before done", rr_grant_valid, 1'b1);
        // Remove request so arbiter won't re-grant immediately after done
        rr_req = '0;
        // Pulse done
        rr_done_pulse();
        // After done, state returns to IDLE, grant cleared on that posedge.
        // No new req, so stays IDLE. Sample at negedge after one more posedge.
        @(posedge clk);
        @(negedge clk);
        check_1b("grant_valid=0 after done", rr_grant_valid, 1'b0);
        check("grant=0 after done", {28'd0, rr_grant}, 32'd0);

        // ---- Summary ----
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
