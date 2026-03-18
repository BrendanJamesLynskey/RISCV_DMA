// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_axi_master;
    import dma_pkg::*;

    // Parameters
    localparam DATA_W     = 32;
    localparam ADDR_W     = 32;
    localparam CLK_PERIOD = 10;
    localparam MEM_DEPTH  = 1024;

    // ----------------------------------------------------------------
    // Clock and reset
    // ----------------------------------------------------------------
    logic clk;
    logic srst;

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // Request interface signals
    // ----------------------------------------------------------------
    arb_req_type_t       req_type;
    logic [ADDR_W-1:0]   req_addr;
    logic [7:0]          req_len;
    logic                req_valid;
    logic                req_done;
    logic                req_error;
    logic [1:0]          req_resp;

    // ----------------------------------------------------------------
    // Data interface signals (read side)
    // ----------------------------------------------------------------
    logic                rd_valid;
    logic [DATA_W-1:0]   rd_data;
    logic                rd_last;
    logic [1:0]          rd_resp;
    logic                rd_ready;

    // ----------------------------------------------------------------
    // Data interface signals (write side)
    // ----------------------------------------------------------------
    logic                wr_valid;
    logic [DATA_W-1:0]   wr_data;
    logic                wr_last;
    logic                wr_ready;
    logic                wr_resp_valid;
    logic [1:0]          wr_resp;

    // ----------------------------------------------------------------
    // AXI4 signals
    // ----------------------------------------------------------------
    logic                m_axi_awvalid;
    logic                m_axi_awready;
    logic [ADDR_W-1:0]   m_axi_awaddr;
    logic [7:0]          m_axi_awlen;
    logic [2:0]          m_axi_awsize;
    logic [1:0]          m_axi_awburst;
    logic                m_axi_wvalid;
    logic                m_axi_wready;
    logic [DATA_W-1:0]   m_axi_wdata;
    logic [DATA_W/8-1:0] m_axi_wstrb;
    logic                m_axi_wlast;
    logic                m_axi_bvalid;
    logic                m_axi_bready;
    logic [1:0]          m_axi_bresp;
    logic                m_axi_arvalid;
    logic                m_axi_arready;
    logic [ADDR_W-1:0]   m_axi_araddr;
    logic [7:0]          m_axi_arlen;
    logic [2:0]          m_axi_arsize;
    logic [1:0]          m_axi_arburst;
    logic                m_axi_rvalid;
    logic                m_axi_rready;
    logic [DATA_W-1:0]   m_axi_rdata;
    logic [1:0]          m_axi_rresp;
    logic                m_axi_rlast;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    dma_axi_master #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W)
    ) u_dut (
        .clk              (clk),
        .srst             (srst),
        .req_type         (req_type),
        .req_addr         (req_addr),
        .req_len          (req_len),
        .req_valid        (req_valid),
        .req_done         (req_done),
        .req_error        (req_error),
        .req_resp         (req_resp),
        .rd_valid         (rd_valid),
        .rd_data          (rd_data),
        .rd_last          (rd_last),
        .rd_resp          (rd_resp),
        .rd_ready         (rd_ready),
        .wr_valid         (wr_valid),
        .wr_data          (wr_data),
        .wr_last          (wr_last),
        .wr_ready         (wr_ready),
        .wr_resp_valid    (wr_resp_valid),
        .wr_resp          (wr_resp),
        .m_axi_awvalid    (m_axi_awvalid),
        .m_axi_awready    (m_axi_awready),
        .m_axi_awaddr     (m_axi_awaddr),
        .m_axi_awlen      (m_axi_awlen),
        .m_axi_awsize     (m_axi_awsize),
        .m_axi_awburst    (m_axi_awburst),
        .m_axi_wvalid     (m_axi_wvalid),
        .m_axi_wready     (m_axi_wready),
        .m_axi_wdata      (m_axi_wdata),
        .m_axi_wstrb      (m_axi_wstrb),
        .m_axi_wlast      (m_axi_wlast),
        .m_axi_bvalid     (m_axi_bvalid),
        .m_axi_bready     (m_axi_bready),
        .m_axi_bresp      (m_axi_bresp),
        .m_axi_arvalid    (m_axi_arvalid),
        .m_axi_arready    (m_axi_arready),
        .m_axi_araddr     (m_axi_araddr),
        .m_axi_arlen      (m_axi_arlen),
        .m_axi_arsize     (m_axi_arsize),
        .m_axi_arburst    (m_axi_arburst),
        .m_axi_rvalid     (m_axi_rvalid),
        .m_axi_rready     (m_axi_rready),
        .m_axi_rdata      (m_axi_rdata),
        .m_axi_rresp      (m_axi_rresp),
        .m_axi_rlast      (m_axi_rlast)
    );

    // ----------------------------------------------------------------
    // AXI4 Slave Memory Model
    // ----------------------------------------------------------------
    reg [DATA_W-1:0] mem [0:MEM_DEPTH-1];

    // Error injection controls
    logic inject_read_error;
    logic inject_write_error;
    logic inject_write_decerr;

    // Slave stall controls
    logic slave_stall_ar;
    logic slave_stall_aw;
    logic slave_stall_w;

    // Read channel state
    reg        rd_active;
    reg [ADDR_W-1:0] rd_addr_base;
    reg [7:0]  rd_beat_cnt;
    reg [7:0]  rd_beat_total;

    // Write channel state
    reg        aw_done;
    reg [ADDR_W-1:0] wr_addr_base;
    reg [7:0]  wr_beat_cnt;
    reg [7:0]  wr_beat_total;
    reg        b_pending;

    // AR channel: arready
    assign m_axi_arready = !slave_stall_ar && !rd_active;

    // AW channel: awready
    assign m_axi_awready = !slave_stall_aw && !aw_done && !b_pending;

    // W channel: wready
    assign m_axi_wready = !slave_stall_w && aw_done && !b_pending;

    // Read data channel
    assign m_axi_rvalid = rd_active;
    assign m_axi_rdata  = mem[(rd_addr_base >> 2) + rd_beat_cnt];
    assign m_axi_rlast  = rd_active && (rd_beat_cnt == rd_beat_total);
    assign m_axi_rresp  = inject_read_error ? 2'b10 : 2'b00; // SLVERR or OKAY

    // Write response channel
    assign m_axi_bvalid = b_pending;
    assign m_axi_bresp  = inject_write_decerr ? 2'b11 :
                           inject_write_error  ? 2'b10 : 2'b00;

    // Read channel FSM
    always @(posedge clk) begin
        if (srst) begin
            rd_active     <= 1'b0;
            rd_addr_base  <= '0;
            rd_beat_cnt   <= 8'd0;
            rd_beat_total <= 8'd0;
        end else begin
            if (!rd_active && m_axi_arvalid && m_axi_arready) begin
                // Capture read request — 1 cycle latency before first data
                rd_active     <= 1'b1;
                rd_addr_base  <= m_axi_araddr;
                rd_beat_cnt   <= 8'd0;
                rd_beat_total <= m_axi_arlen;
            end else if (rd_active && m_axi_rvalid && m_axi_rready) begin
                if (m_axi_rlast) begin
                    rd_active <= 1'b0;
                end else begin
                    rd_beat_cnt <= rd_beat_cnt + 8'd1;
                end
            end
        end
    end

    // Write channel FSM
    always @(posedge clk) begin
        if (srst) begin
            aw_done       <= 1'b0;
            wr_addr_base  <= '0;
            wr_beat_cnt   <= 8'd0;
            wr_beat_total <= 8'd0;
            b_pending     <= 1'b0;
        end else begin
            // AW handshake
            if (!aw_done && !b_pending && m_axi_awvalid && m_axi_awready) begin
                aw_done       <= 1'b1;
                wr_addr_base  <= m_axi_awaddr;
                wr_beat_cnt   <= 8'd0;
                wr_beat_total <= m_axi_awlen;
            end

            // W data beats
            if (aw_done && !b_pending && m_axi_wvalid && m_axi_wready) begin
                mem[(wr_addr_base >> 2) + wr_beat_cnt] <= m_axi_wdata;
                if (m_axi_wlast) begin
                    aw_done   <= 1'b0;
                    b_pending <= 1'b1;
                end else begin
                    wr_beat_cnt <= wr_beat_cnt + 8'd1;
                end
            end

            // B handshake
            if (b_pending && m_axi_bvalid && m_axi_bready) begin
                b_pending <= 1'b0;
            end
        end
    end

    // ----------------------------------------------------------------
    // Pre-load slave memory with known pattern
    // ----------------------------------------------------------------
    initial begin
        for (int i = 0; i < MEM_DEPTH; i++) begin
            mem[i] = 32'hAA000000 + i * 4;
        end
    end

    // ----------------------------------------------------------------
    // Test infrastructure
    // ----------------------------------------------------------------
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

    // ----------------------------------------------------------------
    // Helper tasks
    // ----------------------------------------------------------------
    task automatic reset_dut();
        req_type           = ARB_REQ_NONE;
        req_addr           = '0;
        req_len            = 8'd0;
        req_valid          = 1'b0;
        rd_ready           = 1'b1;
        wr_valid           = 1'b0;
        wr_data            = '0;
        wr_last            = 1'b0;
        inject_read_error  = 1'b0;
        inject_write_error = 1'b0;
        inject_write_decerr = 1'b0;
        slave_stall_ar     = 1'b0;
        slave_stall_aw     = 1'b0;
        slave_stall_w      = 1'b0;
        srst               = 1'b1;
        repeat (3) @(posedge clk);
        srst = 1'b0;
        @(posedge clk);
    endtask

    // Issue a request for 1 cycle
    task automatic issue_req(input arb_req_type_t rtype, input logic [ADDR_W-1:0] addr, input logic [7:0] len);
        @(posedge clk);
        req_type  = rtype;
        req_addr  = addr;
        req_len   = len;
        req_valid = 1'b1;
        @(posedge clk);
        req_valid = 1'b0;
        req_type  = ARB_REQ_NONE;
    endtask

    // Wait for req_done with timeout
    task automatic wait_done(input int timeout_cycles);
        int cnt;
        cnt = 0;
        while (!req_done && cnt < timeout_cycles) begin
            @(posedge clk);
            cnt++;
        end
        if (cnt >= timeout_cycles)
            $display("[WARN] wait_done timed out after %0d cycles", timeout_cycles);
    endtask

    // Drive write data beats from testbench
    // Present data on negedge so it is stable at the following posedge where
    // the DUT samples wr_valid/wr_data/wr_last together with wr_ready.
    task automatic drive_write_data(input logic [ADDR_W-1:0] base_addr, input int num_beats,
                                     input logic [DATA_W-1:0] data_base);
        int beat;
        beat = 0;
        while (beat < num_beats) begin
            @(negedge clk);
            wr_valid = 1'b1;
            wr_data  = data_base + beat * 4;
            wr_last  = (beat == num_beats - 1) ? 1'b1 : 1'b0;
            @(posedge clk);
            // Check handshake: if wr_ready was high at this posedge, beat accepted
            if (wr_ready) begin
                beat++;
            end
            // Otherwise loop and re-present same beat
        end
        @(negedge clk);
        wr_valid = 1'b0;
        wr_last  = 1'b0;
        wr_data  = '0;
    endtask

    // Collect read data beats
    int rd_collect_idx;
    logic [DATA_W-1:0] rd_collected [0:255];
    int rd_collected_count;

    task automatic collect_read_data(input int expected_beats);
        rd_collected_count = 0;
        while (rd_collected_count < expected_beats) begin
            @(posedge clk);
            if (rd_valid && rd_ready) begin
                rd_collected[rd_collected_count] = rd_data;
                rd_collected_count++;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Main test sequence
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("tb_dma_axi_master.vcd");
        $dumpvars(0, tb_dma_axi_master);

        // ==============================================================
        // Test 1: Single-beat read
        // ==============================================================
        $display("\n--- Test 1: Single-beat read ---");
        reset_dut();
        rd_ready = 1'b1;
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0000, 8'd0);
            collect_read_data(1);
        join
        wait_done(50);
        check("T1 read data[0]", rd_collected[0], 32'hAA000000);
        check_1b("T1 req_done", req_done, 1'b1);
        check_1b("T1 req_error", req_error, 1'b0);
        @(posedge clk);

        // ==============================================================
        // Test 2: Burst read (4 beats)
        // ==============================================================
        $display("\n--- Test 2: Burst read (4 beats) ---");
        reset_dut();
        rd_ready = 1'b1;
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0010, 8'd3);
            collect_read_data(4);
        join
        wait_done(50);
        check("T2 read data[0]", rd_collected[0], mem[32'h10 >> 2]);
        check("T2 read data[1]", rd_collected[1], mem[(32'h10 >> 2) + 1]);
        check("T2 read data[2]", rd_collected[2], mem[(32'h10 >> 2) + 2]);
        check("T2 read data[3]", rd_collected[3], mem[(32'h10 >> 2) + 3]);
        check_1b("T2 req_error", req_error, 1'b0);
        @(posedge clk);

        // ==============================================================
        // Test 3: Max burst read (16 beats)
        // ==============================================================
        $display("\n--- Test 3: Max burst read (16 beats) ---");
        reset_dut();
        rd_ready = 1'b1;
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0040, 8'd15);
            collect_read_data(16);
        join
        wait_done(100);
        begin
            int ok;
            ok = 1;
            for (int i = 0; i < 16; i++) begin
                if (rd_collected[i] !== mem[(32'h40 >> 2) + i]) ok = 0;
            end
            check_1b("T3 all 16 beats correct", ok[0], 1'b1);
        end
        check_1b("T3 req_error", req_error, 1'b0);
        @(posedge clk);

        // ==============================================================
        // Test 4: Single-beat write
        // ==============================================================
        $display("\n--- Test 4: Single-beat write ---");
        reset_dut();
        // Clear target location
        mem[32'h100 >> 2] = 32'h0;
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0100, 8'd0);
            drive_write_data(32'h0000_0100, 1, 32'hBB000000);
        join
        wait_done(50);
        check_1b("T4 req_done", req_done, 1'b1);
        check_1b("T4 req_error", req_error, 1'b0);
        check("T4 mem[0x100>>2]", mem[32'h100 >> 2], 32'hBB000000);
        @(posedge clk);

        // ==============================================================
        // Test 5: Burst write (4 beats)
        // ==============================================================
        $display("\n--- Test 5: Burst write (4 beats) ---");
        reset_dut();
        for (int i = 0; i < 4; i++) mem[(32'h200 >> 2) + i] = 32'h0;
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0200, 8'd3);
            drive_write_data(32'h0000_0200, 4, 32'hCC000000);
        join
        wait_done(50);
        check_1b("T5 req_done", req_done, 1'b1);
        check_1b("T5 req_error", req_error, 1'b0);
        check("T5 mem[0]", mem[32'h200 >> 2],       32'hCC000000);
        check("T5 mem[1]", mem[(32'h200 >> 2) + 1], 32'hCC000004);
        check("T5 mem[2]", mem[(32'h200 >> 2) + 2], 32'hCC000008);
        check("T5 mem[3]", mem[(32'h200 >> 2) + 3], 32'hCC00000C);
        @(posedge clk);

        // ==============================================================
        // Test 6: Max burst write (16 beats)
        // ==============================================================
        $display("\n--- Test 6: Max burst write (16 beats) ---");
        reset_dut();
        for (int i = 0; i < 16; i++) mem[(32'h300 >> 2) + i] = 32'h0;
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0300, 8'd15);
            drive_write_data(32'h0000_0300, 16, 32'hDD000000);
        join
        wait_done(100);
        check_1b("T6 req_done", req_done, 1'b1);
        check_1b("T6 req_error", req_error, 1'b0);
        begin
            int ok;
            ok = 1;
            for (int i = 0; i < 16; i++) begin
                if (mem[(32'h300 >> 2) + i] !== (32'hDD000000 + i * 4)) ok = 0;
            end
            check_1b("T6 all 16 beats written correctly", ok[0], 1'b1);
        end
        @(posedge clk);

        // ==============================================================
        // Test 7: Read with SLVERR
        // ==============================================================
        $display("\n--- Test 7: Read with SLVERR ---");
        reset_dut();
        inject_read_error = 1'b1;
        rd_ready = 1'b1;
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0000, 8'd0);
            collect_read_data(1);
        join
        wait_done(50);
        check_1b("T7 req_done", req_done, 1'b1);
        check_1b("T7 req_error", req_error, 1'b1);
        check("T7 req_resp SLVERR", {30'd0, req_resp}, {30'd0, 2'b10});
        inject_read_error = 1'b0;
        @(posedge clk);

        // ==============================================================
        // Test 8: Write with DECERR
        // ==============================================================
        $display("\n--- Test 8: Write with DECERR ---");
        reset_dut();
        inject_write_decerr = 1'b1;
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0400, 8'd0);
            drive_write_data(32'h0000_0400, 1, 32'hEE000000);
        join
        wait_done(50);
        check_1b("T8 req_done", req_done, 1'b1);
        check_1b("T8 req_error", req_error, 1'b1);
        check("T8 req_resp DECERR", {30'd0, req_resp}, {30'd0, 2'b11});
        inject_write_decerr = 1'b0;
        @(posedge clk);

        // ==============================================================
        // Test 9: Back-to-back reads
        // ==============================================================
        $display("\n--- Test 9: Back-to-back reads ---");
        reset_dut();
        rd_ready = 1'b1;
        // First read
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0000, 8'd0);
            collect_read_data(1);
        join
        wait_done(50);
        check_1b("T9 first req_done", req_done, 1'b1);
        check("T9 first read data", rd_collected[0], 32'hAA000000);
        // Wait for req_done to deassert
        @(posedge clk);
        // Second read
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0004, 8'd0);
            collect_read_data(1);
        join
        wait_done(50);
        check_1b("T9 second req_done", req_done, 1'b1);
        check("T9 second read data", rd_collected[0], 32'hAA000004);
        @(posedge clk);

        // ==============================================================
        // Test 10: Back-to-back writes
        // ==============================================================
        $display("\n--- Test 10: Back-to-back writes ---");
        reset_dut();
        mem[32'h500 >> 2] = 32'h0;
        mem[32'h504 >> 2] = 32'h0;
        // First write
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0500, 8'd0);
            drive_write_data(32'h0000_0500, 1, 32'hF1000000);
        join
        wait_done(50);
        check_1b("T10 first req_done", req_done, 1'b1);
        check("T10 first write mem", mem[32'h500 >> 2], 32'hF1000000);
        @(posedge clk);
        // Second write
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0504, 8'd0);
            drive_write_data(32'h0000_0504, 1, 32'hF2000000);
        join
        wait_done(50);
        check_1b("T10 second req_done", req_done, 1'b1);
        check("T10 second write mem", mem[32'h504 >> 2], 32'hF2000000);
        @(posedge clk);

        // ==============================================================
        // Test 11: Read then write
        // ==============================================================
        $display("\n--- Test 11: Read then write ---");
        reset_dut();
        rd_ready = 1'b1;
        mem[32'h600 >> 2] = 32'h0;
        // Read first
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0000, 8'd0);
            collect_read_data(1);
        join
        wait_done(50);
        check_1b("T11 read req_done", req_done, 1'b1);
        check("T11 read data", rd_collected[0], 32'hAA000000);
        @(posedge clk);
        // Then write
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0600, 8'd0);
            drive_write_data(32'h0000_0600, 1, 32'hAB000000);
        join
        wait_done(50);
        check_1b("T11 write req_done", req_done, 1'b1);
        check("T11 write mem", mem[32'h600 >> 2], 32'hAB000000);
        @(posedge clk);

        // ==============================================================
        // Test 12: Slave stall
        // ==============================================================
        $display("\n--- Test 12: Slave stall ---");
        reset_dut();
        rd_ready = 1'b1;
        // Stall AR for several cycles
        slave_stall_ar = 1'b1;
        fork
            issue_req(ARB_REQ_READ, 32'h0000_0000, 8'd0);
            begin
                // Release stall after 5 cycles
                repeat (5) @(posedge clk);
                slave_stall_ar = 1'b0;
            end
            collect_read_data(1);
        join
        wait_done(50);
        check_1b("T12 read completes after AR stall", req_done, 1'b1);
        check("T12 stalled read data", rd_collected[0], 32'hAA000000);
        @(posedge clk);

        // Also test write stall
        reset_dut();
        mem[32'h700 >> 2] = 32'h0;
        slave_stall_w = 1'b1;
        fork
            issue_req(ARB_REQ_WRITE, 32'h0000_0700, 8'd0);
            drive_write_data(32'h0000_0700, 1, 32'hAC000000);
            begin
                repeat (5) @(posedge clk);
                slave_stall_w = 1'b0;
            end
        join
        wait_done(50);
        check_1b("T12 write completes after W stall", req_done, 1'b1);
        check("T12 stalled write mem", mem[32'h700 >> 2], 32'hAC000000);
        @(posedge clk);

        // ---- Summary ----
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
