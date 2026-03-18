// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_top;
    import dma_pkg::*;

    // ================================================================
    // Parameters
    // ================================================================
    localparam int NUM_CH        = 4;
    localparam int DATA_W        = 32;
    localparam int ADDR_W        = 32;
    localparam int MAX_BURST_LEN = 16;
    localparam int CLK_PERIOD    = 10;
    localparam int MEM_WORDS     = 16384;  // 64KB

    // ================================================================
    // Clock and reset
    // ================================================================
    logic clk;
    logic srst;

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ================================================================
    // CPU register interface
    // ================================================================
    logic                reg_wr_en;
    logic                reg_rd_en;
    logic [11:0]         reg_addr;
    logic [DATA_W-1:0]   reg_wr_data;
    logic [DATA_W-1:0]   reg_rd_data;
    logic                reg_rd_valid;

    // ================================================================
    // AXI4 signals
    // ================================================================
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

    // ================================================================
    // Peripheral handshake and interrupt
    // ================================================================
    logic [NUM_CH-1:0]   dreq;
    logic [NUM_CH-1:0]   dack;
    logic                irq;

    // ================================================================
    // DUT instantiation
    // ================================================================
    dma_top #(
        .NUM_CH        (NUM_CH),
        .DATA_W        (DATA_W),
        .ADDR_W        (ADDR_W),
        .MAX_BURST_LEN (MAX_BURST_LEN),
        .ARB_MODE      ("ROUND_ROBIN")
    ) u_dut (
        .clk            (clk),
        .srst           (srst),
        .reg_wr_en      (reg_wr_en),
        .reg_rd_en      (reg_rd_en),
        .reg_addr       (reg_addr),
        .reg_wr_data    (reg_wr_data),
        .reg_rd_data    (reg_rd_data),
        .reg_rd_valid   (reg_rd_valid),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .dreq           (dreq),
        .dack           (dack),
        .irq            (irq)
    );

    // ================================================================
    // Test infrastructure
    // ================================================================
    int pass_count = 0;
    int fail_count = 0;
    int test_count = 0;

    task automatic check(input string name, input logic [31:0] got, input logic [31:0] expected);
        test_count++;
        if (got === expected) begin
            $display("[PASS] %s: got 0x%08h", name, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s: got 0x%08h, expected 0x%08h", name, got, expected);
            fail_count++;
        end
    endtask

    task automatic check_1b(input string name, input logic got, input logic expected);
        test_count++;
        if (got === expected) begin
            $display("[PASS] %s: got %0b", name, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s: got %0b, expected %0b", name, got, expected);
            fail_count++;
        end
    endtask

    // ================================================================
    // AXI4 slave memory model
    // ================================================================
    logic [31:0] axi_mem [0:16383];
    logic inject_axi_error;

    // -- Read channel state --
    logic        rd_active;
    logic [31:0] rd_addr_base;
    logic [7:0]  rd_beats_remaining;
    logic [7:0]  rd_beat_idx;

    // -- Write channel state --
    logic        wr_addr_captured;
    logic [31:0] wr_addr_base;
    logic [7:0]  wr_beats_total;
    logic [7:0]  wr_beat_idx;
    logic        wr_resp_pending;

    // arready and awready always high, wready always high
    assign m_axi_arready = 1'b1;
    assign m_axi_awready = 1'b1;
    assign m_axi_wready  = 1'b1;

    // -- AXI read channel FSM --
    always_ff @(posedge clk) begin
        if (srst) begin
            rd_active          <= 1'b0;
            rd_addr_base       <= '0;
            rd_beats_remaining <= '0;
            rd_beat_idx        <= '0;
            m_axi_rvalid       <= 1'b0;
            m_axi_rdata        <= '0;
            m_axi_rresp        <= 2'b00;
            m_axi_rlast        <= 1'b0;
        end else begin
            if (!rd_active) begin
                // Wait for a read address handshake
                if (m_axi_arvalid && m_axi_arready) begin
                    rd_active          <= 1'b1;
                    rd_addr_base       <= m_axi_araddr;
                    rd_beats_remaining <= m_axi_arlen;  // total beats = arlen+1, remaining after first = arlen
                    rd_beat_idx        <= '0;
                    // Drive first beat immediately
                    m_axi_rvalid       <= 1'b1;
                    m_axi_rdata        <= axi_mem[m_axi_araddr[15:2]];
                    m_axi_rresp        <= inject_axi_error ? 2'b10 : 2'b00;
                    m_axi_rlast        <= (m_axi_arlen == 8'd0);
                end else begin
                    m_axi_rvalid <= 1'b0;
                    m_axi_rlast  <= 1'b0;
                end
            end else begin
                // Active read burst
                if (m_axi_rvalid && m_axi_rready) begin
                    if (m_axi_rlast) begin
                        // Burst complete
                        rd_active    <= 1'b0;
                        m_axi_rvalid <= 1'b0;
                        m_axi_rlast  <= 1'b0;
                    end else begin
                        // Next beat
                        rd_beat_idx        <= rd_beat_idx + 8'd1;
                        rd_beats_remaining <= rd_beats_remaining - 8'd1;
                        m_axi_rdata        <= axi_mem[(rd_addr_base[15:2]) + rd_beat_idx + 1];
                        m_axi_rresp        <= inject_axi_error ? 2'b10 : 2'b00;
                        m_axi_rlast        <= (rd_beats_remaining == 8'd1);
                    end
                end
            end
        end
    end

    // -- AXI write channel FSM --
    always_ff @(posedge clk) begin
        if (srst) begin
            wr_addr_captured <= 1'b0;
            wr_addr_base     <= '0;
            wr_beats_total   <= '0;
            wr_beat_idx      <= '0;
            wr_resp_pending  <= 1'b0;
            m_axi_bvalid     <= 1'b0;
            m_axi_bresp      <= 2'b00;
        end else begin
            // Clear bvalid after accepted
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 1'b0;
            end

            // Capture write address
            if (m_axi_awvalid && m_axi_awready && !wr_addr_captured) begin
                wr_addr_base     <= m_axi_awaddr;
                wr_beats_total   <= m_axi_awlen;
                wr_beat_idx      <= '0;
                wr_addr_captured <= 1'b1;
            end

            // Accept write data
            if (wr_addr_captured && m_axi_wvalid && m_axi_wready) begin
                axi_mem[(wr_addr_base[15:2]) + wr_beat_idx] <= m_axi_wdata;
                wr_beat_idx <= wr_beat_idx + 8'd1;
                if (m_axi_wlast) begin
                    wr_addr_captured <= 1'b0;
                    wr_resp_pending  <= 1'b1;
                end
            end

            // Drive write response
            if (wr_resp_pending && !m_axi_bvalid) begin
                m_axi_bvalid    <= 1'b1;
                m_axi_bresp     <= inject_axi_error ? 2'b10 : 2'b00;
                wr_resp_pending <= 1'b0;
            end
        end
    end

    // ================================================================
    // Helper tasks
    // ================================================================

    task automatic reg_write(input logic [11:0] addr, input logic [31:0] data);
        @(posedge clk);
        reg_wr_en   <= 1'b1;
        reg_addr    <= addr;
        reg_wr_data <= data;
        @(posedge clk);
        reg_wr_en   <= 1'b0;
    endtask

    task automatic reg_read(input logic [11:0] addr, output logic [31:0] data);
        @(posedge clk);
        reg_rd_en <= 1'b1;
        reg_addr  <= addr;
        @(posedge clk);
        reg_rd_en <= 1'b0;
        // rd_valid appears 1 cycle after rd_en
        @(posedge clk);
        data = reg_rd_data;
    endtask

    function automatic logic [11:0] ch_base(input int ch);
        return ch[11:0] * 12'd64;
    endfunction

    // Register offsets within a channel block
    localparam logic [5:0] OFF_CH_CTRL      = 6'h00;
    localparam logic [5:0] OFF_CH_STATUS    = 6'h04;
    localparam logic [5:0] OFF_CH_SRC_ADDR  = 6'h08;
    localparam logic [5:0] OFF_CH_DST_ADDR  = 6'h0C;
    localparam logic [5:0] OFF_CH_XFER_LEN  = 6'h10;
    localparam logic [5:0] OFF_CH_DESC_ADDR = 6'h14;

    // Global register addresses
    localparam logic [11:0] ADDR_IRQ_STATUS = 12'h100;
    localparam logic [11:0] ADDR_IRQ_ENABLE = 12'h104;
    localparam logic [11:0] ADDR_IRQ_CLEAR  = 12'h108;
    localparam logic [11:0] ADDR_VERSION    = 12'h10C;

    task automatic start_channel(input int ch, input logic [31:0] src, input logic [31:0] dst, input logic [31:0] len);
        reg_write(ch_base(ch) + OFF_CH_SRC_ADDR, src);
        reg_write(ch_base(ch) + OFF_CH_DST_ADDR, dst);
        reg_write(ch_base(ch) + OFF_CH_XFER_LEN, len);
        reg_write(ch_base(ch) + OFF_CH_CTRL, 32'h03);  // enable=1, start=1
    endtask

    task automatic start_sg_channel(input int ch, input logic [31:0] desc_addr);
        reg_write(ch_base(ch) + OFF_CH_DESC_ADDR, desc_addr);
        reg_write(ch_base(ch) + OFF_CH_CTRL, 32'h0B);  // enable=1, start=1, sg_en=1
    endtask

    task automatic wait_channel_done(input int ch, input int timeout);
        logic [31:0] status_val;
        int cnt;
        logic done;
        cnt = 0;
        done = 0;
        while (!done) begin
            reg_read(ch_base(ch) + OFF_CH_STATUS, status_val);
            // Check state field OR sticky TC/ERR bits (state may already be IDLE)
            if (status_val[2:0] == CH_DONE || status_val[2:0] == CH_ERROR ||
                status_val[3] || status_val[4]) begin
                done = 1;
            end else begin
                cnt = cnt + 1;
                if (cnt >= timeout) begin
                    $display("[WARN] wait_channel_done ch%0d timed out after %0d reads", ch, timeout);
                    done = 1;
                end
            end
        end
    endtask

    task automatic load_descriptor(input logic [31:0] addr,
                                   input logic [31:0] src,
                                   input logic [31:0] dst,
                                   input logic [31:0] len,
                                   input logic [31:0] next,
                                   input logic [31:0] ctrl);
        // Descriptor layout in memory (5 words, matches dma_channel desc_buf ordering):
        // word 0 = ctrl, word 1 = xfer_len, word 2 = dst_addr,
        // word 3 = src_addr, word 4 = next_desc_addr
        axi_mem[addr[15:2] + 0] = ctrl;
        axi_mem[addr[15:2] + 1] = len;
        axi_mem[addr[15:2] + 2] = dst;
        axi_mem[addr[15:2] + 3] = src;
        axi_mem[addr[15:2] + 4] = next;
    endtask

    // ================================================================
    // Pre-load source data pattern
    // ================================================================
    task automatic preload_mem();
        int i;
        for (i = 0; i < MEM_WORDS; i++) begin
            axi_mem[i] = 32'hDADA0000 + i;
        end
    endtask

    // ================================================================
    // Reset task
    // ================================================================
    task automatic do_reset();
        srst        <= 1'b1;
        reg_wr_en   <= 1'b0;
        reg_rd_en   <= 1'b0;
        reg_addr    <= '0;
        reg_wr_data <= '0;
        dreq        <= '0;
        inject_axi_error <= 1'b0;
        repeat (5) @(posedge clk);
        srst <= 1'b0;
        repeat (2) @(posedge clk);
    endtask

    // ================================================================
    // Main test sequence
    // ================================================================
    initial begin
        $dumpfile("tb_dma_top.vcd");
        $dumpvars(0, tb_dma_top);

        preload_mem();
        do_reset();

        // ============================================================
        // Test 1: Reset
        // ============================================================
        $display("\n=== Test 1: Reset ===");
        check_1b("arvalid after reset", m_axi_arvalid, 1'b0);
        check_1b("awvalid after reset", m_axi_awvalid, 1'b0);
        check_1b("wvalid after reset",  m_axi_wvalid,  1'b0);
        check_1b("irq after reset",     irq,           1'b0);

        // ============================================================
        // Test 2: Register access
        // ============================================================
        $display("\n=== Test 2: Register access ===");
        begin
            logic [31:0] rdata;
            // Write and read back SRC_ADDR for ch0
            reg_write(ch_base(0) + OFF_CH_SRC_ADDR, 32'hDEAD_BEEF);
            reg_read(ch_base(0) + OFF_CH_SRC_ADDR, rdata);
            check("ch0 SRC readback", rdata, 32'hDEAD_BEEF);

            // Write and read back DST_ADDR for ch0
            reg_write(ch_base(0) + OFF_CH_DST_ADDR, 32'h1234_5678);
            reg_read(ch_base(0) + OFF_CH_DST_ADDR, rdata);
            check("ch0 DST readback", rdata, 32'h1234_5678);

            // Write and read back XFER_LEN for ch0
            reg_write(ch_base(0) + OFF_CH_XFER_LEN, 32'h00000040);
            reg_read(ch_base(0) + OFF_CH_XFER_LEN, rdata);
            check("ch0 LEN readback", rdata, 32'h00000040);

            // Read CTRL (enable bit)
            reg_write(ch_base(0) + OFF_CH_CTRL, 32'h01);  // enable only
            reg_read(ch_base(0) + OFF_CH_CTRL, rdata);
            check("ch0 CTRL readback enable", rdata[0], 1'b1);
        end

        // ============================================================
        // Test 3: Simple M2M transfer (1 burst, 16 words = 64 bytes)
        // ============================================================
        $display("\n=== Test 3: Simple M2M transfer (1 burst) ===");
        begin
            int i;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();  // Re-load after reset clears AXI slave state

            // src = 0x0000 (word 0..15), dst = 0x1000 (word 1024..1039)
            start_channel(0, 32'h0000_0000, 32'h0000_1000, 32'd64);
            wait_channel_done(0, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("ch0 tc bit set", {31'b0, rdata[3]}, 32'h1);

            // Verify data
            begin
                int errs;
                errs = 0;
                for (i = 0; i < 16; i++) begin
                    if (axi_mem[1024 + i] !== (32'hDADA0000 + i)) errs = errs + 1;
                end
                check("M2M 16-word copy", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 4: Multi-burst M2M transfer (256 bytes = 64 words = 4 bursts)
        // ============================================================
        $display("\n=== Test 4: Multi-burst M2M transfer ===");
        begin
            int i;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            // src = 0x0000 (word 0..63), dst = 0x2000 (word 2048..2111)
            start_channel(0, 32'h0000_0000, 32'h0000_2000, 32'd256);
            wait_channel_done(0, 10000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("ch0 tc bit set multi-burst", {31'b0, rdata[3]}, 32'h1);

            begin
                int errs;
                errs = 0;
                for (i = 0; i < 64; i++) begin
                    if (axi_mem[2048 + i] !== (32'hDADA0000 + i)) errs = errs + 1;
                end
                check("M2M 64-word copy", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 5: Two channels simultaneously
        // ============================================================
        $display("\n=== Test 5: Two channels simultaneously ===");
        begin
            int i;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            // ch0: src=0x0000 -> dst=0x3000, 64 bytes
            // ch1: src=0x0100 -> dst=0x4000, 64 bytes
            start_channel(0, 32'h0000_0000, 32'h0000_3000, 32'd64);
            start_channel(1, 32'h0000_0100, 32'h0000_4000, 32'd64);

            wait_channel_done(0, 5000);
            wait_channel_done(1, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("ch0 two-ch tc", {31'b0, rdata[3]}, 32'h1);
            reg_read(ch_base(1) + OFF_CH_STATUS, rdata);
            check("ch1 two-ch tc", {31'b0, rdata[3]}, 32'h1);

            begin
                int errs;
                errs = 0;
                for (i = 0; i < 16; i++) begin
                    // ch0 dst word index: 0x3000/4 = 3072
                    if (axi_mem[3072 + i] !== (32'hDADA0000 + i)) errs = errs + 1;
                    // ch1 src word index: 0x0100/4 = 64, dst word index: 0x4000/4 = 4096
                    if (axi_mem[4096 + i] !== (32'hDADA0000 + 64 + i)) errs = errs + 1;
                end
                check("Two-ch data verify", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 6: All 4 channels simultaneously
        // ============================================================
        $display("\n=== Test 6: All 4 channels simultaneously ===");
        begin
            int i, ch;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            // ch0: 0x0000->0x5000, ch1: 0x0100->0x5100, ch2: 0x0200->0x5200, ch3: 0x0300->0x5300
            // Each transfers 64 bytes (16 words)
            for (ch = 0; ch < 4; ch++) begin
                start_channel(ch, ch * 32'h100, 32'h5000 + ch * 32'h100, 32'd64);
            end

            for (ch = 0; ch < 4; ch++) begin
                wait_channel_done(ch, 10000);
            end

            begin
                int errs;
                errs = 0;
                for (ch = 0; ch < 4; ch++) begin
                    reg_read(ch_base(ch) + OFF_CH_STATUS, rdata);
                    if (!rdata[3]) errs = errs + 1; // Check TC bit
                    for (i = 0; i < 16; i++) begin
                        // src word index = ch*64 + i, dst word index = 0x5000/4 + ch*64 + i
                        if (axi_mem[5120 + ch*64 + i] !== (32'hDADA0000 + ch*64 + i))
                            errs = errs + 1;
                    end
                end
                check("4-ch all data verify", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 7: Scatter-gather (2 descriptors)
        // ============================================================
        $display("\n=== Test 7: Scatter-gather (2 descriptors) ===");
        begin
            int i;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            // Descriptor 0 at 0x8000: src=0x0000, dst=0x6000, len=16 bytes (4 words), next=0x8014
            // Descriptor 1 at 0x8014: src=0x0010, dst=0x6010, len=16 bytes (4 words), next=0x0000 (end)
            // desc ctrl: enable=1 => 0x01
            load_descriptor(32'h8000, 32'h0000_0000, 32'h0000_6000, 32'd16, 32'h0000_8014, 32'h01);
            load_descriptor(32'h8014, 32'h0000_0010, 32'h0000_6010, 32'd16, 32'h0000_0000, 32'h01);

            start_sg_channel(0, 32'h0000_8000);
            wait_channel_done(0, 10000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("SG 2-desc tc", {31'b0, rdata[3]}, 32'h1);

            begin
                int errs;
                errs = 0;
                // First descriptor: 4 words from word 0 to word 0x6000/4=6144
                for (i = 0; i < 4; i++) begin
                    if (axi_mem[6144 + i] !== (32'hDADA0000 + i)) errs = errs + 1;
                end
                // Second descriptor: 4 words from word 0x0010/4=4 to word 0x6010/4=6148
                for (i = 0; i < 4; i++) begin
                    if (axi_mem[6148 + i] !== (32'hDADA0000 + 4 + i)) errs = errs + 1;
                end
                check("SG 2-desc data verify", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 8: Scatter-gather (4 descriptors)
        // ============================================================
        $display("\n=== Test 8: Scatter-gather (4 descriptors) ===");
        begin
            int i, d;
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            // 4 descriptors at 0x9000, spaced by 0x14 (5 words = 20 bytes)
            // Each transfers 16 bytes (4 words)
            for (d = 0; d < 4; d++) begin
                logic [31:0] desc_a, src_a, dst_a, next_a, ctrl_a;
                desc_a = 32'h9000 + d * 32'h14;
                src_a  = d * 32'h10;                // 0x00, 0x10, 0x20, 0x30
                dst_a  = 32'h7000 + d * 32'h10;    // 0x7000, 0x7010, 0x7020, 0x7030
                next_a = (d < 3) ? (32'h9000 + (d + 1) * 32'h14) : 32'h0000_0000;
                ctrl_a = 32'h01;  // enable
                load_descriptor(desc_a, src_a, dst_a, 32'd16, next_a, ctrl_a);
            end

            start_sg_channel(0, 32'h0000_9000);
            wait_channel_done(0, 20000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("SG 4-desc tc", {31'b0, rdata[3]}, 32'h1);

            begin
                int errs;
                errs = 0;
                for (d = 0; d < 4; d++) begin
                    for (i = 0; i < 4; i++) begin
                        // src word = d*4 + i, dst word = 0x7000/4 + d*4 + i = 7168 + d*4 + i
                        if (axi_mem[7168 + d*4 + i] !== (32'hDADA0000 + d*4 + i))
                            errs = errs + 1;
                    end
                end
                check("SG 4-desc data verify", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 9: AXI error propagation
        // ============================================================
        $display("\n=== Test 9: AXI error propagation ===");
        begin
            logic [31:0] rdata;
            preload_mem();
            do_reset();
            preload_mem();

            inject_axi_error <= 1'b1;
            start_channel(0, 32'h0000_0000, 32'h0000_A000, 32'd64);
            wait_channel_done(0, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("AXI error ch0 status", rdata[2:0], CH_ERROR);
            check_1b("AXI error irq asserted", irq, 1'b0);  // IRQ not enabled yet, should be 0

            // Enable error IRQ for ch0 and check
            inject_axi_error <= 1'b0;
            do_reset();
            preload_mem();

            // Enable error IRQ for ch0: bit 16 of IRQ_ENABLE
            reg_write(ADDR_IRQ_ENABLE, 32'h0001_0000);
            inject_axi_error <= 1'b1;
            start_channel(0, 32'h0000_0000, 32'h0000_A000, 32'd64);
            wait_channel_done(0, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("AXI error ch0 status with irq_en", rdata[2:0], CH_ERROR);
            check_1b("AXI error irq with err_en", irq, 1'b1);

            inject_axi_error <= 1'b0;
        end

        // ============================================================
        // Test 10: Interrupt enable/clear
        // ============================================================
        $display("\n=== Test 10: Interrupt enable/clear ===");
        begin
            logic [31:0] rdata;
            do_reset();
            preload_mem();

            // Enable TC IRQ for ch0: bit 0 of IRQ_ENABLE
            reg_write(ADDR_IRQ_ENABLE, 32'h0000_0001);

            start_channel(0, 32'h0000_0000, 32'h0000_B000, 32'd64);
            wait_channel_done(0, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("IRQ test ch0 tc", {31'b0, rdata[3]}, 32'h1);
            // Wait a few cycles for irq to propagate
            repeat (3) @(posedge clk);
            check_1b("IRQ asserted after TC", irq, 1'b1);

            // Read IRQ status
            reg_read(ADDR_IRQ_STATUS, rdata);
            check("IRQ_STATUS tc bit0", rdata[0], 1'b1);

            // Clear IRQ by writing 1 to bit 0 of IRQ_CLEAR
            reg_write(ADDR_IRQ_CLEAR, 32'h0000_0001);
            repeat (2) @(posedge clk);
            check_1b("IRQ cleared after write", irq, 1'b0);

            // Now test with IRQ disabled: disable TC IRQ
            reg_write(ADDR_IRQ_ENABLE, 32'h0000_0000);
            // Need to re-start: abort first, then restart
            reg_write(ch_base(0) + OFF_CH_CTRL, 32'h04);  // abort
            repeat (5) @(posedge clk);
            start_channel(0, 32'h0000_0000, 32'h0000_C000, 32'd64);
            wait_channel_done(0, 5000);
            repeat (3) @(posedge clk);
            check_1b("IRQ not fired when disabled", irq, 1'b0);
        end

        // ============================================================
        // Test 11: Abort then restart
        // ============================================================
        $display("\n=== Test 11: Abort then restart ===");
        begin
            logic [31:0] rdata;
            int i;
            do_reset();
            preload_mem();

            // Start a transfer
            start_channel(0, 32'h0000_0000, 32'h0000_D000, 32'd256);

            // Wait a few cycles then abort
            repeat (20) @(posedge clk);
            reg_write(ch_base(0) + OFF_CH_CTRL, 32'h04);  // abort
            repeat (10) @(posedge clk);

            // Check channel is idle after abort
            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("ch0 IDLE after abort", rdata[2:0], CH_IDLE);

            // Full reset to clear arbiter/AXI master state, then restart
            do_reset();
            preload_mem();
            start_channel(0, 32'h0000_0000, 32'h0000_E000, 32'd64);
            wait_channel_done(0, 5000);

            reg_read(ch_base(0) + OFF_CH_STATUS, rdata);
            check("ch0 tc after restart", {31'b0, rdata[3]}, 32'h1);

            begin
                int errs;
                errs = 0;
                for (i = 0; i < 16; i++) begin
                    // dst word index: 0xE000/4 = 14336
                    if (axi_mem[14336 + i] !== (32'hDADA0000 + i)) errs = errs + 1;
                end
                check("Abort-restart data verify", errs[31:0], 32'd0);
            end
        end

        // ============================================================
        // Test 12: Version register
        // ============================================================
        $display("\n=== Test 12: Version register ===");
        begin
            logic [31:0] rdata;
            reg_read(ADDR_VERSION, rdata);
            check("VERSION register", rdata, 32'h0001_0000);
        end

        // ============================================================
        // Summary
        // ============================================================
        $display("\n========================================");
        $display("  RESULTS: %0d / %0d passed", pass_count, test_count);
        if (fail_count == 0)
            $display("  STATUS:  ALL TESTS PASSED");
        else
            $display("  STATUS:  %0d TESTS FAILED", fail_count);
        $display("========================================\n");

        $finish;
    end

    // Global timeout
    initial begin
        #5_000_000;
        $display("[TIMEOUT] Simulation exceeded 5ms limit");
        $finish;
    end

endmodule
