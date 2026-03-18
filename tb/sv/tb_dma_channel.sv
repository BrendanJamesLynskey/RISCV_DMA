// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_channel;
    import dma_pkg::*;

    // Parameters
    localparam int DATA_W        = 32;
    localparam int ADDR_W        = 32;
    localparam int MAX_BURST_LEN = 16;
    localparam int CLK_PERIOD    = 10;
    localparam int MEM_WORDS     = 4096;  // 16KB

    // Clock and reset
    logic clk;
    logic srst;

    // Configuration inputs
    logic                cfg_enable;
    logic                cfg_start;
    logic                cfg_abort;
    logic                cfg_sg_en;
    xfer_type_t          cfg_xfer_type;
    logic [ADDR_W-1:0]   cfg_src_addr;
    logic [ADDR_W-1:0]   cfg_dst_addr;
    logic [31:0]         cfg_xfer_len;
    logic [ADDR_W-1:0]   cfg_desc_addr;

    // Status outputs
    ch_status_t          status;
    logic [ADDR_W-1:0]   cur_src_addr;
    logic [ADDR_W-1:0]   cur_dst_addr;
    logic                tc_pulse;
    logic                err_pulse;

    // Arbiter interface
    logic                arb_req;
    arb_req_type_t       arb_req_type;
    logic [ADDR_W-1:0]   arb_addr;
    logic [7:0]          arb_len;
    logic                arb_grant;

    // AXI read data path
    logic                axi_rd_valid;
    logic [DATA_W-1:0]   axi_rd_data;
    logic                axi_rd_last;
    logic [1:0]          axi_rd_resp;
    logic                axi_rd_ready;

    // AXI write data path
    logic                axi_wr_valid;
    logic [DATA_W-1:0]   axi_wr_data;
    logic                axi_wr_last;
    logic                axi_wr_ready;
    logic                axi_wr_resp_valid;
    logic [1:0]          axi_wr_resp;

    // Peripheral handshake
    logic                dreq;
    logic                dack;

    // DUT
    dma_channel #(
        .CH_ID         (0),
        .DATA_W        (DATA_W),
        .ADDR_W        (ADDR_W),
        .MAX_BURST_LEN (MAX_BURST_LEN)
    ) u_dut (
        .clk            (clk),
        .srst           (srst),
        .cfg_enable     (cfg_enable),
        .cfg_start      (cfg_start),
        .cfg_abort      (cfg_abort),
        .cfg_sg_en      (cfg_sg_en),
        .cfg_xfer_type  (cfg_xfer_type),
        .cfg_src_addr   (cfg_src_addr),
        .cfg_dst_addr   (cfg_dst_addr),
        .cfg_xfer_len   (cfg_xfer_len),
        .cfg_desc_addr  (cfg_desc_addr),
        .status         (status),
        .cur_src_addr   (cur_src_addr),
        .cur_dst_addr   (cur_dst_addr),
        .tc_pulse       (tc_pulse),
        .err_pulse      (err_pulse),
        .arb_req        (arb_req),
        .arb_req_type   (arb_req_type),
        .arb_addr       (arb_addr),
        .arb_len        (arb_len),
        .arb_grant      (arb_grant),
        .axi_rd_valid   (axi_rd_valid),
        .axi_rd_data    (axi_rd_data),
        .axi_rd_last    (axi_rd_last),
        .axi_rd_resp    (axi_rd_resp),
        .axi_rd_ready   (axi_rd_ready),
        .axi_wr_valid   (axi_wr_valid),
        .axi_wr_data    (axi_wr_data),
        .axi_wr_last    (axi_wr_last),
        .axi_wr_ready   (axi_wr_ready),
        .axi_wr_resp_valid (axi_wr_resp_valid),
        .axi_wr_resp    (axi_wr_resp),
        .dreq           (dreq),
        .dack           (dack)
    );

    // Clock generation
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Local memory ----
    logic [31:0] mem [0:MEM_WORDS-1];

    // ---- Test infrastructure ----
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

    // ---- Error injection control ----
    logic inject_rd_err;
    int   inject_rd_err_beat;  // which beat to inject error on
    logic inject_wr_err;

    // ---- Mock arbiter + AXI data path (single always block) ----
    logic [ADDR_W-1:0] cap_addr;
    logic [7:0]        cap_len;
    arb_req_type_t     cap_type;
    logic              rd_active;
    logic              wr_active;
    logic              grant_pending;
    logic              grant_pending2;
    logic [ADDR_W-1:0] rd_addr_cur;
    logic [7:0]        rd_beat_cnt;
    logic [ADDR_W-1:0] wr_addr_cur;

    always @(posedge clk) begin
        if (srst) begin
            arb_grant         <= 1'b0;
            grant_pending     <= 1'b0;
            grant_pending2    <= 1'b0;
            cap_addr          <= '0;
            cap_len           <= '0;
            cap_type          <= ARB_REQ_NONE;
            axi_rd_valid      <= 1'b0;
            axi_rd_data       <= '0;
            axi_rd_last       <= 1'b0;
            axi_rd_resp       <= 2'b00;
            rd_active         <= 1'b0;
            rd_beat_cnt       <= '0;
            rd_addr_cur       <= '0;
            axi_wr_ready      <= 1'b0;
            axi_wr_resp_valid <= 1'b0;
            axi_wr_resp       <= 2'b00;
            wr_active         <= 1'b0;
            wr_addr_cur       <= '0;
        end else begin
            // Default: deassert write response after one cycle
            axi_wr_resp_valid <= 1'b0;

            // ---- Arbiter logic ----
            // Stage 1: detect arb_req, set pending
            if (arb_req && !arb_grant && !grant_pending && !grant_pending2 && !rd_active && !wr_active) begin
                grant_pending <= 1'b1;
                cap_type      <= arb_req_type;
            end

            // Stage 2: re-capture stable addr/len (burst_len now loaded in DUT)
            if (grant_pending) begin
                grant_pending  <= 1'b0;
                grant_pending2 <= 1'b1;
                cap_addr       <= arb_addr;
                cap_len        <= arb_len;
            end

            // Stage 3: assert grant and start data phase
            if (grant_pending2) begin
                arb_grant      <= 1'b1;
                grant_pending2 <= 1'b0;

                // Start read or write data phase immediately with grant
                if (cap_type == ARB_REQ_READ || cap_type == ARB_REQ_DESC) begin
                    rd_active    <= 1'b1;
                    rd_beat_cnt  <= '0;
                    rd_addr_cur  <= cap_addr;
                    axi_rd_valid <= 1'b1;
                    axi_rd_data  <= mem[cap_addr >> 2];
                    axi_rd_last  <= (cap_len == 8'd0) ? 1'b1 : 1'b0;
                    if (inject_rd_err && inject_rd_err_beat == 0)
                        axi_rd_resp <= 2'b10;
                    else
                        axi_rd_resp <= 2'b00;
                end else if (cap_type == ARB_REQ_WRITE) begin
                    wr_active    <= 1'b1;
                    wr_addr_cur  <= cap_addr;
                    axi_wr_ready <= 1'b1;
                end
            end

            // ---- Read data phase ----
            if (rd_active && !grant_pending && !grant_pending2) begin
                if (axi_rd_valid && axi_rd_ready) begin
                    if (axi_rd_last) begin
                        rd_active    <= 1'b0;
                        axi_rd_valid <= 1'b0;
                        axi_rd_last  <= 1'b0;
                        axi_rd_resp  <= 2'b00;
                        arb_grant    <= 1'b0;
                    end else begin
                        rd_beat_cnt <= rd_beat_cnt + 8'd1;
                        rd_addr_cur <= rd_addr_cur + 32'd4;
                        axi_rd_data <= mem[(rd_addr_cur + 32'd4) >> 2];
                        if (rd_beat_cnt + 8'd1 == cap_len)
                            axi_rd_last <= 1'b1;
                        else
                            axi_rd_last <= 1'b0;
                        if (inject_rd_err && (rd_beat_cnt + 1) == inject_rd_err_beat[7:0])
                            axi_rd_resp <= 2'b10;
                        else
                            axi_rd_resp <= 2'b00;
                    end
                end
            end

            // ---- Write data phase ----
            if (wr_active && !grant_pending && !grant_pending2) begin
                if (axi_wr_valid && axi_wr_ready) begin
                    mem[wr_addr_cur >> 2] <= axi_wr_data;
                    wr_addr_cur <= wr_addr_cur + 32'd4;
                    if (axi_wr_last) begin
                        wr_active    <= 1'b0;
                        axi_wr_ready <= 1'b0;
                        arb_grant    <= 1'b0;
                        axi_wr_resp_valid <= 1'b1;
                        if (inject_wr_err)
                            axi_wr_resp <= 2'b10;
                        else
                            axi_wr_resp <= 2'b00;
                    end
                end
            end

            // ---- Deassert grant when data phase ends ----
            if (arb_grant && !rd_active && !wr_active && !grant_pending && !grant_pending2) begin
                arb_grant <= 1'b0;
            end

            // ---- Clear read signals when not active ----
            if (!rd_active && !grant_pending && !grant_pending2) begin
                axi_rd_valid <= 1'b0;
                axi_rd_last  <= 1'b0;
            end

            // ---- Clear write ready when not active ----
            if (!wr_active && !grant_pending && !grant_pending2) begin
                axi_wr_ready <= 1'b0;
            end
        end
    end

    // ---- Helper tasks ----

    task automatic reset_dut();
        cfg_enable    = 0;
        cfg_start     = 0;
        cfg_abort     = 0;
        cfg_sg_en     = 0;
        cfg_xfer_type = XFER_MEM2MEM;
        cfg_src_addr  = '0;
        cfg_dst_addr  = '0;
        cfg_xfer_len  = '0;
        cfg_desc_addr = '0;
        dreq          = 0;
        inject_rd_err = 0;
        inject_rd_err_beat = 0;
        inject_wr_err = 0;
        srst = 1;
        repeat (3) @(posedge clk);
        srst = 0;
        @(posedge clk);
    endtask

    task automatic start_xfer(
        input logic [ADDR_W-1:0] src,
        input logic [ADDR_W-1:0] dst,
        input logic [31:0]       len,
        input xfer_type_t        xtype,
        input logic              sg_en,
        input logic [ADDR_W-1:0] desc_addr
    );
        @(negedge clk);
        cfg_enable    = 1;
        cfg_src_addr  = src;
        cfg_dst_addr  = dst;
        cfg_xfer_len  = len;
        cfg_xfer_type = xtype;
        cfg_sg_en     = sg_en;
        cfg_desc_addr = desc_addr;
        cfg_start     = 1;
        @(negedge clk);
        cfg_start     = 0;
    endtask

    task automatic wait_for_idle(input int timeout_cycles);
        int i;
        reg done;
        done = 0;
        for (i = 0; i < timeout_cycles && !done; i++) begin
            @(posedge clk);
            if (status == CH_IDLE) done = 1;
        end
        if (!done)
            $display("[WARN] wait_for_idle timed out after %0d cycles", timeout_cycles);
    endtask

    task automatic wait_for_tc(input int timeout_cycles);
        int i;
        reg done;
        done = 0;
        for (i = 0; i < timeout_cycles && !done; i++) begin
            @(posedge clk);
            if (tc_pulse) done = 1;
        end
        if (!done)
            $display("[WARN] wait_for_tc timed out after %0d cycles", timeout_cycles);
    endtask

    task automatic wait_for_error(input int timeout_cycles);
        int i;
        reg done;
        done = 0;
        for (i = 0; i < timeout_cycles && !done; i++) begin
            @(posedge clk);
            if (status == CH_ERROR) done = 1;
        end
        if (!done)
            $display("[WARN] wait_for_error timed out after %0d cycles", timeout_cycles);
    endtask

    // Initialize memory with pattern: mem[i] = i * 4 (byte address)
    task automatic init_mem();
        int i;
        for (i = 0; i < MEM_WORDS; i++) begin
            mem[i] = 32'hA000_0000 + i;
        end
    endtask

    // Load a descriptor into local memory
    // Descriptor word order: [0]=ctrl, [1]=xfer_len, [2]=dst_addr, [3]=src_addr, [4]=next_desc_addr
    task automatic load_descriptor(
        input logic [ADDR_W-1:0] desc_addr,
        input logic [31:0]       ctrl,
        input logic [31:0]       xfer_len,
        input logic [ADDR_W-1:0] dst_addr,
        input logic [ADDR_W-1:0] src_addr,
        input logic [ADDR_W-1:0] next_desc_addr
    );
        mem[(desc_addr >> 2) + 0] = ctrl;
        mem[(desc_addr >> 2) + 1] = xfer_len;
        mem[(desc_addr >> 2) + 2] = dst_addr;
        mem[(desc_addr >> 2) + 3] = src_addr;
        mem[(desc_addr >> 2) + 4] = next_desc_addr;
    endtask

    // ======== Tests ========
    initial begin
        $dumpfile("tb_dma_channel.vcd");
        $dumpvars(0, tb_dma_channel);

        // ---- Test 1: Reset ----
        $display("\n--- Test 1: Reset ---");
        reset_dut();
        check("status after reset", {29'd0, status}, {29'd0, CH_IDLE});

        // ---- Test 2: Simple 4-byte transfer (1 beat) ----
        $display("\n--- Test 2: Simple 4-byte transfer ---");
        reset_dut();
        init_mem();
        // src=0x0000 (word 0), dst=0x0800 (word 512)
        mem[512] = 32'h0000_0000;  // Clear destination
        start_xfer(32'h0000_0000, 32'h0000_0800, 32'd4, XFER_MEM2MEM, 1'b0, '0);
        wait_for_tc(200);
        check_1b("T2 tc_pulse", tc_pulse, 1'b1);
        // After tc_pulse, wait one more cycle to ensure IDLE
        @(posedge clk);
        check("T2 status idle", {29'd0, status}, {29'd0, CH_IDLE});
        check("T2 dst data", mem[512], 32'hA000_0000);

        // ---- Test 3: 16-byte transfer (4 beats) ----
        $display("\n--- Test 3: 16-byte transfer (4 beats) ---");
        reset_dut();
        init_mem();
        // src=0x0100 (word 64), dst=0x0900 (word 576)
        begin
            int i;
            for (i = 0; i < 4; i++) mem[576 + i] = 32'h0;
        end
        start_xfer(32'h0000_0100, 32'h0000_0900, 32'd16, XFER_MEM2MEM, 1'b0, '0);
        wait_for_tc(200);
        check_1b("T3 tc_pulse", tc_pulse, 1'b1);
        check("T3 dst[0]", mem[576], 32'hA000_0040);
        check("T3 dst[1]", mem[577], 32'hA000_0041);
        check("T3 dst[2]", mem[578], 32'hA000_0042);
        check("T3 dst[3]", mem[579], 32'hA000_0043);

        // ---- Test 4: 64-byte transfer (16 beats, max burst) ----
        $display("\n--- Test 4: 64-byte transfer (16 beats) ---");
        reset_dut();
        init_mem();
        // src=0x0200 (word 128), dst=0x0A00 (word 640)
        begin
            int i;
            for (i = 0; i < 16; i++) mem[640 + i] = 32'h0;
        end
        start_xfer(32'h0000_0200, 32'h0000_0A00, 32'd64, XFER_MEM2MEM, 1'b0, '0);
        wait_for_tc(300);
        check_1b("T4 tc_pulse", tc_pulse, 1'b1);
        check("T4 dst[0]",  mem[640],  32'hA000_0080);
        check("T4 dst[15]", mem[655], 32'hA000_008F);

        // ---- Test 5: 128-byte transfer (2 bursts of 16 beats) ----
        $display("\n--- Test 5: 128-byte transfer (2 bursts) ---");
        reset_dut();
        init_mem();
        // src=0x0300 (word 192), dst=0x0B00 (word 704)
        begin
            int i;
            for (i = 0; i < 32; i++) mem[704 + i] = 32'h0;
        end
        start_xfer(32'h0000_0300, 32'h0000_0B00, 32'd128, XFER_MEM2MEM, 1'b0, '0);
        wait_for_tc(500);
        check_1b("T5 tc_pulse", tc_pulse, 1'b1);
        check("T5 dst[0]",  mem[704],  32'hA000_00C0);
        check("T5 dst[15]", mem[719], 32'hA000_00CF);
        check("T5 dst[16]", mem[720], 32'hA000_00D0);
        check("T5 dst[31]", mem[735], 32'hA000_00DF);

        // ---- Test 6: Abort mid-transfer ----
        $display("\n--- Test 6: Abort mid-transfer ---");
        reset_dut();
        init_mem();
        // Start a 128-byte transfer and abort after a few cycles
        start_xfer(32'h0000_0300, 32'h0000_0B00, 32'd128, XFER_MEM2MEM, 1'b0, '0);
        // Wait a few cycles for the transfer to start
        repeat (8) @(posedge clk);
        @(negedge clk);
        cfg_abort = 1;
        @(negedge clk);
        cfg_abort = 0;
        @(posedge clk);
        @(posedge clk);
        check("T6 status idle after abort", {29'd0, status}, {29'd0, CH_IDLE});

        // ---- Test 7: AXI read error ----
        $display("\n--- Test 7: AXI read error ---");
        reset_dut();
        init_mem();
        inject_rd_err = 1;
        inject_rd_err_beat = 0;  // Error on first beat
        start_xfer(32'h0000_0000, 32'h0000_0800, 32'd16, XFER_MEM2MEM, 1'b0, '0);
        wait_for_error(200);
        check("T7 status error", {29'd0, status}, {29'd0, CH_ERROR});
        // Wait for err_pulse
        // err_pulse fires while in S_ERROR state
        @(posedge clk);
        check_1b("T7 err_pulse", err_pulse, 1'b1);
        inject_rd_err = 0;
        // Abort to return to idle
        @(negedge clk);
        cfg_abort = 1;
        @(negedge clk);
        cfg_abort = 0;
        @(posedge clk);
        @(posedge clk);

        // ---- Test 8: AXI write error ----
        $display("\n--- Test 8: AXI write error ---");
        reset_dut();
        init_mem();
        inject_wr_err = 1;
        start_xfer(32'h0000_0000, 32'h0000_0800, 32'd4, XFER_MEM2MEM, 1'b0, '0);
        wait_for_error(200);
        check("T8 status error", {29'd0, status}, {29'd0, CH_ERROR});
        @(posedge clk);
        check_1b("T8 err_pulse", err_pulse, 1'b1);
        inject_wr_err = 0;
        @(negedge clk);
        cfg_abort = 1;
        @(negedge clk);
        cfg_abort = 0;
        @(posedge clk);
        @(posedge clk);

        // ---- Test 9: Scatter-gather: 1 descriptor ----
        $display("\n--- Test 9: SG 1 descriptor ---");
        reset_dut();
        init_mem();
        // Descriptor at 0x1000: ctrl=0x01 (enable), xfer_len=16, dst=0x3000, src=0x2000, next=0
        load_descriptor(32'h0000_1000, 32'h0000_0001, 32'd16, 32'h0000_3000, 32'h0000_2000, 32'h0000_0000);
        // Ensure source data at 0x2000 (word 2048) is set
        // init_mem already set mem[2048] = 0xA000_0800, etc.
        begin
            int i;
            for (i = 0; i < 4; i++) mem[3072 + i] = 32'h0;  // Clear dst at 0x3000
        end
        start_xfer('0, '0, '0, XFER_MEM2MEM, 1'b1, 32'h0000_1000);
        wait_for_tc(500);
        check_1b("T9 tc_pulse", tc_pulse, 1'b1);
        check("T9 dst[0]", mem[3072], 32'hA000_0800);
        check("T9 dst[1]", mem[3073], 32'hA000_0801);
        check("T9 dst[2]", mem[3074], 32'hA000_0802);
        check("T9 dst[3]", mem[3075], 32'hA000_0803);

        // ---- Test 10: Scatter-gather: 2-descriptor chain ----
        $display("\n--- Test 10: SG 2-descriptor chain ---");
        reset_dut();
        init_mem();
        // Descriptor 1 at 0x1000: ctrl=0x01, xfer_len=8, dst=0x3000, src=0x2000, next=0x1014
        load_descriptor(32'h0000_1000, 32'h0000_0001, 32'd8, 32'h0000_3000, 32'h0000_2000, 32'h0000_1014);
        // Descriptor 2 at 0x1014: ctrl=0x01, xfer_len=8, dst=0x3008, src=0x2008, next=0
        load_descriptor(32'h0000_1014, 32'h0000_0001, 32'd8, 32'h0000_3008, 32'h0000_2008, 32'h0000_0000);
        // Clear destinations
        begin
            int i;
            for (i = 0; i < 4; i++) begin
                mem[3072 + i] = 32'h0;  // 0x3000
                mem[3074 + i] = 32'h0;  // 0x3008
            end
        end
        start_xfer('0, '0, '0, XFER_MEM2MEM, 1'b1, 32'h0000_1000);
        wait_for_tc(500);
        check_1b("T10 tc_pulse", tc_pulse, 1'b1);
        // Desc 1: 8 bytes = 2 beats from 0x2000 to 0x3000
        check("T10 desc1 dst[0]", mem[3072], 32'hA000_0800);
        check("T10 desc1 dst[1]", mem[3073], 32'hA000_0801);
        // Desc 2: 8 bytes = 2 beats from 0x2008 to 0x3008
        check("T10 desc2 dst[0]", mem[3074], 32'hA000_0802);
        check("T10 desc2 dst[1]", mem[3075], 32'hA000_0803);

        // ---- Test 11: Scatter-gather: descriptor with ctrl.last=1 ----
        $display("\n--- Test 11: SG descriptor with last bit ---");
        reset_dut();
        init_mem();
        // Descriptor at 0x1000: ctrl=0x11 (enable + last), xfer_len=8, dst=0x3000, src=0x2000, next=0x1014
        // Even though next is non-zero, the last bit should terminate the chain
        load_descriptor(32'h0000_1000, 32'h0000_0011, 32'd8, 32'h0000_3000, 32'h0000_2000, 32'h0000_1014);
        // Descriptor 2 at 0x1014 should NOT be fetched
        load_descriptor(32'h0000_1014, 32'h0000_0001, 32'd8, 32'h0000_3100, 32'h0000_2100, 32'h0000_0000);
        begin
            int i;
            for (i = 0; i < 2; i++) mem[3072 + i] = 32'h0;  // 0x3000
            for (i = 0; i < 2; i++) mem[3136 + i] = 32'h0;  // 0x3100
        end
        start_xfer('0, '0, '0, XFER_MEM2MEM, 1'b1, 32'h0000_1000);
        wait_for_tc(500);
        check_1b("T11 tc_pulse", tc_pulse, 1'b1);
        // Desc 1 transfer should happen
        check("T11 desc1 dst[0]", mem[3072], 32'hA000_0800);
        check("T11 desc1 dst[1]", mem[3073], 32'hA000_0801);
        // Desc 2 transfer should NOT happen (dst at 0x3100 = word 3136 should be zero)
        check("T11 desc2 not executed", mem[3136], 32'h0000_0000);

        // ---- Test 12: Peripheral handshake (M2P) ----
        $display("\n--- Test 12: Peripheral handshake M2P ---");
        reset_dut();
        init_mem();
        begin
            int i;
            for (i = 0; i < 4; i++) mem[512 + i] = 32'h0;  // Clear dst at 0x0800
        end
        // Use fork to drive dreq concurrently
        fork
            begin
                // Start the transfer
                start_xfer(32'h0000_0000, 32'h0000_0800, 32'd16, XFER_MEM2PERIPH, 1'b0, '0);
            end
            begin
                // Drive dreq high throughout the transfer
                // Wait for write phase to begin
                @(posedge clk);
                dreq = 1;
            end
        join
        wait_for_tc(500);
        check_1b("T12 tc_pulse", tc_pulse, 1'b1);
        check("T12 dst[0]", mem[512], 32'hA000_0000);
        check("T12 dst[1]", mem[513], 32'hA000_0001);
        check("T12 dst[2]", mem[514], 32'hA000_0002);
        check("T12 dst[3]", mem[515], 32'hA000_0003);
        // Verify dack was asserted at some point during writes
        // (We check it indirectly: data arrived, meaning dack allowed writes through)
        // For a direct check, we'll just verify status went back to idle
        @(posedge clk);
        check("T12 status idle", {29'd0, status}, {29'd0, CH_IDLE});
        dreq = 0;

        // ---- Summary ----
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

    // Safety timeout
    initial begin
        #100000;
        $display("[TIMEOUT] Simulation exceeded 100us");
        $finish;
    end

endmodule
