// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_reg_file;
    import dma_pkg::*;

    // Parameters
    localparam NUM_CH     = 4;
    localparam DATA_W     = 32;
    localparam ADDR_W     = 12;
    localparam CLK_PERIOD = 10;

    // Signals
    logic                clk, srst;
    logic                wr_en, rd_en;
    logic [ADDR_W-1:0]  addr;
    logic [DATA_W-1:0]  wr_data;
    logic [DATA_W-1:0]  rd_data;
    logic                rd_valid;

    logic [NUM_CH-1:0]           ch_enable;
    logic [NUM_CH-1:0]           ch_start;
    logic [NUM_CH-1:0]           ch_abort;
    logic [NUM_CH-1:0]           ch_sg_en;
    logic [NUM_CH*2-1:0]        ch_xfer_type_flat;
    logic [NUM_CH*32-1:0]       ch_src_addr_flat;
    logic [NUM_CH*32-1:0]       ch_dst_addr_flat;
    logic [NUM_CH*32-1:0]       ch_xfer_len_flat;
    logic [NUM_CH*32-1:0]       ch_desc_addr_flat;

    logic [NUM_CH*3-1:0]        ch_status_flat;
    logic [NUM_CH*32-1:0]       ch_cur_src_flat;
    logic [NUM_CH*32-1:0]       ch_cur_dst_flat;
    logic [NUM_CH-1:0]           ch_tc;
    logic [NUM_CH-1:0]           ch_err;

    logic                        irq;

    // DUT
    dma_reg_file #(
        .NUM_CH(NUM_CH),
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W)
    ) u_dut (
        .clk        (clk),
        .srst       (srst),
        .wr_en      (wr_en),
        .rd_en      (rd_en),
        .addr       (addr),
        .wr_data    (wr_data),
        .rd_data    (rd_data),
        .rd_valid   (rd_valid),
        .ch_enable  (ch_enable),
        .ch_start   (ch_start),
        .ch_abort   (ch_abort),
        .ch_sg_en   (ch_sg_en),
        .ch_xfer_type_flat(ch_xfer_type_flat),
        .ch_src_addr_flat (ch_src_addr_flat),
        .ch_dst_addr_flat (ch_dst_addr_flat),
        .ch_xfer_len_flat (ch_xfer_len_flat),
        .ch_desc_addr_flat(ch_desc_addr_flat),
        .ch_status_flat  (ch_status_flat),
        .ch_cur_src_flat (ch_cur_src_flat),
        .ch_cur_dst_flat (ch_cur_dst_flat),
        .ch_tc      (ch_tc),
        .ch_err     (ch_err),
        .irq        (irq)
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

    task automatic reset_dut();
        wr_en   = 0;
        rd_en   = 0;
        addr    = '0;
        wr_data = '0;
        ch_tc   = '0;
        ch_err  = '0;
        for (int i = 0; i < NUM_CH; i++) begin
            ch_status_flat[i*3 +: 3]    = 3'(CH_IDLE);
            ch_cur_src_flat[i*32 +: 32]  = '0;
            ch_cur_dst_flat[i*32 +: 32]  = '0;
        end
        srst = 1;
        repeat (3) @(posedge clk);
        srst = 0;
        @(posedge clk);
    endtask

    task automatic reg_write(input logic [ADDR_W-1:0] a, input logic [DATA_W-1:0] d);
        @(negedge clk);
        wr_en   = 1;
        addr    = a;
        wr_data = d;
        @(negedge clk);
        wr_en   = 0;
        addr    = '0;
        wr_data = '0;
    endtask

    task automatic reg_read(input logic [ADDR_W-1:0] a, output logic [DATA_W-1:0] d);
        @(negedge clk);
        rd_en = 1;
        addr  = a;
        @(negedge clk);
        rd_en = 0;
        addr  = '0;
        d = rd_data;
    endtask

    function automatic logic [ADDR_W-1:0] ch_base(input int ch);
        return ch * 64;
    endfunction

    // ======== Tests ========
    initial begin
        logic [DATA_W-1:0] rdata;

        $dumpfile("tb_dma_reg_file.vcd");
        $dumpvars(0, tb_dma_reg_file);

        // ---- Test 1: Reset defaults ----
        $display("\n--- Test 1: Reset defaults ---");
        reset_dut();
        check("ch_enable after reset", {28'b0, ch_enable}, 32'h0);
        check("ch_sg_en after reset",  {28'b0, ch_sg_en},  32'h0);
        check_1b("irq after reset", irq, 1'b0);
        // Read all channel regs — should be 0
        reg_read(ch_base(0) + 12'h08, rdata);
        check("CH_SRC_ADDR[0] default", rdata, 32'h0);
        reg_read(ch_base(0) + 12'h00, rdata);
        check("CH_CTRL[0] default", rdata, 32'h0);
        reg_read(12'h10C, rdata);
        check("VERSION default", rdata, 32'h0001_0000);

        // ---- Test 2: Write/read CH_SRC_ADDR ch0 ----
        $display("\n--- Test 2: Write/read CH_SRC_ADDR ch0 ---");
        reset_dut();
        reg_write(ch_base(0) + 12'h08, 32'h1000_0000);
        reg_read(ch_base(0) + 12'h08, rdata);
        check("CH_SRC_ADDR readback", rdata, 32'h1000_0000);

        // ---- Test 3: Write/read CH_DST_ADDR ch0 ----
        $display("\n--- Test 3: Write/read CH_DST_ADDR ch0 ---");
        reg_write(ch_base(0) + 12'h0C, 32'h2000_0000);
        reg_read(ch_base(0) + 12'h0C, rdata);
        check("CH_DST_ADDR readback", rdata, 32'h2000_0000);

        // ---- Test 4: Write/read CH_XFER_LEN ch0 ----
        $display("\n--- Test 4: Write/read CH_XFER_LEN ch0 ---");
        reg_write(ch_base(0) + 12'h10, 32'h0000_0100);
        reg_read(ch_base(0) + 12'h10, rdata);
        check("CH_XFER_LEN readback", rdata, 32'h0000_0100);

        // ---- Test 5: Write/read CH_DESC_ADDR ch0 ----
        $display("\n--- Test 5: Write/read CH_DESC_ADDR ch0 ---");
        reg_write(ch_base(0) + 12'h14, 32'h3000_0000);
        reg_read(ch_base(0) + 12'h14, rdata);
        check("CH_DESC_ADDR readback", rdata, 32'h3000_0000);

        // ---- Test 6: Write/read CH_CTRL ----
        $display("\n--- Test 6: Write/read CH_CTRL ---");
        reset_dut();
        // enable=1, start=0, abort=0, sg_en=1, xfer_type=01 (m2p) → 0x19
        reg_write(ch_base(0) + 12'h00, 32'h0000_0019);
        check_1b("ch_enable[0]", ch_enable[0], 1'b1);
        check_1b("ch_sg_en[0]",  ch_sg_en[0],  1'b1);
        reg_read(ch_base(0) + 12'h00, rdata);
        check("CH_CTRL readback", rdata, 32'h0000_0019);

        // ---- Test 7: CH_CTRL start pulse ----
        $display("\n--- Test 7: CH_CTRL start pulse ---");
        reset_dut();
        @(negedge clk);
        wr_en   = 1;
        addr    = ch_base(0);
        wr_data = 32'h0000_0002; // start=1
        @(negedge clk);
        wr_en = 0;
        check_1b("ch_start[0] pulse high", ch_start[0], 1'b1);
        @(negedge clk);
        check_1b("ch_start[0] auto-cleared", ch_start[0], 1'b0);

        // ---- Test 8: CH_STATUS read ----
        $display("\n--- Test 8: CH_STATUS read ---");
        reset_dut();
        ch_status_flat[0*3 +: 3] = 3'(CH_READ); // 3'd2
        reg_read(ch_base(0) + 12'h04, rdata);
        check("CH_STATUS readback", rdata[2:0], 3'd2);

        // ---- Test 9: Multi-channel ----
        $display("\n--- Test 9: Multi-channel register access ---");
        reset_dut();
        for (int ch = 0; ch < NUM_CH; ch++) begin
            reg_write(ch_base(ch) + 12'h08, 32'hAA00_0000 + ch);
            reg_write(ch_base(ch) + 12'h0C, 32'hBB00_0000 + ch);
            reg_write(ch_base(ch) + 12'h10, 32'h0000_0040 * (ch + 1));
        end
        for (int ch = 0; ch < NUM_CH; ch++) begin
            reg_read(ch_base(ch) + 12'h08, rdata);
            check($sformatf("CH_SRC[%0d] readback", ch), rdata, 32'hAA00_0000 + ch);
            reg_read(ch_base(ch) + 12'h0C, rdata);
            check($sformatf("CH_DST[%0d] readback", ch), rdata, 32'hBB00_0000 + ch);
            reg_read(ch_base(ch) + 12'h10, rdata);
            check($sformatf("CH_LEN[%0d] readback", ch), rdata, 32'h0000_0040 * (ch + 1));
        end

        // ---- Test 10: IRQ_STATUS set/clear ----
        $display("\n--- Test 10: IRQ_STATUS set/clear ---");
        reset_dut();
        @(negedge clk);
        ch_tc  = 4'b0001;
        ch_err = 4'b0010;
        @(negedge clk);
        ch_tc  = '0;
        ch_err = '0;
        reg_read(12'h100, rdata);
        check("IRQ_STATUS tc[0]",  {31'b0, rdata[0]},  32'h1);
        check("IRQ_STATUS err[1]", {31'b0, rdata[17]}, 32'h1);
        // Clear tc[0]
        reg_write(12'h108, 32'h0000_0001);
        reg_read(12'h100, rdata);
        check("IRQ_STATUS tc[0] cleared", {31'b0, rdata[0]}, 32'h0);
        check("IRQ_STATUS err[1] still set", {31'b0, rdata[17]}, 32'h1);

        // ---- Test 11: IRQ generation ----
        $display("\n--- Test 11: IRQ generation ---");
        reset_dut();
        reg_write(12'h104, 32'h0000_0001); // tc_enable[0]=1
        check_1b("irq before tc", irq, 1'b0);
        @(negedge clk);
        ch_tc = 4'b0001;
        @(negedge clk);
        ch_tc = '0;
        @(negedge clk);
        check_1b("irq after tc", irq, 1'b1);
        reg_write(12'h108, 32'h0000_0001); // clear
        @(negedge clk);
        check_1b("irq after clear", irq, 1'b0);

        // ---- Test 12: VERSION register ----
        $display("\n--- Test 12: VERSION register ---");
        reset_dut();
        reg_read(12'h10C, rdata);
        check("VERSION", rdata, 32'h0001_0000);

        // ---- Summary ----
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
