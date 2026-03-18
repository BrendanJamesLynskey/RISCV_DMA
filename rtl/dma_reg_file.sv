// Brendan Lynskey 2025
module dma_reg_file
  import dma_pkg::*;
#(
    parameter int NUM_CH = 4,
    parameter int DATA_W = 32,
    parameter int ADDR_W = 12
)(
    input  logic                clk,
    input  logic                srst,

    // CPU side
    input  logic                wr_en,
    input  logic                rd_en,
    input  logic [ADDR_W-1:0]  addr,
    input  logic [DATA_W-1:0]  wr_data,
    output logic [DATA_W-1:0]  rd_data,
    output logic                rd_valid,

    // Per-channel config outputs (packed/flattened for iverilog)
    output logic [NUM_CH-1:0]           ch_enable,
    output logic [NUM_CH-1:0]           ch_start,       // pulse
    output logic [NUM_CH-1:0]           ch_abort,       // pulse
    output logic [NUM_CH-1:0]           ch_sg_en,
    output logic [NUM_CH*2-1:0]         ch_xfer_type_flat,
    output logic [NUM_CH*32-1:0]        ch_src_addr_flat,
    output logic [NUM_CH*32-1:0]        ch_dst_addr_flat,
    output logic [NUM_CH*32-1:0]        ch_xfer_len_flat,
    output logic [NUM_CH*32-1:0]        ch_desc_addr_flat,

    // Per-channel status inputs (packed/flattened for iverilog)
    input  logic [NUM_CH*3-1:0]         ch_status_flat,
    input  logic [NUM_CH*32-1:0]        ch_cur_src_flat,
    input  logic [NUM_CH*32-1:0]        ch_cur_dst_flat,
    input  logic [NUM_CH-1:0]           ch_tc,          // pulse from channels
    input  logic [NUM_CH-1:0]           ch_err,         // pulse from channels

    // Interrupt output
    output logic                        irq
);

    // ---- Per-channel register offsets (within channel block) ----
    localparam logic [5:0] OFF_CH_CTRL      = 6'h00;
    localparam logic [5:0] OFF_CH_STATUS    = 6'h04;
    localparam logic [5:0] OFF_CH_SRC_ADDR  = 6'h08;
    localparam logic [5:0] OFF_CH_DST_ADDR  = 6'h0C;
    localparam logic [5:0] OFF_CH_XFER_LEN  = 6'h10;
    localparam logic [5:0] OFF_CH_DESC_ADDR = 6'h14;
    localparam logic [5:0] OFF_CH_CUR_SRC   = 6'h18;
    localparam logic [5:0] OFF_CH_CUR_DST   = 6'h1C;

    // ---- Global register addresses ----
    localparam logic [ADDR_W-1:0] ADDR_IRQ_STATUS = 12'h100;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_ENABLE = 12'h104;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_CLEAR  = 12'h108;
    localparam logic [ADDR_W-1:0] ADDR_VERSION    = 12'h10C;

    localparam logic [DATA_W-1:0] VERSION_VALUE = 32'h0001_0000;

    // ---- Per-channel control registers ----
    logic [NUM_CH-1:0] ch_enable_r;
    logic [NUM_CH-1:0] ch_sg_en_r;
    logic [1:0]        ch_xfer_type_r [NUM_CH];
    logic [31:0]       ch_src_addr_r  [NUM_CH];
    logic [31:0]       ch_dst_addr_r  [NUM_CH];
    logic [31:0]       ch_xfer_len_r  [NUM_CH];
    logic [31:0]       ch_desc_addr_r [NUM_CH];

    // Start/abort are pulse registers — written by CPU, self-clearing
    logic [NUM_CH-1:0] ch_start_r;
    logic [NUM_CH-1:0] ch_abort_r;

    // ---- Interrupt registers ----
    logic [NUM_CH-1:0] irq_tc_status;
    logic [NUM_CH-1:0] irq_err_status;
    logic [NUM_CH-1:0] irq_tc_enable;
    logic [NUM_CH-1:0] irq_err_enable;

    // ---- Output assignments ----
    assign ch_enable = ch_enable_r;
    assign ch_start  = ch_start_r;
    assign ch_abort  = ch_abort_r;
    assign ch_sg_en  = ch_sg_en_r;

    // ---- Flatten output arrays ----
    genvar gi;
    generate
        for (gi = 0; gi < NUM_CH; gi++) begin : gen_flat
            assign ch_xfer_type_flat[gi*2 +: 2]   = ch_xfer_type_r[gi];
            assign ch_src_addr_flat[gi*32 +: 32]   = ch_src_addr_r[gi];
            assign ch_dst_addr_flat[gi*32 +: 32]   = ch_dst_addr_r[gi];
            assign ch_xfer_len_flat[gi*32 +: 32]   = ch_xfer_len_r[gi];
            assign ch_desc_addr_flat[gi*32 +: 32]  = ch_desc_addr_r[gi];
        end
    endgenerate

    // ---- Unflatten input arrays ----
    ch_status_t ch_status_i [NUM_CH];
    logic [31:0] ch_cur_src_i [NUM_CH];
    logic [31:0] ch_cur_dst_i [NUM_CH];

    generate
        for (gi = 0; gi < NUM_CH; gi++) begin : gen_unflat
            assign ch_status_i[gi]  = ch_status_t'(ch_status_flat[gi*3 +: 3]);
            assign ch_cur_src_i[gi] = ch_cur_src_flat[gi*32 +: 32];
            assign ch_cur_dst_i[gi] = ch_cur_dst_flat[gi*32 +: 32];
        end
    endgenerate

    // ---- IRQ output ----
    assign irq = |((irq_tc_status & irq_tc_enable) | (irq_err_status & irq_err_enable));

    // ---- Address decode helpers ----
    // Channel number = addr[7:6] (each channel block is 64 bytes = 0x40)
    wire [3:0] ch_sel    = addr[7:6];
    wire [5:0] ch_offset = addr[5:0];
    wire       is_global = (addr >= ADDR_IRQ_STATUS);
    wire       ch_valid  = (ch_sel < NUM_CH[3:0]);

    // ---- Write logic ----
    always_ff @(posedge clk) begin
        if (srst) begin
            ch_enable_r    <= '0;
            ch_start_r     <= '0;
            ch_abort_r     <= '0;
            ch_sg_en_r     <= '0;
            irq_tc_enable  <= '0;
            irq_err_enable <= '0;
            for (int i = 0; i < NUM_CH; i++) begin
                ch_xfer_type_r[i] <= 2'b00;
                ch_src_addr_r[i]  <= '0;
                ch_dst_addr_r[i]  <= '0;
                ch_xfer_len_r[i]  <= '0;
                ch_desc_addr_r[i] <= '0;
            end
        end else begin
            // Self-clearing pulses: always clear after one cycle
            ch_start_r <= '0;
            ch_abort_r <= '0;

            if (wr_en) begin
                if (!is_global && ch_valid) begin
                    // Per-channel register write
                    case (ch_offset)
                        OFF_CH_CTRL: begin
                            ch_enable_r[ch_sel]    <= wr_data[0];
                            ch_start_r[ch_sel]     <= wr_data[1];
                            ch_abort_r[ch_sel]     <= wr_data[2];
                            ch_sg_en_r[ch_sel]     <= wr_data[3];
                            ch_xfer_type_r[ch_sel] <= wr_data[5:4];
                        end
                        OFF_CH_SRC_ADDR:  ch_src_addr_r[ch_sel]  <= wr_data;
                        OFF_CH_DST_ADDR:  ch_dst_addr_r[ch_sel]  <= wr_data;
                        OFF_CH_XFER_LEN:  ch_xfer_len_r[ch_sel]  <= wr_data;
                        OFF_CH_DESC_ADDR: ch_desc_addr_r[ch_sel] <= wr_data;
                        default: ; // read-only or reserved
                    endcase
                end else if (is_global) begin
                    // Global register write
                    case (addr)
                        ADDR_IRQ_ENABLE: begin
                            irq_tc_enable  <= wr_data[NUM_CH-1:0];
                            irq_err_enable <= wr_data[NUM_CH+15:16];
                        end
                        ADDR_IRQ_CLEAR: ; // Handled in IRQ status block
                        default: ; // VERSION is read-only
                    endcase
                end
            end
        end
    end

    // ---- IRQ status latch (set by channel pulses, cleared by IRQ_CLEAR write) ----
    // Separate from write logic to handle simultaneous set + clear
    always_ff @(posedge clk) begin
        if (srst) begin
            irq_tc_status  <= '0;
            irq_err_status <= '0;
        end else begin
            // Set on channel pulses
            for (int i = 0; i < NUM_CH; i++) begin
                if (ch_tc[i])
                    irq_tc_status[i] <= 1'b1;
                if (ch_err[i])
                    irq_err_status[i] <= 1'b1;
            end
            // Clear on IRQ_CLEAR write (write-1-to-clear)
            if (wr_en && addr == ADDR_IRQ_CLEAR) begin
                irq_tc_status  <= (irq_tc_status  | ch_tc)  & ~wr_data[NUM_CH-1:0];
                irq_err_status <= (irq_err_status | ch_err) & ~wr_data[NUM_CH+15:16];
            end
        end
    end

    // ---- Read logic ----
    always_ff @(posedge clk) begin
        if (srst) begin
            rd_data  <= '0;
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            rd_data  <= '0;

            if (rd_en) begin
                rd_valid <= 1'b1;

                if (!is_global && ch_valid) begin
                    case (ch_offset)
                        OFF_CH_CTRL: begin
                            rd_data <= '0;
                            rd_data[0]   <= ch_enable_r[ch_sel];
                            // start and abort are self-clearing, read as 0
                            rd_data[3]   <= ch_sg_en_r[ch_sel];
                            rd_data[5:4] <= ch_xfer_type_r[ch_sel];
                        end
                        OFF_CH_STATUS: begin
                            rd_data        <= '0;
                            rd_data[2:0]   <= ch_status_i[ch_sel];
                            rd_data[3]     <= irq_tc_status[ch_sel];
                            rd_data[4]     <= irq_err_status[ch_sel];
                        end
                        OFF_CH_SRC_ADDR:  rd_data <= ch_src_addr_r[ch_sel];
                        OFF_CH_DST_ADDR:  rd_data <= ch_dst_addr_r[ch_sel];
                        OFF_CH_XFER_LEN:  rd_data <= ch_xfer_len_r[ch_sel];
                        OFF_CH_DESC_ADDR: rd_data <= ch_desc_addr_r[ch_sel];
                        OFF_CH_CUR_SRC:   rd_data <= ch_cur_src_i[ch_sel];
                        OFF_CH_CUR_DST:   rd_data <= ch_cur_dst_i[ch_sel];
                        default:          rd_data <= '0;
                    endcase
                end else if (is_global) begin
                    case (addr)
                        ADDR_IRQ_STATUS: begin
                            rd_data <= '0;
                            rd_data[NUM_CH-1:0]    <= irq_tc_status;
                            rd_data[NUM_CH+15:16]  <= irq_err_status;
                        end
                        ADDR_IRQ_ENABLE: begin
                            rd_data <= '0;
                            rd_data[NUM_CH-1:0]    <= irq_tc_enable;
                            rd_data[NUM_CH+15:16]  <= irq_err_enable;
                        end
                        ADDR_VERSION: rd_data <= VERSION_VALUE;
                        default:      rd_data <= '0;
                    endcase
                end
            end
        end
    end

endmodule
