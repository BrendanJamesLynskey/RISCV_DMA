// Brendan Lynskey 2025
module dma_channel
  import dma_pkg::*;
#(
    parameter int CH_ID         = 0,
    parameter int DATA_W        = 32,
    parameter int ADDR_W        = 32,
    parameter int MAX_BURST_LEN = 16
)(
    input  logic                clk,
    input  logic                srst,

    // Configuration (from register file)
    input  logic                cfg_enable,
    input  logic                cfg_start,          // pulse
    input  logic                cfg_abort,          // pulse
    input  logic                cfg_sg_en,
    input  xfer_type_t          cfg_xfer_type,
    input  logic [ADDR_W-1:0]   cfg_src_addr,
    input  logic [ADDR_W-1:0]   cfg_dst_addr,
    input  logic [31:0]         cfg_xfer_len,
    input  logic [ADDR_W-1:0]   cfg_desc_addr,

    // Status outputs (to register file)
    output ch_status_t          status,
    output logic [ADDR_W-1:0]   cur_src_addr,
    output logic [ADDR_W-1:0]   cur_dst_addr,
    output logic                tc_pulse,           // transfer complete
    output logic                err_pulse,          // error

    // Arbiter interface
    output logic                arb_req,
    output arb_req_type_t       arb_req_type,
    output logic [ADDR_W-1:0]   arb_addr,
    output logic [7:0]          arb_len,            // AXI burst length (0-based)
    input  logic                arb_grant,

    // AXI data path (active only when granted)
    input  logic                axi_rd_valid,
    input  logic [DATA_W-1:0]   axi_rd_data,
    input  logic                axi_rd_last,
    input  logic [1:0]          axi_rd_resp,
    output logic                axi_rd_ready,

    output logic                axi_wr_valid,
    output logic [DATA_W-1:0]   axi_wr_data,
    output logic                axi_wr_last,
    input  logic                axi_wr_ready,

    input  logic                axi_wr_resp_valid,
    input  logic [1:0]          axi_wr_resp,

    // Peripheral handshake
    input  logic                dreq,
    output logic                dack
);

    // ---- FSM states ----
    typedef enum logic [3:0] {
        S_IDLE        = 4'd0,
        S_DESC_REQ    = 4'd1,
        S_DESC_WAIT   = 4'd2,
        S_DESC_PARSE  = 4'd3,
        S_READ_REQ    = 4'd4,
        S_READ_WAIT   = 4'd5,
        S_WRITE_REQ   = 4'd6,
        S_WRITE_WAIT  = 4'd7,
        S_NEXT_BURST  = 4'd8,
        S_NEXT_DESC   = 4'd9,
        S_DONE        = 4'd10,
        S_ERROR       = 4'd11
    } ch_state_t;

    // AXI response codes
    localparam logic [1:0] AXI_RESP_OKAY = 2'b00;

    // ---- Internal registers ----
    ch_state_t              state, state_next;
    logic [ADDR_W-1:0]      cur_src;
    logic [ADDR_W-1:0]      cur_dst;
    logic [31:0]            remaining;
    logic [7:0]             burst_len;      // beats for current burst (1-based)
    logic [ADDR_W-1:0]      desc_ptr;
    logic [31:0]            desc_buf [5];
    logic [2:0]             desc_word_cnt;
    xfer_type_t             xfer_type_r;
    logic                   sg_en_r;
    logic [7:0]             wr_beat_cnt;    // write beat counter
    logic                   grant_seen_low; // Ensures stale grant from prev phase is ignored

    // Descriptor parsed fields (from desc_buf)
    logic [31:0]            desc_ctrl;
    logic [31:0]            desc_next_addr;

    // ---- FIFO signals ----
    logic                   fifo_wr_en;
    logic [DATA_W-1:0]      fifo_wr_data;
    logic                   fifo_full;
    logic                   fifo_rd_en;
    logic [DATA_W-1:0]      fifo_rd_data;
    logic                   fifo_empty;
    logic                   fifo_srst;

    // ---- FIFO instance ----
    dma_fifo #(
        .DATA_W (DATA_W),
        .DEPTH  (MAX_BURST_LEN)
    ) u_fifo (
        .clk     (clk),
        .srst    (fifo_srst),
        .wr_en   (fifo_wr_en),
        .wr_data (fifo_wr_data),
        .full    (fifo_full),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .count   ()
    );

    // ---- Burst length computation ----
    localparam int BYTES_PER_BEAT = DATA_W / 8;
    logic [31:0] remaining_beats;
    logic [7:0]  computed_burst_len;

    always @(*) begin
        remaining_beats = remaining / BYTES_PER_BEAT[31:0];
        if (remaining_beats > MAX_BURST_LEN[31:0])
            computed_burst_len = MAX_BURST_LEN[7:0];
        else
            computed_burst_len = remaining_beats[7:0];
    end

    // ---- FIFO reset: assert on srst or cfg_abort ----
    assign fifo_srst = srst | cfg_abort;

    // ---- Status mapping ----
    always @(*) begin
        case (state)
            S_IDLE:                                     status = CH_IDLE;
            S_DESC_REQ, S_DESC_WAIT, S_DESC_PARSE:     status = CH_DESC_FETCH;
            S_READ_REQ, S_READ_WAIT:                    status = CH_READ;
            S_WRITE_REQ, S_WRITE_WAIT:                  status = CH_WRITE;
            S_DONE:                                     status = CH_DONE;
            S_ERROR:                                    status = CH_ERROR;
            S_NEXT_BURST, S_NEXT_DESC:                  status = CH_READ;
            default:                                    status = CH_IDLE;
        endcase
    end

    // ---- Address outputs ----
    assign cur_src_addr = cur_src;
    assign cur_dst_addr = cur_dst;

    // ---- Combinational outputs ----
    always @(*) begin
        // Defaults
        arb_req      = 1'b0;
        arb_req_type = ARB_REQ_NONE;
        arb_addr     = {ADDR_W{1'b0}};
        arb_len      = 8'd0;
        axi_rd_ready = 1'b0;
        axi_wr_valid = 1'b0;
        axi_wr_data  = {DATA_W{1'b0}};
        axi_wr_last  = 1'b0;
        fifo_wr_en   = 1'b0;
        fifo_wr_data = {DATA_W{1'b0}};
        fifo_rd_en   = 1'b0;
        dack         = 1'b0;

        case (state)
            // ---- Descriptor fetch ----
            S_DESC_REQ: begin
                arb_req      = 1'b1;
                arb_req_type = ARB_REQ_DESC;
                arb_addr     = desc_ptr;
                arb_len      = 8'd4;   // 5 words, 0-based
            end

            S_DESC_WAIT: begin
                axi_rd_ready = arb_grant;
            end

            // ---- Read phase ----
            S_READ_REQ: begin
                arb_req      = 1'b1;
                arb_req_type = ARB_REQ_READ;
                arb_addr     = cur_src;
                arb_len      = computed_burst_len - 8'd1;
            end

            S_READ_WAIT: begin
                // Gate rd_ready with dreq for PERIPH2MEM
                if (xfer_type_r == XFER_PERIPH2MEM)
                    axi_rd_ready = !fifo_full & arb_grant & dreq;
                else
                    axi_rd_ready = !fifo_full & arb_grant;

                fifo_wr_en   = axi_rd_valid & axi_rd_ready;
                fifo_wr_data = axi_rd_data;

                // PERIPH2MEM dack
                if (xfer_type_r == XFER_PERIPH2MEM)
                    dack = axi_rd_valid & axi_rd_ready;
            end

            // ---- Write phase ----
            S_WRITE_REQ: begin
                arb_req      = 1'b1;
                arb_req_type = ARB_REQ_WRITE;
                arb_addr     = cur_dst;
                arb_len      = burst_len - 8'd1;
            end

            S_WRITE_WAIT: begin
                // Gate wr_valid with dreq for MEM2PERIPH
                if (xfer_type_r == XFER_MEM2PERIPH)
                    axi_wr_valid = !fifo_empty & arb_grant & dreq;
                else
                    axi_wr_valid = !fifo_empty & arb_grant;

                axi_wr_data = fifo_rd_data;
                fifo_rd_en  = axi_wr_valid & axi_wr_ready;
                axi_wr_last = (wr_beat_cnt == burst_len - 8'd1) ? 1'b1 : 1'b0;

                // MEM2PERIPH dack
                if (xfer_type_r == XFER_MEM2PERIPH)
                    dack = axi_wr_valid & axi_wr_ready;
            end

            default: begin
                // Keep defaults
            end
        endcase
    end

    // ---- Sequential logic ----
    always_ff @(posedge clk) begin
        if (srst) begin
            state         <= S_IDLE;
            cur_src       <= '0;
            cur_dst       <= '0;
            remaining     <= '0;
            burst_len     <= '0;
            desc_ptr      <= '0;
            desc_word_cnt <= '0;
            xfer_type_r   <= XFER_MEM2MEM;
            sg_en_r       <= 1'b0;
            wr_beat_cnt   <= '0;
            tc_pulse      <= 1'b0;
            err_pulse     <= 1'b0;
            desc_buf[0]   <= '0;
            desc_buf[1]   <= '0;
            desc_buf[2]   <= '0;
            desc_buf[3]   <= '0;
            desc_buf[4]   <= '0;
            grant_seen_low <= 1'b1;
        end else if (cfg_abort) begin
            // Abort: return to idle, clear request
            state         <= S_IDLE;
            tc_pulse      <= 1'b0;
            err_pulse     <= 1'b0;
            wr_beat_cnt   <= '0;
            desc_word_cnt <= '0;
            grant_seen_low <= 1'b1;
        end else begin
            // Default: clear single-cycle pulses
            tc_pulse  <= 1'b0;
            err_pulse <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (cfg_start && cfg_enable) begin
                        grant_seen_low <= 1'b1; // Fresh request from idle
                        if (cfg_sg_en) begin
                            desc_ptr      <= cfg_desc_addr;
                            sg_en_r       <= 1'b1;
                            state         <= S_DESC_REQ;
                        end else begin
                            cur_src       <= cfg_src_addr;
                            cur_dst       <= cfg_dst_addr;
                            remaining     <= cfg_xfer_len;
                            xfer_type_r   <= cfg_xfer_type;
                            sg_en_r       <= 1'b0;
                            state         <= S_READ_REQ;
                        end
                    end
                end

                // ---- Descriptor fetch ----
                S_DESC_REQ: begin
                    desc_word_cnt <= '0;
                    if (!arb_grant) grant_seen_low <= 1'b1;
                    if (arb_grant && grant_seen_low) begin
                        state <= S_DESC_WAIT;
                    end
                end

                S_DESC_WAIT: begin
                    if (axi_rd_valid && axi_rd_ready) begin
                        desc_buf[desc_word_cnt] <= axi_rd_data;
                        desc_word_cnt           <= desc_word_cnt + 3'd1;
                        if (axi_rd_last) begin
                            state <= S_DESC_PARSE;
                        end
                    end
                end

                S_DESC_PARSE: begin
                    // desc_buf[0] = ctrl, [1] = xfer_len, [2] = dst_addr,
                    // [3] = src_addr, [4] = next_desc_addr
                    if (!desc_buf[0][DESC_CTRL_EN_BIT]) begin
                        state <= S_ERROR;
                    end else begin
                        cur_src        <= desc_buf[3];
                        cur_dst        <= desc_buf[2];
                        remaining      <= desc_buf[1];
                        xfer_type_r    <= xfer_type_t'(desc_buf[0][DESC_CTRL_XTYPE_HI:DESC_CTRL_XTYPE_LO]);
                        state          <= S_READ_REQ;
                        grant_seen_low <= 1'b0; // Must wait for desc grant to drop
                    end
                end

                // ---- Read phase ----
                S_READ_REQ: begin
                    burst_len <= computed_burst_len;
                    if (!arb_grant) grant_seen_low <= 1'b1;
                    if (arb_grant && grant_seen_low) begin
                        state <= S_READ_WAIT;
                    end
                end

                S_READ_WAIT: begin
                    if (axi_rd_valid && axi_rd_ready) begin
                        if (axi_rd_resp != AXI_RESP_OKAY) begin
                            state <= S_ERROR;
                        end else if (axi_rd_last) begin
                            state          <= S_WRITE_REQ;
                            grant_seen_low <= 1'b0; // Must wait for old grant to drop
                        end
                    end
                end

                // ---- Write phase ----
                S_WRITE_REQ: begin
                    wr_beat_cnt <= '0;
                    if (!arb_grant) grant_seen_low <= 1'b1;
                    if (arb_grant && grant_seen_low) begin
                        state <= S_WRITE_WAIT;
                    end
                end

                S_WRITE_WAIT: begin
                    if (axi_wr_valid && axi_wr_ready) begin
                        wr_beat_cnt <= wr_beat_cnt + 8'd1;
                    end
                    if (axi_wr_resp_valid) begin
                        if (axi_wr_resp != AXI_RESP_OKAY) begin
                            state <= S_ERROR;
                        end else begin
                            state <= S_NEXT_BURST;
                        end
                    end
                end

                // ---- Next burst / descriptor ----
                S_NEXT_BURST: begin
                    remaining <= remaining - {24'd0, burst_len} * BYTES_PER_BEAT[31:0];
                    cur_src   <= cur_src + {{(ADDR_W-32){1'b0}}, {24'd0, burst_len} * BYTES_PER_BEAT[31:0]};
                    cur_dst   <= cur_dst + {{(ADDR_W-32){1'b0}}, {24'd0, burst_len} * BYTES_PER_BEAT[31:0]};
                    grant_seen_low <= 1'b0; // Previous write grant still active
                    if (remaining == {24'd0, burst_len} * BYTES_PER_BEAT[31:0]) begin
                        // remaining will become 0
                        if (sg_en_r)
                            state <= S_NEXT_DESC;
                        else
                            state <= S_DONE;
                    end else begin
                        state <= S_READ_REQ;
                    end
                end

                S_NEXT_DESC: begin
                    // Check next_desc_addr and last bit
                    grant_seen_low <= 1'b0; // Previous grant still active
                    if (desc_buf[4] == '0 || desc_buf[0][DESC_CTRL_LAST_BIT]) begin
                        state <= S_DONE;
                    end else begin
                        desc_ptr <= desc_buf[4];
                        state    <= S_DESC_REQ;
                    end
                end

                S_DONE: begin
                    tc_pulse <= 1'b1;
                    state    <= S_IDLE;
                end

                S_ERROR: begin
                    err_pulse <= 1'b1;
                    // Stay in S_ERROR until cfg_abort (handled above)
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
