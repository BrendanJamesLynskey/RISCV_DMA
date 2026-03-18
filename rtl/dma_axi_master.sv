// Brendan Lynskey 2025
module dma_axi_master
  import dma_pkg::*;
#(
    parameter int DATA_W = 32,
    parameter int ADDR_W = 32
)(
    input  logic                clk,
    input  logic                srst,

    // Request interface (from arbiter)
    input  arb_req_type_t       req_type,
    input  logic [ADDR_W-1:0]   req_addr,
    input  logic [7:0]          req_len,        // AXI burst length (0-based: 0 = 1 beat)
    input  logic                req_valid,
    output logic                req_done,
    output logic                req_error,      // SLVERR or DECERR detected
    output logic [1:0]          req_resp,       // AXI response code

    // Data interface to active channel (muxed by arbiter grant)
    output logic                rd_valid,
    output logic [DATA_W-1:0]   rd_data,
    output logic                rd_last,
    output logic [1:0]          rd_resp,
    input  logic                rd_ready,

    input  logic                wr_valid,
    input  logic [DATA_W-1:0]   wr_data,
    input  logic                wr_last,
    output logic                wr_ready,

    output logic                wr_resp_valid,
    output logic [1:0]          wr_resp,

    // AXI4 Master signals
    output logic                m_axi_awvalid,
    input  logic                m_axi_awready,
    output logic [ADDR_W-1:0]   m_axi_awaddr,
    output logic [7:0]          m_axi_awlen,
    output logic [2:0]          m_axi_awsize,
    output logic [1:0]          m_axi_awburst,
    output logic                m_axi_wvalid,
    input  logic                m_axi_wready,
    output logic [DATA_W-1:0]   m_axi_wdata,
    output logic [DATA_W/8-1:0] m_axi_wstrb,
    output logic                m_axi_wlast,
    input  logic                m_axi_bvalid,
    output logic                m_axi_bready,
    input  logic [1:0]          m_axi_bresp,
    output logic                m_axi_arvalid,
    input  logic                m_axi_arready,
    output logic [ADDR_W-1:0]   m_axi_araddr,
    output logic [7:0]          m_axi_arlen,
    output logic [2:0]          m_axi_arsize,
    output logic [1:0]          m_axi_arburst,
    input  logic                m_axi_rvalid,
    output logic                m_axi_rready,
    input  logic [DATA_W-1:0]   m_axi_rdata,
    input  logic [1:0]          m_axi_rresp,
    input  logic                m_axi_rlast
);

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    typedef enum logic [2:0] {
        AXI_IDLE     = 3'd0,
        AXI_AR       = 3'd1,   // Drive read address channel
        AXI_R        = 3'd2,   // Receive read data
        AXI_AW       = 3'd3,   // Drive write address channel
        AXI_W        = 3'd4,   // Drive write data channel
        AXI_B        = 3'd5    // Wait for write response
    } axi_state_t;

    axi_state_t state, state_next;

    // ----------------------------------------------------------------
    // Registered address/len for the active transaction
    // ----------------------------------------------------------------
    logic [ADDR_W-1:0] addr_r;
    logic [7:0]        len_r;

    // ----------------------------------------------------------------
    // Error tracking
    // ----------------------------------------------------------------
    logic        error_r;       // latched error flag for current transaction
    logic [1:0]  resp_r;        // latched non-OKAY response code

    // ----------------------------------------------------------------
    // Combinational next-state and outputs
    // ----------------------------------------------------------------
    axi_state_t  state_c;
    logic        done_c;
    logic        error_latch_c;
    logic [1:0]  resp_latch_c;

    always @(*) begin
        state_c      = state;
        done_c       = 1'b0;
        error_latch_c = error_r;
        resp_latch_c  = resp_r;

        case (state)
            AXI_IDLE: begin
                if (req_valid) begin
                    case (req_type)
                        ARB_REQ_READ, ARB_REQ_DESC: state_c = AXI_AR;
                        ARB_REQ_WRITE:              state_c = AXI_AW;
                        default:                    state_c = AXI_IDLE;
                    endcase
                end
            end

            AXI_AR: begin
                if (m_axi_arvalid && m_axi_arready)
                    state_c = AXI_R;
            end

            AXI_R: begin
                // Check each beat for errors
                if (m_axi_rvalid && rd_ready) begin
                    if (m_axi_rresp[1]) begin   // SLVERR (2'b10) or DECERR (2'b11)
                        error_latch_c = 1'b1;
                        resp_latch_c  = m_axi_rresp;
                    end
                    if (m_axi_rlast) begin
                        done_c  = 1'b1;
                        state_c = AXI_IDLE;
                    end
                end
            end

            AXI_AW: begin
                if (m_axi_awvalid && m_axi_awready)
                    state_c = AXI_W;
            end

            AXI_W: begin
                if (m_axi_wvalid && m_axi_wready && m_axi_wlast)
                    state_c = AXI_B;
            end

            AXI_B: begin
                if (m_axi_bvalid) begin
                    done_c  = 1'b1;
                    state_c = AXI_IDLE;
                    if (m_axi_bresp[1]) begin   // SLVERR or DECERR
                        error_latch_c = 1'b1;
                        resp_latch_c  = m_axi_bresp;
                    end
                end
            end

            default: state_c = AXI_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Sequential logic
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (srst) begin
            state   <= AXI_IDLE;
            addr_r  <= '0;
            len_r   <= 8'd0;
            error_r <= 1'b0;
            resp_r  <= 2'b00;
            req_done  <= 1'b0;
            req_error <= 1'b0;
            req_resp  <= 2'b00;
            wr_resp_valid <= 1'b0;
            wr_resp       <= 2'b00;
        end else begin
            state   <= state_c;
            error_r <= error_latch_c;
            resp_r  <= resp_latch_c;

            // Capture address and length on request acceptance
            if (state == AXI_IDLE && req_valid) begin
                addr_r  <= req_addr;
                len_r   <= req_len;
                error_r <= 1'b0;
                resp_r  <= 2'b00;
            end

            // req_done is a 1-cycle pulse
            req_done  <= done_c;
            req_error <= done_c ? error_latch_c : 1'b0;
            req_resp  <= done_c ? resp_latch_c  : req_resp;

            // Write response forwarding — pulse for 1 cycle when B handshake occurs
            if (state == AXI_B && m_axi_bvalid) begin
                wr_resp_valid <= 1'b1;
                wr_resp       <= m_axi_bresp;
            end else begin
                wr_resp_valid <= 1'b0;
            end
        end
    end

    // ----------------------------------------------------------------
    // AXI read address channel
    // ----------------------------------------------------------------
    assign m_axi_arvalid = (state == AXI_AR);
    assign m_axi_araddr  = addr_r;
    assign m_axi_arlen   = len_r;
    assign m_axi_arsize  = 3'b010;     // 4 bytes
    assign m_axi_arburst = 2'b01;      // INCR

    // ----------------------------------------------------------------
    // AXI read data channel — pass-through to channel interface
    // ----------------------------------------------------------------
    assign rd_valid    = (state == AXI_R) ? m_axi_rvalid : 1'b0;
    assign rd_data     = m_axi_rdata;
    assign rd_last     = m_axi_rlast;
    assign rd_resp     = m_axi_rresp;
    assign m_axi_rready = (state == AXI_R) ? rd_ready : 1'b0;

    // ----------------------------------------------------------------
    // AXI write address channel
    // ----------------------------------------------------------------
    assign m_axi_awvalid = (state == AXI_AW);
    assign m_axi_awaddr  = addr_r;
    assign m_axi_awlen   = len_r;
    assign m_axi_awsize  = 3'b010;     // 4 bytes
    assign m_axi_awburst = 2'b01;      // INCR

    // ----------------------------------------------------------------
    // AXI write data channel — pass-through from channel interface
    // ----------------------------------------------------------------
    assign m_axi_wvalid = (state == AXI_W) ? wr_valid : 1'b0;
    assign m_axi_wdata  = wr_data;
    assign m_axi_wstrb  = {(DATA_W/8){1'b1}};  // All bytes enabled
    assign m_axi_wlast  = wr_last;
    assign wr_ready     = (state == AXI_W) ? m_axi_wready : 1'b0;

    // ----------------------------------------------------------------
    // AXI write response channel
    // ----------------------------------------------------------------
    assign m_axi_bready = (state == AXI_B);

endmodule
