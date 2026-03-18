// Brendan Lynskey 2025
module dma_arbiter
  import dma_pkg::*;
#(
    parameter int NUM_CH   = 4,
    parameter     ARB_MODE = "ROUND_ROBIN"
)(
    input  logic                clk,
    input  logic                srst,

    input  logic [NUM_CH-1:0]   req,
    input  logic [NUM_CH*2-1:0] req_type_flat,
    input  logic [NUM_CH*32-1:0] req_addr_flat,
    input  logic [NUM_CH*8-1:0] req_len_flat,

    output logic [NUM_CH-1:0]   grant,
    output logic                grant_valid,

    output arb_req_type_t       axi_req_type,
    output logic [31:0]         axi_req_addr,
    output logic [7:0]          axi_req_len,
    output logic                axi_req_valid,
    input  logic                axi_req_done
);

    // ---- Unflatten input arrays ----
    arb_req_type_t req_type_i [NUM_CH];
    logic [31:0]   req_addr_i [NUM_CH];
    logic [7:0]    req_len_i  [NUM_CH];

    genvar gi;
    generate
        for (gi = 0; gi < NUM_CH; gi++) begin : gen_unflat
            assign req_type_i[gi] = arb_req_type_t'(req_type_flat[gi*2 +: 2]);
            assign req_addr_i[gi] = req_addr_flat[gi*32 +: 32];
            assign req_len_i[gi]  = req_len_flat[gi*8 +: 8];
        end
    endgenerate

    // ----------------------------------------------------------------
    // State encoding
    // ----------------------------------------------------------------
    typedef enum logic {
        S_IDLE = 1'b0,
        S_BUSY = 1'b1
    } state_t;

    state_t                     state, state_next;

    // ----------------------------------------------------------------
    // Internal signals
    // ----------------------------------------------------------------
    localparam int PTR_W = $clog2(NUM_CH);

    logic [PTR_W-1:0]  priority_ptr;
    logic [PTR_W-1:0]  winner;
    logic               winner_found;

    // Registered winner index (held during BUSY)
    logic [PTR_W-1:0]  winner_r;

    // ----------------------------------------------------------------
    // Winner selection — combinational
    // ----------------------------------------------------------------
    logic [PTR_W-1:0] rr_idx;
    // Use wider arithmetic to avoid truncation of NUM_CH
    logic [PTR_W:0]   rr_sum;

    always @(*) begin
        winner       = '0;
        winner_found = 1'b0;
        rr_idx       = '0;
        rr_sum       = '0;

        if (ARB_MODE == "FIXED_PRIORITY") begin : fixed_pri
            // Channel 0 = highest priority
            integer i;
            for (i = 0; i < NUM_CH; i = i + 1) begin
                if (req[i] && !winner_found) begin
                    winner       = i[PTR_W-1:0];
                    winner_found = 1'b1;
                end
            end
        end else begin : round_robin
            // Scan from priority_ptr, wrapping around
            integer i;
            for (i = 0; i < NUM_CH; i = i + 1) begin
                if (!winner_found) begin
                    // Compute (priority_ptr + i) mod NUM_CH using wider arithmetic
                    rr_sum = {1'b0, priority_ptr} + i[PTR_W:0];
                    if (rr_sum >= NUM_CH[PTR_W:0])
                        rr_idx = rr_sum[PTR_W-1:0] - NUM_CH[PTR_W-1:0];
                    else
                        rr_idx = rr_sum[PTR_W-1:0];

                    if (req[rr_idx]) begin
                        winner       = rr_idx;
                        winner_found = 1'b1;
                    end
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // Next-state logic — combinational
    // ----------------------------------------------------------------
    always @(*) begin
        state_next = state;

        case (state)
            S_IDLE: begin
                if (winner_found)
                    state_next = S_BUSY;
            end

            S_BUSY: begin
                if (axi_req_done)
                    state_next = S_IDLE;
            end

            default: state_next = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Sequential logic
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (srst) begin
            state        <= S_IDLE;
            priority_ptr <= '0;
            winner_r     <= '0;
            grant        <= '0;
            grant_valid  <= 1'b0;
            axi_req_type <= ARB_REQ_NONE;
            axi_req_addr <= 32'd0;
            axi_req_len  <= 8'd0;
            axi_req_valid <= 1'b0;
        end else begin
            state         <= state_next;
            axi_req_valid <= 1'b0; // default: single-cycle pulse

            case (state)
                S_IDLE: begin
                    if (winner_found) begin
                        // Latch the winner and forward its request
                        winner_r      <= winner;
                        grant         <= '0;
                        grant[winner] <= 1'b1;
                        grant_valid   <= 1'b1;
                        axi_req_type  <= req_type_i[winner];
                        axi_req_addr  <= req_addr_i[winner];
                        axi_req_len   <= req_len_i[winner];
                        axi_req_valid <= 1'b1;
                    end
                end

                S_BUSY: begin
                    if (axi_req_done) begin
                        grant       <= '0;
                        grant_valid <= 1'b0;

                        // Advance priority pointer in round-robin mode
                        if (ARB_MODE == "ROUND_ROBIN") begin
                            if ({1'b0, winner_r} + 1 >= NUM_CH[PTR_W:0])
                                priority_ptr <= '0;
                            else
                                priority_ptr <= winner_r + 1'b1;
                        end
                    end
                end

                default: ; // no action
            endcase
        end
    end

endmodule
