// Brendan Lynskey 2025
module dma_fifo
#(
    parameter int DATA_W = 32,
    parameter int DEPTH  = 16
)(
    input  logic                    clk,
    input  logic                    srst,

    // Write port
    input  logic                    wr_en,
    input  logic [DATA_W-1:0]      wr_data,
    output logic                    full,

    // Read port
    input  logic                    rd_en,
    output logic [DATA_W-1:0]      rd_data,
    output logic                    empty,

    // Status
    output logic [$clog2(DEPTH):0] count
);

    // ---- Internal storage ----
    localparam int PTR_W = $clog2(DEPTH);

    logic [DATA_W-1:0] mem [DEPTH];
    logic [PTR_W-1:0]  wr_ptr;
    logic [PTR_W-1:0]  rd_ptr;
    logic [$clog2(DEPTH):0] count_r;

    assign count = count_r;
    assign full  = (count_r == DEPTH[$clog2(DEPTH):0]);
    assign empty = (count_r == '0);

    // Read data is combinational from the read pointer
    assign rd_data = mem[rd_ptr];

    // ---- Write pointer ----
    always_ff @(posedge clk) begin
        if (srst) begin
            wr_ptr <= '0;
        end else if (wr_en && !full) begin
            wr_ptr <= (wr_ptr == PTR_W'(DEPTH - 1)) ? '0 : wr_ptr + 1'b1;
        end
    end

    // ---- Read pointer ----
    always_ff @(posedge clk) begin
        if (srst) begin
            rd_ptr <= '0;
        end else if (rd_en && !empty) begin
            rd_ptr <= (rd_ptr == PTR_W'(DEPTH - 1)) ? '0 : rd_ptr + 1'b1;
        end
    end

    // ---- Count ----
    always_ff @(posedge clk) begin
        if (srst) begin
            count_r <= '0;
        end else begin
            case ({wr_en && !full, rd_en && !empty})
                2'b10:   count_r <= count_r + 1'b1;
                2'b01:   count_r <= count_r - 1'b1;
                default: count_r <= count_r; // 2'b00 or 2'b11 (simultaneous)
            endcase
        end
    end

    // ---- Memory write ----
    always_ff @(posedge clk) begin
        if (wr_en && !full) begin
            mem[wr_ptr] <= wr_data;
        end
    end

endmodule
