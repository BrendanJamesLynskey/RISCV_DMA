// Brendan Lynskey 2025
module dma_top
  import dma_pkg::*;
#(
    parameter int NUM_CH        = 4,
    parameter int DATA_W        = 32,
    parameter int ADDR_W        = 32,
    parameter int MAX_BURST_LEN = 16,
    parameter     ARB_MODE      = "ROUND_ROBIN"
)(
    input  logic                clk,
    input  logic                srst,

    // ---- CPU register interface ----
    input  logic                reg_wr_en,
    input  logic                reg_rd_en,
    input  logic [11:0]         reg_addr,
    input  logic [DATA_W-1:0]   reg_wr_data,
    output logic [DATA_W-1:0]   reg_rd_data,
    output logic                reg_rd_valid,

    // ---- AXI4 Master — Write Address Channel ----
    output logic                m_axi_awvalid,
    input  logic                m_axi_awready,
    output logic [ADDR_W-1:0]   m_axi_awaddr,
    output logic [7:0]          m_axi_awlen,
    output logic [2:0]          m_axi_awsize,
    output logic [1:0]          m_axi_awburst,

    // ---- AXI4 Master — Write Data Channel ----
    output logic                m_axi_wvalid,
    input  logic                m_axi_wready,
    output logic [DATA_W-1:0]   m_axi_wdata,
    output logic [DATA_W/8-1:0] m_axi_wstrb,
    output logic                m_axi_wlast,

    // ---- AXI4 Master — Write Response Channel ----
    input  logic                m_axi_bvalid,
    output logic                m_axi_bready,
    input  logic [1:0]          m_axi_bresp,

    // ---- AXI4 Master — Read Address Channel ----
    output logic                m_axi_arvalid,
    input  logic                m_axi_arready,
    output logic [ADDR_W-1:0]   m_axi_araddr,
    output logic [7:0]          m_axi_arlen,
    output logic [2:0]          m_axi_arsize,
    output logic [1:0]          m_axi_arburst,

    // ---- AXI4 Master — Read Data Channel ----
    input  logic                m_axi_rvalid,
    output logic                m_axi_rready,
    input  logic [DATA_W-1:0]   m_axi_rdata,
    input  logic [1:0]          m_axi_rresp,
    input  logic                m_axi_rlast,

    // ---- Peripheral handshake ----
    input  logic [NUM_CH-1:0]   dreq,
    output logic [NUM_CH-1:0]   dack,

    // ---- Interrupt ----
    output logic                irq
);

    // ================================================================
    // Register file <-> channel signals
    // Packed (flattened) arrays for iverilog compatibility.
    // ================================================================
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
    logic [NUM_CH-1:0]          ch_tc;
    logic [NUM_CH-1:0]          ch_err;

    // ================================================================
    // Arbiter signals (packed/flattened)
    // ================================================================
    logic [NUM_CH-1:0]           arb_req;
    logic [NUM_CH*2-1:0]        arb_req_type_flat;
    logic [NUM_CH*32-1:0]       arb_req_addr_flat;
    logic [NUM_CH*8-1:0]        arb_req_len_flat;
    logic [NUM_CH-1:0]          arb_grant;
    logic                        arb_grant_valid;

    arb_req_type_t               axi_req_type;
    logic [31:0]                 axi_req_addr;
    logic [7:0]                  axi_req_len;
    logic                        axi_req_valid;
    logic                        axi_req_done;

    // ================================================================
    // AXI master <-> channel data path signals
    // ================================================================
    logic                axi_rd_valid;
    logic [DATA_W-1:0]   axi_rd_data;
    logic                axi_rd_last;
    logic [1:0]          axi_rd_resp;
    logic                axi_rd_ready;

    logic                axi_wr_valid;
    logic [DATA_W-1:0]   axi_wr_data;
    logic                axi_wr_last;
    logic                axi_wr_ready;

    logic                axi_wr_resp_valid;
    logic [1:0]          axi_wr_resp;

    // AXI master error signals
    logic                axi_req_error;
    logic [1:0]          axi_req_resp;

    // ================================================================
    // Register File
    // ================================================================
    dma_reg_file #(
        .NUM_CH (NUM_CH),
        .DATA_W (DATA_W),
        .ADDR_W (12)
    ) u_reg_file (
        .clk             (clk),
        .srst            (srst),
        .wr_en           (reg_wr_en),
        .rd_en           (reg_rd_en),
        .addr            (reg_addr),
        .wr_data         (reg_wr_data),
        .rd_data         (reg_rd_data),
        .rd_valid        (reg_rd_valid),
        .ch_enable       (ch_enable),
        .ch_start        (ch_start),
        .ch_abort        (ch_abort),
        .ch_sg_en        (ch_sg_en),
        .ch_xfer_type_flat(ch_xfer_type_flat),
        .ch_src_addr_flat (ch_src_addr_flat),
        .ch_dst_addr_flat (ch_dst_addr_flat),
        .ch_xfer_len_flat (ch_xfer_len_flat),
        .ch_desc_addr_flat(ch_desc_addr_flat),
        .ch_status_flat  (ch_status_flat),
        .ch_cur_src_flat (ch_cur_src_flat),
        .ch_cur_dst_flat (ch_cur_dst_flat),
        .ch_tc           (ch_tc),
        .ch_err          (ch_err),
        .irq             (irq)
    );

    // ================================================================
    // Per-channel wires (explicit per-element to avoid iverilog
    // unpacked array port connection issues)
    // ================================================================
    // Per-channel AXI data signals (before muxing)
    logic                ch_axi_rd_ready [NUM_CH];
    logic                ch_axi_wr_valid [NUM_CH];
    logic [DATA_W-1:0]   ch_axi_wr_data  [NUM_CH];
    logic                ch_axi_wr_last  [NUM_CH];
    logic                ch_dack         [NUM_CH];

    // ================================================================
    // Channel instances — per-element wiring with packed array slicing
    // ================================================================
    genvar gi;
    generate
        for (gi = 0; gi < NUM_CH; gi++) begin : gen_ch
            // Explicit per-element wires from register file
            wire                 w_cfg_enable    = ch_enable[gi];
            wire                 w_cfg_start     = ch_start[gi];
            wire                 w_cfg_abort     = ch_abort[gi];
            wire                 w_cfg_sg_en     = ch_sg_en[gi];

            // Slice packed arrays for per-channel config
            wire [1:0]           w_cfg_xfer_type = ch_xfer_type_flat[gi*2 +: 2];
            wire [31:0]          w_cfg_src_addr  = ch_src_addr_flat[gi*32 +: 32];
            wire [31:0]          w_cfg_dst_addr  = ch_dst_addr_flat[gi*32 +: 32];
            wire [31:0]          w_cfg_xfer_len  = ch_xfer_len_flat[gi*32 +: 32];
            wire [31:0]          w_cfg_desc_addr = ch_desc_addr_flat[gi*32 +: 32];

            // Status wires back to register file
            wire [2:0]  w_status;
            wire [31:0] w_cur_src;
            wire [31:0] w_cur_dst;

            assign ch_status_flat[gi*3 +: 3]    = w_status;
            assign ch_cur_src_flat[gi*32 +: 32]  = w_cur_src;
            assign ch_cur_dst_flat[gi*32 +: 32]  = w_cur_dst;

            // Arbiter request wires — pack into flat arrays
            wire [1:0]  w_arb_req_type;
            wire [31:0] w_arb_addr;
            wire [7:0]  w_arb_len;

            assign arb_req_type_flat[gi*2 +: 2]   = w_arb_req_type;
            assign arb_req_addr_flat[gi*32 +: 32]  = w_arb_addr;
            assign arb_req_len_flat[gi*8 +: 8]     = w_arb_len;

            dma_channel #(
                .CH_ID         (gi),
                .DATA_W        (DATA_W),
                .ADDR_W        (ADDR_W),
                .MAX_BURST_LEN (MAX_BURST_LEN)
            ) u_channel (
                .clk            (clk),
                .srst           (srst),
                .cfg_enable     (w_cfg_enable),
                .cfg_start      (w_cfg_start),
                .cfg_abort      (w_cfg_abort),
                .cfg_sg_en      (w_cfg_sg_en),
                .cfg_xfer_type  (xfer_type_t'(w_cfg_xfer_type)),
                .cfg_src_addr   (w_cfg_src_addr),
                .cfg_dst_addr   (w_cfg_dst_addr),
                .cfg_xfer_len   (w_cfg_xfer_len),
                .cfg_desc_addr  (w_cfg_desc_addr),
                .status         (w_status),
                .cur_src_addr   (w_cur_src),
                .cur_dst_addr   (w_cur_dst),
                .tc_pulse       (ch_tc[gi]),
                .err_pulse      (ch_err[gi]),
                .arb_req        (arb_req[gi]),
                .arb_req_type   (w_arb_req_type),
                .arb_addr       (w_arb_addr),
                .arb_len        (w_arb_len),
                .arb_grant      (arb_grant[gi]),
                .axi_rd_valid   (axi_rd_valid),
                .axi_rd_data    (axi_rd_data),
                .axi_rd_last    (axi_rd_last),
                .axi_rd_resp    (axi_rd_resp),
                .axi_rd_ready   (ch_axi_rd_ready[gi]),
                .axi_wr_valid   (ch_axi_wr_valid[gi]),
                .axi_wr_data    (ch_axi_wr_data[gi]),
                .axi_wr_last    (ch_axi_wr_last[gi]),
                .axi_wr_ready   (axi_wr_ready),
                .axi_wr_resp_valid (axi_wr_resp_valid),
                .axi_wr_resp    (axi_wr_resp),
                .dreq           (dreq[gi]),
                .dack           (ch_dack[gi])
            );
        end
    endgenerate

    // ================================================================
    // Mux channel AXI data signals based on grant
    // ================================================================
    always @(*) begin
        axi_rd_ready = 1'b0;
        axi_wr_valid = 1'b0;
        axi_wr_data  = {DATA_W{1'b0}};
        axi_wr_last  = 1'b0;

        for (int i = 0; i < NUM_CH; i++) begin
            if (arb_grant[i]) begin
                axi_rd_ready = ch_axi_rd_ready[i];
                axi_wr_valid = ch_axi_wr_valid[i];
                axi_wr_data  = ch_axi_wr_data[i];
                axi_wr_last  = ch_axi_wr_last[i];
            end
        end
    end

    // Mux dack outputs
    generate
        for (gi = 0; gi < NUM_CH; gi++) begin : gen_dack
            assign dack[gi] = ch_dack[gi];
        end
    endgenerate

    // ================================================================
    // Arbiter
    // ================================================================
    dma_arbiter #(
        .NUM_CH  (NUM_CH),
        .ARB_MODE(ARB_MODE)
    ) u_arbiter (
        .clk           (clk),
        .srst          (srst),
        .req           (arb_req),
        .req_type_flat (arb_req_type_flat),
        .req_addr_flat (arb_req_addr_flat),
        .req_len_flat  (arb_req_len_flat),
        .grant         (arb_grant),
        .grant_valid   (arb_grant_valid),
        .axi_req_type  (axi_req_type),
        .axi_req_addr  (axi_req_addr),
        .axi_req_len   (axi_req_len),
        .axi_req_valid (axi_req_valid),
        .axi_req_done  (axi_req_done)
    );

    // ================================================================
    // AXI Master
    // ================================================================
    dma_axi_master #(
        .DATA_W (DATA_W),
        .ADDR_W (ADDR_W)
    ) u_axi_master (
        .clk            (clk),
        .srst           (srst),
        .req_type       (axi_req_type),
        .req_addr       (axi_req_addr),
        .req_len        (axi_req_len),
        .req_valid      (axi_req_valid),
        .req_done       (axi_req_done),
        .req_error      (axi_req_error),
        .req_resp       (axi_req_resp),
        .rd_valid       (axi_rd_valid),
        .rd_data        (axi_rd_data),
        .rd_last        (axi_rd_last),
        .rd_resp        (axi_rd_resp),
        .rd_ready       (axi_rd_ready),
        .wr_valid       (axi_wr_valid),
        .wr_data        (axi_wr_data),
        .wr_last        (axi_wr_last),
        .wr_ready       (axi_wr_ready),
        .wr_resp_valid  (axi_wr_resp_valid),
        .wr_resp        (axi_wr_resp),
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
        .m_axi_rlast    (m_axi_rlast)
    );

endmodule
