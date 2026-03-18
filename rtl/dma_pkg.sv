// Brendan Lynskey 2025
package dma_pkg;

    // ----- Global parameters (overridden at top) -----
    parameter int DMA_DATA_W        = 32;
    parameter int DMA_ADDR_W        = 32;
    parameter int DMA_MAX_BURST_LEN = 16;
    parameter int DMA_NUM_CH        = 4;

    // ----- Descriptor struct (5 x 32-bit = 160 bits) -----
    typedef struct packed {
        logic [31:0] next_desc_addr;   // [159:128] — 0x0000_0000 = end of chain
        logic [31:0] src_addr;         // [127:96]
        logic [31:0] dst_addr;         // [95:64]
        logic [31:0] xfer_len;         // [63:32]  — transfer length in bytes
        logic [31:0] ctrl;             // [31:0]   — see bit fields below
    } dma_desc_t;                      // 160 bits total

    // ----- Descriptor ctrl field bit definitions -----
    //   [0]     — enable: 1 = descriptor is valid
    //   [1]     — irq_en: 1 = generate TC interrupt on completion
    //   [3:2]   — xfer_type: 00 = mem-to-mem, 01 = mem-to-periph, 10 = periph-to-mem
    //   [4]     — last: 1 = ignore next_desc_addr (explicit last)
    //   [31:5]  — reserved

    localparam int DESC_CTRL_EN_BIT       = 0;
    localparam int DESC_CTRL_IRQ_EN_BIT   = 1;
    localparam int DESC_CTRL_XTYPE_LO     = 2;
    localparam int DESC_CTRL_XTYPE_HI     = 3;
    localparam int DESC_CTRL_LAST_BIT     = 4;

    // ----- Transfer type encoding -----
    typedef enum logic [1:0] {
        XFER_MEM2MEM    = 2'b00,
        XFER_MEM2PERIPH = 2'b01,
        XFER_PERIPH2MEM = 2'b10,
        XFER_RESERVED   = 2'b11
    } xfer_type_t;

    // ----- Channel status (readable via register file) -----
    typedef enum logic [2:0] {
        CH_IDLE       = 3'd0,
        CH_DESC_FETCH = 3'd1,
        CH_READ       = 3'd2,
        CH_WRITE      = 3'd3,
        CH_DONE       = 3'd4,
        CH_ERROR      = 3'd5
    } ch_status_t;

    // ----- Arbiter request/grant -----
    typedef enum logic [1:0] {
        ARB_REQ_NONE  = 2'b00,
        ARB_REQ_READ  = 2'b01,
        ARB_REQ_WRITE = 2'b10,
        ARB_REQ_DESC  = 2'b11   // descriptor fetch (is a read)
    } arb_req_type_t;

endpackage
