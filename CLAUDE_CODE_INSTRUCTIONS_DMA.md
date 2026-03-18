# CLAUDE_CODE_INSTRUCTIONS_DMA.md

> **Repo**: `RISCV_DMA`
> **Author line** (first line of every source file): `// Brendan Lynskey 2025`
> **Target simulator**: `iverilog -g2012` + `cocotb` with `SIM=icarus`

---

## 1 — Project Overview

Design and verify a **synthesisable, multi-channel DMA controller** in SystemVerilog, suitable for integration into a RISC-V SoC. The DMA controller provides:

- **Parameterised multi-channel architecture** (default 4 channels).
- **AXI4 master interface** for memory-side burst transfers (INCR bursts, 32-bit data/address).
- **Simple memory-mapped register interface** for CPU configuration of channels.
- **Descriptor-chain (scatter-gather)** support — each channel follows a linked list of transfer descriptors stored in memory.
- **Transfer types**: memory-to-memory (primary), with optional memory-to-peripheral / peripheral-to-memory using `dreq`/`dack` handshake signals.
- **Per-channel interrupts** (transfer-complete, error) combined into a single `irq` output with a readable status register.
- **Round-robin or fixed-priority arbitration** between channels (parameterised).
- **AXI error handling**: SLVERR/DECERR responses halt the faulting channel and raise an interrupt.

The deliverables are: RTL source, SystemVerilog testbenches, CocoTB testbenches, simulation scripts, a technical report (`docs/dma_technical_report.md`), and a README.

---

## 2 — Architecture Specification

### 2.1 Features Table

| Feature | Detail |
|---|---|
| Channels | Parameterised, default `NUM_CH = 4` |
| Data width | 32 bits |
| Address width | 32 bits |
| AXI burst type | INCR (`2'b01`) |
| Max burst length | Parameterised, default `MAX_BURST_LEN = 16` (AXI4 AWLEN/ARLEN = 4'hF → 16 beats) |
| Register interface | Flat memory-mapped, one register set per channel, byte-addressable |
| Descriptor format | 5 × 32-bit words: `{next_desc_addr, src_addr, dst_addr, xfer_len, ctrl}` |
| Arbitration | Parameterised: `ARB_MODE = "ROUND_ROBIN"` or `"FIXED_PRIORITY"` |
| Interrupts | Per-channel TC and ERR, OR-reduced to single `irq` |
| Peripheral handshake | Optional `dreq[NUM_CH-1:0]` / `dack[NUM_CH-1:0]` |
| Reset | Synchronous active-high (`srst`) |
| Clock | Single clock domain (`clk`) |

### 2.2 Top-Level Interface — `dma_top`

```
module dma_top
  import dma_pkg::*;
#(
    parameter int NUM_CH        = 4,
    parameter int DATA_W        = 32,
    parameter int ADDR_W        = 32,
    parameter int MAX_BURST_LEN = 16,
    parameter     ARB_MODE      = "ROUND_ROBIN"   // "ROUND_ROBIN" or "FIXED_PRIORITY"
)(
    input  logic                clk,
    input  logic                srst,

    // ---- CPU register interface (simple memory-mapped) ----
    input  logic                reg_wr_en,
    input  logic                reg_rd_en,
    input  logic [11:0]         reg_addr,       // byte address into register space
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

    // ---- Peripheral handshake (active-high) ----
    input  logic [NUM_CH-1:0]   dreq,
    output logic [NUM_CH-1:0]   dack,

    // ---- Interrupt ----
    output logic                irq
);
```

### 2.3 Module Hierarchy

```
dma_top.sv
├── dma_pkg.sv                 // Package: parameters, typedefs, descriptor struct
├── dma_reg_file.sv            // CPU-facing register file
├── dma_channel.sv             // Per-channel FSM (instantiated NUM_CH times)
├── dma_arbiter.sv             // Round-robin / fixed-priority arbiter
├── dma_axi_master.sv          // AXI4 master burst read/write engine
└── dma_fifo.sv                // Simple synchronous FIFO (read data buffer)
```

### 2.4 Package — `dma_pkg`

File: `rtl/dma_pkg.sv`

```systemverilog
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
        CH_IDLE     = 3'd0,
        CH_DESC_FETCH = 3'd1,
        CH_READ     = 3'd2,
        CH_WRITE    = 3'd3,
        CH_DONE     = 3'd4,
        CH_ERROR    = 3'd5
    } ch_status_t;

    // ----- Arbiter request/grant -----
    typedef enum logic [1:0] {
        ARB_REQ_NONE  = 2'b00,
        ARB_REQ_READ  = 2'b01,
        ARB_REQ_WRITE = 2'b10,
        ARB_REQ_DESC  = 2'b11   // descriptor fetch (is a read)
    } arb_req_type_t;

endpackage
```

### 2.5 Register Map — `dma_reg_file`

Each channel occupies a 64-byte register block (16 × 32-bit registers, only first 8 used). Base address for channel `c` = `c * 64` (i.e., `c * 'h40`).

Global registers occupy offset `0x100` onwards.

#### Per-Channel Registers (base = `ch_num * 0x40`)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `0x00` | `CH_CTRL` | R/W | `[0]` enable, `[1]` start (write-1-to-start, self-clearing), `[2]` abort (write-1-to-abort, self-clearing), `[3]` sg_en (scatter-gather enable), `[5:4]` xfer_type (00=m2m, 01=m2p, 10=p2m) |
| `0x04` | `CH_STATUS` | RO | `[2:0]` ch_status_t, `[3]` tc (transfer complete, write-1-to-clear), `[4]` err (error, write-1-to-clear), `[15:8]` remaining burst count |
| `0x08` | `CH_SRC_ADDR` | R/W | Source address (for non-SG single transfer) |
| `0x0C` | `CH_DST_ADDR` | R/W | Destination address (for non-SG single transfer) |
| `0x10` | `CH_XFER_LEN` | R/W | Transfer length in bytes (for non-SG single transfer) |
| `0x14` | `CH_DESC_ADDR` | R/W | Pointer to first descriptor (when sg_en=1) |
| `0x18` | `CH_CUR_SRC` | RO | Current source address (debug) |
| `0x1C` | `CH_CUR_DST` | RO | Current destination address (debug) |

#### Global Registers (base = `0x100`)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `0x100` | `DMA_IRQ_STATUS` | RO | `[NUM_CH-1:0]` TC per channel, `[NUM_CH+15:16]` ERR per channel |
| `0x104` | `DMA_IRQ_ENABLE` | R/W | `[NUM_CH-1:0]` TC interrupt enable, `[NUM_CH+15:16]` ERR interrupt enable |
| `0x108` | `DMA_IRQ_CLEAR` | WO | Write-1-to-clear: same layout as IRQ_STATUS |
| `0x10C` | `DMA_VERSION` | RO | `32'h0001_0000` (v1.0) |

**Register file ports:**

```
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
    input  logic [ADDR_W-1:0]   addr,
    input  logic [DATA_W-1:0]   wr_data,
    output logic [DATA_W-1:0]   rd_data,
    output logic                rd_valid,

    // Per-channel config outputs (active channel config to dma_channel instances)
    output logic [NUM_CH-1:0]           ch_enable,
    output logic [NUM_CH-1:0]           ch_start,       // pulse
    output logic [NUM_CH-1:0]           ch_abort,       // pulse
    output logic [NUM_CH-1:0]           ch_sg_en,
    output xfer_type_t                  ch_xfer_type [NUM_CH],
    output logic [31:0]                 ch_src_addr  [NUM_CH],
    output logic [31:0]                 ch_dst_addr  [NUM_CH],
    output logic [31:0]                 ch_xfer_len  [NUM_CH],
    output logic [31:0]                 ch_desc_addr [NUM_CH],

    // Per-channel status inputs (from dma_channel instances)
    input  ch_status_t                  ch_status    [NUM_CH],
    input  logic [31:0]                 ch_cur_src   [NUM_CH],
    input  logic [31:0]                 ch_cur_dst   [NUM_CH],
    input  logic [NUM_CH-1:0]           ch_tc,          // pulse from channels
    input  logic [NUM_CH-1:0]           ch_err,         // pulse from channels

    // Interrupt output
    output logic                        irq
);
```

### 2.6 Channel FSM — `dma_channel`

Each channel instance manages the full lifecycle of a transfer or descriptor chain. It issues requests to the arbiter and coordinates with the AXI master.

**FSM states:**

```
typedef enum logic [3:0] {
    S_IDLE        = 4'd0,
    S_DESC_REQ    = 4'd1,   // Request arbiter for descriptor fetch
    S_DESC_WAIT   = 4'd2,   // Wait for 5-word descriptor read
    S_DESC_PARSE  = 4'd3,   // Parse descriptor, validate
    S_READ_REQ    = 4'd4,   // Request arbiter for read burst
    S_READ_WAIT   = 4'd5,   // Wait for read burst completion
    S_WRITE_REQ   = 4'd6,   // Request arbiter for write burst
    S_WRITE_WAIT  = 4'd7,   // Wait for write burst completion
    S_NEXT_BURST  = 4'd8,   // Update addresses, check remaining length
    S_NEXT_DESC   = 4'd9,   // Follow next_desc_addr (if SG)
    S_DONE        = 4'd10,
    S_ERROR       = 4'd11
} ch_state_t;
```

**Flow — non-scatter-gather (sg_en=0):**
1. CPU writes `CH_SRC_ADDR`, `CH_DST_ADDR`, `CH_XFER_LEN`, sets `CH_CTRL.enable=1`, writes `CH_CTRL.start=1`.
2. Channel goes `S_IDLE → S_READ_REQ → S_READ_WAIT → S_WRITE_REQ → S_WRITE_WAIT → S_NEXT_BURST` (loop until `xfer_len` exhausted) `→ S_DONE`.
3. On done: assert `tc` pulse, return to `S_IDLE`.

**Flow — scatter-gather (sg_en=1):**
1. CPU writes `CH_DESC_ADDR`, sets `CH_CTRL.sg_en=1, enable=1, start=1`.
2. Channel goes `S_IDLE → S_DESC_REQ → S_DESC_WAIT → S_DESC_PARSE → S_READ_REQ → ... → S_NEXT_BURST → S_NEXT_DESC → S_DESC_REQ → ...` until `next_desc_addr == 0` or `ctrl.last == 1`, then `→ S_DONE`.

**Burst splitting logic:**
- `remaining = xfer_len` (bytes).
- `burst_len = min(MAX_BURST_LEN, remaining / (DATA_W/8))` (in beats).
- After each read+write burst, `remaining -= burst_len * (DATA_W/8)`, `src_addr += burst_len * (DATA_W/8)`, `dst_addr += burst_len * (DATA_W/8)`.
- When `remaining == 0`, either `S_NEXT_DESC` (SG) or `S_DONE`.

**Peripheral handshake logic:**
- If `xfer_type == MEM2PERIPH`: before each write beat, wait for `dreq[ch_id]`; assert `dack[ch_id]` during write.
- If `xfer_type == PERIPH2MEM`: before each read beat, wait for `dreq[ch_id]`; assert `dack[ch_id]` during read.
- If `xfer_type == MEM2MEM`: ignore `dreq`, `dack` held low.

**Ports:**

```
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

    // AXI data (channel↔AXI master, active only when granted)
    // Read path
    input  logic                axi_rd_valid,
    input  logic [DATA_W-1:0]   axi_rd_data,
    input  logic                axi_rd_last,
    input  logic [1:0]          axi_rd_resp,
    output logic                axi_rd_ready,

    // Write path
    output logic                axi_wr_valid,
    output logic [DATA_W-1:0]   axi_wr_data,
    output logic                axi_wr_last,
    input  logic                axi_wr_ready,

    // Write response
    input  logic                axi_wr_resp_valid,
    input  logic [1:0]          axi_wr_resp,

    // Peripheral handshake
    input  logic                dreq,
    output logic                dack
);
```

**Internal data buffer:**  
Each channel contains a small FIFO (`dma_fifo`, depth = `MAX_BURST_LEN`) to buffer data between read and write phases.

### 2.7 Arbiter — `dma_arbiter`

Receives requests from all channels, grants one at a time. Holds the grant until the granted channel de-asserts `arb_req`.

**Round-robin mode**: Rotate priority pointer after each completed grant.
**Fixed-priority mode**: Channel 0 = highest priority.

**Ports:**

```
module dma_arbiter
  import dma_pkg::*;
#(
    parameter int NUM_CH   = 4,
    parameter     ARB_MODE = "ROUND_ROBIN"
)(
    input  logic                clk,
    input  logic                srst,

    // Channel requests
    input  logic [NUM_CH-1:0]   req,
    input  arb_req_type_t       req_type [NUM_CH],
    input  logic [31:0]         req_addr [NUM_CH],
    input  logic [7:0]          req_len  [NUM_CH],

    // Grants
    output logic [NUM_CH-1:0]   grant,
    output logic                grant_valid,

    // Forwarded to AXI master
    output arb_req_type_t       axi_req_type,
    output logic [31:0]         axi_req_addr,
    output logic [7:0]          axi_req_len,
    output logic                axi_req_valid,
    input  logic                axi_req_done      // AXI master signals completion
);
```

### 2.8 AXI Master — `dma_axi_master`

Drives the AXI4 bus. Receives a request (read or write, address, burst length) from the arbiter and executes it.

**FSM states:**

```
typedef enum logic [2:0] {
    AXI_IDLE     = 3'd0,
    AXI_AR       = 3'd1,   // Drive read address channel
    AXI_R        = 3'd2,   // Receive read data
    AXI_AW       = 3'd3,   // Drive write address channel
    AXI_W        = 3'd4,   // Drive write data channel
    AXI_B        = 3'd5    // Wait for write response
} axi_state_t;
```

**Ports:**

```
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
    // Read data out to channel
    output logic                rd_valid,
    output logic [DATA_W-1:0]   rd_data,
    output logic                rd_last,
    output logic [1:0]          rd_resp,
    input  logic                rd_ready,

    // Write data in from channel
    input  logic                wr_valid,
    input  logic [DATA_W-1:0]   wr_data,
    input  logic                wr_last,
    output logic                wr_ready,

    // Write response out to channel
    output logic                wr_resp_valid,
    output logic [1:0]          wr_resp,

    // AXI4 Master signals (directly to dma_top ports)
    // Write Address
    output logic                m_axi_awvalid,
    input  logic                m_axi_awready,
    output logic [ADDR_W-1:0]   m_axi_awaddr,
    output logic [7:0]          m_axi_awlen,
    output logic [2:0]          m_axi_awsize,
    output logic [1:0]          m_axi_awburst,
    // Write Data
    output logic                m_axi_wvalid,
    input  logic                m_axi_wready,
    output logic [DATA_W-1:0]   m_axi_wdata,
    output logic [DATA_W/8-1:0] m_axi_wstrb,
    output logic                m_axi_wlast,
    // Write Response
    input  logic                m_axi_bvalid,
    output logic                m_axi_bready,
    input  logic [1:0]          m_axi_bresp,
    // Read Address
    output logic                m_axi_arvalid,
    input  logic                m_axi_arready,
    output logic [ADDR_W-1:0]   m_axi_araddr,
    output logic [7:0]          m_axi_arlen,
    output logic [2:0]          m_axi_arsize,
    output logic [1:0]          m_axi_arburst,
    // Read Data
    input  logic                m_axi_rvalid,
    output logic                m_axi_rready,
    input  logic [DATA_W-1:0]   m_axi_rdata,
    input  logic [1:0]          m_axi_rresp,
    input  logic                m_axi_rlast
);
```

**AXI protocol notes:**
- `AWSIZE` / `ARSIZE` = `3'b010` (4 bytes) always for 32-bit data.
- `AWBURST` / `ARBURST` = `2'b01` (INCR).
- `WSTRB` = `4'b1111` for full-word writes.
- `AWLEN` / `ARLEN` = burst length minus 1 (0-based as per AXI4 spec).

### 2.9 FIFO — `dma_fifo`

Simple synchronous FIFO used within each channel to decouple read and write phases.

**Ports:**

```
module dma_fifo
#(
    parameter int DATA_W = 32,
    parameter int DEPTH  = 16
)(
    input  logic                clk,
    input  logic                srst,

    // Write port
    input  logic                wr_en,
    input  logic [DATA_W-1:0]   wr_data,
    output logic                full,

    // Read port
    input  logic                rd_en,
    output logic [DATA_W-1:0]   rd_data,
    output logic                empty,

    // Status
    output logic [$clog2(DEPTH):0] count
);
```

Implementation: pointer-based (wr_ptr, rd_ptr), register array. Full when `count == DEPTH`, empty when `count == 0`.

---

## 3 — Coding Conventions

### 3.1 Language and Tooling
- **SystemVerilog** (`.sv`), targeting `iverilog -g2012`.
- **CRITICAL iverilog rule**: Use `always @(*)` instead of `always_comb` for any combinational block that reads signals driven by submodule outputs. This avoids iverilog infinite re-evaluation loops. Use `always_comb` only for purely local combinational logic (no sub-module outputs). When in doubt, use `always @(*)`.
- Use `always_ff`, `logic`, `typedef enum`, packages for all other SystemVerilog constructs.
- No vendor-specific primitives (no Xilinx, Intel, etc.).

### 3.2 Reset
- Synchronous, active-high: signal name `srst`.
- Every `always_ff` block: `always_ff @(posedge clk)` with `if (srst)` as the first branch.

### 3.3 FSM Pattern
```systemverilog
typedef enum logic [N:0] { S_A, S_B, ... } state_t;
state_t state, state_next;

always_ff @(posedge clk)
    if (srst) state <= S_A;
    else      state <= state_next;

always @(*) begin       // Use always @(*) because arbiter grant comes from submodule
    state_next = state;
    // default outputs...
    case (state)
        S_A: ...
        S_B: ...
    endcase
end
```

### 3.4 Handshake
- All inter-module interfaces use `valid` / `ready` (or AXI `VALID`/`READY`).
- Transaction occurs on the cycle where both `valid && ready` are high.

### 3.5 Naming
- `snake_case` for all signals, modules, files.
- Prefix testbenches with `tb_`.
- Prefix CocoTB test files with `test_`.
- Prefix AXI master outputs with `m_axi_`.
- Channel array indices: `[NUM_CH-1:0]` for packed, `[NUM_CH]` for unpacked.

### 3.6 File Headers
First line of every `.sv`, `.v`, `.py` file:
```
// Brendan Lynskey 2025
```
(Use `#` comment for Python.)

### 3.7 Parameters
- Module-level parameters via `#( parameter int NAME = VALUE )`.
- Package-level parameters in `dma_pkg.sv` for shared types and constants.

---

## 4 — File Structure

```
RISCV_DMA/
├── rtl/
│   ├── dma_pkg.sv
│   ├── dma_fifo.sv
│   ├── dma_reg_file.sv
│   ├── dma_axi_master.sv
│   ├── dma_arbiter.sv
│   ├── dma_channel.sv
│   └── dma_top.sv
├── tb/
│   ├── sv/
│   │   ├── tb_dma_fifo.sv
│   │   ├── tb_dma_reg_file.sv
│   │   ├── tb_dma_axi_master.sv
│   │   ├── tb_dma_arbiter.sv
│   │   ├── tb_dma_channel.sv
│   │   └── tb_dma_top.sv
│   └── cocotb/
│       ├── test_dma_fifo/
│       │   ├── test_dma_fifo.py
│       │   └── Makefile
│       ├── test_dma_reg_file/
│       │   ├── test_dma_reg_file.py
│       │   └── Makefile
│       ├── test_dma_axi_master/
│       │   ├── test_dma_axi_master.py
│       │   └── Makefile
│       ├── test_dma_arbiter/
│       │   ├── test_dma_arbiter.py
│       │   └── Makefile
│       ├── test_dma_channel/
│       │   ├── test_dma_channel.py
│       │   └── Makefile
│       └── test_dma_top/
│           ├── test_dma_top.py
│           └── Makefile
├── scripts/
│   ├── run_sv_tests.sh       # Compile & run all SV testbenches
│   ├── run_cocotb_tests.sh   # Run all CocoTB tests
│   └── run_all.sh            # Run both
├── docs/
│   └── dma_technical_report.md
├── CLAUDE_CODE_INSTRUCTIONS_DMA.md
└── README.md
```

---

## 5 — Implementation Order (Bottom-Up)

### Phase 1: Package and FIFO
1. **`dma_pkg.sv`** — Define all types, parameters, structs.
2. **`dma_fifo.sv`** — Implement pointer-based synchronous FIFO.
3. **`tb_dma_fifo.sv`** — Verify FIFO (see §6).
4. **CocoTB: `test_dma_fifo.py`** — Verify FIFO.

### Phase 2: Register File
5. **`dma_reg_file.sv`** — Implement per-channel registers + globals.
6. **`tb_dma_reg_file.sv`** — Verify register reads/writes, W1C, pulse registers.
7. **CocoTB: `test_dma_reg_file.py`** — Verify register file.

### Phase 3: AXI Master
8. **`dma_axi_master.sv`** — Implement AXI4 burst read/write FSM.
9. **`tb_dma_axi_master.sv`** — Verify with AXI slave stub (testbench memory model).
10. **CocoTB: `test_dma_axi_master.py`** — Verify AXI master.

### Phase 4: Arbiter
11. **`dma_arbiter.sv`** — Implement round-robin and fixed-priority.
12. **`tb_dma_arbiter.sv`** — Verify arbitration fairness and hold.
13. **CocoTB: `test_dma_arbiter.py`** — Verify arbiter.

### Phase 5: Channel
14. **`dma_channel.sv`** — Implement full channel FSM with internal FIFO.
15. **`tb_dma_channel.sv`** — Verify single-transfer and SG chains.
16. **CocoTB: `test_dma_channel.py`** — Verify channel.

### Phase 6: Top-Level Integration
17. **`dma_top.sv`** — Wire everything together.
18. **`tb_dma_top.sv`** — Full system-level tests with AXI memory model.
19. **CocoTB: `test_dma_top.py`** — Full system CocoTB tests.

### Phase 7: Documentation and Scripts
20. Write `scripts/run_sv_tests.sh`, `scripts/run_cocotb_tests.sh`, `scripts/run_all.sh`.
21. Write `docs/dma_technical_report.md`.
22. Write `README.md`.
23. Update parent hardware index.

---

## 6 — Verification Requirements

### Minimum Test Counts

| Module | SV Tests | CocoTB Tests | Total |
|--------|----------|--------------|-------|
| `dma_fifo` | 8 | 6 | 14 |
| `dma_reg_file` | 12 | 8 | 20 |
| `dma_axi_master` | 12 | 8 | 20 |
| `dma_arbiter` | 8 | 5 | 13 |
| `dma_channel` | 12 | 8 | 20 |
| `dma_top` | 12 | 8 | 20 |
| **Total** | **64** | **43** | **107** |

### 6.1 FIFO Tests (`tb_dma_fifo` / `test_dma_fifo`)

**SV tests (≥8):**
1. Reset clears FIFO — verify `empty=1`, `full=0`, `count=0` after reset.
2. Single write/read — write one word, read it back, verify data.
3. Fill to full — write DEPTH words, verify `full=1`, `count=DEPTH`.
4. Full write rejected — write when full, verify count doesn't increment.
5. Drain from full — read all DEPTH words, verify `empty=1`.
6. Empty read — read when empty, verify no change.
7. Concurrent write/read — write and read simultaneously at various fill levels.
8. FIFO data integrity — write known pattern (0xDEAD0000+i), read back, verify order.

**CocoTB tests (≥6):**
1. `test_reset` — Verify clean state after reset.
2. `test_single_word` — Write/read single word.
3. `test_fill_drain` — Fill completely, drain completely.
4. `test_full_flag` — Verify full assertion/de-assertion.
5. `test_empty_flag` — Verify empty assertion/de-assertion.
6. `test_data_integrity` — Randomised write/read with golden model comparison.

### 6.2 Register File Tests (`tb_dma_reg_file` / `test_dma_reg_file`)

**SV tests (≥12):**
1. Reset defaults — all registers return 0 (except VERSION).
2. Write/read CH_SRC_ADDR for channel 0.
3. Write/read CH_DST_ADDR for channel 0.
4. Write/read CH_XFER_LEN for channel 0.
5. Write/read CH_DESC_ADDR for channel 0.
6. Write/read CH_CTRL — verify enable, sg_en, xfer_type outputs.
7. CH_CTRL start pulse — write start bit, verify 1-cycle pulse, then auto-clear.
8. CH_STATUS read — drive ch_status input, verify read-back.
9. Multi-channel — write registers for channel 0, 1, 2, 3, read back all.
10. IRQ_STATUS set/clear — drive tc/err pulses, read IRQ_STATUS, write IRQ_CLEAR, verify cleared.
11. IRQ generation — enable IRQ, trigger TC, verify `irq` output asserted.
12. VERSION register — verify reads `32'h0001_0000`.

**CocoTB tests (≥8):**
1. `test_reset` — All registers at defaults.
2. `test_channel_config` — Write/read per-channel registers.
3. `test_start_pulse` — Verify start bit generates single pulse.
4. `test_abort_pulse` — Verify abort bit generates single pulse.
5. `test_status_readback` — Drive status, verify read.
6. `test_irq_status` — TC/ERR pulse → IRQ_STATUS.
7. `test_irq_clear` — Write-1-to-clear.
8. `test_irq_output` — IRQ enable + status → irq pin.

### 6.3 AXI Master Tests (`tb_dma_axi_master` / `test_dma_axi_master`)

The testbench must include an **AXI4 slave memory model** (simple array-based, responds to reads/writes with configurable latency, can inject SLVERR/DECERR).

**SV tests (≥12):**
1. Single-beat read — request 1-beat read, verify ARVALID/ARADDR/ARLEN, receive data.
2. Burst read (4 beats) — verify ARLEN=3, receive 4 data words with RLAST on last.
3. Max burst read (16 beats) — verify full burst.
4. Single-beat write — request 1-beat write, verify AWVALID/AWADDR, send 1 data word with WLAST=1, receive BRESP=OKAY.
5. Burst write (4 beats) — verify WLAST on 4th beat.
6. Max burst write (16 beats) — full burst write.
7. Read with SLVERR — slave returns RRESP=SLVERR, verify `req_error` asserted.
8. Write with DECERR — slave returns BRESP=DECERR, verify `req_error` asserted.
9. Back-to-back reads — issue two reads sequentially.
10. Back-to-back writes — issue two writes sequentially.
11. Read then write — verify state machine transitions correctly.
12. Slave stall — slave holds ARREADY/WREADY low for several cycles, verify DMA master waits.

**CocoTB tests (≥8):**
1. `test_single_read` — 1-beat read.
2. `test_burst_read` — Multi-beat read.
3. `test_single_write` — 1-beat write.
4. `test_burst_write` — Multi-beat write.
5. `test_read_error` — SLVERR on read.
6. `test_write_error` — DECERR on write.
7. `test_back_to_back` — Sequential operations.
8. `test_slave_stall` — Stall scenarios.

### 6.4 Arbiter Tests (`tb_dma_arbiter` / `test_dma_arbiter`)

**SV tests (≥8):**
1. No requests — verify `grant_valid=0`.
2. Single request — channel 0 requests, verify granted.
3. Two simultaneous — channels 0 and 1, verify round-robin grants 0 first (pointer starts at 0).
4. Round-robin fairness — all 4 channels request, verify cycling grants.
5. Hold-until-done — grant held while `axi_req_done` not asserted.
6. Fixed-priority mode — channel 2 and 0 request, verify 0 always wins.
7. Priority pointer update — after granting channel 1, pointer advances to 2.
8. Grant de-assertion — requester drops `req`, verify `grant_valid` de-asserts.

**CocoTB tests (≥5):**
1. `test_no_requests` — Idle state.
2. `test_single_request` — One channel.
3. `test_round_robin` — Fairness.
4. `test_fixed_priority` — Priority override.
5. `test_hold_grant` — Grant persistence.

### 6.5 Channel Tests (`tb_dma_channel` / `test_dma_channel`)

Requires a mock arbiter + mock AXI data path (testbench drives `arb_grant`, `axi_rd_valid`, etc.).

**SV tests (≥12):**
1. Reset — verify `status == CH_IDLE`.
2. Simple 4-byte transfer — 1 beat, non-SG, verify src→dst data, `tc_pulse`.
3. 16-byte transfer — 4 beats, single burst.
4. 64-byte transfer — 16 beats (max burst).
5. 128-byte transfer — splits into 2 bursts of 16 beats.
6. Abort mid-transfer — assert `cfg_abort`, verify return to IDLE.
7. AXI read error — inject error response, verify `status == CH_ERROR`, `err_pulse`.
8. AXI write error — inject write error, verify `status == CH_ERROR`.
9. Scatter-gather: 1 descriptor — single descriptor, `next=0`.
10. Scatter-gather: 2-descriptor chain — follow linked list.
11. Scatter-gather: descriptor with `ctrl.last=1` — terminates chain.
12. Peripheral handshake (M2P) — verify `dack` asserted during writes when `dreq` high.

**CocoTB tests (≥8):**
1. `test_idle_state` — Reset verification.
2. `test_simple_transfer` — Small transfer.
3. `test_multi_burst` — Transfer requiring multiple bursts.
4. `test_abort` — Mid-transfer abort.
5. `test_read_error` — Error handling.
6. `test_sg_single` — Single descriptor.
7. `test_sg_chain` — Multi-descriptor chain.
8. `test_periph_handshake` — dreq/dack.

### 6.6 Top-Level Tests (`tb_dma_top` / `test_dma_top`)

Requires a full **AXI4 slave memory model** instantiated inside the testbench. The memory model holds both source data and receives destination writes. Descriptors for SG tests are pre-loaded into the memory model.

**SV tests (≥12):**
1. Reset — all AXI outputs deasserted, `irq=0`.
2. Register access — write/read channel 0 registers via register interface.
3. Simple M2M transfer (1 burst) — write src data to AXI mem, configure channel, start, verify dst in AXI mem.
4. Multi-burst M2M transfer — 256 bytes (4 bursts of 16×4B).
5. Two channels simultaneously — channels 0 and 1 transfer concurrently, verify both complete.
6. All 4 channels — stress test, all channels transfer.
7. Scatter-gather (2 descriptors) — pre-load descriptors in memory, verify chain traversal.
8. Scatter-gather (4 descriptors) — longer chain.
9. AXI error propagation — slave returns error, verify channel halts and `irq` asserted.
10. Interrupt enable/clear — verify IRQ_ENABLE masks, IRQ_CLEAR clears.
11. Abort then restart — abort channel, reconfigure, restart, verify success.
12. Version register — read `0x10C`, verify `0x0001_0000`.

**CocoTB tests (≥8):**
1. `test_reset` — Clean state.
2. `test_register_access` — Config round-trip.
3. `test_simple_m2m` — Basic transfer.
4. `test_multi_burst_m2m` — Large transfer.
5. `test_concurrent_channels` — Two channels simultaneously.
6. `test_scatter_gather` — Descriptor chain.
7. `test_error_handling` — AXI error → IRQ.
8. `test_irq_flow` — Full IRQ lifecycle.

---

## 7 — Simulation & Debug Workflow

### 7.1 SV Compilation Commands

**Single module (example: FIFO):**
```bash
iverilog -g2012 -o tb_dma_fifo.vvp \
    rtl/dma_pkg.sv \
    rtl/dma_fifo.sv \
    tb/sv/tb_dma_fifo.sv
vvp tb_dma_fifo.vvp
```

**Top-level:**
```bash
iverilog -g2012 -o tb_dma_top.vvp \
    rtl/dma_pkg.sv \
    rtl/dma_fifo.sv \
    rtl/dma_reg_file.sv \
    rtl/dma_axi_master.sv \
    rtl/dma_arbiter.sv \
    rtl/dma_channel.sv \
    rtl/dma_top.sv \
    tb/sv/tb_dma_top.sv
vvp tb_dma_top.vvp
```

**Always list `dma_pkg.sv` first** in the compilation order.

### 7.2 CocoTB Makefile Template

```makefile
# Brendan Lynskey 2025
SIM = icarus
VERILOG_SOURCES = \
    $(shell pwd)/../../../rtl/dma_pkg.sv \
    $(shell pwd)/../../../rtl/dma_fifo.sv
TOPLEVEL = dma_fifo
MODULE = test_dma_fifo
TOPLEVEL_LANG = verilog
COMPILE_ARGS = -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
```

### 7.3 Debug Loop

1. Write module → compile with `iverilog -g2012` → fix compile errors.
2. Run SV testbench → fix failures → iterate until all `[PASS]`.
3. Run CocoTB test → fix failures → iterate until all pass.
4. Move to next module.

### 7.4 iverilog Gotchas

- **`always_comb` re-evaluation loops**: Use `always @(*)` for combinational blocks reading submodule outputs or signals on inter-module boundaries. `always_comb` is fine for local decode logic.
- **Package import order**: `dma_pkg.sv` must be first in the compile list.
- **Enum width**: Always specify explicit width for `typedef enum logic [N:0]`.
- **Unpacked arrays in ports**: iverilog -g2012 supports unpacked array ports, but if issues arise, flatten to packed and slice in the instantiation.
- **`$stop` vs `$finish`**: Use `$stop` in testbenches (allows waveform inspection). Use `$finish` only at the very end of successful completion.
- **SystemVerilog structs through ports**: If iverilog has trouble with struct ports, pass as packed `logic [159:0]` and cast inside the module.

### 7.5 SV Testbench Template

```systemverilog
// Brendan Lynskey 2025
`timescale 1ns / 1ps

module tb_dma_fifo;
    import dma_pkg::*;

    // Parameters
    localparam DATA_W = 32;
    localparam DEPTH  = 16;
    localparam CLK_PERIOD = 10;

    // Signals
    logic clk, srst;
    // ... DUT signals ...

    // DUT
    dma_fifo #(.DATA_W(DATA_W), .DEPTH(DEPTH)) u_dut ( .* );

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
            $stop;
        end
    endtask

    task automatic reset_dut();
        srst = 1;
        repeat (3) @(posedge clk);
        srst = 0;
        @(posedge clk);
    endtask

    // Tests
    initial begin
        $dumpfile("tb_dma_fifo.vcd");
        $dumpvars(0, tb_dma_fifo);

        reset_dut();

        // Test 1: Reset clears FIFO
        // ...

        // Summary
        $display("========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d", test_count, pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end
endmodule
```

---

## 8 — README Template

The `README.md` should follow this structure:

```markdown
# RISCV_DMA — Multi-Channel DMA Controller

A synthesisable, parameterised multi-channel DMA controller in SystemVerilog,
designed for RISC-V SoC integration. Features AXI4 master interface,
scatter-gather descriptor chains, round-robin/fixed-priority arbitration,
and per-channel interrupt generation.

## Features

- Multi-channel (default 4), parameterised
- AXI4 master with INCR burst support (up to 16 beats)
- Memory-mapped register interface for CPU configuration
- Scatter-gather descriptor chain support
- Memory-to-memory, memory-to-peripheral, peripheral-to-memory transfers
- Per-channel transfer-complete and error interrupts
- Round-robin or fixed-priority channel arbitration
- AXI error detection and channel halt

## Architecture

```
dma_top
├── dma_pkg          — Parameters, types, descriptor struct
├── dma_reg_file     — CPU register interface
├── dma_channel ×N   — Per-channel FSM with internal FIFO
│   └── dma_fifo     — Read data buffer
├── dma_arbiter      — Multi-channel arbitration
└── dma_axi_master   — AXI4 burst read/write engine
```

## Register Map

| Offset | Name | Description |
|--------|------|-------------|
| Per-channel (base = ch × 0x40) | | |
| +0x00 | CH_CTRL | Enable, start, abort, SG enable, transfer type |
| +0x04 | CH_STATUS | Channel state, TC, error flags |
| +0x08 | CH_SRC_ADDR | Source address |
| +0x0C | CH_DST_ADDR | Destination address |
| +0x10 | CH_XFER_LEN | Transfer length (bytes) |
| +0x14 | CH_DESC_ADDR | First descriptor pointer |
| Global registers | | |
| 0x100 | DMA_IRQ_STATUS | Interrupt status |
| 0x104 | DMA_IRQ_ENABLE | Interrupt enable mask |
| 0x108 | DMA_IRQ_CLEAR | Write-1-to-clear |
| 0x10C | DMA_VERSION | Version (0x00010000) |

## Simulation

### Prerequisites
- iverilog (≥ 11.0) with `-g2012` support
- cocotb (≥ 1.8) with icarus backend

### Run all tests
```bash
./scripts/run_all.sh
```

### Run SV tests only
```bash
./scripts/run_sv_tests.sh
```

### Run CocoTB tests only
```bash
./scripts/run_cocotb_tests.sh
```

## Test Results

| Module | SV Tests | CocoTB Tests | Status |
|--------|----------|--------------|--------|
| dma_fifo | X/8 | X/6 | ✅ |
| dma_reg_file | X/12 | X/8 | ✅ |
| dma_axi_master | X/12 | X/8 | ✅ |
| dma_arbiter | X/8 | X/5 | ✅ |
| dma_channel | X/12 | X/8 | ✅ |
| dma_top | X/12 | X/8 | ✅ |
| **Total** | **X/64** | **X/43** | |

## Author

Brendan Lynskey 2025

## Licence

MIT
```

*Replace `X` with actual pass counts after verification.*

---

## 9 — Hardware Index Update

After all tests pass, update the parent hardware index repo:

**Repository**: `https://github.com/BrendanJamesLynskey/Hardware`

**Action**: **Merge** (never replace) a new row into the project table in the `README.md`:

```markdown
| RISCV_DMA | Multi-channel DMA controller — AXI4 master, scatter-gather, round-robin/fixed-priority arbiter | [View](https://github.com/BrendanJamesLynskey/RISCV_DMA) |
```

Do **not** modify, reorder, or remove any existing rows. Append at the appropriate alphabetical position or at the end.

---

## 10 — Stretch Goals

These are **optional** enhancements to implement only after all core tests pass:

1. **AXI4 WRAP burst support** — Add `AWBURST/ARBURST = 2'b10` (wrapping) as a configurable option per descriptor.
2. **Byte-level transfer granularity** — Support non-word-aligned transfers using `WSTRB` partial strobes on first/last beats.
3. **Channel linking** — Completion of one channel triggers automatic start of another channel (chain channels, not just descriptors).
4. **2D / strided transfers** — Add stride registers to support 2D block transfers (useful for image processing DMA).
5. **Data width parameterisation** — Test with `DATA_W = 64` and `DATA_W = 128`, adjusting `AWSIZE`/`ARSIZE` accordingly.
6. **Performance counters** — Cycle counter per channel: total transfer time, bus stall cycles.
7. **AXI outstanding transactions** — Support multiple outstanding read requests (pipelining AR and R channels).

---

## 11 — Checklist — Definition of Done

- [ ] All RTL files in `rtl/` compile cleanly with `iverilog -g2012` (zero warnings desired, zero errors mandatory).
- [ ] `dma_pkg.sv` defines all types, structs, parameters — shared across all modules.
- [ ] `dma_fifo.sv` — 8+ SV tests pass, 6+ CocoTB tests pass.
- [ ] `dma_reg_file.sv` — 12+ SV tests pass, 8+ CocoTB tests pass.
- [ ] `dma_axi_master.sv` — 12+ SV tests pass, 8+ CocoTB tests pass.
- [ ] `dma_arbiter.sv` — 8+ SV tests pass, 5+ CocoTB tests pass.
- [ ] `dma_channel.sv` — 12+ SV tests pass, 8+ CocoTB tests pass.
- [ ] `dma_top.sv` — 12+ SV tests pass, 8+ CocoTB tests pass.
- [ ] **64+ SV tests total, 43+ CocoTB tests total** — all passing.
- [ ] `scripts/run_all.sh` runs everything and reports pass/fail summary.
- [ ] `docs/dma_technical_report.md` — covers architecture, register map, descriptor format, arbitration, test methodology, and results.
- [ ] `README.md` — follows template in §8, with actual test counts.
- [ ] Hardware index updated (merge, never replace).
- [ ] No vendor-specific primitives.
- [ ] All files have `// Brendan Lynskey 2025` as first line.
- [ ] No `always_comb` blocks reading submodule outputs — use `always @(*)` for those.
