# DMA Controller Technical Report

**Project:** RISCV_DMA -- Multi-Channel DMA Controller
**Author:** Brendan Lynskey 2025
**Target Simulator:** iverilog -g2012, cocotb with SIM=icarus

---

## 1. Architecture Overview

The RISCV_DMA controller is a synthesisable, parameterised multi-channel DMA engine designed for integration into a RISC-V SoC. It provides AXI4 master burst transfers, scatter-gather descriptor chains, configurable arbitration, and per-channel interrupt generation.

### 1.1 Module Hierarchy

The design is composed of six modules arranged in the following hierarchy:

```
dma_top.sv
├── dma_pkg.sv                 // Package: parameters, typedefs, descriptor struct
├── dma_reg_file.sv            // CPU-facing register file (1 instance)
├── dma_channel.sv             // Per-channel FSM (NUM_CH instances, default 4)
│   └── dma_fifo.sv            // Synchronous FIFO read-data buffer (1 per channel)
├── dma_arbiter.sv             // Round-robin / fixed-priority arbiter (1 instance)
└── dma_axi_master.sv          // AXI4 master burst read/write engine (1 instance)
```

**dma_top** is the top-level integration module. It instantiates all sub-modules, wires the register file outputs to each channel's configuration inputs, connects each channel's arbiter request signals to the arbiter, and multiplexes the granted channel's AXI data path onto the shared AXI master. A grant-based combinational mux selects which channel drives `axi_wr_valid`, `axi_wr_data`, `axi_wr_last`, and `axi_rd_ready` based on the arbiter's one-hot `grant` vector.

**dma_pkg** defines shared types and constants used across all modules: the `dma_desc_t` packed struct (160-bit descriptor), `xfer_type_t` enumeration for transfer types, `ch_status_t` for channel status, `arb_req_type_t` for arbiter request classification, and descriptor control-field bit-position localparams.

**dma_reg_file** provides the CPU-facing memory-mapped register interface. It decodes the 12-bit byte address to select per-channel registers (address bits [7:6] select the channel, bits [5:0] select the register offset) or global registers (addresses >= 0x100). It outputs configuration signals to channels and latches status/interrupt inputs from them.

**dma_channel** implements the full transfer lifecycle as a 12-state FSM. Each instance contains one `dma_fifo` to decouple the read and write phases of a burst. It issues arbiter requests, coordinates with the AXI master data path, parses scatter-gather descriptors, and generates transfer-complete or error pulses.

**dma_arbiter** receives requests from all channels and grants bus access to one at a time. It supports parameterised round-robin (default) or fixed-priority arbitration. The grant is held until the AXI master signals transaction completion via `axi_req_done`.

**dma_axi_master** drives the AXI4 bus interface. It accepts a request (type, address, burst length) from the arbiter and executes the corresponding AXI read or write transaction using a 6-state FSM.

### 1.2 Top-Level Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_CH` | 4 | Number of DMA channels |
| `DATA_W` | 32 | Data width in bits |
| `ADDR_W` | 32 | Address width in bits |
| `MAX_BURST_LEN` | 16 | Maximum AXI burst length in beats |
| `ARB_MODE` | `"ROUND_ROBIN"` | Arbitration mode (`"ROUND_ROBIN"` or `"FIXED_PRIORITY"`) |

### 1.3 Top-Level Interface

The top-level module `dma_top` exposes:

- **CPU register interface:** `reg_wr_en`, `reg_rd_en`, `reg_addr[11:0]`, `reg_wr_data[31:0]`, `reg_rd_data[31:0]`, `reg_rd_valid` -- a simple memory-mapped read/write port.
- **AXI4 master interface:** Full 5-channel AXI4 master (AW, W, B, AR, R) with 32-bit data and address.
- **Peripheral handshake:** `dreq[NUM_CH-1:0]` (input), `dack[NUM_CH-1:0]` (output) for hardware flow control.
- **Interrupt:** Single `irq` output, OR-reduction of enabled per-channel TC and ERR status bits.
- **Clock and reset:** Single clock `clk`, synchronous active-high reset `srst`.

---

## 2. Register Map

### 2.1 Per-Channel Registers

Each channel occupies a 64-byte block. The base address for channel `c` is `c * 0x40`. Only the first 8 registers (32 bytes) are used; the remaining 32 bytes are reserved.

| Offset | Name | R/W | Bit Fields |
|--------|------|-----|------------|
| `0x00` | `CH_CTRL` | R/W | `[0]` enable, `[1]` start (write-1-to-start, self-clearing), `[2]` abort (write-1-to-abort, self-clearing), `[3]` sg_en (scatter-gather enable), `[5:4]` xfer_type (00=MEM2MEM, 01=MEM2PERIPH, 10=PERIPH2MEM) |
| `0x04` | `CH_STATUS` | RO | `[2:0]` ch_status_t (IDLE/DESC_FETCH/READ/WRITE/DONE/ERROR), `[3]` tc (transfer complete), `[4]` err (error flag) |
| `0x08` | `CH_SRC_ADDR` | R/W | Source address for non-SG single transfer |
| `0x0C` | `CH_DST_ADDR` | R/W | Destination address for non-SG single transfer |
| `0x10` | `CH_XFER_LEN` | R/W | Transfer length in bytes for non-SG single transfer |
| `0x14` | `CH_DESC_ADDR` | R/W | Pointer to first descriptor (when sg_en=1) |
| `0x18` | `CH_CUR_SRC` | RO | Current source address (debug, driven by channel) |
| `0x1C` | `CH_CUR_DST` | RO | Current destination address (debug, driven by channel) |

### 2.2 Global Registers

| Offset | Name | R/W | Bit Fields |
|--------|------|-----|------------|
| `0x100` | `IRQ_STATUS` | RO | `[NUM_CH-1:0]` TC status per channel, `[NUM_CH+15:16]` ERR status per channel |
| `0x104` | `IRQ_ENABLE` | R/W | `[NUM_CH-1:0]` TC interrupt enable, `[NUM_CH+15:16]` ERR interrupt enable |
| `0x108` | `IRQ_CLEAR` | WO | Write-1-to-clear: same bit layout as IRQ_STATUS |
| `0x10C` | `VERSION` | RO | Returns `32'h0001_0000` (version 1.0) |

### 2.3 Interrupt Generation

The interrupt output is computed as:

```
irq = |((irq_tc_status & irq_tc_enable) | (irq_err_status & irq_err_enable))
```

TC status bits are set by `ch_tc` pulses from channels. ERR status bits are set by `ch_err` pulses. Both are cleared by writing a 1 to the corresponding bit in `IRQ_CLEAR`. The IRQ status block handles simultaneous set and clear correctly: on an `IRQ_CLEAR` write, incoming pulses are OR'd with the current status before the clear mask is applied.

---

## 3. Descriptor Format

Scatter-gather descriptors are 5 x 32-bit words (160 bits) stored at ascending addresses in memory. When a channel has `sg_en=1`, it fetches descriptors by issuing a 5-beat AXI read burst starting at the descriptor pointer address.

### 3.1 Descriptor Word Layout

| Word Index | Address Offset | Field |
|------------|---------------|-------|
| 0 | +0x00 | `ctrl` |
| 1 | +0x04 | `xfer_len` |
| 2 | +0x08 | `dst_addr` |
| 3 | +0x0C | `src_addr` |
| 4 | +0x10 | `next_desc_addr` |

The `dma_desc_t` packed struct in `dma_pkg` stores these in reverse order (MSB-first), with `next_desc_addr` at bits [159:128] and `ctrl` at bits [31:0]. However, in memory the words are read at ascending addresses and stored into `desc_buf[0..4]` sequentially, so `desc_buf[0]` = ctrl, `desc_buf[1]` = xfer_len, etc.

### 3.2 Descriptor ctrl Bit Definitions

| Bit(s) | Name | Description |
|--------|------|-------------|
| `[0]` | enable | 1 = descriptor is valid. If 0, the channel transitions to ERROR. |
| `[1]` | irq_en | 1 = generate TC interrupt on completion of this descriptor's transfer |
| `[3:2]` | xfer_type | 00 = MEM2MEM, 01 = MEM2PERIPH, 10 = PERIPH2MEM, 11 = reserved |
| `[4]` | last | 1 = this is the last descriptor; ignore next_desc_addr |
| `[31:5]` | reserved | Must be 0 |

### 3.3 Chain Termination

A descriptor chain ends when either:
- `next_desc_addr == 0x0000_0000` (null pointer), or
- `ctrl.last == 1` (explicit last flag)

---

## 4. Channel FSM

Each `dma_channel` instance implements a 12-state FSM that manages the full lifecycle of a transfer or descriptor chain.

### 4.1 State Definitions

| State | Encoding | Description |
|-------|----------|-------------|
| `S_IDLE` | 4'd0 | Waiting for start command |
| `S_DESC_REQ` | 4'd1 | Requesting arbiter grant for descriptor fetch |
| `S_DESC_WAIT` | 4'd2 | Receiving 5-word descriptor read via AXI |
| `S_DESC_PARSE` | 4'd3 | Parsing and validating descriptor fields |
| `S_READ_REQ` | 4'd4 | Requesting arbiter grant for read burst |
| `S_READ_WAIT` | 4'd5 | Receiving read burst data into FIFO |
| `S_WRITE_REQ` | 4'd6 | Requesting arbiter grant for write burst |
| `S_WRITE_WAIT` | 4'd7 | Sending write burst data from FIFO |
| `S_NEXT_BURST` | 4'd8 | Updating addresses and remaining length |
| `S_NEXT_DESC` | 4'd9 | Checking for next descriptor in chain |
| `S_DONE` | 4'd10 | Transfer complete; emit tc_pulse |
| `S_ERROR` | 4'd11 | Error detected; emit err_pulse, halt until abort |

### 4.2 Non-Scatter-Gather Flow

For `sg_en=0`:

```
S_IDLE -> S_READ_REQ -> S_READ_WAIT -> S_WRITE_REQ -> S_WRITE_WAIT
  -> S_NEXT_BURST -> [if remaining > 0: S_READ_REQ, else: S_DONE] -> S_IDLE
```

1. CPU writes `CH_SRC_ADDR`, `CH_DST_ADDR`, `CH_XFER_LEN`, sets `CH_CTRL.enable=1`, writes `CH_CTRL.start=1`.
2. The channel latches the configuration, computes the first burst length, and requests a read grant.
3. Data is read from the source address into the internal FIFO.
4. The channel then requests a write grant and drains the FIFO to the destination address.
5. After each read-write burst pair, `S_NEXT_BURST` updates the source/destination addresses and decrements the remaining byte count.
6. When `remaining` reaches 0, the FSM transitions to `S_DONE`, emits a `tc_pulse`, and returns to `S_IDLE`.

### 4.3 Scatter-Gather Flow

For `sg_en=1`:

```
S_IDLE -> S_DESC_REQ -> S_DESC_WAIT -> S_DESC_PARSE
  -> S_READ_REQ -> ... -> S_NEXT_BURST -> [if remaining > 0: S_READ_REQ]
  -> S_NEXT_DESC -> [if next != 0 && !last: S_DESC_REQ, else: S_DONE]
```

1. CPU writes `CH_DESC_ADDR`, sets `CH_CTRL.sg_en=1, enable=1, start=1`.
2. The channel fetches a 5-word descriptor from the address in `desc_ptr`.
3. `S_DESC_PARSE` validates the `enable` bit and extracts `src_addr`, `dst_addr`, `xfer_len`, and `xfer_type`.
4. The standard read-write-burst loop executes the transfer described by the current descriptor.
5. `S_NEXT_DESC` checks whether to follow the chain or terminate.

### 4.4 Burst Splitting

The burst splitting logic computes:

```
remaining_beats = remaining / BYTES_PER_BEAT
burst_len = min(MAX_BURST_LEN, remaining_beats)
```

After each burst, the addresses advance by `burst_len * BYTES_PER_BEAT` and `remaining` is decremented by the same amount. The AXI burst length field (0-based) is `burst_len - 1` for data transfers, and 4 for descriptor fetches (5 beats).

### 4.5 Internal FIFO

Each channel contains a `dma_fifo` instance with `DEPTH = MAX_BURST_LEN` (default 16). This pointer-based synchronous FIFO decouples the read and write phases, allowing data to be read from the source in one AXI burst and written to the destination in a subsequent burst. The FIFO is reset on both global reset (`srst`) and channel abort (`cfg_abort`).

---

## 5. Arbitration

The `dma_arbiter` module grants bus access to one channel at a time. It operates as a 2-state FSM: `S_IDLE` (selecting a winner from pending requests) and `S_BUSY` (holding the grant until `axi_req_done`).

### 5.1 Round-Robin Mode (Default)

A `priority_ptr` register tracks the next channel to be considered. On each arbitration cycle, the arbiter scans from `priority_ptr` through all `NUM_CH` channels (wrapping around) and selects the first requesting channel. After a grant completes, `priority_ptr` advances to `winner + 1` (mod NUM_CH), ensuring fair access.

The modular index computation uses wider arithmetic to avoid truncation:

```
rr_sum = priority_ptr + i
rr_idx = (rr_sum >= NUM_CH) ? rr_sum - NUM_CH : rr_sum
```

### 5.2 Fixed-Priority Mode

When `ARB_MODE = "FIXED_PRIORITY"`, channel 0 has the highest priority. The arbiter scans channels 0 through NUM_CH-1 and grants to the first requester. The `priority_ptr` is not advanced.

### 5.3 Grant Semantics

When a winner is found in `S_IDLE`, the arbiter latches the winner index, asserts the corresponding bit in the one-hot `grant` vector, sets `grant_valid`, and forwards the winner's request type, address, and burst length to the AXI master as a single-cycle `axi_req_valid` pulse. The grant is held for the entire AXI transaction. When `axi_req_done` is asserted, the arbiter de-asserts the grant and returns to `S_IDLE`.

### 5.4 Packed Array Ports

The arbiter uses packed (flattened) arrays for its input ports (`req_type_flat`, `req_addr_flat`, `req_len_flat`) and unflattens them internally using generate-block assigns. This avoids iverilog limitations with unpacked array port connections.

---

## 6. AXI4 Master

The `dma_axi_master` module implements a 6-state FSM that translates internal DMA requests into AXI4 bus transactions.

### 6.1 FSM States

| State | Description |
|-------|-------------|
| `AXI_IDLE` | Waiting for request from arbiter |
| `AXI_AR` | Driving read address channel (ARVALID asserted) |
| `AXI_R` | Receiving read data beats until RLAST |
| `AXI_AW` | Driving write address channel (AWVALID asserted) |
| `AXI_W` | Driving write data beats until WLAST |
| `AXI_B` | Waiting for write response (BVALID) |

### 6.2 Request Handling

On `req_valid`, the AXI master latches the address and burst length, clears internal error state, and transitions to either `AXI_AR` (for `ARB_REQ_READ` or `ARB_REQ_DESC`) or `AXI_AW` (for `ARB_REQ_WRITE`).

### 6.3 AXI Protocol Details

- **Burst type:** INCR (`AWBURST`/`ARBURST` = `2'b01`) exclusively.
- **Transfer size:** `AWSIZE`/`ARSIZE` = `3'b010` (4 bytes per beat for 32-bit data width).
- **Write strobes:** `WSTRB` = `4'b1111` (all bytes enabled, full-word writes only).
- **Burst length:** `AWLEN`/`ARLEN` = `req_len` (0-based; 0 = 1 beat, 15 = 16 beats).

### 6.4 Data Path

The AXI master acts as a pass-through for data between the AXI bus and the active channel:

- **Read path:** `rd_valid`, `rd_data`, `rd_last`, `rd_resp` are gated by the `AXI_R` state and driven from the AXI R channel signals. `m_axi_rready` is gated by the channel's `rd_ready`.
- **Write path:** `m_axi_wvalid` is gated by the `AXI_W` state and driven from the channel's `wr_valid`. `wr_ready` is gated by the `AXI_W` state and driven from `m_axi_wready`.

### 6.5 Completion and Error Signalling

`req_done` is a single-cycle pulse asserted when the transaction completes (RLAST received for reads, BVALID received for writes). `req_error` is asserted alongside `req_done` if any beat returned a non-OKAY response. The response code is latched in `resp_r` and forwarded as `req_resp`.

The write response is also forwarded to the active channel via `wr_resp_valid`/`wr_resp` as a single-cycle pulse when the B-channel handshake occurs.

---

## 7. Error Handling

### 7.1 AXI Error Detection

The AXI master detects errors by checking bit [1] of the AXI response codes (`RRESP` and `BRESP`). This catches both SLVERR (`2'b10`) and DECERR (`2'b11`). Errors are latched for the duration of the transaction and reported via `req_error` when `req_done` fires.

### 7.2 Channel Error Response

When a channel receives an error response during `S_READ_WAIT` (via `axi_rd_resp != AXI_RESP_OKAY`) or `S_WRITE_WAIT` (via `axi_wr_resp != AXI_RESP_OKAY`), it transitions to `S_ERROR`. In this state:

- The channel emits an `err_pulse` (single cycle) which sets the corresponding bit in `IRQ_STATUS`.
- The channel halts and remains in `S_ERROR` until a `cfg_abort` is received.
- On abort, the channel returns to `S_IDLE` and the internal FIFO is reset.

### 7.3 Descriptor Validation

During `S_DESC_PARSE`, if the `enable` bit (`ctrl[0]`) of the fetched descriptor is 0, the channel transitions to `S_ERROR`. This catches null or invalid descriptors.

---

## 8. Peripheral Handshake

The DMA controller supports hardware flow control for peripheral transfers using `dreq` (DMA request) and `dack` (DMA acknowledge) signals, one pair per channel.

### 8.1 MEM2PERIPH (xfer_type = 01)

During `S_WRITE_WAIT`, `axi_wr_valid` is gated by `dreq`. The channel only drives write data to the AXI master when the peripheral asserts its request. `dack` is asserted for each beat where `axi_wr_valid && axi_wr_ready`.

### 8.2 PERIPH2MEM (xfer_type = 10)

During `S_READ_WAIT`, `axi_rd_ready` is gated by `dreq`. The channel only accepts read data from the AXI master when the peripheral asserts its request. `dack` is asserted for each beat where `axi_rd_valid && axi_rd_ready`.

### 8.3 MEM2MEM (xfer_type = 00)

The `dreq` signal is ignored and `dack` is held low. Transfers proceed at full AXI bus speed.

---

## 9. Design Decisions

### 9.1 Synchronous Reset (srst)

All sequential logic uses synchronous active-high reset (`srst`) in `always_ff @(posedge clk)` blocks with `if (srst)` as the first branch. This was chosen for compatibility with FPGA synthesis tools and avoids the complexities of asynchronous reset distribution.

### 9.2 always @(*) for iverilog Compatibility

Combinational blocks that read signals driven by submodule outputs use `always @(*)` instead of `always_comb`. This is a deliberate workaround for iverilog, which can enter infinite re-evaluation loops when `always_comb` sensitivity lists include submodule outputs. The `always @(*)` form is used for:

- The channel data-path mux in `dma_top` (reads `ch_axi_rd_ready`, `ch_axi_wr_valid`, etc. from channel sub-instances)
- The channel FSM next-state and output logic (reads `arb_grant` from the arbiter sub-instance)
- The arbiter winner-selection logic (reads `req` from the channel sub-instances)
- The AXI master next-state logic

`always_comb` is reserved for purely local combinational logic with no inter-module signal dependencies.

### 9.3 Packed Array Ports for iverilog Unpacked Array Workaround

iverilog has limitations with unpacked array ports in module instantiations. To work around this, inter-module array connections are flattened into packed (1-D) vectors and sliced using the `+:` part-select operator. For example:

- `ch_xfer_type_flat[NUM_CH*2-1:0]` carries all per-channel transfer types as a packed vector.
- Each channel's 2-bit slice is extracted as `ch_xfer_type_flat[gi*2 +: 2]` in the generate block.
- The register file and arbiter unflatten these internally using matching generate-block assigns.

This pattern is used consistently for all per-channel configuration, status, and arbiter signals that cross module boundaries.

### 9.4 grant_seen_low for Stale Grant Detection

When a channel transitions between phases (e.g., from read to write), the arbiter's grant for the previous phase may still be asserted for one or more cycles. Without protection, the channel could incorrectly interpret this stale grant as approval for its new request.

The `grant_seen_low` flag addresses this:

1. When transitioning out of a granted phase (e.g., `S_READ_WAIT` to `S_WRITE_REQ`), the channel sets `grant_seen_low = 0`.
2. In the `S_*_REQ` states, the channel monitors `arb_grant`. When it sees `arb_grant == 0`, it sets `grant_seen_low = 1`.
3. The channel only accepts a grant when `arb_grant && grant_seen_low`, ensuring the grant is a fresh one for the current request.
4. On initial start from `S_IDLE`, `grant_seen_low` is set to 1 (no stale grant possible).

---

## 10. Test Methodology and Results

### 10.1 Verification Approach

The design is verified using two complementary methodologies:

- **SystemVerilog (SV) testbenches:** Self-checking testbenches compiled with `iverilog -g2012`. Each testbench uses a `check()` task that compares actual vs. expected values and maintains pass/fail counters. Tests produce a summary line indicating total checks, passes, and failures.

- **cocotb testbenches:** Python-based testbenches using the cocotb framework with the Icarus Verilog backend (`SIM=icarus`). These provide higher-level test scenarios with randomised stimulus and golden-model comparisons.

Both SV and cocotb testbenches exist for each of the six modules, following a bottom-up verification strategy: FIFO first, then register file, AXI master, arbiter, channel, and finally the integrated top-level.

### 10.2 SV Testbench Infrastructure

Each SV testbench follows a common pattern:

- Imports `dma_pkg::*` for shared types and constants.
- Instantiates the DUT with default parameters.
- Provides a `reset_dut()` task for clean initialisation.
- Uses a `check()` task that increments `pass_count`/`fail_count` and calls `$stop` on failure for waveform inspection.
- Generates VCD waveform dumps for debugging.
- Prints a final summary with total test/pass/fail counts.

For modules requiring an AXI environment (`dma_axi_master`, `dma_channel`, `dma_top`), the testbenches include AXI slave memory models that respond to reads/writes with configurable behaviour and can inject error responses.

### 10.3 cocotb Testbench Infrastructure

Each cocotb test module is accompanied by a Makefile that specifies the Verilog source list, top-level module, and compilation flags (`-g2012`). `dma_pkg.sv` is always listed first in the source order.

### 10.4 Test Results Summary

| Module | SV Checks | cocotb Tests | Status |
|--------|-----------|--------------|--------|
| dma_fifo | 51 | 6 | PASS |
| dma_reg_file | 36 | 8 | PASS |
| dma_axi_master | 44 | 8 | PASS |
| dma_arbiter | 33 | 5 | PASS |
| dma_channel | 42 | 8 | PASS |
| dma_top | 33 | 8 | PASS |
| **Total** | **239** | **43** | **ALL PASS** |

### 10.5 Test Coverage by Module

**dma_fifo (51 SV checks, 6 cocotb tests):** Reset clears, single write/read, fill to full, full-write rejection, drain from full, empty-read rejection, concurrent read/write at various fill levels, data integrity with sequential patterns.

**dma_reg_file (36 SV checks, 8 cocotb tests):** Reset defaults, per-channel register read/write for all offsets, CH_CTRL start/abort pulse self-clearing, CH_STATUS read-back, multi-channel isolation, IRQ_STATUS set via TC/ERR pulses, IRQ_CLEAR write-1-to-clear, IRQ enable/disable masking, irq output assertion, VERSION register value.

**dma_axi_master (44 SV checks, 8 cocotb tests):** Single-beat and burst reads (1, 4, 16 beats), single-beat and burst writes, SLVERR on read, DECERR on write, back-to-back reads and writes, read-then-write transitions, slave stall scenarios.

**dma_arbiter (33 SV checks, 5 cocotb tests):** No-request idle, single-channel request, two simultaneous requests, round-robin fairness across all 4 channels, grant hold until done, fixed-priority mode, priority pointer advancement, grant de-assertion on requester drop.

**dma_channel (42 SV checks, 8 cocotb tests):** Reset to idle, simple 4-byte transfer, 16-byte single burst, 64-byte max burst, 128-byte multi-burst, abort mid-transfer, AXI read error, AXI write error, scatter-gather single descriptor, 2-descriptor chain, descriptor last-bit termination, MEM2PERIPH peripheral handshake.

**dma_top (33 SV checks, 8 cocotb tests):** Reset state verification, register access round-trip, simple MEM2MEM 1-burst transfer, multi-burst 256-byte transfer, two concurrent channels, scatter-gather 2-descriptor chain, AXI error propagation to IRQ, interrupt enable/clear lifecycle, abort and restart, VERSION register.

---

## 11. Simulation Scripts

Three scripts in the `scripts/` directory automate test execution:

- **`run_sv_tests.sh`** -- Compiles and runs all six SV testbenches using `iverilog -g2012` and `vvp`. Reports pass/fail for each module.
- **`run_cocotb_tests.sh`** -- Runs all six cocotb test suites using `make` in each test directory with `SIM=icarus`.
- **`run_all.sh`** -- Executes both SV and cocotb test suites sequentially and provides a combined summary.

---

## 12. Synthesis Considerations

The design targets generic RTL synthesis with no vendor-specific primitives. Key synthesis characteristics:

- All FIFOs use register arrays (no block RAM inference hints), suitable for the small depth (16 entries).
- Single clock domain with synchronous reset eliminates CDC complexity.
- The AXI master FSM and channel FSMs use explicit state encoding (`logic [N:0]` enumerations) for predictable synthesis.
- All combinational outputs have default assignments to prevent latch inference.
- The grant-based mux in `dma_top` synthesises to a priority-encoded multiplexer (only one grant bit is ever active).
