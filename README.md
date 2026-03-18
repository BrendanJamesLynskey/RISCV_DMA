# RISCV_DMA -- Multi-Channel DMA Controller

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
| Per-channel (base = ch x 0x40) | | |
| +0x00 | CH_CTRL | Enable, start, abort, SG enable, transfer type |
| +0x04 | CH_STATUS | Channel state, TC, error flags |
| +0x08 | CH_SRC_ADDR | Source address |
| +0x0C | CH_DST_ADDR | Destination address |
| +0x10 | CH_XFER_LEN | Transfer length (bytes) |
| +0x14 | CH_DESC_ADDR | First descriptor pointer |
| +0x18 | CH_CUR_SRC | Current source address (read-only) |
| +0x1C | CH_CUR_DST | Current destination address (read-only) |
| Global registers | | |
| 0x100 | DMA_IRQ_STATUS | Interrupt status |
| 0x104 | DMA_IRQ_ENABLE | Interrupt enable mask |
| 0x108 | DMA_IRQ_CLEAR | Write-1-to-clear |
| 0x10C | DMA_VERSION | Version (0x00010000) |

## Simulation

### Prerequisites
- iverilog (>= 11.0) with `-g2012` support
- cocotb (>= 1.8) with icarus backend

### Run all tests
```bash
./scripts/run_all.sh
```

### Run SV tests only
```bash
./scripts/run_sv_tests.sh
```

### Run cocotb tests only
```bash
./scripts/run_cocotb_tests.sh
```

## Test Results

| Module | SV Checks | cocotb Tests | Status |
|--------|-----------|--------------|--------|
| dma_fifo | 51 | 6 | PASS |
| dma_reg_file | 36 | 8 | PASS |
| dma_axi_master | 44 | 8 | PASS |
| dma_arbiter | 33 | 5 | PASS |
| dma_channel | 42 | 8 | PASS |
| dma_top | 33 | 8 | PASS |
| **Total** | **239** | **43** | **ALL PASS** |

## File Structure

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
│       ├── test_dma_reg_file/
│       ├── test_dma_axi_master/
│       ├── test_dma_arbiter/
│       ├── test_dma_channel/
│       └── test_dma_top/
├── scripts/
│   ├── run_sv_tests.sh
│   ├── run_cocotb_tests.sh
│   └── run_all.sh
├── docs/
│   └── dma_technical_report.md
└── README.md
```

## Author

Brendan Lynskey 2025

## Licence

MIT
