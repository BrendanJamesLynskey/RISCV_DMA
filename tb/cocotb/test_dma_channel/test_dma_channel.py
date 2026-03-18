# Brendan Lynskey 2025

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer, First


# ---------------------------------------------------------------------
# Constants from dma_pkg
# ---------------------------------------------------------------------
CH_IDLE       = 0
CH_DESC_FETCH = 1
CH_READ       = 2
CH_WRITE      = 3
CH_DONE       = 4
CH_ERROR      = 5

ARB_REQ_READ  = 1
ARB_REQ_WRITE = 2
ARB_REQ_DESC  = 3


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
async def reset_dut(dut):
    dut.srst.value = 1
    dut.cfg_enable.value = 0
    dut.cfg_start.value = 0
    dut.cfg_abort.value = 0
    dut.cfg_sg_en.value = 0
    dut.cfg_xfer_type.value = 0
    dut.cfg_src_addr.value = 0
    dut.cfg_dst_addr.value = 0
    dut.cfg_xfer_len.value = 0
    dut.cfg_desc_addr.value = 0
    dut.arb_grant.value = 0
    dut.axi_rd_valid.value = 0
    dut.axi_rd_data.value = 0
    dut.axi_rd_last.value = 0
    dut.axi_rd_resp.value = 0
    dut.axi_wr_ready.value = 0
    dut.axi_wr_resp_valid.value = 0
    dut.axi_wr_resp.value = 0
    dut.dreq.value = 0
    await ClockCycles(dut.clk, 3)
    dut.srst.value = 0
    await RisingEdge(dut.clk)


async def start_clock(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())


async def mock_arbiter_and_axi(dut, mem, inject_read_error=False, inject_write_error=False):
    """Concurrent process that grants requests and feeds/consumes AXI data."""
    while True:
        await RisingEdge(dut.clk)
        if int(dut.arb_req.value) == 1 and int(dut.arb_grant.value) == 0:
            req_type = int(dut.arb_req_type.value)
            # Wait 1 cycle for burst_len to stabilize
            await RisingEdge(dut.clk)
            addr = int(dut.arb_addr.value)
            burst_len = int(dut.arb_len.value) + 1

            # Grant
            dut.arb_grant.value = 1

            if req_type in [1, 3]:  # READ or DESC
                # Feed read data
                for i in range(burst_len):
                    dut.axi_rd_valid.value = 1
                    dut.axi_rd_data.value = mem.get((addr >> 2) + i, 0)
                    dut.axi_rd_resp.value = 2 if inject_read_error else 0
                    dut.axi_rd_last.value = 1 if i == burst_len - 1 else 0
                    while True:
                        await RisingEdge(dut.clk)
                        if int(dut.axi_rd_ready.value) == 1:
                            break
                dut.axi_rd_valid.value = 0
                dut.axi_rd_last.value = 0

            elif req_type == 2:  # WRITE
                dut.axi_wr_ready.value = 1
                # Accept write data
                beat = 0
                while True:
                    await RisingEdge(dut.clk)
                    if int(dut.axi_wr_valid.value) == 1:
                        mem[(addr >> 2) + beat] = int(dut.axi_wr_data.value)
                        beat += 1
                        if int(dut.axi_wr_last.value) == 1:
                            break
                dut.axi_wr_ready.value = 0
                # Send write response
                await RisingEdge(dut.clk)
                dut.axi_wr_resp_valid.value = 1
                dut.axi_wr_resp.value = 2 if inject_write_error else 0
                await RisingEdge(dut.clk)
                dut.axi_wr_resp_valid.value = 0

            dut.arb_grant.value = 0


async def wait_for_status(dut, expected, timeout_cycles=2000):
    """Wait until dut.status matches expected value, with timeout."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if int(dut.status.value) == expected:
            return
    raise TimeoutError(
        f"Timed out waiting for status={expected}, "
        f"current status={int(dut.status.value)}"
    )


def load_sg_descriptor(mem, base_word_addr, ctrl, xfer_len, dst_addr, src_addr, next_desc_addr):
    """Load a scatter-gather descriptor into the memory dict."""
    mem[base_word_addr + 0] = ctrl
    mem[base_word_addr + 1] = xfer_len
    mem[base_word_addr + 2] = dst_addr
    mem[base_word_addr + 3] = src_addr
    mem[base_word_addr + 4] = next_desc_addr


# ---------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------
@cocotb.test()
async def test_idle_state(dut):
    """After reset, status should be CH_IDLE (0)."""
    await start_clock(dut)
    await reset_dut(dut)

    status = int(dut.status.value)
    assert status == CH_IDLE, f"Expected CH_IDLE ({CH_IDLE}), got {status}"
    dut._log.info("PASS: Status is CH_IDLE after reset")


@cocotb.test()
async def test_simple_transfer(dut):
    """16-byte transfer (4 beats). Verify data copied from src to dst."""
    await start_clock(dut)
    await reset_dut(dut)

    src_addr = 0x1000
    dst_addr = 0x2000
    xfer_len = 16  # bytes
    num_words = xfer_len // 4

    mem = {}
    for i in range(num_words):
        mem[(src_addr >> 2) + i] = 0xA0 + i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_src_addr.value = src_addr
    dut.cfg_dst_addr.value = dst_addr
    dut.cfg_xfer_len.value = xfer_len
    dut.cfg_xfer_type.value = 0  # MEM2MEM
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    await wait_for_status(dut, CH_DONE)

    for i in range(num_words):
        word = mem.get((dst_addr >> 2) + i, None)
        expected = 0xA0 + i
        assert word == expected, (
            f"Mismatch at dst word {i}: expected 0x{expected:08X}, got "
            f"0x{word:08X}" if word is not None else f"missing"
        )

    mock.kill()
    dut._log.info("PASS: Simple 16-byte transfer verified")


@cocotb.test()
async def test_multi_burst(dut):
    """128-byte transfer (2 bursts of 16 beats). Verify all data."""
    await start_clock(dut)
    await reset_dut(dut)

    src_addr = 0x1000
    dst_addr = 0x3000
    xfer_len = 128  # bytes
    num_words = xfer_len // 4

    mem = {}
    for i in range(num_words):
        mem[(src_addr >> 2) + i] = 0xDEAD0000 + i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_src_addr.value = src_addr
    dut.cfg_dst_addr.value = dst_addr
    dut.cfg_xfer_len.value = xfer_len
    dut.cfg_xfer_type.value = 0
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    await wait_for_status(dut, CH_DONE, timeout_cycles=5000)

    for i in range(num_words):
        word = mem.get((dst_addr >> 2) + i, None)
        expected = 0xDEAD0000 + i
        assert word == expected, (
            f"Mismatch at dst word {i}: expected 0x{expected:08X}, got "
            f"0x{word:08X}" if word is not None else f"missing"
        )

    mock.kill()
    dut._log.info("PASS: Multi-burst 128-byte transfer verified")


@cocotb.test()
async def test_abort(dut):
    """Start a transfer, assert cfg_abort before completion. Verify return to IDLE."""
    await start_clock(dut)
    await reset_dut(dut)

    src_addr = 0x1000
    dst_addr = 0x4000
    xfer_len = 128  # bytes — long enough to abort mid-transfer

    mem = {}
    for i in range(xfer_len // 4):
        mem[(src_addr >> 2) + i] = i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_src_addr.value = src_addr
    dut.cfg_dst_addr.value = dst_addr
    dut.cfg_xfer_len.value = xfer_len
    dut.cfg_xfer_type.value = 0
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    # Wait a few cycles then abort
    await ClockCycles(dut.clk, 10)
    dut.cfg_abort.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_abort.value = 0

    await wait_for_status(dut, CH_IDLE)

    mock.kill()
    dut._log.info("PASS: Abort returned channel to IDLE")


@cocotb.test()
async def test_read_error(dut):
    """Inject a read error on AXI, verify status becomes CH_ERROR (5)."""
    await start_clock(dut)
    await reset_dut(dut)

    src_addr = 0x1000
    dst_addr = 0x5000
    xfer_len = 16

    mem = {}
    for i in range(xfer_len // 4):
        mem[(src_addr >> 2) + i] = i

    mock = cocotb.start_soon(
        mock_arbiter_and_axi(dut, mem, inject_read_error=True)
    )

    dut.cfg_src_addr.value = src_addr
    dut.cfg_dst_addr.value = dst_addr
    dut.cfg_xfer_len.value = xfer_len
    dut.cfg_xfer_type.value = 0
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    await wait_for_status(dut, CH_ERROR)

    mock.kill()
    dut._log.info("PASS: Read error detected, status is CH_ERROR")


@cocotb.test()
async def test_sg_single(dut):
    """Single scatter-gather descriptor. Verify transfer from descriptor."""
    await start_clock(dut)
    await reset_dut(dut)

    desc_addr = 0x8000
    src_addr  = 0x1000
    dst_addr  = 0x6000
    xfer_len  = 16  # bytes
    num_words = xfer_len // 4

    mem = {}

    # Load descriptor at desc_addr
    load_sg_descriptor(
        mem,
        base_word_addr=desc_addr >> 2,
        ctrl=1,             # enable
        xfer_len=xfer_len,
        dst_addr=dst_addr,
        src_addr=src_addr,
        next_desc_addr=0,   # end of chain
    )

    # Load source data
    for i in range(num_words):
        mem[(src_addr >> 2) + i] = 0xBEEF0000 + i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_desc_addr.value = desc_addr
    dut.cfg_sg_en.value = 1
    dut.cfg_xfer_type.value = 0
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    await wait_for_status(dut, CH_DONE, timeout_cycles=5000)

    for i in range(num_words):
        word = mem.get((dst_addr >> 2) + i, None)
        expected = 0xBEEF0000 + i
        assert word == expected, (
            f"SG single: mismatch at word {i}: expected 0x{expected:08X}, "
            f"got 0x{word:08X}" if word is not None else f"missing"
        )

    mock.kill()
    dut._log.info("PASS: Single SG descriptor transfer verified")


@cocotb.test()
async def test_sg_chain(dut):
    """Two-descriptor scatter-gather chain. Verify both transfers."""
    await start_clock(dut)
    await reset_dut(dut)

    desc0_addr = 0x8000
    desc1_addr = 0x8100
    src0_addr  = 0x1000
    dst0_addr  = 0x6000
    src1_addr  = 0x2000
    dst1_addr  = 0x7000
    xfer_len   = 16  # bytes per descriptor
    num_words  = xfer_len // 4

    mem = {}

    # Descriptor 0 -> points to descriptor 1
    load_sg_descriptor(
        mem,
        base_word_addr=desc0_addr >> 2,
        ctrl=1,
        xfer_len=xfer_len,
        dst_addr=dst0_addr,
        src_addr=src0_addr,
        next_desc_addr=desc1_addr,
    )

    # Descriptor 1 -> end of chain
    load_sg_descriptor(
        mem,
        base_word_addr=desc1_addr >> 2,
        ctrl=1,
        xfer_len=xfer_len,
        dst_addr=dst1_addr,
        src_addr=src1_addr,
        next_desc_addr=0,
    )

    # Source data for descriptor 0
    for i in range(num_words):
        mem[(src0_addr >> 2) + i] = 0xAAAA0000 + i

    # Source data for descriptor 1
    for i in range(num_words):
        mem[(src1_addr >> 2) + i] = 0xBBBB0000 + i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_desc_addr.value = desc0_addr
    dut.cfg_sg_en.value = 1
    dut.cfg_xfer_type.value = 0
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    await wait_for_status(dut, CH_DONE, timeout_cycles=10000)

    # Verify descriptor 0 data
    for i in range(num_words):
        word = mem.get((dst0_addr >> 2) + i, None)
        expected = 0xAAAA0000 + i
        assert word == expected, (
            f"SG chain desc0: mismatch at word {i}: expected 0x{expected:08X}, "
            f"got 0x{word:08X}" if word is not None else f"missing"
        )

    # Verify descriptor 1 data
    for i in range(num_words):
        word = mem.get((dst1_addr >> 2) + i, None)
        expected = 0xBBBB0000 + i
        assert word == expected, (
            f"SG chain desc1: mismatch at word {i}: expected 0x{expected:08X}, "
            f"got 0x{word:08X}" if word is not None else f"missing"
        )

    mock.kill()
    dut._log.info("PASS: Two-descriptor SG chain verified")


@cocotb.test()
async def test_periph_handshake(dut):
    """MEM2PERIPH transfer with dreq/dack handshake."""
    await start_clock(dut)
    await reset_dut(dut)

    src_addr = 0x1000
    dst_addr = 0x9000
    xfer_len = 16  # bytes
    num_words = xfer_len // 4

    mem = {}
    for i in range(num_words):
        mem[(src_addr >> 2) + i] = 0xCAFE0000 + i

    mock = cocotb.start_soon(mock_arbiter_and_axi(dut, mem))

    dut.cfg_src_addr.value = src_addr
    dut.cfg_dst_addr.value = dst_addr
    dut.cfg_xfer_len.value = xfer_len
    dut.cfg_xfer_type.value = 1  # MEM2PERIPH
    dut.cfg_enable.value = 1
    dut.cfg_start.value = 1
    await RisingEdge(dut.clk)
    dut.cfg_start.value = 0

    # Drive dreq high to allow the transfer to proceed
    dut.dreq.value = 1

    # Monitor dack — it should pulse during the write phase
    dack_seen = False
    for _ in range(2000):
        await RisingEdge(dut.clk)
        if int(dut.dack.value) == 1:
            dack_seen = True
        status = int(dut.status.value)
        if status == CH_DONE or status == CH_IDLE:
            break

    dut.dreq.value = 0

    assert dack_seen, "Expected dack pulse during MEM2PERIPH transfer"

    mock.kill()
    dut._log.info("PASS: Peripheral handshake with dack verified")
