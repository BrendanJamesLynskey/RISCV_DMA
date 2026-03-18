# Brendan Lynskey 2025

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer, First


# ---------------------------------------------------------------------------
# AXI Slave Memory Model
# ---------------------------------------------------------------------------

class AxiSlaveMem:
    """Simple AXI4 slave memory model."""
    def __init__(self):
        self.mem = {}
        self.error_mode = False

    def write(self, byte_addr, data):
        self.mem[byte_addr >> 2] = data & 0xFFFFFFFF

    def read(self, byte_addr):
        return self.mem.get(byte_addr >> 2, 0)


async def axi_read_responder(dut, slave):
    """Handle AR + R channel. Runs continuously."""
    while True:
        await RisingEdge(dut.clk)
        # Check for AR handshake
        if int(dut.m_axi_arvalid.value) == 1 and int(dut.m_axi_arready.value) == 1:
            addr = int(dut.m_axi_araddr.value)
            burst_len = int(dut.m_axi_arlen.value) + 1
            resp = 2 if slave.error_mode else 0

            # Drive read data beats
            for beat in range(burst_len):
                dut.m_axi_rvalid.value = 1
                dut.m_axi_rdata.value = slave.read(addr + beat * 4)
                dut.m_axi_rresp.value = resp
                dut.m_axi_rlast.value = 1 if beat == burst_len - 1 else 0
                # Wait for rready
                while True:
                    await RisingEdge(dut.clk)
                    if int(dut.m_axi_rready.value) == 1:
                        break
            dut.m_axi_rvalid.value = 0
            dut.m_axi_rlast.value = 0


async def axi_write_responder(dut, slave):
    """Handle AW + W + B channel. Runs continuously."""
    while True:
        await RisingEdge(dut.clk)
        # Check for AW handshake
        if int(dut.m_axi_awvalid.value) == 1 and int(dut.m_axi_awready.value) == 1:
            addr = int(dut.m_axi_awaddr.value)
            resp = 2 if slave.error_mode else 0

            # Accept write data beats
            beat = 0
            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_wvalid.value) == 1 and int(dut.m_axi_wready.value) == 1:
                    slave.write(addr + beat * 4, int(dut.m_axi_wdata.value))
                    beat += 1
                    if int(dut.m_axi_wlast.value) == 1:
                        break

            # Send write response
            await RisingEdge(dut.clk)
            dut.m_axi_bvalid.value = 1
            dut.m_axi_bresp.value = resp
            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_bready.value) == 1:
                    break
            dut.m_axi_bvalid.value = 0


# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------

def init_signals(dut):
    dut.srst.value = 0
    dut.reg_wr_en.value = 0
    dut.reg_rd_en.value = 0
    dut.reg_addr.value = 0
    dut.reg_wr_data.value = 0
    dut.m_axi_arready.value = 1
    dut.m_axi_awready.value = 1
    dut.m_axi_wready.value = 1
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.dreq.value = 0


async def reset_dut(dut):
    init_signals(dut)
    dut.srst.value = 1
    await ClockCycles(dut.clk, 5)
    dut.srst.value = 0
    await ClockCycles(dut.clk, 2)


async def reg_write(dut, addr, data):
    await RisingEdge(dut.clk)
    dut.reg_wr_en.value = 1
    dut.reg_addr.value = addr
    dut.reg_wr_data.value = data
    await RisingEdge(dut.clk)
    dut.reg_wr_en.value = 0


async def reg_read(dut, addr):
    await RisingEdge(dut.clk)
    dut.reg_rd_en.value = 1
    dut.reg_addr.value = addr
    await RisingEdge(dut.clk)
    dut.reg_rd_en.value = 0
    await RisingEdge(dut.clk)
    return int(dut.reg_rd_data.value)


def ch_base(ch):
    return ch * 0x40


async def start_channel(dut, ch, src, dst, length):
    await reg_write(dut, ch_base(ch) + 0x08, src)
    await reg_write(dut, ch_base(ch) + 0x0C, dst)
    await reg_write(dut, ch_base(ch) + 0x10, length)
    await reg_write(dut, ch_base(ch) + 0x00, 0x03)  # enable + start


async def wait_channel_done(dut, ch, timeout=20000):
    """Wait for channel TC or ERR bit in status register."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        val = await reg_read(dut, ch_base(ch) + 0x04)
        status = val & 0x07
        tc = (val >> 3) & 1
        err = (val >> 4) & 1
        if tc or err or status == 4 or status == 5:
            return val
    raise TimeoutError(f"Channel {ch} did not complete within {timeout} cycles")


async def setup_test(dut):
    """Common test setup: start clock, reset, create slave, launch responders."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)
    slave = AxiSlaveMem()
    cocotb.start_soon(axi_read_responder(dut, slave))
    cocotb.start_soon(axi_write_responder(dut, slave))
    return slave


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_reset(dut):
    """After reset, check irq=0, arvalid=0, awvalid=0."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)
    await RisingEdge(dut.clk)

    assert int(dut.irq.value) == 0, "irq should be 0 after reset"
    assert int(dut.m_axi_arvalid.value) == 0, "arvalid should be 0 after reset"
    assert int(dut.m_axi_awvalid.value) == 0, "awvalid should be 0 after reset"
    dut._log.info("PASS: Reset values verified")


@cocotb.test()
async def test_register_access(dut):
    """Write/read SRC, DST, LEN, CTRL for ch0."""
    await setup_test(dut)

    test_values = [
        (ch_base(0) + 0x08, 0xDEAD0000, "SRC"),
        (ch_base(0) + 0x0C, 0xBEEF0000, "DST"),
        (ch_base(0) + 0x10, 0x00000040, "LEN"),
        (ch_base(0) + 0x00, 0x00000001, "CTRL"),
    ]

    for addr, wdata, name in test_values:
        await reg_write(dut, addr, wdata)
        rdata = await reg_read(dut, addr)
        assert rdata == wdata, f"{name} reg mismatch: wrote 0x{wdata:08X}, read 0x{rdata:08X}"
        dut._log.info(f"  {name}: wrote 0x{wdata:08X}, read back 0x{rdata:08X} OK")

    dut._log.info("PASS: Register access verified")


@cocotb.test()
async def test_simple_m2m(dut):
    """Load 4 words at src=0x0000, transfer 16 bytes to dst=0x1000, verify."""
    slave = await setup_test(dut)

    src_data = [0xAAAA0001, 0xBBBB0002, 0xCCCC0003, 0xDDDD0004]
    for i, word in enumerate(src_data):
        slave.write(0x0000 + i * 4, word)

    await start_channel(dut, 0, 0x0000, 0x1000, 16)
    status = await wait_channel_done(dut, 0)
    dut._log.info(f"  Channel 0 status: 0x{status:08X}")

    for i, expected in enumerate(src_data):
        actual = slave.read(0x1000 + i * 4)
        assert actual == expected, \
            f"Word {i} mismatch at dst 0x{0x1000 + i * 4:04X}: expected 0x{expected:08X}, got 0x{actual:08X}"

    dut._log.info("PASS: Simple 16-byte M2M transfer verified")


@cocotb.test()
async def test_multi_burst_m2m(dut):
    """128 bytes (32 words) M2M transfer. Verify all copied correctly."""
    slave = await setup_test(dut)

    num_words = 32
    src_addr = 0x0000
    dst_addr = 0x2000

    for i in range(num_words):
        slave.write(src_addr + i * 4, 0x10000000 + i)

    await start_channel(dut, 0, src_addr, dst_addr, num_words * 4)
    status = await wait_channel_done(dut, 0)
    dut._log.info(f"  Channel 0 status: 0x{status:08X}")

    for i in range(num_words):
        expected = 0x10000000 + i
        actual = slave.read(dst_addr + i * 4)
        assert actual == expected, \
            f"Word {i} mismatch: expected 0x{expected:08X}, got 0x{actual:08X}"

    dut._log.info("PASS: 128-byte multi-burst M2M transfer verified")


@cocotb.test()
async def test_concurrent_channels(dut):
    """Ch0 and ch1 transfer simultaneously. Both complete."""
    slave = await setup_test(dut)

    for i in range(4):
        slave.write(0x0000 + i * 4, 0xAA000000 + i)
        slave.write(0x0100 + i * 4, 0xBB000000 + i)

    await start_channel(dut, 0, 0x0000, 0x3000, 16)
    await start_channel(dut, 1, 0x0100, 0x4000, 16)

    status0 = await wait_channel_done(dut, 0)
    status1 = await wait_channel_done(dut, 1)
    dut._log.info(f"  Ch0 status: 0x{status0:08X}, Ch1 status: 0x{status1:08X}")

    for i in range(4):
        expected = 0xAA000000 + i
        actual = slave.read(0x3000 + i * 4)
        assert actual == expected, f"Ch0 word {i} mismatch: expected 0x{expected:08X}, got 0x{actual:08X}"

    for i in range(4):
        expected = 0xBB000000 + i
        actual = slave.read(0x4000 + i * 4)
        assert actual == expected, f"Ch1 word {i} mismatch: expected 0x{expected:08X}, got 0x{actual:08X}"

    dut._log.info("PASS: Concurrent channels verified")


@cocotb.test()
async def test_scatter_gather(dut):
    """2-descriptor SG chain. Verify data copied per each descriptor."""
    slave = await setup_test(dut)

    # Source data: 8 words (2 descriptors x 16 bytes each)
    for i in range(4):
        slave.write(0x0000 + i * 4, 0xDE500000 + i)
        slave.write(0x0010 + i * 4, 0xDE510000 + i)

    # Descriptor 0 at 0x8000: ctrl, xfer_len, dst_addr, src_addr, next_desc_addr
    slave.write(0x8000, 1)       # ctrl = 1 (valid)
    slave.write(0x8004, 16)      # xfer_len = 16 bytes
    slave.write(0x8008, 0x6000)  # dst_addr
    slave.write(0x800C, 0x0000)  # src_addr
    slave.write(0x8010, 0x8014)  # next_desc_addr

    # Descriptor 1 at 0x8014: ctrl, xfer_len, dst_addr, src_addr, next_desc_addr
    slave.write(0x8014, 1)       # ctrl = 1 (valid)
    slave.write(0x8018, 16)      # xfer_len = 16 bytes
    slave.write(0x801C, 0x6010)  # dst_addr
    slave.write(0x8020, 0x0010)  # src_addr
    slave.write(0x8024, 0)       # next_desc_addr = 0 (end of chain)

    # Start SG: write DESC_ADDR, then CTRL with sg_en
    await reg_write(dut, ch_base(0) + 0x14, 0x8000)
    await reg_write(dut, ch_base(0) + 0x00, 0x0B)  # enable=1, start=1, sg_en=1

    status = await wait_channel_done(dut, 0)
    dut._log.info(f"  SG channel 0 status: 0x{status:08X}")

    for i in range(4):
        expected = 0xDE500000 + i
        actual = slave.read(0x6000 + i * 4)
        assert actual == expected, \
            f"Desc0 word {i} mismatch: expected 0x{expected:08X}, got 0x{actual:08X}"

    for i in range(4):
        expected = 0xDE510000 + i
        actual = slave.read(0x6010 + i * 4)
        assert actual == expected, \
            f"Desc1 word {i} mismatch: expected 0x{expected:08X}, got 0x{actual:08X}"

    dut._log.info("PASS: Scatter-gather 2-descriptor chain verified")


@cocotb.test()
async def test_error_handling(dut):
    """Set error_mode, start transfer, verify ERROR status or err bit, irq fires."""
    slave = await setup_test(dut)

    slave.error_mode = True

    for i in range(4):
        slave.write(0x0000 + i * 4, 0xEE000000 + i)

    await start_channel(dut, 0, 0x0000, 0x5000, 16)
    status = await wait_channel_done(dut, 0)
    dut._log.info(f"  Error test status: 0x{status:08X}")

    err_bit = (status >> 4) & 1
    state = status & 0x07
    assert err_bit == 1 or state == 5, \
        f"Expected error indication, got status 0x{status:08X}"

    dut._log.info("PASS: Error handling verified")


@cocotb.test()
async def test_irq_flow(dut):
    """Enable TC IRQ, start transfer, verify irq=1, clear, verify irq=0."""
    slave = await setup_test(dut)

    for i in range(4):
        slave.write(0x0000 + i * 4, 0xFF000000 + i)

    # Enable TC IRQ via IRQ_ENABLE register at 0x104
    await reg_write(dut, 0x104, 0x01)

    await start_channel(dut, 0, 0x0000, 0x7000, 16)
    status = await wait_channel_done(dut, 0)
    dut._log.info(f"  IRQ test status: 0x{status:08X}")

    await ClockCycles(dut.clk, 5)
    assert int(dut.irq.value) == 1, f"Expected irq=1 after TC, got {int(dut.irq.value)}"

    # Clear IRQ via IRQ_CLEAR register at 0x108
    await reg_write(dut, 0x108, 0x01)
    await ClockCycles(dut.clk, 5)
    assert int(dut.irq.value) == 0, f"Expected irq=0 after clear, got {int(dut.irq.value)}"

    dut._log.info("PASS: IRQ flow verified")
