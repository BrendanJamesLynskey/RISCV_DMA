# Brendan Lynskey 2025
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


NUM_CH = 4


async def reset_dut(dut):
    """Apply synchronous reset."""
    dut.srst.value = 1
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    dut.addr.value = 0
    dut.wr_data.value = 0
    dut.ch_tc.value = 0
    dut.ch_err.value = 0
    dut.ch_status_flat.value = 0
    dut.ch_cur_src_flat.value = 0
    dut.ch_cur_dst_flat.value = 0
    await ClockCycles(dut.clk, 3)
    dut.srst.value = 0
    await RisingEdge(dut.clk)


async def reg_write(dut, addr, data):
    """Write a register."""
    await RisingEdge(dut.clk)
    dut.wr_en.value = 1
    dut.addr.value = addr
    dut.wr_data.value = data
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    dut.addr.value = 0
    dut.wr_data.value = 0


async def reg_read(dut, addr):
    """Read a register. Returns data after rd_valid."""
    await RisingEdge(dut.clk)
    dut.rd_en.value = 1
    dut.addr.value = addr
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    dut.addr.value = 0
    # rd_data is registered, available after this edge
    await RisingEdge(dut.clk)
    return int(dut.rd_data.value)


def ch_base(ch):
    return ch * 0x40


@cocotb.test()
async def test_reset(dut):
    """Test 1: All registers at defaults after reset."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    assert int(dut.ch_enable.value) == 0
    assert int(dut.ch_sg_en.value) == 0
    assert int(dut.irq.value) == 0

    # VERSION register
    val = await reg_read(dut, 0x10C)
    assert val == 0x00010000, f"VERSION: 0x{val:08X}"

    # Channel 0 SRC_ADDR
    val = await reg_read(dut, ch_base(0) + 0x08)
    assert val == 0, f"CH_SRC_ADDR default: 0x{val:08X}"


@cocotb.test()
async def test_channel_config(dut):
    """Test 2: Write/read per-channel registers."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vals = {
        0x08: 0x10000000,  # SRC_ADDR
        0x0C: 0x20000000,  # DST_ADDR
        0x10: 0x00000100,  # XFER_LEN
        0x14: 0x30000000,  # DESC_ADDR
    }

    for ch in range(NUM_CH):
        for offset, val in test_vals.items():
            await reg_write(dut, ch_base(ch) + offset, val + ch)

    for ch in range(NUM_CH):
        for offset, val in test_vals.items():
            rd = await reg_read(dut, ch_base(ch) + offset)
            expected = val + ch
            assert rd == expected, \
                f"Ch{ch} offset 0x{offset:02X}: got 0x{rd:08X}, expected 0x{expected:08X}"


@cocotb.test()
async def test_start_pulse(dut):
    """Test 3: Verify start bit generates single pulse."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Write start bit (bit 1 of CH_CTRL)
    await reg_write(dut, ch_base(0), 0x00000002)

    # ch_start should pulse for one cycle then auto-clear
    # After reg_write, the pulse has already happened
    # Verify it's cleared by reading CTRL (start reads as 0)
    val = await reg_read(dut, ch_base(0))
    assert (val & 0x02) == 0, f"Start bit should auto-clear, got 0x{val:08X}"


@cocotb.test()
async def test_abort_pulse(dut):
    """Test 4: Verify abort bit generates single pulse."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Write abort bit (bit 2 of CH_CTRL)
    await reg_write(dut, ch_base(0), 0x00000004)

    # Verify auto-clear
    val = await reg_read(dut, ch_base(0))
    assert (val & 0x04) == 0, f"Abort bit should auto-clear, got 0x{val:08X}"


@cocotb.test()
async def test_status_readback(dut):
    """Test 5: Drive status, verify read."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Set channel 0 status to CH_READ (3'd2), channel 1 to CH_WRITE (3'd3)
    dut.ch_status_flat.value = (3 << (1 * 3)) | (2 << (0 * 3))
    await RisingEdge(dut.clk)

    val = await reg_read(dut, ch_base(0) + 0x04)
    assert (val & 0x07) == 2, f"Ch0 status: expected 2, got {val & 0x07}"

    val = await reg_read(dut, ch_base(1) + 0x04)
    assert (val & 0x07) == 3, f"Ch1 status: expected 3, got {val & 0x07}"


@cocotb.test()
async def test_irq_status(dut):
    """Test 6: TC/ERR pulse -> IRQ_STATUS."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Pulse ch_tc[0] and ch_err[1]
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0b0001
    dut.ch_err.value = 0b0010
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0
    dut.ch_err.value = 0

    val = await reg_read(dut, 0x100)
    assert (val & 0x01) == 1, f"TC[0] not set in IRQ_STATUS: 0x{val:08X}"
    assert ((val >> 17) & 0x01) == 1, f"ERR[1] not set in IRQ_STATUS: 0x{val:08X}"


@cocotb.test()
async def test_irq_clear(dut):
    """Test 7: Write-1-to-clear IRQ_STATUS."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Set TC[0] and ERR[1]
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0b0001
    dut.ch_err.value = 0b0010
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0
    dut.ch_err.value = 0

    # Clear TC[0] only
    await reg_write(dut, 0x108, 0x00000001)

    val = await reg_read(dut, 0x100)
    assert (val & 0x01) == 0, f"TC[0] should be cleared: 0x{val:08X}"
    assert ((val >> 17) & 0x01) == 1, f"ERR[1] should remain set: 0x{val:08X}"

    # Clear ERR[1]
    await reg_write(dut, 0x108, 0x00020000)
    val = await reg_read(dut, 0x100)
    assert ((val >> 17) & 0x01) == 0, f"ERR[1] should be cleared: 0x{val:08X}"


@cocotb.test()
async def test_irq_output(dut):
    """Test 8: IRQ enable + status -> irq pin."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable TC interrupt for channel 0
    await reg_write(dut, 0x104, 0x00000001)
    await RisingEdge(dut.clk)
    assert int(dut.irq.value) == 0, "IRQ should be low with no status"

    # Pulse TC[0]
    dut.ch_tc.value = 0b0001
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.irq.value) == 1, "IRQ should be high after TC pulse"

    # Clear it
    await reg_write(dut, 0x108, 0x00000001)
    await RisingEdge(dut.clk)
    assert int(dut.irq.value) == 0, "IRQ should be low after clear"

    # Verify disabled IRQ doesn't assert
    await reg_write(dut, 0x104, 0x00000000)  # disable all
    dut.ch_tc.value = 0b0001
    await RisingEdge(dut.clk)
    dut.ch_tc.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.irq.value) == 0, "IRQ should stay low when disabled"
