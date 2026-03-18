# Brendan Lynskey 2025
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random


async def reset_dut(dut):
    """Apply synchronous reset for 3 cycles."""
    dut.srst.value = 1
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    dut.wr_data.value = 0
    await ClockCycles(dut.clk, 3)
    dut.srst.value = 0
    await RisingEdge(dut.clk)


async def write_word(dut, data):
    """Write a single word to the FIFO. Takes 1 clock cycle."""
    await RisingEdge(dut.clk)
    dut.wr_en.value = 1
    dut.wr_data.value = data
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    dut.wr_data.value = 0


async def read_word(dut):
    """Read a single word from the FIFO. Takes 1 clock cycle."""
    await RisingEdge(dut.clk)
    data = int(dut.rd_data.value)  # sample combinational rd_data
    dut.rd_en.value = 1
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    return data


DEPTH = 16


@cocotb.test()
async def test_reset(dut):
    """Test 1: Verify clean state after reset."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    assert dut.empty.value == 1, f"Expected empty=1, got {dut.empty.value}"
    assert dut.full.value == 0, f"Expected full=0, got {dut.full.value}"
    assert int(dut.count.value) == 0, f"Expected count=0, got {int(dut.count.value)}"


@cocotb.test()
async def test_single_word(dut):
    """Test 2: Write/read single word."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Write
    await write_word(dut, 0xCAFEBABE)
    await RisingEdge(dut.clk)  # let count update
    assert dut.empty.value == 0, "FIFO should not be empty after write"
    assert int(dut.count.value) == 1, f"Expected count=1, got {int(dut.count.value)}"

    # Read
    data = await read_word(dut)
    assert data == 0xCAFEBABE, f"Expected 0xCAFEBABE, got 0x{data:08X}"
    await RisingEdge(dut.clk)
    assert dut.empty.value == 1, "FIFO should be empty after read"


@cocotb.test()
async def test_fill_drain(dut):
    """Test 3: Fill completely, drain completely."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Fill
    for i in range(DEPTH):
        await write_word(dut, 0xA0000000 + i)

    await RisingEdge(dut.clk)
    assert dut.full.value == 1, "FIFO should be full"
    assert int(dut.count.value) == DEPTH, f"Expected count={DEPTH}, got {int(dut.count.value)}"

    # Drain and verify order
    for i in range(DEPTH):
        data = await read_word(dut)
        assert data == 0xA0000000 + i, f"Word {i}: expected 0x{0xA0000000+i:08X}, got 0x{data:08X}"

    await RisingEdge(dut.clk)
    assert dut.empty.value == 1, "FIFO should be empty after drain"
    assert int(dut.count.value) == 0, f"Expected count=0, got {int(dut.count.value)}"


@cocotb.test()
async def test_full_flag(dut):
    """Test 4: Verify full assertion/de-assertion."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Fill to full
    for i in range(DEPTH):
        assert dut.full.value == 0, f"full should be 0 at count {i}"
        await write_word(dut, i)

    await RisingEdge(dut.clk)
    assert dut.full.value == 1, "full should be 1 when FIFO is full"

    # Attempt write when full — count should not change
    count_before = int(dut.count.value)
    await write_word(dut, 0xDEADDEAD)
    await RisingEdge(dut.clk)
    assert int(dut.count.value) == count_before, "Count should not change on full write"

    # Read one — full should de-assert
    await read_word(dut)
    await RisingEdge(dut.clk)
    assert dut.full.value == 0, "full should de-assert after one read"


@cocotb.test()
async def test_empty_flag(dut):
    """Test 5: Verify empty assertion/de-assertion."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    assert dut.empty.value == 1, "empty should be 1 initially"

    # Write one — empty should de-assert
    await write_word(dut, 0x12345678)
    await RisingEdge(dut.clk)
    assert dut.empty.value == 0, "empty should de-assert after write"

    # Read it back — empty should re-assert
    await read_word(dut)
    await RisingEdge(dut.clk)
    assert dut.empty.value == 1, "empty should re-assert after read"

    # Read when empty — should be a no-op
    count_before = int(dut.count.value)
    dut.rd_en.value = 1
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.count.value) == count_before, "Count should not change on empty read"


@cocotb.test()
async def test_data_integrity(dut):
    """Test 6: Randomised write/read with golden model comparison."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    random.seed(42)
    golden = []

    # Phase 1: Interleaved random writes and reads
    writes = 0
    reads = 0
    for iteration in range(100):
        await RisingEdge(dut.clk)  # settle — ensure count is up to date
        cur_count = int(dut.count.value)
        can_write = cur_count < DEPTH
        can_read = cur_count > 0

        if can_write and (not can_read or random.random() < 0.6):
            data = random.randint(0, 0xFFFFFFFF)
            golden.append(data)
            dut.wr_en.value = 1
            dut.wr_data.value = data
            await RisingEdge(dut.clk)
            dut.wr_en.value = 0
            writes += 1
        elif can_read:
            rd = int(dut.rd_data.value)
            dut.rd_en.value = 1
            await RisingEdge(dut.clk)
            dut.rd_en.value = 0
            expected = golden.pop(0)
            reads += 1
            assert rd == expected, \
                f"Iter {iteration} Read #{reads} mismatch: expected 0x{expected:08X}, got 0x{rd:08X} (writes={writes})"

    # Phase 2: Drain remaining and verify
    while True:
        await RisingEdge(dut.clk)
        if int(dut.count.value) == 0:
            break
        rd = int(dut.rd_data.value)
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        dut.rd_en.value = 0
        expected = golden.pop(0)
        reads += 1
        assert rd == expected, \
            f"Drain Read #{reads} mismatch: expected 0x{expected:08X}, got 0x{rd:08X}"

    assert len(golden) == 0, f"Golden model still has {len(golden)} entries (writes={writes}, reads={reads})"
