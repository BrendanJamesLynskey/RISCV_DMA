# Brendan Lynskey 2025

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

# Request type encodings from dma_pkg
ARB_REQ_READ  = 0b01
ARB_REQ_WRITE = 0b10
ARB_REQ_DESC  = 0b11

# AXI response codes
AXI_RESP_OKAY   = 0b00
AXI_RESP_SLVERR = 0b10
AXI_RESP_DECERR = 0b11


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

async def reset_dut(dut):
    """Apply synchronous reset for a few cycles and initialise all inputs."""
    dut.srst.value = 1
    dut.req_type.value = 0
    dut.req_addr.value = 0
    dut.req_len.value = 0
    dut.req_valid.value = 0
    dut.rd_ready.value = 0
    dut.wr_valid.value = 0
    dut.wr_data.value = 0
    dut.wr_last.value = 0
    # AXI slave side defaults
    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    await ClockCycles(dut.clk, 5)
    dut.srst.value = 0
    await RisingEdge(dut.clk)


async def issue_request(dut, req_type, addr, length):
    """Drive a request on the request interface.  *length* is 0-based."""
    await RisingEdge(dut.clk)
    dut.req_type.value = req_type
    dut.req_addr.value = addr
    dut.req_len.value = length
    dut.req_valid.value = 1
    await RisingEdge(dut.clk)
    dut.req_valid.value = 0


async def wait_for_done(dut, timeout_cycles=200):
    """Poll req_done and return once it pulses high."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if int(dut.req_done.value) == 1:
            return
    raise RuntimeError("Timed out waiting for req_done")


# ---------------------------------------------------------------------------
# AXI Slave Responder Coroutines
# ---------------------------------------------------------------------------

async def axi_slave_read(dut, mem, error_resp=0):
    """Respond to AR+R channel."""
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) == 1:
            addr = int(dut.m_axi_araddr.value)
            burst_len = int(dut.m_axi_arlen.value) + 1
            dut.m_axi_arready.value = 1
            await RisingEdge(dut.clk)
            dut.m_axi_arready.value = 0
            # Drive read data beats
            for i in range(burst_len):
                dut.m_axi_rvalid.value = 1
                dut.m_axi_rdata.value = mem.get((addr >> 2) + i, 0)
                dut.m_axi_rresp.value = error_resp
                dut.m_axi_rlast.value = 1 if i == burst_len - 1 else 0
                while True:
                    await RisingEdge(dut.clk)
                    if int(dut.m_axi_rready.value) == 1:
                        break
            dut.m_axi_rvalid.value = 0
            dut.m_axi_rlast.value = 0


async def axi_slave_write(dut, mem, error_resp=0):
    """Respond to AW+W+B channels."""
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value) == 1:
            addr = int(dut.m_axi_awaddr.value)
            burst_len = int(dut.m_axi_awlen.value) + 1
            dut.m_axi_awready.value = 1
            await RisingEdge(dut.clk)
            dut.m_axi_awready.value = 0
            # Accept write data beats
            dut.m_axi_wready.value = 1
            beat = 0
            while beat < burst_len:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_wvalid.value) == 1:
                    mem[(addr >> 2) + beat] = int(dut.m_axi_wdata.value)
                    beat += 1
            dut.m_axi_wready.value = 0
            # Send write response
            await RisingEdge(dut.clk)
            dut.m_axi_bvalid.value = 1
            dut.m_axi_bresp.value = error_resp
            while True:
                await RisingEdge(dut.clk)
                if int(dut.m_axi_bready.value) == 1:
                    break
            dut.m_axi_bvalid.value = 0


async def axi_slave_read_stall(dut, mem, stall_cycles=5, error_resp=0):
    """Like axi_slave_read but delays arready by *stall_cycles*."""
    while True:
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value) == 1:
            addr = int(dut.m_axi_araddr.value)
            burst_len = int(dut.m_axi_arlen.value) + 1
            # Stall before accepting
            for _ in range(stall_cycles):
                await RisingEdge(dut.clk)
            dut.m_axi_arready.value = 1
            await RisingEdge(dut.clk)
            dut.m_axi_arready.value = 0
            # Drive read data beats
            for i in range(burst_len):
                dut.m_axi_rvalid.value = 1
                dut.m_axi_rdata.value = mem.get((addr >> 2) + i, 0)
                dut.m_axi_rresp.value = error_resp
                dut.m_axi_rlast.value = 1 if i == burst_len - 1 else 0
                while True:
                    await RisingEdge(dut.clk)
                    if int(dut.m_axi_rready.value) == 1:
                        break
            dut.m_axi_rvalid.value = 0
            dut.m_axi_rlast.value = 0


async def drive_write_data(dut, data_list):
    """Drive wr_valid / wr_data / wr_last when wr_ready is high."""
    idx = 0
    total = len(data_list)
    dut.wr_valid.value = 1
    dut.wr_data.value = data_list[idx]
    dut.wr_last.value = 1 if total == 1 else 0
    while idx < total:
        await RisingEdge(dut.clk)
        if int(dut.wr_ready.value) == 1:
            idx += 1
            if idx < total:
                dut.wr_data.value = data_list[idx]
                dut.wr_last.value = 1 if idx == total - 1 else 0
    dut.wr_valid.value = 0
    dut.wr_last.value = 0


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_single_read(dut):
    """1-beat read — verify rd_data matches memory."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {0x100 >> 2: 0xDEADBEEF}
    cocotb.start_soon(axi_slave_read(dut, mem))

    dut.rd_ready.value = 1
    await issue_request(dut, ARB_REQ_READ, 0x100, 0)  # 1 beat

    # Collect read data
    rd_values = []
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.rd_valid.value) == 1:
            rd_values.append(int(dut.rd_data.value))
            if int(dut.rd_last.value) == 1:
                break

    await wait_for_done(dut)
    assert len(rd_values) == 1, f"Expected 1 beat, got {len(rd_values)}"
    assert rd_values[0] == 0xDEADBEEF, f"Data mismatch: {rd_values[0]:#010x}"
    dut._log.info("test_single_read PASSED")


@cocotb.test()
async def test_burst_read(dut):
    """4-beat burst read — verify all data beats."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    base = 0x200
    expected = [0xA0, 0xA1, 0xA2, 0xA3]
    mem = {(base >> 2) + i: v for i, v in enumerate(expected)}
    cocotb.start_soon(axi_slave_read(dut, mem))

    dut.rd_ready.value = 1
    await issue_request(dut, ARB_REQ_READ, base, 3)  # 4 beats (len=3)

    rd_values = []
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.rd_valid.value) == 1:
            rd_values.append(int(dut.rd_data.value))
            if int(dut.rd_last.value) == 1:
                break

    await wait_for_done(dut)
    assert rd_values == expected, f"Data mismatch: {rd_values} != {expected}"
    dut._log.info("test_burst_read PASSED")


@cocotb.test()
async def test_single_write(dut):
    """1-beat write — verify data reaches AXI slave memory."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {}
    cocotb.start_soon(axi_slave_write(dut, mem))

    cocotb.start_soon(drive_write_data(dut, [0xCAFEBABE]))
    await issue_request(dut, ARB_REQ_WRITE, 0x300, 0)

    await wait_for_done(dut)
    assert mem.get(0x300 >> 2) == 0xCAFEBABE, f"Write data mismatch: {mem}"
    dut._log.info("test_single_write PASSED")


@cocotb.test()
async def test_burst_write(dut):
    """4-beat burst write."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {}
    cocotb.start_soon(axi_slave_write(dut, mem))

    write_data = [0x10, 0x20, 0x30, 0x40]
    cocotb.start_soon(drive_write_data(dut, write_data))
    await issue_request(dut, ARB_REQ_WRITE, 0x400, 3)

    await wait_for_done(dut)
    for i, val in enumerate(write_data):
        got = mem.get((0x400 >> 2) + i)
        assert got == val, f"Beat {i}: expected {val:#x}, got {got:#x}"
    dut._log.info("test_burst_write PASSED")


@cocotb.test()
async def test_read_error(dut):
    """Read with SLVERR — verify req_error asserts."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {0x500 >> 2: 0x0}
    cocotb.start_soon(axi_slave_read(dut, mem, error_resp=AXI_RESP_SLVERR))

    dut.rd_ready.value = 1
    await issue_request(dut, ARB_REQ_READ, 0x500, 0)

    await wait_for_done(dut)
    assert int(dut.req_error.value) == 1, "Expected req_error to be asserted"
    assert int(dut.req_resp.value) == AXI_RESP_SLVERR, \
        f"Expected SLVERR, got {int(dut.req_resp.value):#04b}"
    dut._log.info("test_read_error PASSED")


@cocotb.test()
async def test_write_error(dut):
    """Write with DECERR — verify req_error asserts."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {}
    cocotb.start_soon(axi_slave_write(dut, mem, error_resp=AXI_RESP_DECERR))

    cocotb.start_soon(drive_write_data(dut, [0x0]))
    await issue_request(dut, ARB_REQ_WRITE, 0x600, 0)

    await wait_for_done(dut)
    assert int(dut.req_error.value) == 1, "Expected req_error to be asserted"
    assert int(dut.req_resp.value) == AXI_RESP_DECERR, \
        f"Expected DECERR, got {int(dut.req_resp.value):#04b}"
    dut._log.info("test_write_error PASSED")


@cocotb.test()
async def test_back_to_back(dut):
    """Sequential read then write — both must complete."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {0x700 >> 2: 0xBEEF0001}
    cocotb.start_soon(axi_slave_read(dut, mem))
    cocotb.start_soon(axi_slave_write(dut, mem))

    # --- Read phase ---
    dut.rd_ready.value = 1
    await issue_request(dut, ARB_REQ_READ, 0x700, 0)

    rd_values = []
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.rd_valid.value) == 1:
            rd_values.append(int(dut.rd_data.value))
            if int(dut.rd_last.value) == 1:
                break

    await wait_for_done(dut)
    assert rd_values == [0xBEEF0001], f"Read data mismatch: {rd_values}"
    dut.rd_ready.value = 0

    # --- Write phase ---
    cocotb.start_soon(drive_write_data(dut, [0x12345678]))
    await issue_request(dut, ARB_REQ_WRITE, 0x800, 0)

    await wait_for_done(dut)
    assert mem.get(0x800 >> 2) == 0x12345678, f"Write data mismatch: {mem}"
    dut._log.info("test_back_to_back PASSED")


@cocotb.test()
async def test_slave_stall(dut):
    """Delay arready for several cycles — DUT must wait patiently."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    mem = {0x900 >> 2: 0x55AA55AA}
    cocotb.start_soon(axi_slave_read_stall(dut, mem, stall_cycles=8))

    dut.rd_ready.value = 1
    await issue_request(dut, ARB_REQ_READ, 0x900, 0)

    rd_values = []
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.rd_valid.value) == 1:
            rd_values.append(int(dut.rd_data.value))
            if int(dut.rd_last.value) == 1:
                break

    await wait_for_done(dut)
    assert len(rd_values) == 1, f"Expected 1 beat, got {len(rd_values)}"
    assert rd_values[0] == 0x55AA55AA, f"Data mismatch: {rd_values[0]:#010x}"
    dut._log.info("test_slave_stall PASSED")
