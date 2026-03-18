# Brendan Lynskey 2025

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


# Constants from dma_pkg
ARB_REQ_READ  = 0b01
ARB_REQ_WRITE = 0b10
ARB_REQ_DESC  = 0b11

NUM_CH = 4


async def reset_dut(dut):
    """Reset the DUT and clear all inputs."""
    dut.srst.value = 1
    dut.req.value = 0
    dut.axi_req_done.value = 0
    # set flat array ports to 0
    dut.req_type_flat.value = 0
    dut.req_addr_flat.value = 0
    dut.req_len_flat.value = 0
    await ClockCycles(dut.clk, 3)
    dut.srst.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_no_requests(dut):
    """Test that grant_valid stays low when no channels are requesting."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Hold no requests for several cycles
    dut.req.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        assert dut.grant_valid.value == 0, \
            f"grant_valid should be 0 with no requests, got {dut.grant_valid.value}"


@cocotb.test()
async def test_single_request(dut):
    """Test that a single request from channel 0 is granted."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Channel 0 requests a read
    dut.req.value = 0b0001
    dut.req_type_flat.value = ARB_REQ_READ << (0 * 2)
    dut.req_addr_flat.value = 0x1000_0000 << (0 * 32)
    dut.req_len_flat.value = 16 << (0 * 8)

    # Wait for grant
    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.grant_valid.value == 1:
            break

    assert dut.grant_valid.value == 1, "grant_valid should be 1 for single request"
    grant = dut.grant.value.integer
    assert grant & 0x1, f"grant[0] should be set, got grant=0x{grant:x}"

    # Check AXI output reflects channel 0 request
    assert dut.axi_req_valid.value == 1, "axi_req_valid should be 1"
    assert dut.axi_req_type.value == ARB_REQ_READ, \
        f"axi_req_type should be READ (0b01), got {dut.axi_req_type.value}"
    assert dut.axi_req_addr.value == 0x1000_0000, \
        f"axi_req_addr mismatch: got 0x{dut.axi_req_addr.value.integer:08x}"
    assert dut.axi_req_len.value == 16, \
        f"axi_req_len should be 16, got {dut.axi_req_len.value}"


@cocotb.test()
async def test_round_robin(dut):
    """Test round-robin cycling when all 4 channels request simultaneously."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # All channels request simultaneously
    dut.req.value = 0b1111
    type_flat = 0
    addr_flat = 0
    len_flat = 0
    for i in range(NUM_CH):
        type_flat |= ARB_REQ_WRITE << (i * 2)
        addr_flat |= (0x2000_0000 + (i * 0x1000)) << (i * 32)
        len_flat |= (8 + i) << (i * 8)
    dut.req_type_flat.value = type_flat
    dut.req_addr_flat.value = addr_flat
    dut.req_len_flat.value = len_flat

    granted_order = []

    for cycle in range(NUM_CH):
        # Wait for a grant
        for _ in range(10):
            await RisingEdge(dut.clk)
            if dut.grant_valid.value == 1:
                break

        assert dut.grant_valid.value == 1, \
            f"grant_valid should be 1 on round-robin cycle {cycle}"

        grant = dut.grant.value.integer
        # Determine which channel was granted (one-hot)
        granted_ch = None
        for ch in range(NUM_CH):
            if grant & (1 << ch):
                granted_ch = ch
                break
        assert granted_ch is not None, f"No channel granted, grant=0x{grant:x}"
        granted_order.append(granted_ch)

        # Pulse axi_req_done to advance to next grant
        dut.axi_req_done.value = 1
        await RisingEdge(dut.clk)
        dut.axi_req_done.value = 0

    # Verify round-robin order: should be sequential 0,1,2,3
    expected = [0, 1, 2, 3]
    assert granted_order == expected, \
        f"Round-robin order should be {expected}, got {granted_order}"


@cocotb.test()
async def test_fixed_priority(dut):
    """Test that the lowest-numbered requesting channel wins on first arbitration
    (round-robin pointer starts at 0 after reset)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Only channels 1 and 3 request (skip 0 and 2)
    dut.req.value = 0b1010
    dut.req_type_flat.value = (ARB_REQ_READ << (1 * 2)) | (ARB_REQ_DESC << (3 * 2))
    dut.req_addr_flat.value = (0x3000_0000 << (1 * 32)) | (0x4000_0000 << (3 * 32))
    dut.req_len_flat.value = (4 << (1 * 8)) | (1 << (3 * 8))

    # Wait for grant
    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.grant_valid.value == 1:
            break

    assert dut.grant_valid.value == 1, "grant_valid should be 1"
    grant = dut.grant.value.integer

    # With RR pointer starting at 0 after reset, the next eligible channel
    # scanning from 0 should be channel 1 (the lowest requesting channel).
    assert grant & (1 << 1), \
        f"Channel 1 should win first arbitration (lowest requesting), got grant=0b{grant:04b}"

    # Complete first grant and check channel 3 gets next
    dut.axi_req_done.value = 1
    await RisingEdge(dut.clk)
    dut.axi_req_done.value = 0

    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.grant_valid.value == 1:
            break

    grant = dut.grant.value.integer
    assert grant & (1 << 3), \
        f"Channel 3 should win second arbitration, got grant=0b{grant:04b}"


@cocotb.test()
async def test_hold_grant(dut):
    """Test that a grant is held when axi_req_done is not pulsed."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Channel 0 requests
    dut.req.value = 0b0001
    dut.req_type_flat.value = ARB_REQ_WRITE << (0 * 2)
    dut.req_addr_flat.value = 0x5000_0000 << (0 * 32)
    dut.req_len_flat.value = 32 << (0 * 8)

    # Wait for initial grant
    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.grant_valid.value == 1:
            break

    assert dut.grant_valid.value == 1, "grant_valid should be 1"
    initial_grant = dut.grant.value.integer
    assert initial_grant & 0x1, "Channel 0 should be granted"

    # Hold for many cycles without pulsing axi_req_done
    for cycle in range(20):
        await RisingEdge(dut.clk)
        assert dut.grant_valid.value == 1, \
            f"grant_valid dropped at hold cycle {cycle}"
        assert dut.grant.value.integer == initial_grant, \
            f"grant changed at hold cycle {cycle}: expected 0x{initial_grant:x}, " \
            f"got 0x{dut.grant.value.integer:x}"
