#!/bin/bash
# Brendan Lynskey 2025
# Compile and run all SystemVerilog testbenches

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RTL="$PROJ_DIR/rtl"
TB="$PROJ_DIR/tb/sv"

PASS=0
FAIL=0

run_test() {
    local name="$1"
    shift
    echo "========================================"
    echo "  $name"
    echo "========================================"
    local vvp="/tmp/${name}.vvp"
    if iverilog -g2012 -o "$vvp" "$@" 2>&1; then
        if vvp "$vvp" 2>&1 | tee /tmp/${name}.log | tail -5; then
            if grep -q "ALL TESTS PASSED" /tmp/${name}.log 2>/dev/null; then
                PASS=$((PASS + 1))
                echo "  >>> $name: PASSED"
            else
                FAIL=$((FAIL + 1))
                echo "  >>> $name: FAILED"
            fi
        else
            FAIL=$((FAIL + 1))
            echo "  >>> $name: FAILED (runtime error)"
        fi
    else
        FAIL=$((FAIL + 1))
        echo "  >>> $name: FAILED (compile error)"
    fi
    echo ""
}

run_test tb_dma_fifo \
    "$RTL/dma_pkg.sv" "$RTL/dma_fifo.sv" "$TB/tb_dma_fifo.sv"

run_test tb_dma_reg_file \
    "$RTL/dma_pkg.sv" "$RTL/dma_reg_file.sv" "$TB/tb_dma_reg_file.sv"

run_test tb_dma_axi_master \
    "$RTL/dma_pkg.sv" "$RTL/dma_axi_master.sv" "$TB/tb_dma_axi_master.sv"

run_test tb_dma_arbiter \
    "$RTL/dma_pkg.sv" "$RTL/dma_arbiter.sv" "$TB/tb_dma_arbiter.sv"

run_test tb_dma_channel \
    "$RTL/dma_pkg.sv" "$RTL/dma_fifo.sv" "$RTL/dma_channel.sv" "$TB/tb_dma_channel.sv"

run_test tb_dma_top \
    "$RTL/dma_pkg.sv" "$RTL/dma_fifo.sv" "$RTL/dma_reg_file.sv" \
    "$RTL/dma_axi_master.sv" "$RTL/dma_arbiter.sv" "$RTL/dma_channel.sv" \
    "$RTL/dma_top.sv" "$TB/tb_dma_top.sv"

echo "========================================"
echo "  SV TEST SUMMARY"
echo "  Passed: $PASS / $((PASS + FAIL))"
echo "========================================"

if [ $FAIL -ne 0 ]; then
    echo "SOME SV TESTS FAILED"
    exit 1
else
    echo "ALL SV TESTS PASSED"
fi
