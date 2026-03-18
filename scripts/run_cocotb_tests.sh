#!/bin/bash
# Brendan Lynskey 2025
# Run all CocoTB tests

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
COCOTB="$PROJ_DIR/tb/cocotb"

PASS=0
FAIL=0

TESTS=(
    test_dma_fifo
    test_dma_reg_file
    test_dma_axi_master
    test_dma_arbiter
    test_dma_channel
    test_dma_top
)

for t in "${TESTS[@]}"; do
    echo "========================================"
    echo "  CocoTB: $t"
    echo "========================================"
    cd "$COCOTB/$t"
    make clean 2>/dev/null || true
    if make 2>&1 | tee /tmp/cocotb_${t}.log | tail -5; then
        if grep -q "FAIL=0" /tmp/cocotb_${t}.log 2>/dev/null; then
            PASS=$((PASS + 1))
            echo "  >>> $t: PASSED"
        else
            FAIL=$((FAIL + 1))
            echo "  >>> $t: FAILED"
        fi
    else
        FAIL=$((FAIL + 1))
        echo "  >>> $t: FAILED"
    fi
    echo ""
done

echo "========================================"
echo "  COCOTB TEST SUMMARY"
echo "  Passed: $PASS / $((PASS + FAIL))"
echo "========================================"

if [ $FAIL -ne 0 ]; then
    echo "SOME COCOTB TESTS FAILED"
    exit 1
else
    echo "ALL COCOTB TESTS PASSED"
fi
