#!/bin/bash
# Brendan Lynskey 2025
# Run all SystemVerilog and CocoTB tests

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo "  Running SystemVerilog tests..."
echo "============================================"
if "$SCRIPT_DIR/run_sv_tests.sh"; then
    SV_OK=1
else
    SV_OK=0
fi

echo ""
echo "============================================"
echo "  Running CocoTB tests..."
echo "============================================"
if "$SCRIPT_DIR/run_cocotb_tests.sh"; then
    COCOTB_OK=1
else
    COCOTB_OK=0
fi

echo ""
echo "============================================"
echo "  FINAL SUMMARY"
echo "============================================"
if [ $SV_OK -eq 1 ] && [ $COCOTB_OK -eq 1 ]; then
    echo "  ALL TESTS PASSED"
    exit 0
else
    [ $SV_OK -eq 0 ] && echo "  SV: SOME FAILURES"
    [ $COCOTB_OK -eq 0 ] && echo "  CocoTB: SOME FAILURES"
    exit 1
fi
