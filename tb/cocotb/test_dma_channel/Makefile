# Brendan Lynskey 2025
SIM = icarus
VERILOG_SOURCES = \
    $(shell pwd)/../../../rtl/dma_pkg.sv \
    $(shell pwd)/../../../rtl/dma_fifo.sv \
    $(shell pwd)/../../../rtl/dma_channel.sv
TOPLEVEL = dma_channel
COCOTB_TEST_MODULES = test_dma_channel
TOPLEVEL_LANG = verilog
COMPILE_ARGS = -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
