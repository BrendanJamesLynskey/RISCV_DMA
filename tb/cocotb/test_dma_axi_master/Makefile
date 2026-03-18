# Brendan Lynskey 2025
SIM = icarus
VERILOG_SOURCES = \
    $(shell pwd)/../../../rtl/dma_pkg.sv \
    $(shell pwd)/../../../rtl/dma_axi_master.sv
TOPLEVEL = dma_axi_master
COCOTB_TEST_MODULES = test_dma_axi_master
TOPLEVEL_LANG = verilog
COMPILE_ARGS = -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
