# Brendan Lynskey 2025
SIM = icarus
VERILOG_SOURCES = \
    $(shell pwd)/../../../rtl/dma_pkg.sv \
    $(shell pwd)/../../../rtl/dma_reg_file.sv
TOPLEVEL = dma_reg_file
COCOTB_TEST_MODULES = test_dma_reg_file
TOPLEVEL_LANG = verilog
COMPILE_ARGS = -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
