# Brendan Lynskey 2025
SIM = icarus
VERILOG_SOURCES = \
    $(shell pwd)/../../../rtl/dma_pkg.sv \
    $(shell pwd)/../../../rtl/dma_fifo.sv \
    $(shell pwd)/../../../rtl/dma_reg_file.sv \
    $(shell pwd)/../../../rtl/dma_axi_master.sv \
    $(shell pwd)/../../../rtl/dma_arbiter.sv \
    $(shell pwd)/../../../rtl/dma_channel.sv \
    $(shell pwd)/../../../rtl/dma_top.sv
TOPLEVEL = dma_top
COCOTB_TEST_MODULES = test_dma_top
TOPLEVEL_LANG = verilog
COMPILE_ARGS = -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
