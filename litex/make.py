#!/usr/bin/env python3

#
# This file is based on Linux-on-LiteX-VexRiscv
#
# Copyright (c) 2019-2021, Linux-on-LiteX-VexRiscv Developers
#           (c) 2021 Marek Materzok
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse
import os

from litex.soc.cores.cpu import VexRiscvSMP
from litex.soc.integration.builder import Builder

from litespi.modules import *
from litespi.opcodes import SpiNorFlashOpCodes as Codes

from soc_mimiker import SoCMimiker

kB = 1024

# Board definition----------------------------------------------------------------------------------

class Board:
    soc_kwargs = {"integrated_rom_size": 0x10000, "l2_size": 0}
    def __init__(self, soc_cls=None, soc_capabilities={}, soc_constants={}, bitstream_ext=""):
        self.soc_cls          = soc_cls
        self.soc_capabilities = soc_capabilities
        self.soc_constants    = soc_constants
        self.bitstream_ext    = bitstream_ext

    def load(self, filename):
        prog = self.platform.create_programmer()
        prog.load_bitstream(filename)

    def flash(self, filename):
        prog = self.platform.create_programmer()
        prog.flash(0, filename)

# Icesugar Pro support ------------------------------------------------------------------------------------

class IcesugarPro(Board):
    spiflash = W25Q256JV(Codes.READ_1_1_1)
    soc_kwargs = {
        "sys_clk_freq" : int(50e6), # 48MHz default.
        "l2_size"      : 2048,      # Use Wishbone and L2 for memory accesses.
    }
    def __init__(self):
        from litex_boards.targets import muselab_icesugar_pro
        Board.__init__(self, muselab_icesugar_pro.BaseSoC, soc_capabilities={
            # Communication
            "serial",
            # GPIO
            # pin collision with user_led
            #"rgb_led",
            # Storage
            "sdcard",
            # USRMCLK issue unsolved in litex_boards
            #"spiflash",
        }, bitstream_ext=".bit")

#---------------------------------------------------------------------------------------------------
# Build
#---------------------------------------------------------------------------------------------------

supported_boards = {
    "icesugar_pro":    IcesugarPro,
}

def main():
    description = "Mimiker on LiteX-VexRiscv\n\n"
    description += "Available boards:\n"
    for name in supported_boards.keys():
        description += "- " + name + "\n"
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--board",          required=True,            help="FPGA board")
    parser.add_argument("--device",         default=None,             help="FPGA device")
    parser.add_argument("--variant",        default=None,             help="FPGA board variant")
    parser.add_argument("--toolchain",      default=None,             help="Toolchain use to build")
    parser.add_argument("--build",          action="store_true",      help="Build bitstream")
    parser.add_argument("--load",           action="store_true",      help="Load bitstream (to SRAM)")
    parser.add_argument("--fdtoverlays",    default="",               help="Device Tree Overlays to apply")
    VexRiscvSMP.args_fill(parser)
    args = parser.parse_args()

    board_name = args.board
    board = supported_boards[board_name]()
    soc_kwargs = Board.soc_kwargs
    soc_kwargs.update(board.soc_kwargs)

    # CPU parameters ---------------------------------------------------------------------------
    # Do memory accesses through Wishbone and L2 cache when L2 size is configured.
    args.with_wishbone_memory = soc_kwargs["l2_size"] != 0
    VexRiscvSMP.args_read(args)

    # SoC parameters ---------------------------------------------------------------------------
    if args.device is not None:
        soc_kwargs.update(device=args.device)
    if args.variant is not None:
        soc_kwargs.update(variant=args.variant)
    if args.toolchain is not None:
        soc_kwargs.update(toolchain=args.toolchain)

    # SoC creation -----------------------------------------------------------------------------
    soc = SoCMimiker(board.soc_cls, **soc_kwargs)
    board.platform = soc.platform

    # SoC constants ----------------------------------------------------------------------------
    for k, v in board.soc_constants.items():
        soc.add_constant(k, v)

    # SoC peripherals --------------------------------------------------------------------------

    # Build ------------------------------------------------------------------------------------
    build_dir = os.path.join("build", board_name)
    builder   = Builder(soc,
        output_dir   = os.path.join("build", board_name),
        bios_options = ["TERM_MINI"],
        csr_json     = os.path.join(build_dir, "csr.json"),
        csr_csv      = os.path.join(build_dir, "csr.csv")
    )
    builder.build(run=args.build, build_name=board_name)

    # DTS --------------------------------------------------------------------------------------
    soc.generate_dts(board_name)
    soc.compile_dts(board_name, args.fdtoverlays)

    # DTB --------------------------------------------------------------------------------------
    soc.combine_dtb(board_name, args.fdtoverlays)

    # REPL -------------------------------------------------------------------------------------
    soc.generate_repl(board_name)

    # Load FPGA bitstream ----------------------------------------------------------------------
    if args.load:
        board.load(filename=os.path.join(builder.gateware_dir, soc.build_name + board.bitstream_ext))

if __name__ == "__main__":
    main()



