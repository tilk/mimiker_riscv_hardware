#!/usr/bin/env python3

#
# This file is based on Linux-on-LiteX-VexRiscv
#
# Copyright (c) 2019-2021, Linux-on-LiteX-VexRiscv Developers
#           (c) 2021-2022 Marek Materzok
# SPDX-License-Identifier: BSD-2-Clause

import json
import argparse

from litex.soc.cores.cpu import VexRiscvSMP
from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone

from litedram import modules as litedram_modules
from litedram.phy.model import SDRAMPHYModel
from litex.tools.litex_sim import sdram_module_nphases, get_sdram_phy_settings
from litedram.core.controller import ControllerSettings

from litex.tools.litex_json2dts_linux import generate_dts

from soc_mimiker import SoCMimiker

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data",  Pins(8)),
    )
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Supervisor ---------------------------------------------------------------------------------------

class Supervisor(Module, AutoCSR):
    def __init__(self):
        self._finish  = CSR()  # controlled from CPU
        self.finish = Signal() # controlled from logic
        self.sync += If(self._finish.re | self.finish, Finish())

# SimSoC -------------------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self, 
        init_memories=False, 
        sdram_module     = "MT48LC16M16",
        sdram_data_width = 32,
        sdram_verbosity  = 0,
        **kwargs):

        ram_init = []
        if init_memories:
            ram_init = get_mem_data({
                "images/Image":       "0x00000000",
                "images/rv32.dtb":    "0x00ef0000",
                "images/rootfs.cpio": "0x02000000",
                "images/opensbi.bin": "0x00f00000"
            }, "little")

        platform = Platform()
        sys_clk_freq = int(100e6)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform,
            clk_freq  = sys_clk_freq,
            uart_name = "sim",
            integrated_rom_size = 0x10000,
            **kwargs)

        self.add_config("DISABLE_DELAYS")

        # Supervisor -------------------------------------------------------------------------------
        self.submodules.supervisor = Supervisor()

        # SDRAM ------------------------------------------------------------------------------------
        sdram_clk_freq   = int(100e6) # FIXME: use 100MHz timings
        sdram_module_cls = getattr(litedram_modules, sdram_module)
        sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
        sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
        phy_settings     = get_sdram_phy_settings(
            memtype    = sdram_module.memtype,
            data_width = sdram_data_width,
            clk_freq   = sdram_clk_freq)
        self.submodules.sdrphy = SDRAMPHYModel(
            module    = sdram_module,
            settings  = phy_settings,
            clk_freq  = sdram_clk_freq,
            verbosity = sdram_verbosity,
            init      = ram_init)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = sdram_module,
            origin        = self.mem_map["main_ram"],
            l2_cache_size = 0)
        self.add_constant("SDRAM_TEST_DISABLE") # Skip SDRAM test to avoid corrupting pre-initialized contents.

    def do_finalize(self):
        self.add_constant("ROM_BOOT_ADDRESS", self.bus.regions["opensbi"].origin)
        SoCCore.do_finalize(self)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Mimiker on LiteX-VexRiscv Simulation")
    parser.add_argument("--trace",                action="store_true",     help="enable VCD tracing")
    parser.add_argument("--trace-start",          default=0,               help="cycle to start VCD tracing")
    parser.add_argument("--trace-end",            default=-1,              help="cycle to end VCD tracing")
    parser.add_argument("--opt-level",            default="O3",            help="compilation optimization level")
    parser.add_argument("--threads",              default=1,               help="Set number of threads (default=1)")
    VexRiscvSMP.args_fill(parser)
    args = parser.parse_args()

    VexRiscvSMP.args_read(args)
    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")
    board_name = "sim"
    build_dir  = os.path.join("build", board_name)

    for i in range(2): # a bit silly, but...
        soc = SoCMimiker(SimSoC,
            init_memories = i != 0)
        builder = Builder(soc, output_dir=build_dir,
            compile_gateware = i != 0 ,
            csr_json         = os.path.join(build_dir, "csr.json"))
        builder.build(sim_config=sim_config,
            run         = i != 0,
            opt_level   = args.opt_level,
            trace       = args.trace,
            trace_start = int(args.trace_start),
            trace_end   = int(args.trace_end),
            threads     = args.threads)
        if i == 0:
            soc.generate_dts(board_name)
            soc.compile_dts(board_name)

if __name__ == "__main__":
    main()


