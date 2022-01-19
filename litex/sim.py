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
    ),
    ("jtag", 0,
        Subsignal("tdi", Pins(1)),
        Subsignal("tdo", Pins(1)),
        Subsignal("tck", Pins(1)),
        Subsignal("tms", Pins(1)),
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

# JTAG TAP FSM -------------------------------------------------------------------------------------
# author: github.com/jevinskie
# TODO: merge to LiteX

class JTAGTAPFSM(Module):
    def __init__(self, tms: Signal, tck: Signal, use_ieee_encoding=False, expose_signals=True):
        self.submodules.fsm = fsm = ClockDomainsRenamer("jtag")(FSM())
        # Debug counter
        # self.tck_cnt = tck_cnt = Signal(16)
        # self.sync.jtag += tck_cnt.eq(tck_cnt + 1)
        fsm.act('test_logic_reset',
            If(~tms, NextState('run_test_idle'))
        )
        fsm.act('run_test_idle',
            If( tms, NextState('select_dr_scan'))
        )
        # DR
        fsm.act('select_dr_scan',
            If(~tms, NextState('capture_dr')    ).Else(NextState('select_ir_scan'))
        )
        fsm.act('capture_dr',
            If(~tms, NextState('shift_dr')      ).Else(NextState('exit1_dr'))
        )
        fsm.act('shift_dr',
            If( tms, NextState('exit1_dr'))
        )
        fsm.act('exit1_dr',
            If(~tms, NextState('pause_dr')      ).Else(NextState('update_dr'))
        )
        fsm.act('pause_dr',
            If( tms, NextState('exit2_dr'))
        )
        fsm.act('exit2_dr',
            If( tms, NextState('update_dr')     ).Else(NextState('shift_dr'))
        )
        fsm.act('update_dr',
            If( tms, NextState('select_dr_scan')).Else(NextState('run_test_idle'))
        )
        # IR
        fsm.act('select_ir_scan',
            If(~tms, NextState('capture_ir')    ).Else(NextState('test_logic_reset'))
        )
        fsm.act('capture_ir',
            If(~tms, NextState('shift_ir')      ).Else(NextState('exit1_ir'))
        )
        fsm.act('shift_ir',
            If( tms, NextState('exit1_ir'))
        )
        fsm.act('exit1_ir',
            If(~tms, NextState('pause_ir')      ).Else(NextState('update_ir'))
        )
        fsm.act('pause_ir',
            If( tms, NextState('exit2_ir'))
        )
        fsm.act('exit2_ir',
            If( tms, NextState('update_ir')     ).Else(NextState('shift_ir'))
        )
        fsm.act('update_ir',
            If( tms, NextState('select_dr_scan')).Else(NextState('run_test_idle'))
        )
        if use_ieee_encoding:
            # 11491.1-2013 Table 6-3 "State assignments for example TAP controller"  page 36 pdf page 58
            self.fsm.encoding = {
                'exit2_dr': 0,
                'exit1_dr': 1,
                'shift_dr': 2,
                'pause_dr': 3,
                'select_ir_scan': 4,
                'update_dr': 5,
                'capture_dr': 6,
                'select_dr_scan': 7,
                'exit2_ir': 8,
                'exit1_ir': 9,
                'shift_ir': 0xA,
                'pause_ir': 0xB,
                'run_test_idle': 0xC,
                'update_ir': 0xD,
                'capture_ir': 0xE,
                'test_logic_reset': 0xF,
            }
        if expose_signals:
            for state_name in fsm.actions:
                sig = fsm.ongoing(state_name)
                SHOUTING_NAME = state_name.upper()
                hcs_name = SHOUTING_NAME
                hcs = Signal(name=hcs_name)
                setattr(self, hcs_name, hcs)
                self.comb += hcs.eq(sig)

# Softcore TAP -------------------------------------------------------------------------------------
# TODO: merge to LiteX

class JTAGPort(Module):
    def __init__(self, insn, with_enable=False):
        self.insn = insn

        self.tck = Signal()
        self.tdi = Signal()
        self.tdo = Signal()

        self.reset = Signal()
        self.capture = Signal()
        self.shift = Signal()
        self.update = Signal()
        if with_enable:
            self.enable = Signal()

class SoftJTAG(Module):
    def __init__(self, insn_width, pads=None):
        self.insn_width = insn_width
        self.ports = []

        # JTAG signals
        self.trst = trst = Signal()
        self.tck = tck = Signal()
        self.tms = tms = Signal()
        self.tdi = tdi = Signal()
        self.tdo = tdo = Signal()

        # JTAG clock domains
        self.clock_domains.cd_jtag  = cd_jtag  = ClockDomain()
        self.clock_domains.cd_jtagn = cd_jtagn = ClockDomain()

        self.comb += cd_jtag.clk.eq(tck)
        self.comb += cd_jtag.rst.eq(trst)
        self.comb += cd_jtagn.clk.eq(~tck)
        self.comb += cd_jtagn.rst.eq(trst)

        # JTAG TAP state machine
        self.submodules.tap_fsm = tap_fsm = JTAGTAPFSM(tms, tck)

        self.reset = tap_fsm.TEST_LOGIC_RESET
        self.capture = tap_fsm.CAPTURE_DR
        self.shift = tap_fsm.SHIFT_DR
        self.update = tap_fsm.UPDATE_DR

        # JTAG TAP logic
        insnShift = Signal(insn_width)
        self.insn = insn = Signal(insn_width)
        self.bypass = bypass = Signal()
        self.tdoComb = tdoComb = Signal()
        self.sync.jtag += If(tap_fsm.TEST_LOGIC_RESET,
                insnShift.eq(0)
            ).Elif(tap_fsm.CAPTURE_IR,
                insnShift.eq(5) # ???
            ).Elif(tap_fsm.SHIFT_IR,
                insnShift.eq(Cat(insnShift[1:], tdi))
            )
        self.sync.jtagn += If(tap_fsm.TEST_LOGIC_RESET,
                insn.eq(0)
            ).Elif(tap_fsm.UPDATE_IR,
                insn.eq(insnShift)
            )
        self.sync.jtag += bypass.eq(tdi)
        self.sync.jtagn += tdo.eq(tdoComb)

        if pads:
            self.connect(pads)

    def connect(self, pads):
        if hasattr(pads, 'trst'):
            self.comb += self.trst.eq(pads.trst)
        else:
            self.comb += self.trst.eq(0)
        self.comb += self.tck.eq(pads.tck)
        self.comb += self.tms.eq(pads.tms)
        self.comb += self.tdi.eq(pads.tdi)
        self.comb += pads.tdo.eq(self.tdo)

    def add_port(self, port):
        self.ports.append(port)

    def do_finalize(self):

        insncase = {}
        insncase[C(-1, (self.insn_width, False))] = self.tdo.eq(self.bypass)
        for port in self.ports:
            self.comb += port.tdi.eq(self.tdi)
            self.comb += port.tck.eq(self.tck)
            enable = self.insn == port.insn
            if hasattr(port, 'enable'):
                self.comb += port.enable.eq(enable)
                self.comb += port.reset.eq(self.reset)
                self.comb += port.capture.eq(self.capture)
                self.comb += port.shift.eq(self.shift)
                self.comb += port.update.eq(self.update)
            else:
                self.comb += port.reset.eq(self.reset & enable)
                self.comb += port.capture.eq(self.capture & enable)
                self.comb += port.shift.eq(self.shift & enable)
                self.comb += port.update.eq(self.update & enable)
            insncase[C(port.insn, (self.insn_width, False))] = self.tdo.eq(port.tdo)

        self.comb += Case(self.insn, insncase)

# SimSoC -------------------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self, 
        init_memories=False, 
        sdram_module     = "MT48LC16M16",
        sdram_data_width = 32,
        sdram_verbosity  = 0,
        jtag             = False,
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
        
        # JTAG -------------------------------------------------------------------------------------
        if jtag:
            softJtag = SoftJTAG(4)
            vexPort = JTAGPort(0, True)
            softJtag.add_port(vexPort)
            softJtag.connect(platform.request("jtag"))
            self.comb += self.cpu.jtag_clk.eq(vexPort.tck)
            self.comb += self.cpu.jtag_tdi.eq(vexPort.tdi)
            self.comb += vexPort.tdo.eq(self.cpu.jtag_clk)
            self.comb += self.cpu.jtag_enable.eq(vexPort.enable)
            self.comb += self.cpu.jtag_capture.eq(vexPort.capture)
            self.comb += self.cpu.jtag_shift.eq(vexPort.shift)
            self.comb += self.cpu.jtag_update.eq(vexPort.update)
            self.comb += self.cpu.jtag_reset.eq(vexPort.reset)
            self.submodules += softJtag
            self.submodules += vexPort

    def do_finalize(self):
        self.add_constant("ROM_BOOT_ADDRESS", self.bus.regions["opensbi"].origin)
        SoCCore.do_finalize(self)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Mimiker on LiteX-VexRiscv Simulation")
    parser.add_argument("--trace",                action="store_true",      help="enable VCD tracing")
    parser.add_argument("--trace-start",          default=0,                help="cycle to start VCD tracing")
    parser.add_argument("--trace-end",            default=-1,               help="cycle to end VCD tracing")
    parser.add_argument("--opt-level",            default="O3",             help="compilation optimization level")
    parser.add_argument("--threads",              default=1,                help="set number of threads (default=1)")
    parser.add_argument("--jtagremote",           action="store_true",      help="enable debugging via jtagremote")
    VexRiscvSMP.args_fill(parser)
    args = parser.parse_args()

    VexRiscvSMP.args_read(args)
    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")
    if args.jtagremote:
        sim_config.add_module("jtagremote", "jtag", args={"port": 1234})
    board_name = "sim"
    build_dir  = os.path.join("build", board_name)

    for i in range(2): # a bit silly, but...
        soc = SoCMimiker(SimSoC,
            init_memories = i != 0,
            jtag = args.jtagremote)
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


