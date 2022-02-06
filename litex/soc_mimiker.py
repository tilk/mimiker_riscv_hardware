#
# This file is based on Linux-on-LiteX-VexRiscv
#
# Copyright (c) 2019-2021, Linux-on-LiteX-VexRiscv Developers
#           (c) 2021, Marek Materzok
# SPDX-License-Identifier: BSD-2-Clause

#!/usr/bin/env python3

import os
import json
import shutil
import subprocess

from litex.soc.cores.cpu import VexRiscvSMP
from migen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.gpio import GPIOOut, GPIOIn
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.xadc import XADC
from litex.soc.cores.pwm import PWM
from litex.soc.cores.icap import ICAPBitstream
from litex.soc.cores.clock import S7MMCM

from litex.tools.litex_json2dts_linux import generate_dts
from litex.tools.litex_json2renode import generate_repl

# SoCMimiker -----------------------------------------------------------------------------------------

def SoCMimiker(soc_cls, **kwargs):
    class _SoCMimiker(soc_cls):
        csr_map = {**soc_cls.csr_map, **{
            "ctrl":       0,
            "uart":       2,
            "timer0":     3,
        }}
        interrupt_map = {**soc_cls.interrupt_map, **{
            "uart":       1,
            "timer0":     2,
        }}
        mem_map = {**soc_cls.mem_map, **{
            "csr":          0xf0000000,
        }}

        def __init__(self, cpu_variant="linux", uart_baudrate=1e6, **kwargs):

            # SoC ----------------------------------------------------------------------------------
            soc_cls.__init__(self,
                cpu_type       = "vexriscv_smp",
                cpu_variant    = cpu_variant,
                uart_baudrate  = uart_baudrate,
                max_sdram_size = 0x40000000, # Limit mapped SDRAM to 1GB.
                **kwargs)

            # Add linker region for OpenSBI
            self.add_memory_region("opensbi", self.mem_map["main_ram"] + 0x00f00000, 0x80000, type="cached+linker")

        # DTS generation ---------------------------------------------------------------------------
        def generate_dts(self, board_name):
            json_src = os.path.join("build", board_name, "csr.json")
            dts = os.path.join("build", board_name, "{}.dts".format(board_name))

            with open(json_src) as json_file, open(dts, "w") as dts_file:
                dts_content = generate_dts(json.load(json_file), polling=False, initrd_start=0x2000000, initrd_size=0x3000000)
                dts_file.write(dts_content)

        # DTS compilation --------------------------------------------------------------------------
        def compile_dts(self, board_name, symbols=False):
            dts = os.path.join("build", board_name, "{}.dts".format(board_name))
            dtb = os.path.join("build", board_name, "{}.dtb".format(board_name))
            subprocess.check_call(
                "dtc -@ -O dtb -o clint.dtbo clint.dts", shell=True)
            subprocess.check_call(
                "dtc -@ -O dtb -o {} {}".format(dtb, dts), shell=True)

        # DTB combination --------------------------------------------------------------------------
        def combine_dtb(self, board_name, overlays=""):
            dtb_in = os.path.join("build", board_name, "{}.dtb".format(board_name))
            dtb_out = os.path.join("images", "rv32.dtb")
            overlays += " clint.dtbo"
            subprocess.check_call(
                "fdtoverlay -i {} -o {} {}".format(dtb_in, dtb_out, overlays), shell=True)

        # REPL generation --------------------------------------------------------------------------
        def generate_repl(self, board_name):
            json_src = os.path.join("build", board_name, "csr.json")
            repl = os.path.join("build", board_name, "{}.repl".format(board_name))

            with open(json_src) as json_file, open(repl, "w") as repl_file:
                resc_content = generate_repl(json.load(json_file), {}, [])
                repl_file.write(resc_content)

    return _SoCMimiker(**kwargs)

