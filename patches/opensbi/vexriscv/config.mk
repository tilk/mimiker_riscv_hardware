# vim: tabstop=8 shiftwidth=8 noexpandtab:

# Compiler flags.
platform-cppflags-y =
platform-cflags-y =
platform-asflags-y =
platform-ldflags-y =

# RISC-V GCC options are described here:
#   https://gcc.gnu.org/onlinedocs/gcc/RISC-V-Options.html#RISC-V-Options
PLATFORM_RISCV_XLEN = 32
PLATFORM_RISCV_ABI = ilp32
PLATFORM_RISCV_ISA = rv32ima_zicsr_zifencei
PLATFORM_RISCV_CODE_MODEL = medany

# Firmware load address configuration.
FW_TEXT_START=0x40f00000

# We use the jump firmware configuration.
FW_JUMP=y

# This needs to be 4MB aligned for 32-bit system.
FW_JUMP_ADDR=0x40000000

# Use the same address as QEMU.
FW_JUMP_FDT_ADDR=0x40ef0000
