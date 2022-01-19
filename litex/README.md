# Mimiker on LiteX-VexRiscv

## Preparing

Install [LiteX](https://github.com/enjoy-digital/litex).

Copy the following files to the `images` directory:

* `Image` -- the kernel image,
* `rootfs.cpio` -- the rootfs image,
* `opensbi.bin` -- the OpenSBI image.

## Running Verilator simulation

Run `./sim.py`.

## Building bitstream to Arty A7

This is for SoC developers. Xilinx Vivado is required.

Run:

```
./make.py --board arty_a7 --build
```

## Loading bitstream and booting

Before this step, the bitstream needs to be built.
If not a SoC developer, the bitstream image, `arty_a7.bit`, has to be copied to `build/arty_a7/gateware`.

Run:

```
./make.py --board arty_a7 --load
```

To boot LiteX, find the `ttyUSB` device connected to Arty, and run:

```
lxterm --speed 1e6 /dev/ttyUSB? --images images/boot.json
```

