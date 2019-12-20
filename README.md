# Audio RTDM driver

Xenomai real-time audio driver for TI PCM3168A codec on the [Elk Pi hat](https://elk.audio/dev-kit/).

## Building

Have the kernel sources and an ARMv7 cross-compilation toolchain on your host machine, then do:

(https://github.com/elk-audio/elkpi-sdk) available on the host machine, then:

```
$ export KERNEL_PATH=<path to kernel source tree>
$ export CROSS_COMPILE=<arm compiler prefix>
$ make
```

It's possible to just use the official [Elk Audio OS cross-compiling SDK](https://github.com/elk-audio/elkpi-sdk), in which case you don't have to set the `CROSS_COMPILE` environment variable since the SDK will do it automatically.

As an alternative, you can build using the devshell option of Bitbake if you have all the Yocto layers ready on the host machine:

```
$ bitbake -c devshell virtual/kernel
$ export KERNEL_PATH=<path to kernel source in your bitbake tmp build files>
$ make
```

By default CV gates support is enabled with the definition `BCM2835_I2S_CVGATES_SUPPORT`.
It is currently set in the `Makefile` script with this line:

```
ccflags-y += -DBCM2835_i2S_CVGATES_SUPPORT
```

Just remove the line if you want to compile without CV gate support.

## Usage Example
To load the driver as an out-of-tree module, run as sudo:

```
$ insmod pcm3168a-elk.ko
$ insmod bcm2835-i2s-elk.ko
$ insmod audio_rtdm.ko audio_buffer_size=<BUFFER SIZE>
```

If the modules are installed already as part of the Kernel you can just do instead:

```
 $ modprobe audio_rtdm. audio_buffer_size=<BUFFER SIZE>
```

---
Copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
