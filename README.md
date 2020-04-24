# Audio RTDM driver

Xenomai based real time audio driver for Elk Pi hat

## Building

With kernel source and cross compiler setup on host:

`export KERNEL_PATH=<path to kernel source>`

`export CROSS_COMPILE=<arm compiler prefix>`

`make`

With Yocto layers setup and using devshell option of Bitbake:

`bitbake -c devshell virtual/kernel`

`export KERNEL_PATH=<path to kernel source in your bitbake tmp build files>`

`make`

By default CV gates support is enabled by the definition *BCM2835_i2S_CVGATES_SUPPORT*.
It is passed to Makefile with this line in the Makefile (remove this line to disable CV gates support)

`ccflags-y += -DBCM2835_i2S_CVGATES_SUPPORT`

## Usage Example
To load the driver out of tree :

`insmod pcm3168a-elk.ko`

`insmod bcm2835-i2s-elk.ko`

`insmod audio_rtdm.ko audio_buffer_size=<BUFFER SIZE>`

If the modules are installed already as part of the Kernel:

 `modprobe audio_rtdm. audio_buffer_size=<BUFFER SIZE>`