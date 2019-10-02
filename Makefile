obj-m += audio_rtdm.o
audio_rtdm-objs := rpi-rtdm-audio.o rpi-pcm3168a.o rpi-bcm2835-i2s.o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*
