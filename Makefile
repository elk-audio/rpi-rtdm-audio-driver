obj-m += rpi-rtdm-audio-driver.o
rpi-rtdm-audio-driver-objs := rpi-rtdm-audio.o rpi-rtdm-codec-utils.o rpi-rtdm-i2s.o
all:
	$(MAKE) ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} -C /home/nitin/work/rpi/raspi/rpi-xenomai M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*