ccflags-y += -DBCM2835_I2S_CVGATES_SUPPORT
obj-m += pcm3168a-elk.o
obj-m += pcm5122-elk.o
obj-m += bcm2835-i2s-elk.o
obj-m += audio-rtdm.o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH)  M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*
