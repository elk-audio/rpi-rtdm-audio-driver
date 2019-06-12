/**
 * @file bcm2835-rtdm-audio-driver.c
 * @author Nitin Kulkarni
 * @brief Initial version of real-time audio driver for rpi
 * @version 0.1
 * @date 2019-05-31
 * 
 * @copyright Copyright Mind Music Labs (c) 2019
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <asm/barrier.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "rpi-rtdm-codec-utils.h"

#define I2S_INTERRUPT		85
#define RTDM_SUBCLASS_GPIO	0
#define DEVICE_NAME		"rtdm_audio"
#define RTAUDIO_PROFILE_VER	1

MODULE_AUTHOR("Nitin Kulkarni");
MODULE_DESCRIPTION("RTDM audio driver for RPi");
MODULE_LICENSE("GPL");

struct audio_dev_context {
	rtdm_event_t irq_event;
	ipipe_spinlock_t lock;
};

static int audio_driver_open(struct rtdm_fd *fd, int oflags) {
	printk("audio_rtdm: audio_open.\n");
	return 0;
}

static void audio_driver_close(struct rtdm_fd *fd) {
	printk("audio_rtdm: audio_close.\n");
}

static struct rtdm_driver audio_driver = {
	.profile_info		= RTDM_PROFILE_INFO(gpio,
						RTDM_CLASS_EXPERIMENTAL,
						RTDM_SUBCLASS_GENERIC,
						RTAUDIO_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.context_size		= sizeof( struct audio_dev_context),
	.ops = {
		.open		= audio_driver_open,
		.close		= audio_driver_close,
	},
};

static struct rtdm_device device = {
	.driver = &audio_driver,
	.label = DEVICE_NAME,
};

static int __init audio_rtdm_driver_init(void) {
	int ret;

	printk("audio_rtdm: Audio RTDM driver starting init ...\n");

	ret = rtdm_dev_register(&device);
	if (ret) {
		rtdm_dev_unregister(&device);
		printk(KERN_INFO "audio_rtdm: driver init failed\n");
		return ret;
	}

	printk(KERN_INFO "audio_rtdm: driver initialized\n");
	
	if (i2c_init())
		printk("i2c_init failed\n");
	
	return 0;
}

static void __exit audio_rtdm_driver_exit(void) {
	printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	if (i2c_exit())
		printk("i2c_exit failed\n");
	rtdm_dev_unregister(&device);
}

module_init(audio_rtdm_driver_init)
module_exit(audio_rtdm_driver_exit)