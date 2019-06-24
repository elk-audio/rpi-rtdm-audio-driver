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
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/io.h>




/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "rpi-rtdm-codec-utils.h"
#include "rpi-rtdm-i2s.h"

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

static struct rtdm_device rtdm_audio_device = {
	.driver = &audio_driver,
	.label = DEVICE_NAME,
};



static int audio_rtdm_driver_probe(struct platform_device *pdev) {
	int ret;

	//printk("audio_rtdm: Audio RTDM driver starting init ...\n");

	ret = rtdm_dev_register(&rtdm_audio_device);
	if (ret) {
		rtdm_dev_unregister(&rtdm_audio_device);
		printk(KERN_INFO "audio_rtdm: driver init failed\n");
		return ret;
	}
	
	if (rpi_rtdm_i2c_init())
		printk("audio_rtdm_driver_probe: rpi_rtdm_i2c_init failed\n");

	if (rpi_rtdm_i2s_init(pdev))
		printk("audio_rtdm_driver_probe: rpi_rtdm_i2s_init failed\n");
	
	printk(KERN_INFO "audio_rtdm: driver initialized\n");
	return 0;
}

static int audio_rtdm_driver_remove(struct platform_device *pdev) {
	printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	if (rpi_rtdm_i2c_exit())
		printk("i2c_exit failed\n");
	rtdm_dev_unregister(&rtdm_audio_device);
	rpi_rtdm_remove_irq();
	return 0;
}

static const struct of_device_id rpi_i2s_of_match[] = {
	{ .compatible = "brcm,bcm2835-i2s", },
	{},
};

MODULE_DEVICE_TABLE(of, rpi_i2s_of_match);

static struct platform_driver rpi_i2s_rtdm_driver = {
	.probe		= audio_rtdm_driver_probe,
	.remove		= audio_rtdm_driver_remove,
	.driver		= {
		.name	= "bcm2835-i2s",
		.of_match_table = rpi_i2s_of_match,
	},
};

module_platform_driver(rpi_i2s_rtdm_driver);