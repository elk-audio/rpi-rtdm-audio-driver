// SPDX-License-Identifier: GPL-2.0
/*
 * @brief Initial version of real-time audio driver for rpi
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "rpi-audio-rtdm.h"
#include "elk-pi-config.h"
#include "hifi-berry-config.h"
#include "hifi-berry-pro-config.h"
#include "pcm3168a-elk.h"
#include "pcm5122-elk.h"
#include "pcm1863-elk.h"
#include "bcm2835-i2s-elk.h"

MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_DESCRIPTION("RTDM audio driver for RPi");
MODULE_LICENSE("GPL");

#define DEFAULT_AUDIO_SAMPLING_RATE			48000
#define DEFAULT_AUDIO_NUM_INPUT_CHANNELS		8
#define DEFAULT_AUDIO_NUM_OUTPUT_CHANNELS		8
#define DEFAULT_AUDIO_NUM_CODEC_CHANNELS		8
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		64
#define DEFAULT_AUDIO_CODEC_FORMAT			INT24_LJ
#define DEFAULT_AUDIO_LOW_LATENCY_VAL			1
#define PLATFORM_TYPE					NATIVE_AUDIO

static uint audio_ver_maj = AUDIO_RTDM_VERSION_MAJ;
module_param(audio_ver_maj, uint, 0644);

static uint audio_ver_min = AUDIO_RTDM_VERSION_MIN;
module_param(audio_ver_min, uint, 0644);

static uint audio_ver_rev = AUDIO_RTDM_VERSION_VER;
module_param(audio_ver_rev, uint, 0644);

static uint audio_buffer_size = DEFAULT_AUDIO_N_FRAMES_PER_BUFFER;
module_param(audio_buffer_size, uint, 0644);

static uint audio_input_channels = DEFAULT_AUDIO_NUM_INPUT_CHANNELS;
module_param(audio_input_channels, uint, 0444);

static uint audio_output_channels = DEFAULT_AUDIO_NUM_OUTPUT_CHANNELS;
module_param(audio_output_channels, uint, 0444);

static uint audio_sampling_rate = DEFAULT_AUDIO_SAMPLING_RATE;
module_param(audio_sampling_rate, uint, 0444);

static uint audio_format = DEFAULT_AUDIO_CODEC_FORMAT;
module_param(audio_format, uint, 0444);

static uint platform_type = PLATFORM_TYPE;
module_param(platform_type, uint, 0444);

static char *audio_hat = "elk-pi";
module_param(audio_hat, charp, 0644);

static uint audio_enable_low_latency = DEFAULT_AUDIO_LOW_LATENCY_VAL;
module_param(audio_enable_low_latency, uint, 0644);

struct audio_dev_context {
	struct audio_rtdm_dev *i2s_dev;
	uint64_t user_proc_calls;
};

static ssize_t audio_buffer_size_show(struct class *cls, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_buffer_size);
}

static ssize_t audio_buffer_size_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	unsigned long bs;
	ssize_t result;
	result = sscanf(buf, "%lu", &bs);
	if (result != 1)
		return -EINVAL;
	audio_buffer_size = bs;
	return size;
}

static ssize_t audio_hat_show(struct class *cls, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", audio_hat);
}

static ssize_t audio_sampling_rate_show(struct class *cls, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_sampling_rate);
}

static CLASS_ATTR_RW(audio_buffer_size);
static CLASS_ATTR_RO(audio_hat);
static CLASS_ATTR_RO(audio_sampling_rate);

static struct attribute *audio_rtdm_class_attrs[] = {
	&class_attr_audio_buffer_size.attr,
	&class_attr_audio_hat.attr,
	&class_attr_audio_sampling_rate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(audio_rtdm_class);

struct class audio_rtdm_class = {
	.name        = "audio_rtdm",
	.class_groups = audio_rtdm_class_groups,
};

static int audio_driver_open(struct rtdm_fd *fd, int oflags)
{
	struct audio_dev_context *dev_context;
	printk(KERN_INFO "audio_rtdm: audio_open.\n");
	dev_context = (struct audio_dev_context *)rtdm_fd_to_private(fd);
	dev_context->i2s_dev = bcm2835_get_i2s_dev();
	dev_context->i2s_dev->wait_flag = 0;
	dev_context->user_proc_calls = 0;
	dev_context->i2s_dev->kinterrupts = 0;
	dev_context->i2s_dev->buffer_idx = 0;
	rtdm_event_init(&dev_context->i2s_dev->irq_event, 0);
	bcm2835_i2s_buffers_setup(audio_buffer_size, audio_output_channels);
	return 0;
}

static void audio_driver_close(struct rtdm_fd *fd)
{
	int i;
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	struct audio_rtdm_buffers *i2s_buffer = dev_context->i2s_dev->buffer;
	int *tx = i2s_buffer->tx_buf;

	printk(KERN_INFO "audio_rtdm: audio_close.\n");
	rtdm_event_destroy(&dev_context->i2s_dev->irq_event);
	if (dev_context->i2s_dev->wait_flag) {
		for (i = 0; i < i2s_buffer->buffer_len/4; i++) {
			tx[i] = 0;
		}
		dev_context->i2s_dev->wait_flag = 0;
	}
	bcm2835_i2s_exit();
}

static int audio_driver_mmap_nrt(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	struct audio_rtdm_buffers *i2s_buffer = dev_context->i2s_dev->buffer;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return dma_mmap_coherent(dev_context->i2s_dev->dma_rx->device->dev,
		vma,
		i2s_buffer->rx_buf, i2s_buffer->rx_phys_addr,
		RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE);
}

static int audio_driver_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
								void __user *arg)
{
	int result;
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
					rtdm_fd_to_private(fd);
	struct audio_rtdm_dev *dev = dev_context->i2s_dev;

	switch (request) {

	case AUDIO_IRQ_WAIT:
	{
		result = rtdm_event_wait(&dev->irq_event);
		if (result < 0) {
			printk(KERN_ERR "rtdm_event_wait failed\n");
			return result;
		}
		dev_context->user_proc_calls = dev->kinterrupts;
		result = dev->buffer_idx;
		return !result;
	}

	case AUDIO_PROC_START:
	{
		bcm2835_i2s_start_stop(dev, BCM2835_I2S_START_CMD);
		return 0;
	}

	case AUDIO_PROC_STOP:
	{
		bcm2835_i2s_start_stop(dev, BCM2835_I2S_STOP_CMD);
		return 0;
	}

	case AUDIO_USERPROC_FINISHED:
	{
		result = (dev->kinterrupts -
				dev_context->user_proc_calls);
		return result;
	}

	default:
		printk(KERN_WARNING "audio_rtdm : audio_ioctl_rt: invalid value"
							" %d\n", request);
		return -EINVAL;
	}
}

static struct rtdm_driver audio_driver = {
	.profile_info		= RTDM_PROFILE_INFO(gpio,
						RTDM_CLASS_EXPERIMENTAL,
						RTDM_SUBCLASS_GENERIC,
						RTAUDIO_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.context_size		= sizeof(struct audio_dev_context),
	.ops = {
		.open		= audio_driver_open,
		.close		= audio_driver_close,
		.mmap		= audio_driver_mmap_nrt,
		.ioctl_rt	= audio_driver_ioctl_rt,
	},
};

static struct rtdm_device rtdm_audio_device = {
	.driver = &audio_driver,
	.label = DEVICE_NAME,
};


int audio_rtdm_init(void)
{
	int ret, num_codec_channels = DEFAULT_AUDIO_NUM_CODEC_CHANNELS;

	ret = class_register(&audio_rtdm_class);
	if (ret < 0)
		return ret;

	if (!realtime_core_enabled()) {
		printk(KERN_ERR "audio_rtdm: rt core not enabled\n");
		return -ENODEV;
	}

	if (!strcmp(audio_hat, "hifi-berry")) {
		printk(KERN_INFO "audio_rtdm: hifi-berry hat\n");
		if (pcm5122_codec_init(HIFI_BERRY_DAC_MODE,
				HIFI_BERRY_SAMPLING_RATE,
				audio_enable_low_latency)) {
			printk(KERN_ERR "audio_rtdm: codec init failed\n");
			return -1;
		}
		audio_input_channels = HIFI_BERRY_NUM_INPUT_CHANNELS;
		audio_output_channels = HIFI_BERRY_NUM_OUTPUT_CHANNELS;
		num_codec_channels = HIFI_BERRY_NUM_CODEC_CHANNELS;
		audio_format = HIFI_BERRY_CODEC_FORMAT;
		audio_sampling_rate = HIFI_BERRY_SAMPLING_RATE;
	} else if (!strcmp(audio_hat, "hifi-berry-pro")) {
		printk(KERN_INFO "audio_rtdm: hifi-berry-pro hat\n");
		if (pcm1863_codec_init(audio_enable_low_latency)) {
			printk(KERN_ERR "audio_rtdm: pcm3168 codec failed\n");
			return -1;
		}
		if (pcm5122_codec_init(HIFI_BERRY_PRO_DAC_MODE,
					HIFI_BERRY_PRO_SAMPLING_RATE,
					audio_enable_low_latency)) {
			printk(KERN_ERR "audio_rtdm: pcm5122 codec failed\n");
			return -1;
		}
		audio_input_channels = HIFI_BERRY_PRO_NUM_INPUT_CHANNELS;
		audio_output_channels = HIFI_BERRY_PRO_NUM_OUTPUT_CHANNELS;
		num_codec_channels = HIFI_BERRY_PRO_NUM_CODEC_CHANNELS;
		audio_format = HIFI_BERRY_PRO_CODEC_FORMAT;
		audio_sampling_rate = HIFI_BERRY_PRO_SAMPLING_RATE;
	} else if (!strcmp(audio_hat, "elk-pi")) {
		printk(KERN_INFO "audio_rtdm: elk-pi hat\n");
		if (pcm3168a_codec_init()) {
			printk(KERN_ERR "audio_rtdm: codec init failed\n");
			return -1;
		}
		audio_input_channels = ELK_PI_NUM_INPUT_CHANNELS;
		audio_output_channels = ELK_PI_NUM_OUTPUT_CHANNELS;
		num_codec_channels = ELK_PI_NUM_CODEC_CHANNELS;
		audio_format = ELK_PI_CODEC_FORMAT;
		audio_sampling_rate = ELK_PI_SAMPLING_RATE;
	} else {
		printk(KERN_ERR "audio_rtdm: Unsupported hat\n");
	}

	if (bcm2835_i2s_init(audio_hat)) {
		printk(KERN_ERR "audio_rtdm: i2s init failed\n");
		return -1;
	}
	ret = rtdm_dev_register(&rtdm_audio_device);
	if (ret) {
		rtdm_dev_unregister(&rtdm_audio_device);
		printk(KERN_ERR "audio_rtdm:driver init failed\n");
		return ret;
	}

	printk(KERN_INFO "audio_rtdm: driver initialized\n");
	return 0;
}

void audio_rtdm_exit(void)
{
	printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	if (!strcmp(audio_hat, "hifi-berry")) {
		pcm5122_codec_exit();
	} else if (!strcmp(audio_hat, "elk-pi")) {
		pcm3168a_codec_exit();
	}
	class_unregister(&audio_rtdm_class);
	rtdm_dev_unregister(&rtdm_audio_device);
}

module_init(audio_rtdm_init)
module_exit(audio_rtdm_exit)
