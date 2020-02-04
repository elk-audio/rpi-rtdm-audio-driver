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

/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "pcm3168a-elk.h"
#include "bcm2835-i2s-elk.h"
#include "audio-rtdm.h"

MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_DESCRIPTION("RTDM audio driver for RPi");
MODULE_LICENSE("GPL");

static uint audio_ver_maj = AUDIO_RTDM_VERSION_MAJ;
module_param(audio_ver_maj, uint, 0644);

static uint audio_ver_min = AUDIO_RTDM_VERSION_MIN;
module_param(audio_ver_min, uint, 0644);

static uint audio_ver_rev = AUDIO_RTDM_VERSION_VER;
module_param(audio_ver_rev, uint, 0644);

static uint audio_buffer_size = DEFAULT_AUDIO_N_FRAMES_PER_BUFFER;
module_param(audio_buffer_size, uint, 0644);

static uint audio_channels = DEFAULT_AUDIO_N_CHANNELS;
module_param(audio_channels, uint, 0644);

static uint audio_sampling_rate = DEFAULT_AUDIO_SAMPLING_RATE;
module_param(audio_sampling_rate, uint, 0444);

struct audio_dev_context {
	struct audio_rtdm_dev *i2s_dev;
	uint64_t user_proc_calls;
};

static int audio_driver_open(struct rtdm_fd *fd, int oflags)
{
	struct audio_dev_context *dev_context;
	printk(KERN_INFO "audio_rtdm: audio_open.\n");
	dev_context = (struct audio_dev_context *)rtdm_fd_to_private(fd);
	dev_context->i2s_dev = bcm2835_get_i2s_dev();
	dev_context->i2s_dev->wait_flag = 0;
	dev_context->user_proc_calls = 0;
	rtdm_event_init(&dev_context->i2s_dev->irq_event, 0);
	return 0;
}

static void audio_driver_close(struct rtdm_fd *fd)
{
	int i;
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	int *tx = dev_context->i2s_dev->buffer->tx_buf;
	printk(KERN_INFO "audio_rtdm: audio_close.\n");
	rtdm_event_destroy(&dev_context->i2s_dev->irq_event);
	if (dev_context->i2s_dev->wait_flag) {
		for (i = 0; i < dev_context->i2s_dev->buffer->buffer_len; i++) {
			tx[i] = 0;
		}
		dev_context->i2s_dev->wait_flag = 0;
	}
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
		dev->wait_flag = 1;
		return 0;
	}

	case AUDIO_PROC_STOP:
	{
		dev->wait_flag = 0;
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
	int ret;

	if (!realtime_core_enabled()) {
		printk(KERN_ERR "audio_rtdm: rt core not enabled\n");
		return -ENODEV;
	}
	msleep(100);
	if (pcm3168a_codec_init()) {
		printk(KERN_ERR "audio_rtdm: codec init failed\n");
		return -1;
	}
	msleep(100);
	if (bcm2835_i2s_init(audio_buffer_size, audio_channels)) {
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
	pcm3168a_codec_exit();
	bcm2835_i2s_exit();
	rtdm_dev_unregister(&rtdm_audio_device);
}

module_init(audio_rtdm_init)
module_exit(audio_rtdm_exit)