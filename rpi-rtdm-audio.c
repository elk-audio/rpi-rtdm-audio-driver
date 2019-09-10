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
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>


/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "rpi-pcm3168a.h"
#include "rpi-bcm2835-i2s.h"
#include "rpi-rtdm-audio.h"

MODULE_AUTHOR("Nitin Kulkarni");
MODULE_DESCRIPTION("RTDM audio driver for RPi");
MODULE_LICENSE("GPL");

static uint audio_buffer_size = DEFAULT_AUDIO_N_FRAMES_PER_BUFFER;
module_param(audio_buffer_size, uint, 0644);

static uint audio_channels = DEFAULT_AUDIO_N_CHANNELS;
module_param(audio_channels, uint, 0644);

static uint audio_ver_maj = AUDIO_RTDM_VERSION_MAJ;
module_param(audio_ver_maj, uint, 0644);

static uint audio_ver_min = AUDIO_RTDM_VERSION_MIN;
module_param(audio_ver_min, uint, 0644);

static uint audio_ver_rev = AUDIO_RTDM_VERSION_VER;
module_param(audio_ver_rev, uint, 0644);

struct audio_dev_context {
	ipipe_spinlock_t lock;
	struct rpi_i2s_dev *i2s_dev;
	uint64_t user_proc_completions;
	uint64_t			kinterrupts;
	unsigned			buffer_idx;
	rtdm_task_t *audio_task;
};

extern struct rpi_i2s_dev *rpi_device_i2s;
dma_cookie_t cookie_tx;
dma_cookie_t cookie_rx;

/* static void audio_intr_handler(void *ctx)
{
	unsigned long flags;
	struct audio_dev_context *dev_context = ctx;
	int err,i, pos;
	uint64_t next_wake_up_ns;
	static uint64_t periodic_wake_up_ns;
	int *tx = (int *)  dev_context->i2s_dev->buffer->tx_buf;
	periodic_wake_up_ns = rtdm_clock_read_monotonic();

	while (!rtdm_task_should_stop()) {

		periodic_wake_up_ns += 10000000;

		rtdm_event_signal(&dev_context->i2s_dev->irq_event);
		err = rtdm_task_sleep_abs(periodic_wake_up_ns,
						 RTDM_TIMERMODE_ABSOLUTE);
		if (err) {
			printk("rtdm_task_sleep_abs failed Error code: %d\n",
									err);
			break;
		}
	}
} */

static int audio_driver_open(struct rtdm_fd *fd, int oflags) {
	struct audio_dev_context *dev_context;
	struct rpi_i2s_dev *dev = rpi_device_i2s;
	struct i2s_buffers_info *i2s_buffer = rpi_device_i2s->buffer;
	printk("audio_rtdm: audio_open.\n");
	dev_context = (struct audio_dev_context *)rtdm_fd_to_private(fd);
	dev_context->i2s_dev = dev;

	bcm28835_i2s_prepare_and_submit(dev);
	memset(i2s_buffer->rx_buf, 0,
			i2s_buffer->buffer_len * 2);
	dev_context->user_proc_completions = 0;
	dev_context->buffer_idx = 0;
	dev_context->kinterrupts = 0;
	rtdm_event_init(&dev_context->i2s_dev->irq_event, 0);
	return 0;
}

static void audio_driver_close(struct rtdm_fd *fd)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	struct rpi_i2s_dev *dev = dev_context->i2s_dev;
	enum dma_status status;
	struct dma_tx_state state;
	printk("audio_rtdm: audio_close.\n");
	status = dmaengine_tx_status(dev->dma_tx, cookie_tx, &state);
	printk("tx status = %d residue = %d\n",status, state.residue);
	status = dmaengine_tx_status(dev->dma_rx, cookie_rx, &state);
	printk("rx status = %d residue = %d\n",status, state.residue);
	
	rtdm_event_destroy(&dev_context->i2s_dev->irq_event);
}

static int audio_driver_mmap_nrt(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	struct i2s_buffers_info *i2s_buffer = dev_context->i2s_dev->buffer;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return rtdm_mmap_kmem(vma, i2s_buffer->rx_buf);
}

static int audio_driver_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
								void __user *arg) {
	int result,i, pos;
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
					rtdm_fd_to_private(fd);

	switch(request) {

	case AUDIO_IRQ_WAIT:
	{
		if ((result = rtdm_event_wait(&dev_context->i2s_dev->irq_event)
		) < 0) {
			return result;
		}
		else {
			dev_context->kinterrupts++;
			result = dev_context->buffer_idx;
			dev_context->buffer_idx = ~(dev_context->buffer_idx) & 0x00000001;
			
			return result;
		}
		
		return 0;
	}

	case AUDIO_PROC_START:
	{		
		bcm2835_i2s_clear_fifos(dev_context->i2s_dev, true, true);
		
		for (i = 0; i < (24 + DEFAULT_AUDIO_N_CHANNELS);
		 i++) {
			rpi_reg_write(dev_context->i2s_dev->base_addr,
					BCM2835_I2S_FIFO_A_REG, 0);
		 }
		bcm2835_i2s_start_stop(dev_context->i2s_dev,
					BCM2835_I2S_START_CMD);

		return 0;
	}
	
	case AUDIO_PROC_STOP:
	{
		dmaengine_terminate_async(dev_context->i2s_dev->dma_tx);
		dmaengine_terminate_async(dev_context->i2s_dev->dma_rx);
		bcm2835_i2s_start_stop(dev_context->i2s_dev, 
					BCM2835_I2S_STOP_CMD);
		return 0;
	}

	case AUDIO_USERPROC_FINISHED:
	{
		wmb();
		dev_context->user_proc_completions++;

		return (dev_context->kinterrupts -
		dev_context->user_proc_completions);
	}

	default:
		printk(KERN_WARNING "audio_rtdm : audio_ioctl_rt: invalid value"
							" %d\n", request);
		return -EINVAL;
	}
}

/* static int audio_driver_ioctl_nrt(struct rtdm_fd *fd, unsigned int request,
				void __user *arg)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
					rtdm_fd_to_private(fd);
	int i, result = 0;
	int *rx = (int *)  dev_context->i2s_dev->buffer->rx_buf;
	int *tx = (int *)  dev_context->i2s_dev->buffer->tx_buf;
	struct i2s_buffers_info *i2s_buffer = dev_context->i2s_dev->buffer;

	switch(request) {

	case AUDIO_PROC_START:
	{
		dev_context->audio_task = kcalloc(1,sizeof(rtdm_task_t),
								GFP_KERNEL);
		result = rtdm_task_init(dev_context->audio_task,
					 "audio_intr_handler",
					audio_intr_handler, dev_context,
					95, 0);
		if (result) {
			printk("audio rtdm: rtdm_task_init failed\n");
			return -EINVAL;
		}
		return 0;
	}
	

	case AUDIO_PROC_STOP:
	{
		if (dev_context->audio_task) {
			rtdm_event_destroy(&dev_context->i2s_dev->irq_event);
			rtdm_task_destroy(dev_context->audio_task);
			kfree(dev_context->audio_task);
			dev_context->audio_task = NULL;
		}
		return 0;
	}
	default:
		printk("audio_rtdm: audio_ioctl_nrt invalid value %d\n",
								request);
		return -EINVAL;
	}
	return result;
} */

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
		.mmap		= audio_driver_mmap_nrt,
		.ioctl_rt	= audio_driver_ioctl_rt,
		//.ioctl_nrt	= audio_driver_ioctl_nrt,
	},
};

static struct rtdm_device rtdm_audio_device = {
	.driver = &audio_driver,
	.label = DEVICE_NAME,
};


static int audio_rtdm_driver_probe(struct platform_device *pdev) {
	int ret;

	if (!realtime_core_enabled()) {
		printk("realtime_core_enabled returned false ! \n");
		return -ENODEV;
	}

	if (rpi_pcm3168a_codec_init()) {
		printk("audio_rtdm_driver_probe: codec_init failed\n");
		return -1;
	}
	msleep(300);

	if (bcm2835_i2s_init(pdev)) {
		printk("audio_rtdm_driver_probe: i2s_init failed\n");
		return -1;
	}
	ret = rtdm_dev_register(&rtdm_audio_device);
	if (ret) {
		rtdm_dev_unregister(&rtdm_audio_device);
		printk(KERN_INFO "audio_rtdm: driver init failed\n");
		return ret;
	}

	printk(KERN_INFO "audio_rtdm: driver initialized\n");
	return 0;
}

static int audio_rtdm_driver_remove(struct platform_device *pdev) {
	printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	if (rpi_pcm3168a_codec_exit())
		printk("i2c_exit failed\n");
	bcm2835_i2s_exit(pdev);
	rtdm_dev_unregister(&rtdm_audio_device);
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