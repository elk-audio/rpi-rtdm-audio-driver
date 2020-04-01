// SPDX-License-Identifier: GPL-2.0
/*
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#ifndef AUDIO_RTDM_H
#define AUDIO_RTDM_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define RTDM_SUBCLASS_GPIO	0
#define DEVICE_NAME		"audio_rtdm"
#define RTAUDIO_PROFILE_VER	1
#define AUDIO_RTDM_VERSION_MAJ	0
#define AUDIO_RTDM_VERSION_MIN	2
#define AUDIO_RTDM_VERSION_VER	0

#define AUDIO_IOC_MAGIC		'r'

/* ioctl request to wait on dma callback */
#define AUDIO_IRQ_WAIT			_IO(AUDIO_IOC_MAGIC, 1)
/* This ioctl not used anymore but kept for backwards compatibility */
#define AUDIO_IMMEDIATE_SEND		_IOW(AUDIO_IOC_MAGIC, 2, int)
/* ioctl request to start receiving audio callbacks */
#define AUDIO_PROC_START		_IO(AUDIO_IOC_MAGIC, 3)
/* ioctl to inform the driver the user space process has completed */
#define AUDIO_USERPROC_FINISHED		_IOW(AUDIO_IOC_MAGIC, 4, int)
/* ioctl to stop receiving audio callbacks */
#define AUDIO_PROC_STOP			_IO(AUDIO_IOC_MAGIC, 5)

struct audio_rtdm_buffers {
	uint32_t 	 	*cv_gate_out;
	uint32_t 	 	*cv_gate_in;
	void			*tx_buf;
	void			*rx_buf;
	size_t			buffer_len;
	size_t			period_len;
	dma_addr_t		tx_phys_addr;
	dma_addr_t		rx_phys_addr;
};

/* General audio rtdm device struct */
struct audio_rtdm_dev {
	struct device			*dev;
	void __iomem			*i2s_base_addr;
	struct dma_chan			*dma_tx;
	struct dma_chan			*dma_rx;
	struct dma_async_tx_descriptor 	*tx_desc;
	struct dma_async_tx_descriptor	*rx_desc;
	dma_addr_t			fifo_dma_addr;
	unsigned			addr_width;
	unsigned			dma_burst_size;
	struct audio_rtdm_buffers	*buffer;
	rtdm_event_t 			irq_event;
	unsigned			wait_flag;
	unsigned			buffer_idx;
	uint64_t			kinterrupts;
	struct clk			*clk;
	bool				bcm_master_mode;
	int				clk_rate;
	char 				*audio_hat;
};
#endif