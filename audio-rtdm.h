/**
 * @file audio-rtdm.h
 * @author Nitin Kulkarni
 * @version 0.1
 *
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#ifndef RPI_RTDM_AUDIO_H
#define RPI_RTDM_AUDIO_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define RTDM_SUBCLASS_GPIO	0
#define DEVICE_NAME		"audio_rtdm"
#define RTAUDIO_PROFILE_VER	1
#define AUDIO_RTDM_VERSION_MAJ 	0
#define AUDIO_RTDM_VERSION_MIN	2
#define AUDIO_RTDM_VERSION_VER	0

#define DEFAULT_AUDIO_SAMPLING_RATE			48000
#define DEFAULT_AUDIO_N_CHANNELS			8
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		64

#define AUDIO_IOC_MAGIC		'r'


#define AUDIO_IRQ_WAIT			_IO(AUDIO_IOC_MAGIC, 1)
#define AUDIO_IMMEDIATE_SEND		_IOW(AUDIO_IOC_MAGIC, 2, int)
#define AUDIO_PROC_START		_IO(AUDIO_IOC_MAGIC, 3)
#define AUDIO_USERPROC_FINISHED		_IOW(AUDIO_IOC_MAGIC, 4, int)
#define AUDIO_PROC_STOP			_IO(AUDIO_IOC_MAGIC, 5)

struct audio_rtdm_buffers {
	uint32_t 	 	*cv_gate_out;
	uint32_t 	 	*cv_gate_in;
	void			*tx_buf;
	void			*rx_buf;
	size_t			buffer_len;
	size_t			period_len;
	dma_addr_t 			tx_phys_addr;
	dma_addr_t 			rx_phys_addr;
};

/* General audio rtdm device struct */
struct audio_rtdm_dev {
	struct device			*dev;
	void __iomem			*base_addr;
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
};
#endif