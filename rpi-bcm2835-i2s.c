/**
 * @file rpi-rtdm-i2s.c
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief I2S module of theRTDM audio driver.
 * A lot of stuff is based on the mainline I2S module by Florian Meier
 * @version 0.1
 * @date 2019-06-11
 * 
 * @copyright MIND music labs (c) 2019
 * 
 */
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <asm/barrier.h>
#include <linux/spinlock.h>
#include <rtdm/driver.h>
#include <linux/ipipe_domain.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/mm.h>
#include <linux/gpio.h>

#include "rpi-bcm2835-i2s.h"
#include "rpi-rtdm-audio.h"

/**
 * Cv gate gpio pin definitions
 */
#define 	NUM_OF_CV_OUTS	4
#define 	NUM_OF_CV_INS	2
#define	 	CV_GATE_OUT1	17
#define 	CV_GATE_OUT2	27
#define 	CV_GATE_OUT3	22
#define 	CV_GATE_OUT4	23
#define 	CV_GATE_IN1	24
#define 	CV_GATE_IN2	25

static uint audio_buffer_size = DEFAULT_AUDIO_N_FRAMES_PER_BUFFER;
module_param(audio_buffer_size, uint, 0644);

static uint audio_channels = DEFAULT_AUDIO_N_CHANNELS;
module_param(audio_channels, uint, 0644);

struct rpi_audio_driver *rpi_device_i2s;
extern dma_cookie_t cookie_tx;
extern dma_cookie_t cookie_rx;
static uint64_t  new_wakeup, old_wakeup = 0, diff;

static int cv_gate_out[NUM_OF_CV_OUTS] =
 {CV_GATE_OUT1, CV_GATE_OUT2, CV_GATE_OUT3, CV_GATE_OUT4};

static int cv_gate_in[NUM_OF_CV_INS] =
 {CV_GATE_IN1, CV_GATE_IN2};

void bcm2835_i2s_clear_fifos(struct rpi_audio_driver *dev,
				    bool tx, bool rx)
{
	uint32_t csreg, sync;
	uint32_t i2s_active_state;
	uint32_t off;
	uint32_t clr;
	int timeout = 1000;

	off =  tx ? BCM2835_I2S_TXON : 0;
	off |= rx ? BCM2835_I2S_RXON : 0;

	clr =  tx ? BCM2835_I2S_TXCLR : 0;
	clr |= rx ? BCM2835_I2S_RXCLR : 0;

	/* Backup the current state */
	rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &csreg);
	i2s_active_state = csreg & (BCM2835_I2S_RXON | BCM2835_I2S_TXON);

	/* Stop I2S module */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG, off, 0);

	/*
	 * Clear the FIFOs
	 * Requires at least 2 PCM clock cycles to take effect
	 */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG, clr, clr);

	rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &sync);
	sync &= BCM2835_I2S_SYNC;

	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_SYNC, ~sync);

	/* Wait for the SYNC flag changing it's state */
	while (--timeout) {
		rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &csreg);
		if ((csreg & BCM2835_I2S_SYNC) != sync)
			break;
	}

	/* Restore I2S state */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXON | BCM2835_I2S_TXON, i2s_active_state);
}

void bcm2835_i2s_start_stop(struct rpi_audio_driver *dev, int cmd)
{
	uint32_t mask, val, discarded = 0;
	int32_t tmp[9], samples[2] = {0xff, 0xff};
	wmb();
	mask = BCM2835_I2S_RXON | BCM2835_I2S_TXON;

	if (cmd == BCM2835_I2S_START_CMD) {
		rpi_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, mask);
		/*Assumption: According to my colleague Mr. Sharan Yagneswar the last two channels from pcm3168 are always zero &
		the probability of getting two successive zero values in any other two channels is as much as earth being hit by an asteroid
		*/
		while (samples[0] != 0 || samples[1] != 0) {
			rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG,
					 &val);
			if (val & BCM2835_I2S_RXD) {
				wmb();
				rpi_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG,
					 0x00);
				samples[1] = samples[0];
				rmb();
				rpi_reg_read(dev->base_addr, BCM2835_I2S_FIFO_A_REG,
					 &samples[0]);
					 tmp[discarded] = samples[0];
					 discarded++;
			}
		}
		printk(KERN_INFO "audio_rtdm: %d sampls discarded\n",discarded);
	}
	else {
		rpi_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, 0);
	}
}

static void bcm2835_i2s_dma_callback(void *data)
{
	int i;
	uint32_t val;
	struct rpi_audio_driver *dev = data;

	new_wakeup = rtdm_clock_read_monotonic();
	if (!old_wakeup) {
		old_wakeup = new_wakeup;
	} else {
		diff = new_wakeup - old_wakeup;
		if (diff < IRQ_FILTER_TIME_NS) {
			return;
		}
		old_wakeup = new_wakeup;
	}

	dev->kinterrupts++;
	dev->buffer_idx = ~(dev->buffer_idx) & 0x1;
	if (dev->wait_flag)
	{
		rtdm_event_signal(&dev->irq_event);
		for (i = 0; i < NUM_OF_CV_OUTS; i++) {
			val = (unsigned long) *dev->buffer->cv_gate_out &
			 BIT(i);
			gpio_set_value(cv_gate_out[i], val);
		}
		val = 0;
		for (i = 0; i < NUM_OF_CV_INS; i++) {
		val |= gpio_get_value(cv_gate_in[i]) << i;
		}
		*dev->buffer->cv_gate_in = val;
	}
}

static struct dma_async_tx_descriptor *
bcm2835_i2s_dma_prepare_cyclic(struct rpi_audio_driver *dev,
			enum dma_transfer_direction dir)
{
	struct dma_slave_config cfg;
	struct dma_chan *chan;
	int  flags;
	struct dma_async_tx_descriptor * desc;
	struct rpi_buffers_info *audio_buffers = dev->buffer;

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = dir;

	if (dir == DMA_MEM_TO_DEV) {
		cfg.dst_addr = dev->fifo_dma_addr;
		cfg.dst_addr_width = dev->addr_width;
		cfg.dst_maxburst = dev->dma_burst_size;
		chan = dev->dma_tx;
		flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan, audio_buffers->tx_phys_addr,audio_buffers->buffer_len, audio_buffers->period_len, dir,
				       flags);
	} else {
		cfg.src_addr = dev->fifo_dma_addr;
		cfg.src_addr_width = dev->addr_width;
		cfg.src_maxburst = dev->dma_burst_size;
		chan = dev->dma_rx;
		flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan, audio_buffers->rx_phys_addr,audio_buffers->buffer_len, audio_buffers->period_len, dir,
				       flags);
	}
	return desc;
}

static int bcm2835_i2s_dma_prepare(struct rpi_audio_driver *dev)
{
	int err;
	dev->tx_desc = bcm2835_i2s_dma_prepare_cyclic(dev, DMA_MEM_TO_DEV);
	if (!dev->tx_desc) {
		dev_err(dev->dev,
			"failed to get DMA TX descriptor\n");
		err = -EBUSY;
		return err;
	}

	dev->rx_desc = bcm2835_i2s_dma_prepare_cyclic(dev, DMA_DEV_TO_MEM);
	if (!dev->rx_desc) {
		dev_err(dev->dev,
			"failed to get DMA RX descriptor\n");
		err = -EBUSY;
		dmaengine_terminate_async(dev->dma_tx);
		return err;
	}
	dev->rx_desc->callback = bcm2835_i2s_dma_callback;
	dev->rx_desc->callback_param = dev;
	return 0;
}

static void bcm2835_i2s_submit_dma(struct rpi_audio_driver *dev)
{
	cookie_rx = dmaengine_submit(dev->rx_desc);
	cookie_tx = dmaengine_submit(dev->tx_desc);

	dma_async_issue_pending(dev->dma_rx);
	dma_async_issue_pending(dev->dma_tx);
}

static int bcm2835_i2s_dma_setup(struct rpi_audio_driver *rpi_dev)
{
	struct device *dev = (struct device*) rpi_dev->dev;

	rpi_dev->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!rpi_dev->dma_tx) {
		return -ENODEV;
	}

	rpi_dev->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!rpi_dev->dma_rx) {
		dma_release_channel(rpi_dev->dma_tx);
		rpi_dev->dma_tx = NULL;
		return -ENODEV;
	}

	printk(KERN_INFO "bcm2835_i2s_dma_setup: Successful.\n");
	return 0;
}

static int bcm2835_init_cv_gates(void) {
	int  i, ret;
	for ( i = 0; i < NUM_OF_CV_OUTS; i++) {
		if ((ret = gpio_request(cv_gate_out[i], "cv_out_gate")) < 0) {
			printk(KERN_ERR "audio_rtdm: failed to get cv out\n");
			return ret;
		}

		if ((ret = gpio_direction_output(cv_gate_out[i], 0)) < 0) {
			printk(KERN_ERR "audio_rtdm: failed to set gpio dir\n");
			return ret;
		}
	}
	for ( i = 0; i < NUM_OF_CV_INS; i++) {
		if ((ret = gpio_request(cv_gate_in[i], "cv_in_gate")) < 0) {
			printk(KERN_ERR "audio_rtdm: failed to get cv out\n");
			return ret;
		}

		if ((ret = gpio_direction_input(cv_gate_in[i])) < 0) {
			printk(KERN_ERR "audio_rtdm: failed to set gpio dir\n");
			return ret;
		}
	}
	return ret;
}

static void bcm2835_i2s_configure(struct rpi_audio_driver * dev)
{
	unsigned int data_length, framesync_length;
	unsigned int slots, slot_width;
	int frame_length;
	unsigned int rx_ch1_pos, rx_ch2_pos, tx_ch1_pos, tx_ch2_pos;
	unsigned int mode, format;
	bool bit_clock_master = false;
	bool frame_sync_master = false;
	bool frame_start_falling_edge = true;

	data_length = 32;
	mode = 0;
	slots = 2;
	slot_width = 32;
	frame_length = slots * slot_width;
	format = BCM2835_I2S_CHEN | BCM2835_I2S_CHWEX;
	format |= BCM2835_I2S_CHWID((data_length-8)&0xf);
	framesync_length = frame_length / 2;
	frame_start_falling_edge = false;

	rx_ch1_pos = 0;
	rx_ch2_pos = 32; /* calculated manually for now */
	tx_ch1_pos = 0;
	tx_ch2_pos = 32;
	/* CH2 format is the same as for CH1 */
	format = BCM2835_I2S_CH1(format) | BCM2835_I2S_CH2(format);

	mode |= BCM2835_I2S_FLEN(frame_length - 1);
	mode |= BCM2835_I2S_FSLEN(framesync_length);

	if (!bit_clock_master)
		mode |= BCM2835_I2S_CLKDIS | BCM2835_I2S_CLKM | BCM2835_I2S_CLKI;

	if (!frame_sync_master)
		mode |= BCM2835_I2S_FSM;

	if (frame_start_falling_edge)
		mode |= BCM2835_I2S_FSI;

	rpi_reg_write(dev->base_addr, BCM2835_I2S_MODE_A_REG, mode);

	rpi_reg_write(dev->base_addr, BCM2835_I2S_RXC_A_REG,format
		| BCM2835_I2S_CH1_POS(rx_ch1_pos)
		| BCM2835_I2S_CH2_POS(rx_ch2_pos));

	rpi_reg_write(dev->base_addr, BCM2835_I2S_TXC_A_REG,format
		|BCM2835_I2S_CH1_POS(tx_ch1_pos)
		| BCM2835_I2S_CH2_POS(tx_ch2_pos));

	dev_dbg(dev->dev, "rx pos: %d,%d tx pos: %d,%d\n",
		rx_ch1_pos, rx_ch2_pos, tx_ch1_pos, tx_ch2_pos);

	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_MODE_A_REG,
			BCM2835_I2S_CLKDIS, 0);
	/* Setup the DMA parameters */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXTHR(1)
			| BCM2835_I2S_TXTHR(1)
			| BCM2835_I2S_DMAEN, 0xffffffff);

	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_DREQ_A_REG,
			  BCM2835_I2S_TX_PANIC(BCM2835_DMA_TX_PANIC_THR)
			| BCM2835_I2S_RX_PANIC(BCM2835_DMA_RX_PANIC_THR)
			| BCM2835_I2S_TX(BCM2835_DMA_THR_TX)
			| BCM2835_I2S_RX(BCM2835_DMA_THR_RX), 0xffffffff);
}

void bcm2835_i2s_enable(struct rpi_audio_driver *dev)
{
	/* Disable RAM STBY */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_STBY, BCM2835_I2S_STBY);

	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_INTEN_A_REG,
			BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR , BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR);

	/* Enable PCM block */
	rpi_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_EN , BCM2835_I2S_EN);
}

static void bcm2835_i2s_clear_regs(struct rpi_audio_driver *dev)
{
	rpi_reg_write(dev->base_addr, BCM2835_I2S_CS_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_MODE_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_RXC_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_TXC_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_DREQ_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_INTEN_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, 0);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_GRAY_REG, 0);
}

int bcm2835_i2s_init(struct platform_device *pdev)
{
	int i;
	struct rpi_audio_driver *dev;
	int ret = 0;
	struct resource *mem_resource;
	void __iomem *base;
	const __be32 *addr;
	dma_addr_t dma_base, dummy_phys_addr;
	struct rpi_buffers_info *audio_buffer;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	rpi_device_i2s = dev;

	mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_resource);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "devm_ioremap_resource failed.");
		return PTR_ERR(base);
	}
	dev->base_addr = base;

	addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "could not get DMA-register address\n");
		return -EINVAL;
	}

	dma_base = be32_to_cpup(addr);
	dev->fifo_dma_addr = dma_base + BCM2835_I2S_FIFO_A_REG;
	dev->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dev->dma_burst_size = 2;
	dev->dev = &pdev->dev;
	rtdm_event_init(&dev->irq_event, 0);

	bcm2835_i2s_clear_regs(dev);
	bcm2835_i2s_configure(dev);
	bcm2835_init_cv_gates();

	if (bcm2835_i2s_dma_setup(dev))
		return -ENODEV;

	audio_buffer = kcalloc(1, sizeof(struct rpi_buffers_info),
		GFP_KERNEL);

	if (!audio_buffer) {
		dev_err(&pdev->dev, "couldn't allocate audio_buffer\n");
		return -ENOMEM;
	}
	dev->buffer = audio_buffer;
	audio_buffer->rx_buf = dma_zalloc_coherent(dev->dma_rx->device->dev,
				NUM_OF_PAGES * PAGE_SIZE, &dummy_phys_addr,
				GFP_KERNEL);
	if (!audio_buffer->rx_buf ) {
		dev_err(&pdev->dev, "couldn't allocate coherent_mem\n");
		return -ENOMEM;
	}
	audio_buffer->rx_phys_addr = dummy_phys_addr;
	audio_buffer->period_len = NUM_OF_WORDS * sizeof(uint32_t);
	audio_buffer->buffer_len = 2 * audio_buffer->period_len;
	audio_buffer->tx_buf = audio_buffer->rx_buf +
			audio_buffer->buffer_len;
	audio_buffer->tx_phys_addr = dummy_phys_addr + audio_buffer->buffer_len;
	audio_buffer->cv_gate_out = audio_buffer->rx_buf +
			audio_buffer->buffer_len * 2;
	audio_buffer->cv_gate_in = audio_buffer->rx_buf +
			audio_buffer->buffer_len * 2 + sizeof(uint32_t);
	*audio_buffer->cv_gate_out = 0x0f;

	ret = bcm2835_i2s_dma_prepare(dev);
	if (ret)
		dev_err(&pdev->dev,"bcm2835_i2s_dma_prepare failed!\n");

	bcm2835_i2s_enable(dev);

	bcm2835_i2s_clear_fifos(dev, true, true);

	for (i = 0; i < (BCM2835_DMA_THR_TX + DEFAULT_AUDIO_N_CHANNELS); i++)
		rpi_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG, 0);

	msleep(10);

	bcm2835_i2s_submit_dma(dev);
	msleep(500);
	bcm2835_i2s_start_stop(dev, BCM2835_I2S_START_CMD);

	return ret;
}

void bcm2835_i2s_exit(struct platform_device *pdev)
{
	struct rpi_buffers_info *audio_buffers = rpi_device_i2s->buffer;
	dmaengine_terminate_async(rpi_device_i2s->dma_tx);
	dmaengine_terminate_async(rpi_device_i2s->dma_rx);
	bcm2835_i2s_start_stop(rpi_device_i2s, BCM2835_I2S_STOP_CMD);
	devm_iounmap(&pdev->dev, (void *)rpi_device_i2s->base_addr);
	dma_free_coherent(rpi_device_i2s->dma_rx->device->dev,
				NUM_OF_PAGES * PAGE_SIZE, audio_buffers->rx_buf,
				audio_buffers->rx_phys_addr);
	dma_release_channel(rpi_device_i2s->dma_tx);
	dma_release_channel(rpi_device_i2s->dma_rx);
	kfree(audio_buffers);
	devm_kfree(&pdev->dev, (void *)rpi_device_i2s);
}
