// SPDX-License-Identifier: GPL-2.0
/*
 * @brief I2S module of the RTDM audio driver.
 *	  A lot of stuff is based on the mainline I2S module by Florian Meier
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <rtdm/driver.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/clk.h>

#include "pcm3168a-elk.h"
#include "bcm2835-i2s-elk.h"
#include "elk-pi-config.h"

#define BCM2835_PCM_WORD_LEN 	32
#define BCM2835_PCM_SLOTS	2

static struct audio_rtdm_dev *audio_static_dev;

#ifdef BCM2835_I2S_CVGATES_SUPPORT
static int cv_gate_out[NUM_OF_CVGATE_OUTS] = { CVGATE_OUTS_LIST };
static int cv_gate_in[NUM_OF_CVGATE_INS] = { CVGATE_INS_LIST };
#endif

void bcm2835_i2s_clear_fifos(struct audio_rtdm_dev *audio_dev,
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
	rpi_reg_read(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, &csreg);
	i2s_active_state = csreg & (BCM2835_I2S_RXON | BCM2835_I2S_TXON);

	/* Stop I2S module */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, off, 0);

	/*
	 * Clear the FIFOs
	 * Requires at least 2 PCM clock cycles to take effect
	 */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, clr, clr);

	rpi_reg_read(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, &sync);
	sync &= BCM2835_I2S_SYNC;

	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_SYNC, ~sync);

	/* Wait for the SYNC flag changing it's state */
	while (--timeout) {
		rpi_reg_read(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, &csreg);
		if ((csreg & BCM2835_I2S_SYNC) != sync)
			break;
	}

	/* Restore I2S state */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXON | BCM2835_I2S_TXON, i2s_active_state);
}

static void bcm2835_i2s_synch_frame(struct audio_rtdm_dev *audio_dev,
					uint32_t mask)
{
	uint32_t val, discarded = 0;
	int32_t tmp[9], samples[2] = {0xff, 0xff};

	rpi_reg_update_bits(audio_dev->i2s_base_addr,
		BCM2835_I2S_CS_A_REG, mask, mask);
	/* Make sure channels are aligned in right order.
	Last two channels from pcm3168 are always zero &
	the probability of getting two successive zero values is nearly impossible */
	while (samples[0] != 0 || samples[1] != 0) {
		rpi_reg_read(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
					&val);
		if (val & BCM2835_I2S_RXD) {
			rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_FIFO_A_REG,
					0x00);
			samples[1] = samples[0];
			rpi_reg_read(audio_dev->i2s_base_addr, BCM2835_I2S_FIFO_A_REG,
					&samples[0]);
					tmp[discarded] = samples[0];
					discarded++;
		}
	}
	printk(KERN_INFO "bcm2835-i2s: %d samples discarded\n",
							discarded);
}

void bcm2835_i2s_start_stop(struct audio_rtdm_dev *audio_dev, int cmd)
{
	uint32_t mask;
	wmb();
	mask = BCM2835_I2S_RXON | BCM2835_I2S_TXON;

	if (cmd == BCM2835_I2S_START_CMD) {
		if (!strcmp(audio_dev->audio_hat, "elk-pi")) {
			bcm2835_i2s_synch_frame(audio_dev, mask);
		} else {
			rpi_reg_update_bits(audio_dev->i2s_base_addr,
				BCM2835_I2S_CS_A_REG, mask, mask);
		}
	} else {
		rpi_reg_update_bits(audio_dev->i2s_base_addr,
			BCM2835_I2S_CS_A_REG, mask, 0);
	}
}

static void bcm2835_i2s_dma_callback(void *data)
{
	int i;
	uint32_t val;
	struct audio_rtdm_dev *audio_dev = data;

	audio_dev->kinterrupts++;
	audio_dev->buffer_idx = ~(audio_dev->buffer_idx) & 0x1;
	if (audio_dev->wait_flag) {
		rtdm_event_signal(&audio_dev->irq_event);
#ifdef BCM2835_I2S_CVGATES_SUPPORT
		if (audio_dev->cv_gate_enabled) {
			for (i = 0; i < NUM_OF_CVGATE_OUTS; i++) {
				val = (unsigned long) *audio_dev->buffer->cv_gate_out &
				BIT(i);
				gpio_set_value(cv_gate_out[i], val);
			}
			val = 0;
			for (i = 0; i < NUM_OF_CVGATE_INS; i++) {
			val |= gpio_get_value(cv_gate_in[i]) << i;
			}
			*audio_dev->buffer->cv_gate_in = val;
		}
#endif
	}
}

static struct dma_async_tx_descriptor *
bcm2835_i2s_dma_prepare_cyclic(struct audio_rtdm_dev *audio_dev,
			enum dma_transfer_direction dir)
{
	struct dma_slave_config cfg;
	struct dma_chan *chan;
	int  flags;
	struct dma_async_tx_descriptor *desc;
	struct audio_rtdm_buffers *audio_buffers = audio_dev->buffer;

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = dir;

	if (dir == DMA_MEM_TO_DEV) {
		cfg.dst_addr = audio_dev->fifo_dma_addr;
		cfg.dst_addr_width = audio_dev->addr_width;
		cfg.dst_maxburst = audio_dev->dma_burst_size;
		chan = audio_dev->dma_tx;
		flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(audio_dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan,
		audio_buffers->tx_phys_addr, audio_buffers->buffer_len,
		audio_buffers->period_len, dir, flags);
	} else if (dir == DMA_DEV_TO_MEM) {
		cfg.src_addr = audio_dev->fifo_dma_addr;
		cfg.src_addr_width = audio_dev->addr_width;
		cfg.src_maxburst = audio_dev->dma_burst_size;
		chan = audio_dev->dma_rx;
		flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(audio_dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan,
		audio_buffers->rx_phys_addr, audio_buffers->buffer_len,
		audio_buffers->period_len, dir, flags);
	} else {
		printk(KERN_ERR "bcm2835-i2s: unsupported dma direction\n");
		return NULL;
	}
	return desc;
}

static int bcm2835_i2s_dma_prepare(struct audio_rtdm_dev *audio_dev)
{
	int err;
	audio_dev->tx_desc = bcm2835_i2s_dma_prepare_cyclic(audio_dev, DMA_MEM_TO_DEV);
	if (!audio_dev->tx_desc) {
		dev_err(audio_dev->dev,
			"failed to get DMA TX descriptor\n");
		err = -EBUSY;
		return err;
	}

	audio_dev->rx_desc = bcm2835_i2s_dma_prepare_cyclic(audio_dev, DMA_DEV_TO_MEM);
	if (!audio_dev->rx_desc) {
		dev_err(audio_dev->dev,
			"failed to get DMA RX descriptor\n");
		err = -EBUSY;
		dmaengine_terminate_async(audio_dev->dma_tx);
		return err;
	}
	audio_dev->rx_desc->callback = bcm2835_i2s_dma_callback;
	audio_dev->rx_desc->callback_param = audio_dev;
	return 0;
}

static void bcm2835_i2s_submit_dma(struct audio_rtdm_dev *audio_dev)
{
	dmaengine_submit(audio_dev->rx_desc);
	dmaengine_submit(audio_dev->tx_desc);

	dma_async_issue_pending(audio_dev->dma_rx);
	dma_async_issue_pending(audio_dev->dma_tx);
}

static int bcm2835_i2s_dma_setup(struct audio_rtdm_dev *audio_dev)
{
	struct device *dev = (struct device *) audio_dev->dev;

	audio_dev->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!audio_dev->dma_tx) {
		return -ENODEV;
	}

	audio_dev->dma_tx->private = "rtdm-tx-irq";

	/* Note: This dmaengine_resume is a way to enter the dma backend
	and get rtdm irqs. The above initialized string is used as an
	identifier to recognize which channels need to be real-time safe */

	dmaengine_resume(audio_dev->dma_tx);
	audio_dev->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!audio_dev->dma_rx) {
		dma_release_channel(audio_dev->dma_tx);
		audio_dev->dma_tx = NULL;
		return -ENODEV;
	}
	audio_dev->dma_rx->private = "rtdm-rx-irq";
	dmaengine_resume(audio_dev->dma_rx);
	printk(KERN_INFO "bcm2835-i2s: dma setup successful.\n");
	return 0;
}

#ifdef BCM2835_I2S_CVGATES_SUPPORT
static int bcm2835_init_cv_gates(void)
{
	int  i, ret;
	for (i = 0; i < NUM_OF_CVGATE_OUTS; i++) {
		ret = gpio_request(cv_gate_out[i], "cv_out_gate");
		if (ret < 0) {
			printk(KERN_ERR "bcm2835-i2s: failed to get cv out\n");
			return ret;
		}
		ret = gpio_direction_output(cv_gate_out[i], 0);
		if (ret < 0) {
			printk(KERN_ERR "bcm2835-i2s: failed to set gpio dir\n");
			return ret;
		}
	}
	for (i = 0; i < NUM_OF_CVGATE_INS; i++) {
		ret = gpio_request(cv_gate_in[i], "cv_in_gate");
		if (ret < 0) {
			printk(KERN_ERR "bcm2835-i2s: failed to get cv out\n");
			return ret;
		}
		ret = gpio_direction_input(cv_gate_in[i]);
		if (ret < 0) {
			printk(KERN_ERR "bcm2835-i2s: failed to set gpio dir\n");
			return ret;
		}
	}
	return ret;
}

static void bcm2835_free_cv_gates(void)
{
	int i;
	for (i = 0; i < NUM_OF_CVGATE_OUTS; i++)
		gpio_free(cv_gate_out[i]);

	for (i = 0; i < NUM_OF_CVGATE_INS; i++)
		gpio_free(cv_gate_in[i]);
}
#endif

static void bcm2835_i2s_configure(struct audio_rtdm_dev *audio_dev)
{
	unsigned int data_length, framesync_length;
	unsigned int slots, slot_width;
	int frame_length, bclk_rate;
	unsigned int ch1_pos, ch2_pos;
	unsigned int mode = 0, format = 0;
	bool bit_clock_master = false;
	bool frame_sync_master = false;
	bool frame_start_falling_edge = true;

	data_length = BCM2835_PCM_WORD_LEN;
	slots = BCM2835_PCM_SLOTS;
	slot_width = BCM2835_PCM_WORD_LEN;
	frame_length = slots * slot_width;
	format = BCM2835_I2S_CHEN | BCM2835_I2S_CHWEX;
	format |= BCM2835_I2S_CHWID((data_length-8)&0xf);
	framesync_length = frame_length / 2;
	frame_start_falling_edge = false;
	if (!strcmp(audio_dev->audio_hat, "hifi-berry")) {
		bit_clock_master = true;
		frame_sync_master = true;
		bclk_rate = frame_length * DEFAULT_AUDIO_SAMPLING_RATE;
		if (clk_set_rate(audio_dev->clk, bclk_rate))
			printk(KERN_ERR "bcm2835_i2s_configure: clk_set_rate failed\n");

		audio_dev->clk_rate = bclk_rate;
		mode = BCM2835_I2S_CLKI;
		ch1_pos = 1;
		ch2_pos = 33;
		clk_prepare_enable(audio_dev->clk);
	} else if (!strcmp(audio_dev->audio_hat, "hifi-berry-pro")) {
		ch1_pos = 1;
		ch2_pos = 33;
	} else {
		ch1_pos = 0;
		ch2_pos = 32; /* calculated manually for now */
	}
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

	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_MODE_A_REG, mode);

	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_RXC_A_REG, format
		| BCM2835_I2S_CH1_POS(ch1_pos)
		| BCM2835_I2S_CH2_POS(ch2_pos));

	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_TXC_A_REG, format
		|BCM2835_I2S_CH1_POS(ch1_pos)
		| BCM2835_I2S_CH2_POS(ch2_pos));

	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_MODE_A_REG,
			BCM2835_I2S_CLKDIS, 0);
	/* Setup the DMA parameters */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXTHR(1)
			| BCM2835_I2S_TXTHR(1)
			| BCM2835_I2S_DMAEN, 0xffffffff);

	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_DREQ_A_REG,
			  BCM2835_I2S_TX_PANIC(BCM2835_DMA_TX_PANIC_THR)
			| BCM2835_I2S_RX_PANIC(BCM2835_DMA_RX_PANIC_THR)
			| BCM2835_I2S_TX(BCM2835_DMA_THR_TX)
			| BCM2835_I2S_RX(BCM2835_DMA_THR_RX), 0xffffffff);
}

static void bcm2835_i2s_enable(struct audio_rtdm_dev *audio_dev)
{
	/* Disable RAM STBY */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_STBY, BCM2835_I2S_STBY);

	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_INTEN_A_REG,
			BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR,
			BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR);

	/* Enable PCM block */
	rpi_reg_update_bits(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_EN, BCM2835_I2S_EN);
}

static void bcm2835_i2s_clear_regs(struct audio_rtdm_dev *audio_dev)
{
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_CS_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_MODE_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_RXC_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_TXC_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_DREQ_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_INTEN_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_INTSTC_A_REG, 0);
	rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_GRAY_REG, 0);
}

int bcm2835_i2s_init(int audio_buffer_size, int audio_channels,
			char *audio_hat)
{
	int ret, i;
	dma_addr_t dummy_phys_addr;
	struct audio_rtdm_dev *audio_dev = audio_static_dev;
	struct audio_rtdm_buffers *audio_buffer = audio_dev->buffer;
	audio_dev->audio_hat = audio_hat;

	printk(KERN_INFO "Elk hat: %s\n", audio_dev->audio_hat);

	audio_buffer->rx_buf = dma_zalloc_coherent(audio_dev->dma_rx->device->dev,
	RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
	&dummy_phys_addr,
	GFP_KERNEL);
	if (!audio_buffer->rx_buf) {
		printk(KERN_ERR "bcm2835-i2s: couldn't allocate dma mem\n");
		return -ENOMEM;
	}
	audio_buffer->rx_phys_addr = dummy_phys_addr;
	audio_buffer->period_len = audio_buffer_size * audio_channels
					 * sizeof(uint32_t);
	audio_buffer->buffer_len = 2 * audio_buffer->period_len;
	audio_buffer->tx_buf = audio_buffer->rx_buf +
			audio_buffer->buffer_len;
	audio_buffer->tx_phys_addr = dummy_phys_addr + audio_buffer->buffer_len;
	audio_buffer->cv_gate_out = audio_buffer->rx_buf +
			audio_buffer->buffer_len * 2;
	audio_buffer->cv_gate_in = audio_buffer->rx_buf +
			audio_buffer->buffer_len * 2 + sizeof(uint32_t);
	*audio_buffer->cv_gate_out = 0x0f;

	ret = bcm2835_i2s_dma_prepare(audio_dev);
	if (ret) {
		printk(KERN_ERR "bcm2835-i2s: dma_prepare failed\n");
		return -EINVAL;
	}

	if (!strcmp(audio_dev->audio_hat, "elk-pi")) {
		audio_dev->cv_gate_enabled = true;
		bcm2835_init_cv_gates();
	}
	bcm2835_i2s_clear_regs(audio_dev);
	bcm2835_i2s_configure(audio_dev);
	bcm2835_i2s_enable(audio_dev);

	bcm2835_i2s_clear_fifos(audio_dev, true, true);

	for (i = 0; i < (BCM2835_DMA_THR_TX + audio_channels); i++)
		rpi_reg_write(audio_dev->i2s_base_addr, BCM2835_I2S_FIFO_A_REG, 0);

	bcm2835_i2s_submit_dma(audio_dev);
	msleep(10);
	bcm2835_i2s_start_stop(audio_dev, BCM2835_I2S_START_CMD);

	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_i2s_init);

struct audio_rtdm_dev *bcm2835_get_i2s_dev(void)
{
	return audio_static_dev;
}
EXPORT_SYMBOL_GPL(bcm2835_get_i2s_dev);

int bcm2835_i2s_exit(void)
{
	int ret = 0;
	struct audio_rtdm_dev *audio_dev = audio_static_dev;
	ret = dmaengine_terminate_async(audio_dev->dma_tx);
	if (ret < 0) {
		printk(KERN_ERR "bcm2835-i2s: dmaengine_terminate_async \
		 failed\n");
		return ret;
	}
	ret = dmaengine_terminate_async(audio_dev->dma_rx);
	if (ret < 0) {
		printk(KERN_ERR "bcm2835-i2s: dmaengine_terminate_async \
		 failed\n");
		return ret;
	}
	dma_free_coherent(audio_dev->dma_rx->device->dev,
				RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
				audio_dev->buffer->rx_buf,
				audio_dev->buffer->rx_phys_addr);
	dma_release_channel(audio_dev->dma_tx);
	dma_release_channel(audio_dev->dma_rx);
	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_i2s_exit);

int bcm2835_i2s_probe(struct platform_device *pdev)
{
	struct audio_rtdm_dev *audio_dev;
	int ret = 0;
	struct resource *mem_resource;
	void __iomem *base;
	const __be32 *addr;
	dma_addr_t dma_base;
	struct audio_rtdm_buffers *audio_buffer;

	audio_dev = devm_kzalloc(&pdev->dev, sizeof(*audio_dev),
			   GFP_KERNEL);
	if (!audio_dev)
		return -ENOMEM;

	audio_dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(audio_dev->clk)) {
		dev_err(&pdev->dev, "could not get clk: %ld\n",
			PTR_ERR(audio_dev->clk));
		return PTR_ERR(audio_dev->clk);
	}

	audio_static_dev = audio_dev;

	mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_resource);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "devm_ioremap_resource failed.");
		return PTR_ERR(base);
	}
	audio_dev->i2s_base_addr = base;

	addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "could not get DMA-register address\n");
		return -EINVAL;
	}

	dma_base = be32_to_cpup(addr);
	audio_dev->fifo_dma_addr = dma_base + BCM2835_I2S_FIFO_A_REG;
	audio_dev->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	audio_dev->dma_burst_size = 2;
	audio_dev->dev = &pdev->dev;
	rtdm_event_init(&audio_dev->irq_event, 0);

	if (bcm2835_i2s_dma_setup(audio_dev))
		return -ENODEV;

	audio_buffer = kcalloc(1, sizeof(struct audio_rtdm_buffers),
		GFP_KERNEL);

	if (!audio_buffer) {
		dev_err(&pdev->dev, "couldn't allocate audio_buffer\n");
		return -ENOMEM;
	}
	audio_dev->buffer = audio_buffer;
	return ret;
}

static int bcm2835_i2s_remove(struct platform_device *pdev)
{
	struct audio_rtdm_buffers *audio_buffers =
					audio_static_dev->buffer;

	bcm2835_i2s_start_stop(audio_static_dev, BCM2835_I2S_STOP_CMD);
	kfree(audio_buffers);

#ifdef BCM2835_I2S_CVGATES_SUPPORT
	if (audio_static_dev->cv_gate_enabled)
		bcm2835_free_cv_gates();
#endif
	devm_iounmap(&pdev->dev, (void *)audio_static_dev->i2s_base_addr);
	devm_kfree(&pdev->dev, (void *)audio_static_dev);
	return 0;
}

static const struct of_device_id bcm2835_i2s_of_match[] = {
	{ .compatible = "brcm,bcm2835-i2s", },
	{},
};

MODULE_DEVICE_TABLE(of, bcm2835_i2s_of_match);

static struct platform_driver bcm2835_i2s_driver = {
	.probe		= bcm2835_i2s_probe,
	.remove		= bcm2835_i2s_remove,
	.driver		= {
		.name	= "bcm2835-i2s",
		.of_match_table = bcm2835_i2s_of_match,
	},
};

module_platform_driver(bcm2835_i2s_driver);
MODULE_DESCRIPTION("BCM2835 I2S interface for ELK Pi");
MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_LICENSE("GPL");