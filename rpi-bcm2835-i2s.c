/**
 * @file rpi-rtdm-i2s.c
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief 
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

#include "rpi-bcm2835-i2s.h"

#define DEVICE_NAME "rtdm_audio"

#define DEFAULT_AUDIO_N_CHANNELS			8
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		64
#define NUM_OF_WORDS (DEFAULT_AUDIO_N_CHANNELS * \
DEFAULT_AUDIO_N_FRAMES_PER_BUFFER)
#define NUM_OF_PAGES					4

static unsigned long i2s_interrupts = 0;
module_param(i2s_interrupts, ulong, 0644);

static struct rpi_i2s_dev *rpi_device_i2s;
static rtdm_irq_t  i2s_rtdm_irq;

static void i2s_loopback_task(void *ctx)
{
	struct rpi_i2s_dev *dev = ctx;
	int i;
	static int pos = 0;
	static int *txbuf, *rxbuf;
	struct i2s_buffers_info *i2s_buffer = dev->buffer;
	txbuf = (int *) i2s_buffer->tx_buf;
	rxbuf = (int *) i2s_buffer->rx_buf;
	printk("i2s_loopback_task: started\n");
	while(!rtdm_task_should_stop()){
		rtdm_event_wait(&dev->irq_event);

		for (i = 0; i < NUM_OF_WORDS; i+=8) {
				txbuf[i + pos] = rxbuf[i + pos];
				txbuf[i + pos + 1] = rxbuf[i + pos + 1];
				txbuf[i + pos + 2] = rxbuf[i + pos + 2];
				txbuf[i + pos + 3] = rxbuf[i + pos + 3];
				txbuf[i + pos + 4] = rxbuf[i + pos + 4];
				txbuf[i + pos + 5] = rxbuf[i + pos + 5];
				txbuf[i + pos + 6] = rxbuf[i + pos + 4];
				txbuf[i + pos + 7] = rxbuf[i + pos + 5];
		}
		i2s_interrupts++;
		pos = NUM_OF_WORDS * (i2s_interrupts % 2);
	}
}

static void bcm2835_i2s_clear_fifos(struct rpi_i2s_dev *dev,
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

static int bcm2835_i2s_intr(rtdm_irq_t *irqh)
{
	struct rpi_i2s_dev *dev = rtdm_irq_get_arg(irqh, struct rpi_i2s_dev);
	uint32_t val;

	rpi_reg_read(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, &val);
	bcm2835_i2s_clear_fifos(dev, true, true);
	rpi_reg_write(dev->base_addr, BCM2835_I2S_INTSTC_A_REG,
		BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR);
	printk("bcm2835_i2s_intr: BCM2835_I2S_INTSTC_A_REG = 0x%08X\n",val);

	return RTDM_IRQ_HANDLED;
}

static void bcm2835_i2s_start_stop(struct rpi_i2s_dev *dev, int cmd)
{
	uint32_t mask, val, discarded = 0;
	int32_t tmp[9], samples[2] = {0xff, 0xff};
	wmb();
	mask = BCM2835_I2S_RXON | BCM2835_I2S_TXON;
 
	if (cmd == RPI_I2S_START_CMD) {
		rpi_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, mask);

		while (samples[0] != 0 || samples[1] != 0) {
			rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG,
					 &val);
			if (val & BCM2835_I2S_RXD) {
				rpi_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG,
					 0x00);
					 
				samples[1] = samples[0];
				rpi_reg_read(dev->base_addr, BCM2835_I2S_FIFO_A_REG,
					 &samples[0]);
					 tmp[discarded] = samples[0];
					 discarded++;
			}
		}
		printk("bcm2835_i2s_start_stop: %d samples discarded\n",discarded);
		for ( val = 0; val < discarded; val++)
			printk("%x\n",tmp[val]);

	}
	else {
		rpi_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, 0);
	}
}

static void bcm2835_i2s_dma_callback(void *data)
{
	struct rpi_i2s_dev *dev = data;
	
	//printk("bcm2835_i2s_dma_callback: BCM2835_I2S_CS_A_REG = 0x%08X\n",val);

	rtdm_event_signal(&dev->irq_event);
}

static struct dma_async_tx_descriptor *
bcm2835_i2s_dma_prepare_cyclic(struct rpi_i2s_dev *dev,
			enum dma_transfer_direction dir)
{
	struct dma_slave_config cfg;
	struct dma_chan *chan;
	int  flags;
	struct dma_async_tx_descriptor * desc;
	struct i2s_buffers_info *i2s_buffer = dev->buffer;

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
		desc = dmaengine_prep_dma_cyclic(chan, i2s_buffer->tx_phys_addr, i2s_buffer->buffer_len, i2s_buffer->period_len, dir,
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
		desc = dmaengine_prep_dma_cyclic(chan, i2s_buffer->rx_phys_addr, i2s_buffer->buffer_len, i2s_buffer->period_len, dir,
				       flags);
	}
	return desc;
}

static int bcm2835_i2s_dma_prepare(struct rpi_i2s_dev *dev)
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

static void bcm2835_i2s_start_dma(struct rpi_i2s_dev *dev)
{
	dmaengine_submit(dev->rx_desc);
	dmaengine_submit(dev->tx_desc);

	dma_async_issue_pending(dev->dma_rx);
	dma_async_issue_pending(dev->dma_tx);
}

static int bcm2835_i2s_dma_setup(struct rpi_i2s_dev *rpi_dev)
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

	printk("bcm2835_i2s_dma_setup: Successful.\n");
	return 0;
}

static void bcm2835_i2s_configure(struct rpi_i2s_dev * dev)
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
			  BCM2835_I2S_TX_PANIC(8)
			| BCM2835_I2S_RX_PANIC(32)
			| BCM2835_I2S_TX(16)
			| BCM2835_I2S_RX(16), 0xffffffff);
}

static void bcm2835_i2s_enable(struct rpi_i2s_dev *dev)
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

static void bcm2835_i2s_clear_regs(struct rpi_i2s_dev *dev)
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
	struct rpi_i2s_dev *dev;
	int i;
	int ret = 0;
	struct resource *mem_resource;
	void __iomem *base;
	const __be32 *addr;
	dma_addr_t dma_base, dummy_phys_addr;
	uint32_t val;
	struct i2s_buffers_info *i2s_buffer;
	int *tmp;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->irq = RPI_I2S_IRQ_NUM;
	rpi_device_i2s = dev;

	ret = rtdm_irq_request(&i2s_rtdm_irq, dev->irq, bcm2835_i2s_intr,
			       0, "rt-i2s", dev);
	if(ret) {
		printk("bcm2835_i2s_init: irq request failed !\n");
		return -1;
	}

	mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_resource);
	if (IS_ERR(base)) {
		printk("bcm2835_i2s_init: devm_ioremap_resource failed.");
		return PTR_ERR(base);
	}
	dev->base_addr = base;

	addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "bcm2835_i2s_init: could not get DMA-register address\n");
		return -EINVAL;
	}

	dma_base = be32_to_cpup(addr);
	dev->fifo_dma_addr = dma_base + BCM2835_I2S_FIFO_A_REG;
	dev->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dev->dma_burst_size = 2;
	dev->dev = &pdev->dev;
	rtdm_event_init(&dev->irq_event, 0);
	
	dev->i2s_task = kcalloc(1,sizeof(rtdm_task_t), GFP_KERNEL);

	bcm2835_i2s_clear_regs(dev);
	bcm2835_i2s_configure(dev);

	if (bcm2835_i2s_dma_setup(dev))
		return -ENODEV;

	i2s_buffer = kcalloc(1, sizeof(struct i2s_buffers_info),
		GFP_KERNEL);

	if (!i2s_buffer) {
		printk("bcm2835_i2s_init: couldn't allocate i2s_buffer\n");
		return -ENOMEM;
	}
	dev->buffer = i2s_buffer;
	i2s_buffer->rx_buf = dma_zalloc_coherent(dev->dma_rx->device->dev,
				NUM_OF_PAGES * PAGE_SIZE, &dummy_phys_addr,
				GFP_KERNEL);
	if (!i2s_buffer->rx_buf ) {
		printk("bcm2835_i2s_init: couldn't allocate coherent_mem\n");
		return -ENOMEM;
	}
	i2s_buffer->rx_phys_addr = dummy_phys_addr;
	i2s_buffer->period_len = NUM_OF_WORDS * sizeof(uint32_t);
	i2s_buffer->buffer_len = 2 * i2s_buffer->period_len;
	i2s_buffer->tx_buf = i2s_buffer->rx_buf +
			i2s_buffer->buffer_len;
	i2s_buffer->tx_phys_addr = dummy_phys_addr + i2s_buffer->buffer_len;

	ret = bcm2835_i2s_dma_prepare(dev);
	if (ret)
		printk("bcm2835_i2s_dma_prepare failed!\n");

	bcm2835_i2s_enable(dev);
	/* Clear the fifos */
	bcm2835_i2s_clear_fifos(dev, true, true);

	tmp = (int *) i2s_buffer->tx_buf;

	for (i = 0; i < NUM_OF_WORDS * 2; i++) {
		tmp[i] = 0;
	}
	
	for (i =0; i < 24; i++)
		rpi_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG, 0);

	msleep(10);
	/* print all the registers just for debug*/
	rpi_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_CS_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_MODE_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_MODE_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_RXC_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_RXC_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_TXC_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_TXC_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_INTEN_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_INTEN_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_INTSTC_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_DREQ_A_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_DREQ_A_REG = 0x%08X\n",val);
	rpi_reg_read(dev->base_addr, BCM2835_I2S_GRAY_REG, &val);
	printk("bcm2835_i2s_init: BCM2835_I2S_GRAY_REG = 0x%08X\n",val);

	bcm2835_i2s_start_dma(dev);

	msleep(500);
	ret = rtdm_task_init(dev->i2s_task,
					 "i2s_driver_task",
					i2s_loopback_task, dev,
					95, 0);
	if (ret) {
		printk("i2s_init: rtdm_task_init failed\n");
		return -EINVAL;
	}
	msleep(1000);
	printk("i2s_init: starting transfers!\n");
	bcm2835_i2s_start_stop(dev, RPI_I2S_START_CMD);
	
	return ret;
}

void bcm2835_i2s_exit(struct platform_device *pdev)
{
	struct i2s_buffers_info *i2s_buffer = rpi_device_i2s->buffer;
	rtdm_irq_free(&i2s_rtdm_irq);
	devm_iounmap(&pdev->dev, (void *)rpi_device_i2s->base_addr);
	dma_free_coherent(rpi_device_i2s->dma_rx->device->dev,
				4 * PAGE_SIZE, i2s_buffer->rx_buf,
				i2s_buffer->rx_phys_addr);
	dma_release_channel(rpi_device_i2s->dma_tx);
	dma_release_channel(rpi_device_i2s->dma_rx);
	kfree(i2s_buffer);
	devm_kfree(&pdev->dev, (void *)rpi_device_i2s);
}