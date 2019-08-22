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

#include "rpi-rtdm-i2s.h"

#define RPI_I2S_STOP_CMD 0
#define RPI_I2S_START_CMD 1

#define DEVICE_NAME "rtdm_audio"

#define DEFAULT_AUDIO_N_CHANNELS			2
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		32
#define NUM_OF_WORDS (DEFAULT_AUDIO_N_CHANNELS * \
DEFAULT_AUDIO_N_FRAMES_PER_BUFFER)

static unsigned long i2s_interrupts = 0;
module_param(i2s_interrupts, ulong, 0644);

static unsigned long wakeup_period_max = 0;
module_param(wakeup_period_max, ulong, 0644);

static unsigned long wakeup_period_min = 5000000000;
module_param(wakeup_period_min, ulong, 0644);

static unsigned long stop_i2s = 60;
module_param(stop_i2s, ulong, 0644);

static struct rpi_i2s_dev *rpi_device_i2s;
static struct i2s_transfer *drv_i2s_transfer;
static dma_addr_t dummy_phys_addr;
static rtdm_irq_t  i2s_rtdm_irq;

static void i2s_loopback_task(void *ctx)
{
	struct rpi_i2s_dev *dev = ctx;
	int i,j;
	static int pos = 0;
	static int *txbuf, *rxbuf;
	static uint64_t  diff_ns, current_wakeup_ns, prev_wakeup_ns = 0;
	txbuf = (int *) drv_i2s_transfer->tx_buf;
	rxbuf = (int *) drv_i2s_transfer->rx_buf;
	printk("i2s_loopback_task: started\n");
	while(!rtdm_task_should_stop()){
		rtdm_event_wait(&dev->irq_event);
		current_wakeup_ns = rtdm_clock_read_monotonic();

		for (i = 0; i < NUM_OF_WORDS; i++) {
				txbuf[i + pos] = rxbuf[i + pos];
				/* txbuf[i + pos + 1] = rxbuf[i + pos + 1];
				txbuf[i + pos + 2] = rxbuf[i + pos + 2];
				txbuf[i + pos + 3] = rxbuf[i + pos + 3];
				txbuf[i + pos + 4] = rxbuf[i + pos + 4];
				txbuf[i + pos + 5] = rxbuf[i + pos + 5];
				txbuf[i + pos + 6] = rxbuf[i + pos + 0];
				txbuf[i + pos + 7] = rxbuf[i + pos + 1]; */

			/* if(!prev_wakeup_ns) {

				raw_printk(" 0x%08X\n",rxbuf[i  + pos]);
			} */
		}

		if(prev_wakeup_ns) {
			diff_ns = current_wakeup_ns - prev_wakeup_ns;
			wakeup_period_max = max(diff_ns, wakeup_period_max);
			wakeup_period_min = min(diff_ns, wakeup_period_min);
		}	
		prev_wakeup_ns = current_wakeup_ns;
		i2s_interrupts++;
		pos = NUM_OF_WORDS * (i2s_interrupts % 2);
	}
}

static void rpi_rtdm_clear_fifos(struct rpi_i2s_dev *dev,
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
	i2s_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &csreg);
	i2s_active_state = csreg & (BCM2835_I2S_RXON | BCM2835_I2S_TXON);

	/* Stop I2S module */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG, off, 0);

	/*
	 * Clear the FIFOs
	 * Requires at least 2 PCM clock cycles to take effect
	 */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG, clr, clr);

	i2s_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &sync);
	sync &= BCM2835_I2S_SYNC;

	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_SYNC, ~sync);

	/* Wait for the SYNC flag changing it's state */
	while (--timeout) {
		i2s_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &csreg);
		if ((csreg & BCM2835_I2S_SYNC) != sync)
			break;
	}

	/* Restore I2S state */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXON | BCM2835_I2S_TXON, i2s_active_state);
}

static int rtdm_i2s_intr(rtdm_irq_t *irqh)
{
	struct rpi_i2s_dev *dev = rtdm_irq_get_arg(irqh, struct rpi_i2s_dev);
	uint32_t val;

	i2s_reg_read(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, &val);
	rpi_rtdm_clear_fifos(dev, true, true);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_INTSTC_A_REG,
		BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR);
	printk("bcm2835_i2s_intr: BCM2835_I2S_INTSTC_A_REG = 0x%08X\n",val);

	return RTDM_IRQ_HANDLED;
}

static int rpi_i2s_setup_clock(struct rpi_i2s_dev *dev)
{
	int ret;
	unsigned long bclk_rate = (48000 * 64);

	ret = clk_set_rate(dev->clk, bclk_rate);
	if (ret)
		return ret;
	dev->clk_rate = bclk_rate;
	clk_prepare_enable(dev->clk);
	dev->clk_prepared = true;
	return 0;
}

static void rpi_rtdm_i2s_start_stop(struct rpi_i2s_dev *dev, int cmd)
{
	uint32_t mask;
	wmb();
	mask = BCM2835_I2S_RXON | BCM2835_I2S_TXON;

	if (cmd == RPI_I2S_START_CMD) {
		i2s_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, mask);
	}
	else {
		i2s_reg_update_bits(dev->base_addr,
			BCM2835_I2S_CS_A_REG, mask, 0);
	}
}

static void rpi_rtdm_dma_callback(void *data)
{
	uint32_t val;
	struct rpi_i2s_dev *dev = data;
	
	//printk("rpi_rtdm_dma_callback: BCM2835_I2S_CS_A_REG = 0x%08X\n",val);


	rtdm_event_signal(&dev->irq_event);
}

static struct dma_async_tx_descriptor *
rpi_rtdm_dma_prepare_cyclic(struct rpi_i2s_dev *dev,
			enum dma_transfer_direction dir, struct i2s_transfer *transfer)
{
	struct dma_slave_config cfg;
	struct dma_chan *chan;
	int ret, flags;
	struct dma_async_tx_descriptor * desc;

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = dir;

	if (dir == DMA_MEM_TO_DEV) {
		cfg.dst_addr = dev->fifo_dma_addr;
		cfg.dst_addr_width = dev->addr_width;
		cfg.dst_maxburst = dev->dma_burst_size;
		chan = dev->dma_tx;
		flags = /* DMA_CTRL_REUSE | */ DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan, transfer->tx_phys_addr, transfer->buffer_len, transfer->period_len, dir,
				       flags);
	} else {
		cfg.src_addr = dev->fifo_dma_addr;
		cfg.src_addr_width = dev->addr_width;
		cfg.src_maxburst = dev->dma_burst_size;
		chan = dev->dma_rx;
		flags = /* DMA_CTRL_REUSE | */ DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

		if (dmaengine_slave_config(chan, &cfg)) {
			dev_warn(dev->dev, "DMA slave config failed\n");
			return NULL;
		}
		desc = dmaengine_prep_dma_cyclic(chan, transfer->rx_phys_addr, transfer->buffer_len, transfer->period_len, dir,
				       flags);
	}
	return desc;
}

static int rpi_rtdm_dma_prepare(struct rpi_i2s_dev *dev,
		struct i2s_transfer *transfer)
{
	int err;

	transfer->tx_desc = rpi_rtdm_dma_prepare_cyclic(dev, DMA_MEM_TO_DEV,
	transfer);
	if (!transfer->tx_desc) {
		dev_err(dev->dev,
			"failed to get DMA TX descriptor\n");
		err = -EBUSY;
		return err;
	}

	transfer->rx_desc = rpi_rtdm_dma_prepare_cyclic(dev, DMA_DEV_TO_MEM,
	transfer);
	if (!transfer->rx_desc) {
		dev_err(dev->dev,
			"failed to get DMA RX descriptor\n");
		err = -EBUSY;
		dmaengine_terminate_async(dev->dma_tx);
		return err;
	}
	transfer->rx_desc->callback = rpi_rtdm_dma_callback;
	transfer->rx_desc->callback_param = dev;
	return 0;
}

static void rpi_rtdm_start_dma(struct rpi_i2s_dev *dev,
				struct i2s_transfer *transfer)
{
	dmaengine_submit(transfer->rx_desc);
	dmaengine_submit(transfer->tx_desc);

	dma_async_issue_pending(dev->dma_rx);
	dma_async_issue_pending(dev->dma_tx);
}

static int rpi_rtdm_dma_setup(struct rpi_i2s_dev *rpi_dev)
{
	struct device *dev = (struct device*) rpi_dev->dev;

	rpi_dev->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!rpi_dev->dma_tx) {
		printk("rpi_rtdm_dma_setup: request for dma_tx chan failed\n");
		return -ENODEV;
	}

	rpi_dev->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!rpi_dev->dma_rx) {
		printk("rpi_rtdm_dma_setup: request for dma_rx chan failed\n");
		dma_release_channel(rpi_dev->dma_tx);
		rpi_dev->dma_tx = NULL;
		return -ENODEV;
	}

	printk("rpi_rtdm_dma_setup: Successful.\n");
	return 0;
}

static void rpi_rtdm_configure_i2s(struct rpi_i2s_dev * dev)
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

	i2s_reg_write(dev->base_addr, BCM2835_I2S_MODE_A_REG, mode);

	i2s_reg_write(dev->base_addr, BCM2835_I2S_RXC_A_REG,format
		| BCM2835_I2S_CH1_POS(rx_ch1_pos)
		| BCM2835_I2S_CH2_POS(rx_ch2_pos));

	i2s_reg_write(dev->base_addr, BCM2835_I2S_TXC_A_REG,format
		|BCM2835_I2S_CH1_POS(tx_ch1_pos)
		| BCM2835_I2S_CH2_POS(tx_ch2_pos));

	dev_dbg(dev->dev, "rx pos: %d,%d tx pos: %d,%d\n",
		rx_ch1_pos, rx_ch2_pos, tx_ch1_pos, tx_ch2_pos);

	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_MODE_A_REG,
			BCM2835_I2S_CLKDIS, 0);
	/* Setup the DMA parameters */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXTHR(1)
			| BCM2835_I2S_TXTHR(1)
			| BCM2835_I2S_DMAEN, 0xffffffff);

	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_DREQ_A_REG,
			  BCM2835_I2S_TX_PANIC(8)
			| BCM2835_I2S_RX_PANIC(48)
			| BCM2835_I2S_TX(16)
			| BCM2835_I2S_RX(16), 0xffffffff);
}

static void rpi_rtdm_i2s_enable(struct rpi_i2s_dev *dev)
{
	/* Disable RAM STBY */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_STBY, BCM2835_I2S_STBY);

	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_INTEN_A_REG,
			BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR , BCM2835_I2S_INT_TXERR | BCM2835_I2S_INT_RXERR);

	/* Enable PCM block */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_EN , BCM2835_I2S_EN);
}

static void rpi_rtdm_clear_regs(struct rpi_i2s_dev *dev)
{
	i2s_reg_write(dev->base_addr, BCM2835_I2S_CS_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_MODE_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_RXC_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_TXC_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_DREQ_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_INTEN_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, 0);
	i2s_reg_write(dev->base_addr, BCM2835_I2S_GRAY_REG, 0);
}

int rpi_rtdm_i2s_init(struct platform_device *pdev)
{
	struct rpi_i2s_dev *dev;
	int i,j;
	int ret = 0;
	struct resource *mem_resource;
	void __iomem *base;
	const __be32 *addr;
	dma_addr_t dma_base;
	uint32_t val;
	void *coherent_mem;
	int *tmp;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->clk_prepared = false;
	dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dev->clk)) {
		dev_err(&pdev->dev, "could not get clk: %ld\n",
			PTR_ERR(dev->clk));
		return PTR_ERR(dev->clk);
	}
	dev->irq = RPI_I2S_IRQ_NUM;
	rpi_device_i2s = dev;

	ret = rtdm_irq_request(&i2s_rtdm_irq, dev->irq, rtdm_i2s_intr,
			       0, "rt-i2s", dev);
	if(ret) {
		printk("rpi_rtdm_i2s_init: irq request failed !\n");
		return -1;
	}

	mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_resource);
	if (IS_ERR(base)) {
		printk("rpi_rtdm_i2s_init: devm_ioremap_resource failed.");
		return PTR_ERR(base);
	}
	dev->base_addr = base;

	addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "rpi_rtdm_i2s_init: could not get DMA-register address\n");
		return -EINVAL;
	}

	dma_base = be32_to_cpup(addr);
	dev->fifo_dma_addr = dma_base + BCM2835_I2S_FIFO_A_REG;
	dev->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dev->dma_burst_size = 2;
	dev->dev = &pdev->dev;
	rtdm_event_init(&dev->irq_event, 0);
	
	dev->i2s_task = kcalloc(1,sizeof(rtdm_task_t), GFP_KERNEL);

	rpi_rtdm_clear_regs(dev);
	rpi_rtdm_configure_i2s(dev);

	if (rpi_rtdm_dma_setup(dev))
		return -ENODEV;

	coherent_mem = dma_zalloc_coherent(dev->dma_rx->device->dev,
				4 * PAGE_SIZE, &dummy_phys_addr, GFP_KERNEL);
	if (!coherent_mem) {
		printk("rpi_rtdm_i2s_init: couldn't allocate coherent_mem\n");
		return -ENOMEM;
	}

	
	drv_i2s_transfer = kcalloc(1, sizeof(struct i2s_transfer),
		GFP_KERNEL);

	if (!drv_i2s_transfer) {
		printk("rpi_rtdm_i2s_init: couldn't allocate drv_i2s_transfer\n");
		return -ENOMEM;
	}

	drv_i2s_transfer->rx_buf = coherent_mem;
	drv_i2s_transfer->rx_phys_addr = dummy_phys_addr;
	drv_i2s_transfer->period_len = NUM_OF_WORDS * sizeof(uint32_t);
	drv_i2s_transfer->buffer_len = 2 * drv_i2s_transfer->period_len;
	drv_i2s_transfer->tx_buf = drv_i2s_transfer->rx_buf +
			drv_i2s_transfer->buffer_len;
	drv_i2s_transfer->tx_phys_addr = dummy_phys_addr + drv_i2s_transfer->buffer_len;

	printk("drv_i2s_transfer->buffer_len = %d \n drv_i2s_transfer->period_len = %d\n",drv_i2s_transfer->buffer_len,drv_i2s_transfer->period_len);
	printk("rx_buf = %p and tx_buf = %p, rx_phys_addr = %p tx_phys_addr = %p fifo_dma_addr = %p\n",drv_i2s_transfer->rx_buf,drv_i2s_transfer->tx_buf, drv_i2s_transfer->rx_phys_addr, drv_i2s_transfer->tx_phys_addr, dev->fifo_dma_addr);

	ret = rpi_rtdm_dma_prepare(dev, drv_i2s_transfer);
	if (ret)
		printk("rpi_rtdm_dma_prepare failed!\n");

	rpi_i2s_setup_clock(dev);
	rpi_rtdm_i2s_enable(dev);
	/* Clear the fifos */
	rpi_rtdm_clear_fifos(dev, true, true);

	tmp = (int *) drv_i2s_transfer->tx_buf;

	for (i = 0; i < NUM_OF_WORDS * 2; i++) {
		tmp[i] = 0;
	}
	
	for (i =0; i < 16; i++)
		i2s_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG, 0);

	msleep(10);
	/* print all the registers just for debug*/
	i2s_reg_read(dev->base_addr, BCM2835_I2S_CS_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_CS_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_MODE_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_MODE_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_RXC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_RXC_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_TXC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_TXC_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_INTEN_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_INTEN_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_INTSTC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_INTSTC_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_DREQ_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_DREQ_A_REG = 0x%08X\n",val);
	i2s_reg_read(dev->base_addr, BCM2835_I2S_GRAY_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_GRAY_REG = 0x%08X\n",val);

	rpi_rtdm_start_dma(dev, drv_i2s_transfer);

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
	rpi_rtdm_i2s_start_stop(dev, RPI_I2S_START_CMD);
	

	/* dmaengine_terminate_async(dev->dma_rx);
	dmaengine_terminate_async(dev->dma_tx);
	rtdm_task_destroy(dev->i2s_task);
	rtdm_event_destroy(&dev->irq_event); */
	
	return ret;
}

void rpi_rtdm_i2s_exit(struct platform_device *pdev)
{
	rtdm_irq_free(&i2s_rtdm_irq);
	devm_iounmap(&pdev->dev, (void *)rpi_device_i2s->base_addr);
	dma_free_coherent(rpi_device_i2s->dma_rx->device->dev,
				4 * PAGE_SIZE, drv_i2s_transfer->rx_buf, 
				dummy_phys_addr);
	dma_release_channel(rpi_device_i2s->dma_tx);
	dma_release_channel(rpi_device_i2s->dma_rx);
	kfree(drv_i2s_transfer);
	devm_kfree(&pdev->dev, (void *)rpi_device_i2s);
}