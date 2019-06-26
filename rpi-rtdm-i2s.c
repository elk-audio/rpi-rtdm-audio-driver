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

/* #include <linux/irq.h>
#include <linux/interrupt.h> */

#include "rpi-rtdm-i2s.h"

#define RPI_I2S_STOP_CMD 0
#define RPI_I2S_START_CMD 1

#define DEVICE_NAME "rtdm_audio"

static unsigned long i2s_interrupts = 0;
module_param(i2s_interrupts, ulong, 0644);

static unsigned long i2s_irq = 85;
module_param(i2s_irq, ulong, 0644);

static unsigned long rx_fifo_errors = 0;
module_param(rx_fifo_errors, ulong, 0644);
static unsigned long tx_fifo_errors = 0;
module_param(tx_fifo_errors, ulong, 0644);

struct rpi_i2s_dev *rpi_device_i2s;

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

/* static irqreturn_t rpi_i2s_interrupt(int irq, void *dev_id)
{
	uint32_t  i, intr_status, cs_a, fifo_reg, err;
	unsigned long flags;
	struct rpi_i2s_dev *dev = rtdm_irq_get_arg(irqh, struct rpi_i2s_dev);
	uint32_t *i2s_intreg = i2s_regbase + BCM2835_I2S_INTEN_A_REG;
	uint32_t *i2s_intstatus = i2s_regbase + BCM2835_I2S_INTSTC_A_REG;
	*i2s_intreg = 0x00;
	printk("rpi_i2s_rtdm_interrupt !");
	
	//raw_spin_lock_irqsave(&dev->lock, flags);
	
	i2s_interrupts++;
	//raw_spin_unlock_irqrestore(&dev->lock, flags);
	
	*i2s_intstatus = 0x0F;
	return IRQ_HANDLED;
} */

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

static void rpi_rtdm_configure_i2s(struct rpi_i2s_dev * dev)
{
	unsigned int data_length, framesync_length;
	unsigned int slots, slot_width;
	int frame_length;
	unsigned int rx_ch1_pos, rx_ch2_pos, tx_ch1_pos, tx_ch2_pos;
	unsigned int mode, format;
	bool bit_clock_master = false;
	bool frame_sync_master = false;
	bool frame_start_falling_edge = false;

	data_length = 32;
	mode = 0;
	slots = 2;
	slot_width = 32;
	frame_length = slots * slot_width;
	format = BCM2835_I2S_CHEN | BCM2835_I2S_CHWEX;
	format |= BCM2835_I2S_CHWID((data_length-8)&0xf);
	framesync_length = frame_length / 2;
	frame_start_falling_edge = true;

	rx_ch1_pos = 1;
	rx_ch2_pos = 33; /* calculated manually for now */
	tx_ch1_pos = 1;
	tx_ch2_pos = 33;
	/* CH2 format is the same as for CH1 */
	format = BCM2835_I2S_CH1(format) | BCM2835_I2S_CH2(format);

	mode |= BCM2835_I2S_FLEN(frame_length - 1);
	mode |= BCM2835_I2S_FSLEN(framesync_length);

	if (!bit_clock_master)
		mode |= BCM2835_I2S_CLKDIS | BCM2835_I2S_CLKM;

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

	/* dev_dbg(dev->dev, "sampling rate: %d bclk rate: %d\n",
		params_rate(params), bclk_rate); */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_MODE_A_REG,
			BCM2835_I2S_CLKDIS, 0);
}

static void rpi_rtdm_i2s_enable(struct rpi_i2s_dev *dev)
{	
	/* Disable RAM STBY */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_STBY, BCM2835_I2S_STBY);

	/* Enable PCM block */
	i2s_reg_update_bits(dev->base_addr, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_EN, BCM2835_I2S_EN);	
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
	int i;
	int ret = 0;
	struct resource *mem_resource;
	void __iomem *base;
	const __be32 *addr;
	dma_addr_t dma_base;
	uint32_t val;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	rpi_device_i2s = dev;
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
	dev->dev = &pdev->dev;
	//printk("i2s_init: dma_base = %p \n",dma_base);
	/* TO DO: Dma support */
	if (rpi_rtdm_dma_setup(dev))
		return -ENODEV;
	rpi_rtdm_clear_regs(dev);
	rpi_rtdm_configure_i2s(dev);
	rpi_rtdm_i2s_enable(dev);
	/* Clear the fifos */
	rpi_rtdm_clear_fifos(dev, true, true);

	for(i = 0; i < 64; i++)
		i2s_reg_write(dev->base_addr, BCM2835_I2S_FIFO_A_REG, 0);

	msleep(5);
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

	msleep(1000);
	/* start the i2s controller */
	//rpi_rtdm_i2s_start_stop(dev, RPI_I2S_START_CMD);
	return ret;
}

void rpi_rtdm_remove_irq(void)
{
	
}