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
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/barrier.h>
#include <linux/spinlock.h>
#include <rtdm/driver.h>
#include <linux/ipipe_domain.h>

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

static rtdm_irq_t i2s_irq_handle;
struct rpi_i2s_dev *rpidev;
static volatile void *i2s_regbase;


static void rpi_rtdm_clear_fifos(struct rpi_i2s_dev *dev,
				    bool tx, bool rx)
{
	uint32_t csreg;
	uint32_t i2s_active_state;
	uint32_t off;
	uint32_t clr;

	off =  tx ? BCM2835_I2S_TXON : 0;
	off |= rx ? BCM2835_I2S_RXON : 0;

	clr =  tx ? BCM2835_I2S_TXCLR : 0;
	clr |= rx ? BCM2835_I2S_RXCLR : 0;

	/* Backup the current state */
	regmap_read(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, &csreg);
	i2s_active_state = csreg & (BCM2835_I2S_RXON | BCM2835_I2S_TXON);

	/* Stop I2S module */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, off, 0);

	/*
	 * Clear the FIFOs
	 * Requires at least 2 PCM clock cycles to take effect
	 */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, clr, clr);

	/* Wait for some clock cycles */
	//msleep(5);

	/* Restore I2S state */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_RXON | BCM2835_I2S_TXON, i2s_active_state);
}

static  int rpi_i2s_rtdm_interrupt(rtdm_irq_t *irqh)
//static irqreturn_t rpi_i2s_interrupt(int irq, void *dev_id)
{
	uint32_t  i, intr_status, cs_a, fifo_reg, err;
	unsigned long flags;
	struct rpi_i2s_dev *dev = rtdm_irq_get_arg(irqh, struct rpi_i2s_dev);
	uint32_t *i2s_intreg = i2s_regbase + BCM2835_I2S_INTEN_A_REG;
	uint32_t *i2s_intstatus = i2s_regbase + BCM2835_I2S_INTSTC_A_REG;
	*i2s_intreg = 0x00;
	printk("rpi_i2s_rtdm_interrupt !");
	
	//raw_spin_lock_irqsave(&dev->lock, flags);
	
	regmap_read(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, &cs_a);
	
	// Check if RX Fifo has overflow error
	if (cs_a & BCM2835_I2S_CS_RXERR) {
		err = 1;
		rx_fifo_errors++;
	}
	// Check if TX Fifo has overflow error
	if (cs_a & BCM2835_I2S_CS_TXERR) {
		err = 1;
		tx_fifo_errors++;
	}

	// Check if RX Fifo crossed RXTHR num of samples
	if (cs_a & BCM2835_I2S_RXR) {
		for (i = 0; i < 64; i++) {
			regmap_read(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, 
			&cs_a);
			if (cs_a & BCM2835_I2S_TXD) {
				regmap_read(dev->i2s_regmap,
				BCM2835_I2S_FIFO_A_REG,
				&fifo_reg);
				regmap_write(dev->i2s_regmap,
				BCM2835_I2S_FIFO_A_REG,
				fifo_reg);
			}
		}
	}
	if (err) {
		// Clear the fifos
		rpi_rtdm_clear_fifos(dev, true, true);
		cs_a = BCM2835_I2S_CS_TXERR | BCM2835_I2S_CS_RXERR;
		regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG,
		cs_a, cs_a);
	}
	intr_status = BCM2835_I2S_INT_RXERR |
			BCM2835_I2S_INT_TXERR |
			BCM2835_I2S_INT_RXR |
			BCM2835_I2S_INT_TXW;
	regmap_write(dev->i2s_regmap, BCM2835_I2S_INTSTC_A_REG, intr_status);
	i2s_interrupts++;
	//raw_spin_unlock_irqrestore(&dev->lock, flags);
	
	*i2s_intstatus = 0x0F;
	return RTDM_IRQ_HANDLED;
}

static bool rpi_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BCM2835_I2S_CS_A_REG:
	case BCM2835_I2S_FIFO_A_REG:
	case BCM2835_I2S_INTSTC_A_REG:
	case BCM2835_I2S_GRAY_REG:
		return true;
	default:
		return false;
	};
}

static bool rpi_i2s_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BCM2835_I2S_FIFO_A_REG:
		return true;
	default:
		return false;
	};
}

static const struct reg_default rpi_i2s_reg_defaults[] = {
	{0x00, 0x00000000},
	{0x08, 0x00000000},
	{0x0c, 0x00000000},
	{0x10, 0x00000000},
	{0x14, 0x00000000},
	{0x18, 0x00000000},
	{0x1c, 0x00000000},
	{0x20, 0x00000000},
};

static const struct regmap_config rpi_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = BCM2835_I2S_GRAY_REG,
	.reg_defaults = rpi_i2s_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rpi_i2s_reg_defaults),
	.precious_reg = rpi_i2s_precious_reg,
	.volatile_reg = rpi_i2s_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static void rpi_rtdm_i2s_start_stop(struct rpi_i2s_dev *dev, int cmd)
{
	uint32_t mask;
	wmb();
	mask = BCM2835_I2S_RXON | BCM2835_I2S_TXON;

	if (cmd == RPI_I2S_START_CMD) {
		regmap_update_bits(dev->i2s_regmap,
			BCM2835_I2S_CS_A_REG, mask, mask);
	}
	else {
		regmap_update_bits(dev->i2s_regmap,
			BCM2835_I2S_CS_A_REG, mask, 0);
	}
}

static void rpi_rtdm_configure_i2s(struct rpi_i2s_dev * dev)
{
	unsigned int data_length, framesync_length;
	unsigned int slots, slot_width;
	int frame_length;
	unsigned int rx_mask, tx_mask;
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

	regmap_write(dev->i2s_regmap, BCM2835_I2S_MODE_A_REG, mode);

	regmap_write(dev->i2s_regmap, BCM2835_I2S_RXC_A_REG,format
		| BCM2835_I2S_CH1_POS(rx_ch1_pos)
		| BCM2835_I2S_CH2_POS(rx_ch2_pos));

	regmap_write(dev->i2s_regmap, BCM2835_I2S_TXC_A_REG,format
		|BCM2835_I2S_CH1_POS(tx_ch1_pos)
		| BCM2835_I2S_CH2_POS(tx_ch2_pos));

	dev_dbg(dev->dev, "rx pos: %d,%d tx pos: %d,%d\n",
		rx_ch1_pos, rx_ch2_pos, tx_ch1_pos, tx_ch2_pos);

	/* dev_dbg(dev->dev, "sampling rate: %d bclk rate: %d\n",
		params_rate(params), bclk_rate); */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_MODE_A_REG,
			BCM2835_I2S_CLKDIS, 0);
}

static void rpi_rtdm_i2s_enable(struct rpi_i2s_dev *dev)
{	
	/* Disable RAM STBY */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_STBY, BCM2835_I2S_STBY);

	/* Enable PCM block */
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG,
			BCM2835_I2S_EN, BCM2835_I2S_EN);	
}

static void rpi_rtdm_enable_intrrupt(struct rpi_i2s_dev *dev)
{
	uint32_t cs_a, int_en;

	cs_a = BCM2835_I2S_TXTHR(0x00);
	regmap_update_bits(dev->i2s_regmap, BCM2835_I2S_CS_A_REG,
			cs_a, cs_a);
	int_en = BCM2835_I2S_INT_TXW;
	/* Set the INTEN_A reg for the rx interrupt */
	regmap_write(dev->i2s_regmap, BCM2835_I2S_INTEN_A_REG, int_en);
}

int rpi_rtdm_i2s_init(struct platform_device *pdev)
{
	struct rpi_i2s_dev *dev;
	int irq,i;
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
	rpidev = dev;
	mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem_resource);
	i2s_regbase = base;
	if (IS_ERR(base)) {
		printk("rpi_rtdm_i2s_init: devm_ioremap_resource failed.");
		return PTR_ERR(base);
	}

	dev->i2s_regmap = devm_regmap_init_mmio(&pdev->dev, base,
				&rpi_regmap_config);
	if (IS_ERR(dev->i2s_regmap)) {
		printk("rpi_rtdm_i2s_init: devm_regmap_init_mmio failed.");
		return PTR_ERR(dev->i2s_regmap);
	}

	addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "rpi_rtdm_i2s_init: could not get DMA-register address\n");
		return -EINVAL;
	}
	
	dma_base = be32_to_cpup(addr);
	//printk("i2s_init: dma_base = %p \n",dma_base);
	/* TO DO: Dma support */

	
	irq_set_irq_type(i2s_irq, IRQ_TYPE_EDGE_RISING);
	ret = rtdm_irq_request(&i2s_irq_handle, i2s_irq,
				rpi_i2s_rtdm_interrupt, RTDM_IRQTYPE_EDGE, "rpi_i2s", dev);
	if (ret) {
		printk(KERN_ERR "rpi_rtdm_i2s_init failed to get \
		 interrupt\n");
	}

	rpi_rtdm_i2s_enable(dev);
	rpi_rtdm_configure_i2s(dev);
	/* Clear the fifos */
	rpi_rtdm_clear_fifos(dev, true, true);

	/* print all the registers just for debug*/
	/* regmap_read(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_CS_A_REG = 0x%08X and val in base is %08X\n",val,*(int*)base);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_MODE_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_MODE_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_RXC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_RXC_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_TXC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_TXC_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_INTEN_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_INTEN_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_INTSTC_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_INTSTC_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_DREQ_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_DREQ_A_REG = 0x%08X\n",val);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_GRAY_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_GRAY_REG = 0x%08X\n",val);

	printk("rpi_rtdm_i2s_init: using i2s_irq %d\n",i2s_irq); */
	msleep(50);
	for (i=0; i < 64; i++) {
		regmap_write(dev->i2s_regmap,
				BCM2835_I2S_FIFO_A_REG,
				0x00000000);
	}
	
	rpi_rtdm_enable_intrrupt(dev);

	msleep(1000);
	/* start the i2s controller */
	rpi_rtdm_i2s_start_stop(dev, RPI_I2S_START_CMD);
	msleep(10);

	regmap_read(dev->i2s_regmap, BCM2835_I2S_CS_A_REG, &val);
	printk("rpi_rtdm_i2s_init: BCM2835_I2S_CS_A_REG = 0x%08X\n",val);
	
	return ret;
}

void rpi_rtdm_remove_irq(void) {
	free_irq(i2s_irq, rpidev);
	//rtdm_irq_free(&i2s_irq_handle);
}