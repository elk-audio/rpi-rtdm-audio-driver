/**
 * @file rpi-rtdm-i2s.h
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief 
 * @version 0.1
 * @date 2019-06-11
 * 
 * @copyright MIND Music Labs (c) 2019
 * 
 */
#ifndef RPI_RTDM_I2S_H
#define RPI_RTDM_I2S_H

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/ipipe.h>
#include <linux/ipipe_domain.h>
#include <asm/barrier.h>
#include <linux/err.h>
#include <linux/sizes.h>
#include <linux/scatterlist.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#define RPI_I2S_IRQ_NUM 85

#define BCM2835_I2S_PERIPHERAL_BASE	0x20203000
#define RPI_I2S_STOP_CMD 0
#define RPI_I2S_START_CMD 1

/* I2S registers */
#define BCM2835_I2S_CS_A_REG		0x00
#define BCM2835_I2S_FIFO_A_REG		0x04
#define BCM2835_I2S_MODE_A_REG		0x08
#define BCM2835_I2S_RXC_A_REG		0x0c
#define BCM2835_I2S_TXC_A_REG		0x10
#define BCM2835_I2S_DREQ_A_REG		0x14
#define BCM2835_I2S_INTEN_A_REG		0x18
#define BCM2835_I2S_INTSTC_A_REG	0x1c
#define BCM2835_I2S_GRAY_REG		0x20

/* I2S register settings */
/* CS_A Register Masks */
#define BCM2835_I2S_STBY		BIT(25)
#define BCM2835_I2S_SYNC		BIT(24)
#define BCM2835_I2S_RXSEX		BIT(23)
#define BCM2835_I2S_RXF		BIT(22)
#define BCM2835_I2S_TXE		BIT(21)
#define BCM2835_I2S_RXD		BIT(20)
#define BCM2835_I2S_TXD		BIT(19)
#define BCM2835_I2S_RXR		BIT(18)
#define BCM2835_I2S_TXW		BIT(17)
#define BCM2835_I2S_CS_RXERR		BIT(16)
#define BCM2835_I2S_CS_TXERR		BIT(15)
#define BCM2835_I2S_RXSYNC		BIT(14)
#define BCM2835_I2S_TXSYNC		BIT(13)
#define BCM2835_I2S_DMAEN		BIT(9)
#define BCM2835_I2S_RXTHR(v)		((v) << 7)
#define BCM2835_I2S_TXTHR(v)		((v) << 5)
#define BCM2835_I2S_RXCLR		BIT(4)
#define BCM2835_I2S_TXCLR		BIT(3)
#define BCM2835_I2S_TXON		BIT(2)
#define BCM2835_I2S_RXON		BIT(1)
#define BCM2835_I2S_EN			(1)

/* MODE_A Register Masks */
#define BCM2835_I2S_CLKDIS		BIT(28)
#define BCM2835_I2S_PDMN		BIT(27)
#define BCM2835_I2S_PDME		BIT(26)
#define BCM2835_I2S_FRXP		BIT(25)
#define BCM2835_I2S_FTXP		BIT(24)
#define BCM2835_I2S_CLKM		BIT(23)
#define BCM2835_I2S_CLKI		BIT(22)
#define BCM2835_I2S_FSM		BIT(21)
#define BCM2835_I2S_FSI		BIT(20)
#define BCM2835_I2S_FLEN(v)		((v) << 10)
#define BCM2835_I2S_FSLEN(v)		(v)

/* RXC_A Register Masks */
#define BCM2835_I2S_CHWEX		BIT(15)
#define BCM2835_I2S_CHEN		BIT(14)
#define BCM2835_I2S_CHPOS(v)		((v) << 4)
#define BCM2835_I2S_CHWID(v)		(v)
#define BCM2835_I2S_CH1(v)		((v) << 16)
#define BCM2835_I2S_CH2(v)		(v)
#define BCM2835_I2S_CH1_POS(v)		BCM2835_I2S_CH1(BCM2835_I2S_CHPOS(v))
#define BCM2835_I2S_CH2_POS(v)		BCM2835_I2S_CH2(BCM2835_I2S_CHPOS(v))

/* DREQ_A Register Masks*/
#define BCM2835_I2S_TX_PANIC(v)	((v) << 24)
#define BCM2835_I2S_RX_PANIC(v)	((v) << 16)
#define BCM2835_I2S_TX(v)		((v) << 8)
#define BCM2835_I2S_RX(v)		(v)

#define BCM2835_I2S_INT_RXERR		BIT(3)
#define BCM2835_I2S_INT_TXERR		BIT(2)
#define BCM2835_I2S_INT_RXR		BIT(1)
#define BCM2835_I2S_INT_TXW		BIT(0)

/* Frame length register is 10 bit, maximum length 1024 */
#define BCM2835_I2S_MAX_FRAME_LENGTH	1024

static inline void rpi_reg_write(void *base_addr, uint32_t reg_addr,
				uint32_t value)
{
	uint32_t *reg = base_addr + reg_addr;
	wmb();
	*reg = value;
}

static inline void rpi_reg_update_bits(void *base_addr, uint32_t reg_addr,
				uint32_t mask, uint32_t value)
{
	uint32_t *reg = base_addr + reg_addr;
	wmb();
	*reg &= (~mask);
	*reg |= (mask & value);
}

static inline void rpi_reg_read(void *base_addr, uint32_t reg_addr,
			uint32_t *value)
{
	uint32_t *reg = base_addr + reg_addr;
	wmb();
	*value = *reg;
	rmb();
	//printk("i2s_reg_read:  reg ptr = %p\n",reg);
}

struct i2s_buffers_info {
	void			*tx_buf;
	void			*rx_buf;
	size_t			buffer_len;
	size_t			period_len;
	dma_addr_t 			tx_phys_addr;
	dma_addr_t 			rx_phys_addr;
};

/* General device struct */
struct rpi_i2s_dev {
	struct device			*dev;
	void __iomem			*base_addr;
	struct dma_chan			*dma_tx;
	struct dma_chan			*dma_rx;
	struct dma_async_tx_descriptor 	*tx_desc;
	struct dma_async_tx_descriptor	*rx_desc;
	dma_addr_t			fifo_dma_addr;
	unsigned			addr_width;
	unsigned			dma_burst_size;
	struct i2s_buffers_info		*buffer;
	int 				irq;
	rtdm_task_t *i2s_task;
	rtdm_event_t irq_event;
};

#define MAX_DMA_LEN		SZ_64K

int bcm2835_i2s_init(struct platform_device *pdev);
void bcm2835_i2s_exit(struct platform_device *pdev);



#endif