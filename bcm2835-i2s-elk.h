// SPDX-License-Identifier: GPL-2.0
/**
 * @file bcm2835-i2s-elk.h
 * @author Nitin Kulkarni
 * @version 0.1
 *
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 *
 */
#ifndef RPI_BCM2835_I2S_H
#define RPI_BCM2835_I2S_H

#include <linux/bitops.h>
#include <asm/barrier.h>
#include <linux/err.h>
#include <linux/sizes.h>

#include "audio-rtdm.h"

#define BCM2835_I2S_IRQ_NUM 85

#define BCM2835_I2S_PERIPHERAL_BASE	0x20203000
#define BCM2835_I2S_STOP_CMD 		0
#define BCM2835_I2S_START_CMD 		1
#define BCM2835_DMA_THR_TX		24
#define BCM2835_DMA_THR_RX		8
#define BCM2835_DMA_TX_PANIC_THR	8
#define BCM2835_DMA_RX_PANIC_THR	40

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
#define NUM_OF_PAGES			20

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
	rmb();
	*value = *reg;
}

extern int bcm2835_i2s_init(int audio_buffer_size,
						 int audio_channels);
extern int bcm2835_i2s_exit(void);
extern struct audio_rtdm_dev *bcm2835_get_i2s_dev(void);

#endif