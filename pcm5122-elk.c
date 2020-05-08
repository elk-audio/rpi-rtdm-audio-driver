// SPDX-License-Identifier: GPL-2.0
/*
 * @brief The PCM5122 codec driver for ELK  PI. A lot of stuff is hardcoded for
 *	now, idea is to make it runtime configurable ideally. This module is
 *	based on the mainline pcm3168a driver by Damien Horsley.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "pcm5122-elk.h"

#define PCM5122_I2C_BUS_NUM 	1
#define PCM5122_SCLK_RATE 	24576000
#define PCM5122_BCLK_RATE	3072000

static struct i2c_board_info i2c_pcm5122_board_info[] =  {
	{
		I2C_BOARD_INFO("pcm-5122", 0x4d),
	}
};

static int pcm5122_reg_write(struct i2c_client *dev,
				unsigned int reg, unsigned int val)
{
	int ret;
	char cmd[2];
	cmd[0] = reg & 0xff;
	cmd[1] = val;
	ret = i2c_master_send(dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to write reg\n");
		return ret;
	}
	return 0;
}

static int pcm5122_config_codec(struct i2c_client *dev,
				int mode, int sampling_freq, bool enable_low_latency)
{
	int ret = -1;
	printk("pcm5122_config_codec: mode = %d\n", mode);

	if (sampling_freq != 48000) {
		printk(KERN_ERR "pcm5122: Unsupported sampling freq %d",
			sampling_freq);
			return ret;
	}

	/* select page 0*/
	if (pcm5122_reg_write(dev, 0x00, 0x00)) {
		return ret;
	}
	/* power off */
	if (pcm5122_reg_write(dev, PCM512x_POWER, PCM512x_RQPD)) {
		return ret;
	}
	/* power on in stand by mode */
	if (pcm5122_reg_write(dev, PCM512x_POWER, PCM512x_RQST)) {
		return ret;
	}
	/* reset regs */
	if (pcm5122_reg_write(dev, PCM512x_RESET, PCM512x_RSTM | PCM512x_RSTR)) {
		return ret;
	}
	/* I2S format 32-bit SE */
	if (pcm5122_reg_write(dev, PCM512x_I2S_1, PCM512x_AFMT_I2S |
						PCM512x_ALEN_32)) {
		return ret;
	}

	if (enable_low_latency) {
		if (pcm5122_reg_write(dev, PCM512x_DSP_PROGRAM,
			PCM512x_LOW_LATENCY_IIR)) {
			return ret;
		}
	}

	if (mode == PCM5122_MASTER_MODE) {
		/* enable GPIO3 for the clk generator */
		if (pcm5122_reg_write(dev, PCM512x_GPIO_EN, PCM512x_G3OE)) {
			return ret;
		}

		if (pcm5122_reg_write(dev, PCM512x_GPIO_OUTPUT_3, 0x02)) {
			return ret;
		}

		if (pcm5122_reg_write(dev, PCM512x_GPIO_CONTROL_1, 0x04)) {
			return ret;
		}

		if (pcm5122_reg_write(dev, PCM512x_BCLK_LRCLK_CFG, PCM512x_LRKO |
					PCM512x_BCKO)) {
			return ret;
		}
		/* set the bit clk divider from sclk*/
		if (pcm5122_reg_write(dev, PCM512x_MASTER_CLKDIV_1,
		PCM5122_SCLK_RATE/PCM5122_BCLK_RATE - 1)) {
			return ret;
		}
		/* set the LR clk divider from bit clk*/
		if (pcm5122_reg_write(dev, PCM512x_MASTER_CLKDIV_2,
		PCM5122_BCLK_RATE/sampling_freq - 1)) {
			return ret;
		}

		if (pcm5122_reg_write(dev, PCM512x_MASTER_MODE, PCM512x_RLRK |
			PCM512x_RBCK)) {
			return ret;
		}
	} else {
		if (pcm5122_reg_write(dev, PCM512x_PLL_REF, PCM512x_SREF_BCK)) {
			return ret;
		}

		if (pcm5122_reg_write(dev, PCM512x_DAC_REF, PCM512x_SDAC_PLL)) {
			return ret;
		}
	}

	if (pcm5122_reg_write(dev, PCM512x_ERROR_DETECT, PCM512x_IDSK |
				PCM512x_IDBK | PCM512x_IDSK |
		PCM512x_IDCH)) {
			return ret;
	}
	/* get out of standby mode */
	if (pcm5122_reg_write(dev, PCM512x_POWER, 0x00)) {
			return ret;
	}
	return 0;
}

int pcm5122_codec_init(int mode, int sampling_freq, bool enable_low_latency)
{
	struct i2c_client *client = NULL;
	struct i2c_adapter *adapter = NULL;
	adapter = i2c_get_adapter(PCM5122_I2C_BUS_NUM);
	if (!adapter) {
		printk(KERN_ERR "pcm5122: Failed to get i2c adapter\n");
		return -1;
	}

	client = i2c_new_device(adapter, i2c_pcm5122_board_info);
	if (!client) {
		printk(KERN_ERR "pcm5122: Failed to get i2c client 5122\n");
		return -1;
	}

	if (pcm5122_config_codec(client, mode, sampling_freq, enable_low_latency)) {
		printk(KERN_ERR "pcm5122-elk: config_codec failed\n");
		return -1;
	}
	i2c_unregister_device(client);
	i2c_put_adapter(adapter);
	printk(KERN_INFO "pcm5122-elk: codec configured\n");
	return 0;
}
EXPORT_SYMBOL_GPL(pcm5122_codec_init);

void pcm5122_codec_exit(void)
{
	printk(KERN_INFO "pcm5122-elk: unregister i2c-client\n");
}
EXPORT_SYMBOL_GPL(pcm5122_codec_exit);

int pcm5122_init(void)
{
	printk(KERN_INFO "pcm5122-elk: module init\n");
	return 0;
}

void pcm5122_exit(void)
{
	printk(KERN_INFO "pcm5122-elk: module exit\n");
}

module_init(pcm5122_init)
module_exit(pcm5122_exit)

MODULE_DESCRIPTION("PCM5122 I2C codec driver for ELK Pi");
MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_LICENSE("GPL");
