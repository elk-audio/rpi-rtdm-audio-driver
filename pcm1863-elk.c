// SPDX-License-Identifier: GPL-2.0
/*
 * @brief The PCM1863 codec driver for ELK  PI. A lot of stuff is hardcoded for
 *	now, idea is to make it runtime configurable ideally. This module is
 *	based on the mainline pcm3168a driver by Damien Horsley.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "pcm1863-elk.h"
#include "elk-pi-config.h"

#define PCM1863_I2C_BUS_NUM 1

static struct i2c_board_info i2c_pcm1863_board_info[] =  {
	{
		I2C_BOARD_INFO("pcm-5122", 0x4a),
	}
};

static int pcm1863_reg_write(struct i2c_client *dev,
				unsigned int reg, unsigned int val)
{
	int ret;
	char cmd[2];
	cmd[0] = reg & 0xff;
	cmd[1] = val;
	ret = i2c_master_send(dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm1863: Failed to write reg\n");
		return ret;
	}
	return 0;
}

static int pcm1863_config_codec(struct i2c_client *dev)
{
	int ret = -1;
	/* select page 0*/
	if(pcm1863_reg_write(dev, PCM186X_PAGE, 0x00)) {
		return ret;
	}
	/* reset registers */
	if(pcm1863_reg_write(dev, PCM186X_PAGE, PCM186X_RESET)) {
		return ret;
	}

	if(pcm1863_reg_write(dev, PCM186X_CLK_CTRL,
				 PCM186X_CLK_CTRL_CLKDET_EN)) {
		return ret;
	}

	if(pcm1863_reg_write(dev, PCM186X_PLL_CTRL,
				 PCM186X_PLL_CTRL_REF_SEL)) {
		return ret;
	}

	if(pcm1863_reg_write(dev, PCM186X_PCM_CFG,
				 PCM186X_PCM_CFG_RX_WLEN_32 |
				 PCM186X_PCM_CFG_TX_WLEN_32 |
				 PCM186X_PCM_CFG_FMT_I2S)) {
		return ret;
	}
	return 0;
}

int pcm1863_codec_init(void)
{
	struct i2c_client *client = NULL;
	struct i2c_adapter *adapter = NULL;
	adapter = i2c_get_adapter(PCM1863_I2C_BUS_NUM);
	if (!adapter) {
		printk(KERN_ERR "pcm1863-elk: Failed to get i2c adapter\n");
		return -1;
	}

	client = i2c_new_device(adapter, i2c_pcm1863_board_info);
	if (!client) {
		printk( KERN_ERR "pcm1863-elk: Failed to get i2c client\n");
		return -1;
	}

	if (pcm1863_config_codec(client)) {
		printk(KERN_ERR "pcm1863-elk: config_codec failed\n");
		return -1;
	}
	i2c_unregister_device(client);
	i2c_put_adapter(adapter);
	printk(KERN_INFO "pcm1863-elk: codec configured\n");
	return 0;
}
EXPORT_SYMBOL_GPL(pcm1863_codec_init);

void pcm1863_codec_exit(void)
{
	printk(KERN_INFO "pcm1863-elk: unregister i2c-client\n");
}
EXPORT_SYMBOL_GPL(pcm1863_codec_exit);

int pcm1863_init(void)
{
	printk(KERN_INFO "pcm1863-elk: module init\n");
	return 0;
}

void pcm1863_exit(void)
{
	printk(KERN_INFO "pcm1863-elk: module exit\n");
}

module_init(pcm1863_init)
module_exit(pcm1863_exit)

MODULE_DESCRIPTION("PCM5122 I2C codec driver for ELK Pi");
MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_LICENSE("GPL");
