// SPDX-License-Identifier: GPL-2.0
/*
 * @brief The PCM168a codec driver for ELK  PI. A lot of stuff is hardcoded for
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

#define PCM5122_I2C_BUS_NUM 1

static struct i2c_board_info i2c_pcm5122_board_info[] =  {
	{
		I2C_BOARD_INFO("pcm-5122", 0x4d),
	}
};

static int pcm5122_config_codec(struct i2c_client *i2c_client_dev)
{
	char cmd[2];
	int ret;
	/* select page 0*/
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to select page 0\n");
		return ret;
	}
	/* power off */
	cmd[0] = PCM512x_POWER & 0xff;
	cmd[1] = 0x00 | PCM512x_RQPD;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to power down\n");
		return ret;
	}
	/* power on in stand by mode */
	cmd[0] = PCM512x_POWER & 0xff;
	cmd[1] = 0x00 | PCM512x_RQST;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to powerup\n");
		return ret;
	}
	msleep(5);
	cmd[0] = PCM512x_RESET & 0xff;
	cmd[1] = 0x00 | PCM512x_RSTM | PCM512x_RSTR;

	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to set stby mode\n");
		return ret;
	}
	msleep(5);
	cmd[0] = PCM512x_I2S_1  & 0xff;
	cmd[1] = PCM512x_AFMT_I2S | PCM512x_ALEN_32;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to reset device\n");
		return ret;
	}
	msleep(5);
	cmd[0] = PCM512x_PLL_REF  & 0xff;
	cmd[1] = PCM512x_SREF_BCK;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to reset device\n");
		return ret;
	}
	msleep(5);
	cmd[0] = PCM512x_DAC_REF  & 0xff;
	cmd[1] = PCM512x_SDAC_PLL;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to reset device\n");
		return ret;
	}
	msleep(5);
	cmd[0] = PCM512x_ERROR_DETECT  & 0xff;
	cmd[1] = PCM512x_IDSK | PCM512x_IDBK | PCM512x_IDSK |
		PCM512x_IDCH;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to reset device\n");
		return ret;
	}
	cmd[0] = PCM512x_POWER  & 0xff;
	cmd[1] = 0x00;
	ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2);
	if (ret < 0) {
		printk("pcm5122: Failed to reset device\n");
		return ret;
	}
	return 0;
}

int pcm5122_codec_init(void)
{
	struct i2c_adapter *adapter = i2c_get_adapter(PCM5122_I2C_BUS_NUM);
	struct i2c_client *client = i2c_new_device
					(adapter, i2c_pcm5122_board_info);

	if (pcm5122_config_codec(client)) {
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
