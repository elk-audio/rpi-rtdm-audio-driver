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
#include <linux/gpio.h>

#include "pcm3168a-elk.h"

#define PCM3168A_CODEC_RST_PIN  16
#define PCM3168A_CPLD_RST_PIN 	4
#define PCM3168A_I2C_BUS_NUM	1

/* generated with si5351 tool to generate 24.576 Mhz clk */
static uint8_t clkgen_reg_val_lookup[CLKGEN_NUM_OF_REGS][2] = {
		{0x07, 0x00},
		{0x09, 0xFF},
		{0x0A, 0xFF},
		{0x0C, 0x00},
		{0x0D, 0x00},
		{0x0F, 0x00},
		{0x10, 0x0F},
		{0x11, 0x8C},
		{0x12, 0x8C},
		{0x13, 0x8C},
		{0x14, 0x8C},
		{0x15, 0x8C},
		{0x16, 0x8C},
		{0x17, 0x8C},
		{0x1A, 0x04},
		{0x1B, 0x65},
		{0x1C, 0x00},
		{0x1D, 0x0E},
		{0x1E, 0x9C},
		{0x1F, 0x00},
		{0x20, 0x02},
		{0x21, 0x74},
		{0x2A, 0x00},
		{0x2B, 0x02},
		{0x2C, 0x00},
		{0x2D, 0x10},
		{0x2E, 0x40},
		{0x2F, 0x00},
		{0x30, 0x00},
		{0x31, 0x00},
		{0x5A, 0x00},
		{0x5B, 0x00},
		{0x95, 0x00},
		{0x96, 0x00},
		{0x97, 0x00},
		{0x98, 0x00},
		{0x99, 0x00},
		{0x9A, 0x00},
		{0x9B, 0x00},
		{0xA2, 0x00},
		{0xA3, 0x00},
		{0xA4, 0x00},
		{0xB7, 0x92}
	};

static struct i2c_board_info i2c_clkgen_board_info[] =  {
	{
		I2C_BOARD_INFO("clk-gen", 0x60),
	}
};

static struct i2c_board_info i2c_pcm3168a_board_info[] =  {
	{
		I2C_BOARD_INFO("pcm-3168a", 0x44),
	}
};

static int pcm3168_reg_write(struct i2c_client *dev,
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

static int pcm3168a_config_clk_gen(struct i2c_client *dev)
{
	uint8_t *cmd = &clkgen_reg_val_lookup[0][0];
	int i, ret = -1;

	for (i = CLKGEN_CLK0_CNTRL_REG; i <= CLKGEN_CLK7_CNTRL_REG; i++) {
		if(pcm3168_reg_write(dev, i, CLKGEN_CLK_PWR_DWN_MASK))
		return ret;
	}
	msleep(50);
	for (i = 0; i < CLKGEN_NUM_OF_REGS; i++) {
		if(pcm3168_reg_write(dev, cmd[0], cmd[1]))
			return ret;
		cmd+=2;
		msleep(5);
	}
	if (pcm3168_reg_write(dev, CLKGEN_PLL_RESET_REG,
		CLKGEN_PLL_RESET_MASK)){
		return ret;
	}
	msleep(5);
	if(pcm3168_reg_write(dev, CLKGEN_OUTPUT_EN_CNTRL_REG,
		CLKGEN_EN_OUTPUT_MASK)) {
		return ret;
	}
	return 0;
}

static int pcm3168a_config_codec(struct i2c_client *i2c_client_dev)
{
	int ret = -1;

	if(pcm3168_reg_write(i2c_client_dev, PCM_DAC_CNTRL_TWO_REG,
		0x00 | DAC_CHAN_0_1_DISABLED_MODE_MASK |
			DAC_CHAN_2_3_DISABLED_MODE_MASK |
			DAC_CHAN_4_5_DISABLED_MODE_MASK |
			DAC_CHAN_6_7_DISABLED_MODE_MASK)) {
		return ret;
	}

	if(pcm3168_reg_write(i2c_client_dev, PCM_ADC_CNTRL_TWO_REG,
		0x00 | ADC_CHAN_0_1_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_ENABLE_MASK)) {
		return ret;
	}

	if(pcm3168_reg_write(i2c_client_dev, PCM_DAC_CNTRL_ONE_REG,
		0x00 | DAC_SLAVE_MODE_MASK | DAC_LJ_24_BIT_TDM_MODE_MASK)) {
		return ret;
	}

	if(pcm3168_reg_write(i2c_client_dev, PCM_DAC_CNTRL_THREE_REG,
		0x00 | DAC_MASTER_VOLUME_CONTROL_MODE_MASK |
			DAC_ATTEN_SPEED_SLOW_MASK |
			DAC_DEMPH_DISABLE_MASK)) {
		return ret;
	}

	if(pcm3168_reg_write(i2c_client_dev, PCM_ADC_CONTROL_THREE_REG,
		0x00 | ADC_MASTER_VOLUME_CONTROL_MODE_MASK |
			ADC_ATTEN_SPEED_SLOW_MASK)) {
		return ret;
	}
	/**
	* ADC settings
	* -> Master where master clock is 512xfs
	* -> data format is left justified 24 bit TDM
	*/
	if(pcm3168_reg_write(i2c_client_dev, PCM_ADC_CNTRL_ONE_REG,
		0x00 | ADC_MASTER_MODE_512xFS |
			ADC_LJ_24_BIT_TDM_MODE_MASK)) {
		return ret;
	}

	// Power up both DAC and ADC
	if(pcm3168_reg_write(i2c_client_dev, PCM_ADC_CNTRL_TWO_REG,
		0x00 | ADC_CHAN_0_1_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_4_5_NO_HPF_MASK)) {
		return ret;
	}

	if(pcm3168_reg_write(i2c_client_dev, PCM_DAC_CNTRL_TWO_REG,
		0x00 | DAC_CHAN_0_1_NORMAL_MODE_MASK |
			DAC_CHAN_2_3_NORMAL_MODE_MASK |
			DAC_CHAN_4_5_NORMAL_MODE_MASK |
			DAC_CHAN_6_7_NORMAL_MODE_MASK)) {
		return ret;
	}
	return 0;
}

int pcm3168a_codec_init(void)
{
	int ret;
	struct i2c_adapter *adapter = i2c_get_adapter(PCM3168A_I2C_BUS_NUM);
	struct i2c_client *client = i2c_new_device
					(adapter, i2c_clkgen_board_info);

	ret = gpio_request(PCM3168A_CODEC_RST_PIN, "CODEC_RST");
	if (ret < 0) {
		printk(KERN_ERR "pcm3168a-elk: Failed to get CODEC_RST_GPIO\n");
		return ret;
	}
	ret = gpio_request(PCM3168A_CPLD_RST_PIN, "SIKA_RST");
	if (ret < 0) {
		printk(KERN_ERR "pcm3168a-elk: Failed to get CPLD_RST\n");
		return ret;
	}
	gpio_direction_output(PCM3168A_CPLD_RST_PIN, 1);
	gpio_direction_output(PCM3168A_CODEC_RST_PIN, 0);
	if (pcm3168a_config_clk_gen(client))
		printk(KERN_ERR "pcm3168a-elk: clk generator config failed\n");
	msleep(200); // let the clk settle
	gpio_direction_output(PCM3168A_CODEC_RST_PIN, 1);
	msleep(5);
	i2c_unregister_device(client);
	client = i2c_new_device(adapter, i2c_pcm3168a_board_info);
	if (pcm3168a_config_codec(client)) {
		printk(KERN_ERR "pcm31681-elk: config_codec failed\n");
		return -1;
	}
	msleep(5);
	gpio_direction_output(PCM3168A_CPLD_RST_PIN, 0);
	i2c_unregister_device(client);
	i2c_put_adapter(adapter);
	printk(KERN_INFO "pcm31681-elk: codec configured\n");
	return 0;
}
EXPORT_SYMBOL_GPL(pcm3168a_codec_init);

void pcm3168a_codec_exit(void)
{
	printk(KERN_INFO "pcm31681-elk: unregister i2c-client\n");
	gpio_free(PCM3168A_CODEC_RST_PIN);
	gpio_free(PCM3168A_CPLD_RST_PIN);
}
EXPORT_SYMBOL_GPL(pcm3168a_codec_exit);

int pcm3168a_init(void)
{
	printk(KERN_INFO "pcm31681-elk: module init\n");
	return 0;
}

void pcm3168a_exit(void)
{
	printk(KERN_INFO "pcm31681-elk: module exit\n");
}

module_init(pcm3168a_init)
module_exit(pcm3168a_exit)

MODULE_DESCRIPTION("PCM3168A I2C codec driver for ELK Pi");
MODULE_AUTHOR("Nitin Kulkarni (nitin@elk.audio)");
MODULE_LICENSE("GPL");