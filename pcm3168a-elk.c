/**
 * @file rpi-rtdm-i2c.c
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief The PCM168a codec driver for ELK  PI. A lot of stuff is hardcoded for *	now, idea is to make it runtime configurable ideally. This module is  	*	based on the mainline pcm3168a driver by Damien Horsley.
 * @version 0.1
 * 
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 * 
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>
#include <linux/interrupt.h>

#include "pcm3168a-elk.h"

#define PCM3168A_CODEC_RST_PIN  16
#define PCM3168A_CPLD_RST_PIN 	4
#define PCM3168A_I2C_BUS_NUM	1

static struct i2c_client* i2c_device_client;

static uint8_t clkgen_reg_val_lookup[CLKGEN_NUM_OF_REGS][2] =
	{
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

static int pcm3168a_config_clk_gen(struct i2c_client* i2c_client_dev) {
	char cmd[2];
	int i, ret = 0;

	for (i = CLKGEN_CLK0_CNTRL_REG; i <= CLKGEN_CLK7_CNTRL_REG; i++) {
		cmd[0] = i;
		cmd[1] = CLKGEN_CLK_PWR_DWN_MASK;
		if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd,2))
		< 0) {
			printk("audio_rtdm: i2c_master_send failed to send cmd\n");
			return ret;
		}
	}
	msleep(50);
	for (i = 0; i < CLKGEN_NUM_OF_REGS; i++) {
		if (i2c_master_send(i2c_client_dev, &clkgen_reg_val_lookup[i][0]
		, 2) < 0) {
			printk("audio_rtdm: Faild to send look up\n");
		}
		msleep(20);
	}

	cmd[0] = CLKGEN_PLL_RESET_REG;
	cmd[1] = CLKGEN_PLL_RESET_MASK;

	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd\n");
		return ret;
	}
	msleep(50);
	cmd[0] = CLKGEN_OUTPUT_EN_CNTRL_REG;
	cmd[1] = CLKGEN_EN_OUTPUT_MASK;

	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd\n");
		return ret;
	}
	return 0;
}

static int pcm3168a_config_codec(struct i2c_client* i2c_client_dev) {
	char cmd[2];
	int ret;

	cmd[0] = PCM_DAC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | DAC_CHAN_0_1_DISABLED_MODE_MASK |
			DAC_CHAN_2_3_DISABLED_MODE_MASK |
			DAC_CHAN_4_5_DISABLED_MODE_MASK |
			DAC_CHAN_6_7_DISABLED_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd  to \
		PCM_DAC_CNTRL_TWO_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_ADC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | ADC_CHAN_0_1_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_ENABLE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd to \
		PCM_ADC_CNTRL_TWO_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_ONE_REG;
	cmd[1] = 0x00 | DAC_SLAVE_MODE_MASK | DAC_LJ_24_BIT_TDM_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd to \
		PCM_DAC_CNTRL_ONE_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_THREE_REG;
	cmd[1] = 0x00 | DAC_MASTER_VOLUME_CONTROL_MODE_MASK |
			DAC_ATTEN_SPEED_SLOW_MASK |
			DAC_DEMPH_DISABLE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to \
		DAC_CONTROL_3_REG_DATA\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_ADC_CONTROL_THREE_REG;
	cmd[1] = 0x00 | ADC_MASTER_VOLUME_CONTROL_MODE_MASK |
			ADC_ATTEN_SPEED_SLOW_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("audio_rtdm: i2c_master_send failed to send cmd to \
		PCM_ADC_CONTROL_THREE_REG\n");
		return ret;
	}
	msleep(10);
	/**
	* ADC settings
	* -> Master where master clock is 512xfs
	* -> data format is left justified 24 bit TDM
	*/
	cmd[0] = PCM_ADC_CNTRL_ONE_REG;
	cmd[1] = 0x00 | ADC_MASTER_MODE_512xFS |
			ADC_LJ_24_BIT_TDM_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk(KERN_ERR "audio_rtdm: i2c_master_send failed for \
		PCM_ADC_CNTRL_ONE_REG\n");
		return ret;
	}
	msleep(10);
	// Power up both DAC and ADC
	cmd[0] = PCM_ADC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | ADC_CHAN_0_1_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_4_5_NO_HPF_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk(KERN_ERR "audio_rtdm: i2c_master_send failed to send cmd \
		to power up adcs\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | DAC_CHAN_0_1_NORMAL_MODE_MASK |
			DAC_CHAN_2_3_NORMAL_MODE_MASK |
			DAC_CHAN_4_5_NORMAL_MODE_MASK |
			DAC_CHAN_6_7_NORMAL_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk(KERN_ERR "audio_rtdm: i2c_master_send failed to send cmd to \
		PCM_DAC_CNTRL_REG_TWO to power up dacs\n");
		return ret;
	}

	return 0;
}

int pcm3168a_codec_init(void) {
	int ret;
	struct i2c_adapter *adapter = i2c_get_adapter(PCM3168A_I2C_BUS_NUM);
	struct i2c_client *client = i2c_new_device
					(adapter, i2c_clkgen_board_info);

	if (pcm3168a_config_clk_gen(client))
		printk(KERN_ERR "pcm3168a-elk: rpi_config_clk_gen failed\n");

	if ((ret = gpio_request(PCM3168A_CODEC_RST_PIN, "CODEC_RST")) < 0) {
		printk(KERN_ERR "pcm3168a-elk: Failed to get CODEC_RST_GPIO\n");
		return ret;
	}
	if ((ret = gpio_request(PCM3168A_CPLD_RST_PIN, "SIKA_RST")) < 0) {
		printk(KERN_ERR "pcm3168a-elk: Failed to get SIKA_RST\n");
		return ret;
	}
	gpio_direction_output(PCM3168A_CPLD_RST_PIN, 1);
	gpio_direction_output(PCM3168A_CODEC_RST_PIN, 0);
	msleep(5);
	gpio_direction_output(PCM3168A_CODEC_RST_PIN, 1);
	msleep(5);
	i2c_unregister_device(client);
	client = i2c_new_device(adapter,i2c_pcm3168a_board_info);
	if (pcm3168a_config_codec(client)) {
		printk(KERN_ERR "pcm31681-elk: config_codec failed\n");
		return -1;
	}
	msleep(5);
	gpio_direction_output(PCM3168A_CPLD_RST_PIN, 0);
	printk(KERN_INFO "pcm31681-elk: codec configured\n");
	return 0;
}
EXPORT_SYMBOL_GPL(pcm3168a_codec_init);

void pcm3168a_codec_exit(void) {
	printk(KERN_INFO "pcm31681-elk: unregister i2c-client\n");
	i2c_unregister_device(i2c_device_client);
	gpio_free(PCM3168A_CODEC_RST_PIN);
	gpio_free(PCM3168A_CPLD_RST_PIN);
}
EXPORT_SYMBOL_GPL(pcm3168a_codec_exit);

int pcm3168a_init(void) {
	printk(KERN_INFO "pcm31681-elk: module init\n");
	return 0;
}

void pcm3168a_exit(void) {
	printk(KERN_INFO "pcm31681-elk: module exit\n");
}

module_init(pcm3168a_init)
module_exit(pcm3168a_exit)

MODULE_DESCRIPTION("PCM3168A I2C codec driver for ELK Pi");
MODULE_AUTHOR("Nitin Kulkarni");
MODULE_LICENSE("GPL v2");