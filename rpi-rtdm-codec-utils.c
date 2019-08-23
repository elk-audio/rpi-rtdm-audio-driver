/**
 * @file rpi-rtdm-i2c.c
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief 
 * @version 0.1
 * @date 2019-06-11
 * 
 * @copyright Mind music labs (c) 2019
 * 
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>
#include <linux/interrupt.h>

#include "rpi-rtdm-codec-utils.h"

#define CODEC_RST_GPIO  16
#define SIKA_CPLD_RST 4
#define GPIO_TRIGGER_NONE		0x0 /* unspecified */
#define GPIO_TRIGGER_EDGE_RISING	0x1
#define GPIO_TRIGGER_EDGE_FALLING	0x2
#define GPIO_TRIGGER_LEVEL_HIGH		0x4
#define GPIO_TRIGGER_LEVEL_LOW		0x8
#define GPIO_TRIGGER_MASK		0xf

static struct i2c_adapter* i2c_device_adapter;
static struct i2c_client* i2c_client_device;

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

static int config_clk_gen(struct i2c_client* i2c_client_dev) {
	char cmd[2];
	int i, ret = 0;
	//printk("i2c_init: i2c_client_dev for clkgen: %p \n", i2c_client_dev);

	for (i = CLKGEN_CLK0_CNTRL_REG; i <= CLKGEN_CLK7_CNTRL_REG; i++) {
		cmd[0] = i;
		cmd[1] = CLKGEN_CLK_PWR_DWN_MASK;
		if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd,2))
		< 0) {
			printk("i2c_init: i2c_master_send failed to send cmd\n");
			return ret;
		}
	}
	msleep(50);
	for (i = 0; i < CLKGEN_NUM_OF_REGS; i++) {
		if (i2c_master_send(i2c_client_dev, &clkgen_reg_val_lookup[i][0]
		, 2) < 0) {
			printk("i2c_init: Faild to send look up\n");
		}
		msleep(20);
	}

	cmd[0] = CLKGEN_PLL_RESET_REG;
	cmd[1] = CLKGEN_PLL_RESET_MASK;

	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("i2c_init: i2c_master_send failed to send cmd\n");
		return ret;
	}
	msleep(50);
	cmd[0] = CLKGEN_OUTPUT_EN_CNTRL_REG;
	cmd[1] = CLKGEN_EN_OUTPUT_MASK;

	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("i2c_init: i2c_master_send failed to send cmd\n");
		return ret;
	}
	msleep(50);
	//printk("i2c_init: done with clk gen configuration\n");
	return 0;
}

static int rpi_rtdm_config_codec(struct i2c_client* i2c_client_dev) {
	char cmd[2];
	int ret;

	cmd[0] = PCM_DAC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | DAC_CHAN_0_1_DISABLED_MODE_MASK |
			DAC_CHAN_2_3_DISABLED_MODE_MASK |
			DAC_CHAN_4_5_DISABLED_MODE_MASK |
			DAC_CHAN_6_7_DISABLED_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd  to PCM_DAC_CNTRL_TWO_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_ADC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | ADC_CHAN_0_1_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_ENABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_ENABLE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_ADC_CNTRL_TWO_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_ONE_REG;
	cmd[1] = 0x00 | DAC_SLAVE_MODE_MASK | DAC_I2S_24_BIT_TDM_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_DAC_CNTRL_ONE_REG\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_THREE_REG;
	cmd[1] = 0x00 | DAC_MASTER_VOLUME_CONTROL_MODE_MASK |
			DAC_ATTEN_SPEED_SLOW_MASK |
			DAC_DEMPH_DISABLE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to DAC_CONTROL_3_REG_DATA\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_ADC_CONTROL_THREE_REG;
	cmd[1] = 0x00 | ADC_MASTER_VOLUME_CONTROL_MODE_MASK |
			ADC_ATTEN_SPEED_SLOW_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_ADC_CONTROL_THREE_REG\n");
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
			ADC_I2S_24_BIT_TDM_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_ADC_CNTRL_ONE_REG\n");
		return ret;
	}
	msleep(10);
	// Power up both DAC and ADC
	cmd[0] = PCM_ADC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | ADC_CHAN_0_1_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_2_3_POWER_SAVE_DISABLE_MASK |
			ADC_CHAN_4_5_POWER_SAVE_DISABLE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_ADC_CNTRL_TWO_REG to power up adcs\n");
		return ret;
	}
	msleep(10);
	cmd[0] = PCM_DAC_CNTRL_TWO_REG;
	cmd[1] = 0x00 | DAC_CHAN_0_1_NORMAL_MODE_MASK |
			DAC_CHAN_2_3_NORMAL_MODE_MASK |
			DAC_CHAN_4_5_NORMAL_MODE_MASK |
			DAC_CHAN_6_7_NORMAL_MODE_MASK;
	if ((ret = i2c_master_send(i2c_client_dev, (const char *)cmd, 2)) < 0) {
		printk("config_codec: i2c_master_send failed to send cmd to PCM_DAC_CNTRL_REG_TWO to power up dacs\n");
		return ret;
	}

	return 0;
}

int rpi_rtdm_codec_init(void) {
	int ret;
	// Setup device
	i2c_device_adapter = i2c_get_adapter(1);
	i2c_client_device = i2c_new_device(i2c_device_adapter, i2c_clkgen_board_info);

	if (config_clk_gen(i2c_client_device))
		printk("i2c_init::config_clk_gen failed\n");

	if ((ret = gpio_request(CODEC_RST_GPIO, "CODEC_RST")) < 0) {
		printk("i2c_init: Failed to get CODEC_RST_GPIO\n");
		return ret;
	}
	if ((ret = gpio_request(SIKA_CPLD_RST, "SIKA_RST")) < 0) {
		printk("i2c_init: Failed to get SIKA_RST\n");
		return ret;
	}
	gpio_direction_output(SIKA_CPLD_RST, 0);
	gpio_direction_output(CODEC_RST_GPIO, 0);
	msleep(50);
	gpio_direction_output(CODEC_RST_GPIO, 1);
	i2c_unregister_device(i2c_client_device);
	i2c_client_device = i2c_new_device(i2c_device_adapter, i2c_pcm3168a_board_info);

	if (rpi_rtdm_config_codec(i2c_client_device))
		printk("i2c_init::config_codec failed\n");

	return 0;
}

static rtdm_irq_t  bcm_rtdm_irq;

int irq_handler(rtdm_irq_t *irq_handle)
{
  // We got IRQ !
   rtdm_printk("RPI_GPIO RTDM, INterrupt !\n");

  return RTDM_IRQ_HANDLED;
}

static int request_gpio_irq(unsigned int gpio, int trigger)
{
	int ret, irq_trigger, irq;

	ret = gpio_request(gpio, "gpio-in");
		if (ret) {
			if (ret != -EPROBE_DEFER)
				printk("can not request GPIO%d\n", gpio);
			return ret;
		}

	ret = gpio_direction_input(gpio);
	if (ret) {
		printk("cannot set GPIO%d as input\n", gpio);
		return ret;
	}

	gpio_export(gpio, true);
	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		printk("irq < 0 ! \n");
		return -1;
	}

	irq_trigger = 0;
	if (trigger & GPIO_TRIGGER_EDGE_RISING)
		irq_trigger |= IRQ_TYPE_EDGE_RISING;
	if (trigger & GPIO_TRIGGER_EDGE_FALLING)
		irq_trigger |= IRQ_TYPE_EDGE_FALLING;
	if (trigger & GPIO_TRIGGER_LEVEL_HIGH)
		irq_trigger |= IRQ_TYPE_LEVEL_HIGH;
	if (trigger & GPIO_TRIGGER_LEVEL_LOW)
		irq_trigger |= IRQ_TYPE_LEVEL_LOW;

	if (irq_trigger)
		irq_set_irq_type(irq, irq_trigger);
	
	ret = rtdm_irq_request(&bcm_rtdm_irq, irq, irq_handler,
			       0, "rt-gpio", THIS_MODULE->name);
	if (ret) {
		printk( "cannot request GPIO%d interrupt\n", gpio);
		return ret;
	}

	rtdm_irq_enable(&bcm_rtdm_irq);
	printk("GPIO RTDM irq setup successfully!\n");
	return 0;
}

/* int rpi_rtdm_gpio_test(void) {
	int ret;
	request_gpio_irq(TEST_GPIO_IN, GPIO_TRIGGER_EDGE_RISING);

	if ((ret = gpio_request(CODEC_RST_GPIO, "CODEC_RST")) < 0) {
		printk("i2c_init: Failed to get CODEC_RST_GPIO\n");
		return ret;
	}
	gpio_direction_output(CODEC_RST_GPIO, 0);
	msleep(50);
	gpio_direction_output(CODEC_RST_GPIO, 1);

	return 0;
} */

int rpi_rtdm_codec_exit(void) {
	printk(KERN_INFO "i2c-exit: unregister i2c-client\n");
	/* i2c_unregister_device(i2c_client_device);
	gpio_free(CODEC_RST_GPIO); */
	return 0;
}