/**
 * @file rpi-rtdm-i2c.h
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief 
 * @version 0.1
 * 
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 * 
 */
#ifndef RPI_RTDM_CODEC_UTILS_H
#define RPI_RTDM_CODEC_UTILS_H

/* Clock generator related definitions */

#define CLKGEN_CLK0_CNTRL_REG 		16
#define CLKGEN_CLK7_CNTRL_REG 		23
#define CLKGEN_CLK_PWR_DWN_MASK 	0x80
#define CLKGEN_NUM_OF_REGS 		43
#define CLKGEN_PLL_RESET_REG 		177
#define CLKGEN_PLL_RESET_MASK 		0xA0
#define CLKGEN_OUTPUT_EN_CNTRL_REG 	3
#define CLKGEN_EN_OUTPUT_MASK 		0x00

/* Codec related definitions */
/* Codec Reg addr */
#define PCM_RST_CNTRL_REG		0x40
#define PCM_DAC_CNTRL_ONE_REG		0x41
#define PCM_DAC_CNTRL_TWO_REG		0x42
#define PCM_DAC_OUTPUT_PHASE_REG	0x43
#define PCM_DAC_MUTE_CNTRL_REG		0x44
#define PCM_DAC_CNTRL_THREE_REG		0x46
#define PCM_DAC_VOL_CNTRL_REG		0x47
#define PCM_ADC_SAMPLING_MODE_REG	0x50
#define PCM_ADC_CNTRL_ONE_REG		0x51
#define PCM_ADC_CNTRL_TWO_REG		0x52
#define PCM_ADC_INPUT_CONFIG_REG	0x53
#define PCM_ADC_INPUT_PHASE_REG		0x54
#define PCM_ADC_SOFTMUTE_REG		0x55
#define PCM_ADC_CONTROL_THREE_REG	0x57
#define PCM_ADC_VOL_CNTRL_REG		0x58

/* ResetControl Masks*/
#define PCM_MODE_CTRL_RESET_MASK		0x00
#define PCM_SYSTEM_RESET_MASK			0x00
#define DAC_SAMPLING_MODE_AUTO_MASK		0x00
#define DAC_SAMPLING_MODE_SINGLE_MASK		0x01
#define DAC_SAMPLING_MODE_DUAL_MASK		0x02
#define DAC_SAMPLING_MODE_QUAD_MASK		0x03

/* DAC Control one masks */
// bit 7
#define DAC_POWER_SAVE_ENABLE_MODE_MASK		0x00
#define DAC_POWER_SAVE_DISABLE_MODE_MASK	0x80
// bit 6:4
#define DAC_SLAVE_MODE_MASK			0x00
#define DAC_MASTER_MODE_768xFS			0x10
#define DAC_MASTER_MODE_512xFS			0x20
#define DAC_MASTER_MODE_384xFS			0x30
#define DAC_MASTER_MODE_256xFS			0x40
#define DAC_MASTER_MODE_192xFS			0x50
#define DAC_MASTER_MODE_128xFS			0x60
// bits 3:0
#define DAC_I2S_24_BIT_MODE_MASK		0x00 
#define DAC_LJ_24_BIT_MODE_MASK			0x01
#define DAC_RJ_24_BIT_MODE_MASK			0x02
#define DAC_RJ_16_BIT_MODE_MASK			0x03
#define DAC_I2S_24_BIT_DSP_MODE_MASK		0x04
#define DAC_LJ_24_BIT_DSP_MODE_MASK		0x05
#define DAC_I2S_24_BIT_TDM_MODE_MASK		0x06
#define DAC_LJ_24_BIT_TDM_MODE_MASK		0x07
#define DAC_I2S_24_BIT_HS_TDM_MODE_MASK		0x08
#define DAC_LJ_24_BIT_HS_TDM_MODE_MASK		0x09

/* DAC Control two masks */
#define DAC_CHAN_0_1_NORMAL_MODE_MASK		0x00
#define DAC_CHAN_0_1_DISABLED_MODE_MASK		0x10
#define DAC_CHAN_2_3_NORMAL_MODE_MASK		0x00
#define DAC_CHAN_2_3_DISABLED_MODE_MASK		0x20
#define DAC_CHAN_4_5_NORMAL_MODE_MASK		0x00
#define DAC_CHAN_4_5_DISABLED_MODE_MASK		0x40
#define DAC_CHAN_6_7_NORMAL_MODE_MASK		0x00
#define DAC_CHAN_6_7_DISABLED_MODE_MASK		0x80
#define DAC_CHAN_0_1_SHARP_ROLLOFF_ENABLE_MASK	0x00
#define DAC_CHAN_0_1_SLOW_ROLLOFF_ENABLE_MASK	0x01
#define DAC_CHAN_2_3_SHARP_ROLLOFF_ENABLE_MASK	0x00
#define DAC_CHAN_2_3_SLOW_ROLLOFF_ENABLE_MASK	0x02
#define DAC_CHAN_4_5_SHARP_ROLLOFF_ENABLE_MASK	0x00
#define DAC_CHAN_4_5_SLOW_ROLLOFF_ENABLE_MASK	0x04
#define DAC_CHAN_6_7_SHARP_ROLLOFF_ENABLE_MASK	0x00
#define DAC_CHAN_6_7_SLOW_ROLLOFF_ENABLE_MASK	0x08

/* DAC Output phase masks */
#define DAC_CHAN_0_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_0_PHASE_INVERT_MASK		0x01
#define DAC_CHAN_1_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_1_PHASE_INVERT_MASK		0x02
#define DAC_CHAN_2_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_2_PHASE_INVERT_MASK		0x04
#define DAC_CHAN_3_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_3_PHASE_INVERT_MASK		0x08
#define DAC_CHAN_4_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_4_PHASE_INVERT_MASK		0x10
#define DAC_CHAN_5_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_5_PHASE_INVERT_MASK		0x20
#define DAC_CHAN_6_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_6_PHASE_INVERT_MASK		0x40
#define DAC_CHAN_7_PHASE_NO_INVERT_MASK		0x00
#define DAC_CHAN_7_PHASE_INVERT_MASK		0x80

/* DAC mute control masks */
#define DAC_CHAN_0_NO_MUTE_MASK			0x00
#define DAC_CHAN_0_MUTE_MASK			0x01
#define DAC_CHAN_1_NO_MUTE_MASK			0x00
#define DAC_CHAN_1_MUTE_MASK			0x02
#define DAC_CHAN_2_NO_MUTE_MASK			0x00
#define DAC_CHAN_2_MUTE_MASK			0x04
#define DAC_CHAN_3_NO_MUTE_MASK			0x00
#define DAC_CHAN_3_MUTE_MASK			0x08
#define DAC_CHAN_4_NO_MUTE_MASK			0x00
#define DAC_CHAN_4_MUTE_MASK			0x10
#define DAC_CHAN_5_NO_MUTE_MASK			0x00
#define DAC_CHAN_5_MUTE_MASK			0x20
#define DAC_CHAN_6_NO_MUTE_MASK			0x00
#define DAC_CHAN_6_MUTE_MASK			0x40
#define DAC_CHAN_7_NO_MUTE_MASK			0x00
#define DAC_CHAN_7_MUTE_MASK			0x80

/* DAC Control three masks */
#define DAC_SINGLE_CHANNEL_VOLUME_CONTROL_MODE_MASK	0x00
#define DAC_MASTER_VOLUME_CONTROL_MODE_MASK		0x80
#define DAC_ATTEN_SPEED_FAST_MASK			0x00
#define DAC_ATTEN_SPEED_SLOW_MASK			0x40
#define DAC_DEMPH_DISABLE_MASK				0x00
#define DAC_DEMPH_ENABLE_48K_MASK			0x10
#define DAC_DEMPH_ENABLE_44_1K_MASK			0x20
#define DAC_DEMPH_ENABLE_32K_MASK			0x30

/* ADCSamplingMode */
#define ADC_SAMPLING_MODE_AUTO_MASK			0x00
#define ADC_SAMPLING_MODE_SINGLE_RATE_MASK		0x01
#define ADC_SAMPLING_MODE_DUAL_RATE_MASK		0x02

/* ADCControl1 */
#define ADC_SLAVE_MODE_MASK				0x00
#define ADC_MASTER_MODE_768xFS				0x10
#define ADC_MASTER_MODE_512xFS				0x20
#define ADC_MASTER_MODE_384xFS				0x30
#define ADC_MASTER_MODE_256xFS				0x40
#define ADC_I2S_24_BIT_MODE_MASK			0x00
#define ADC_LJ_24_BIT_MODE_MASK				0x01
#define ADC_RJ_24_BIT_MODE_MASK				0x02
#define ADC_RJ_16_BIT_MODE_MASK				0x03
#define ADC_I2S_24_BIT_DSP_MODE_MASK			0x04
#define ADC_LJ_24_BIT_DSP_MODE_MASK			0x05
#define ADC_I2S_24_BIT_TDM_MODE_MASK			0x06
#define ADC_LJ_24_BIT_TDM_MODE_MASK			0x07

/* ADCControl2 */
#define ADC_CHAN_0_1_POWER_SAVE_DISABLE_MASK		0x00
#define ADC_CHAN_0_1_POWER_SAVE_ENABLE_MASK		0x10
#define ADC_CHAN_2_3_POWER_SAVE_DISABLE_MASK		0x00
#define ADC_CHAN_2_3_POWER_SAVE_ENABLE_MASK		0x20
#define ADC_CHAN_4_5_POWER_SAVE_DISABLE_MASK		0x00
#define ADC_CHAN_4_5_POWER_SAVE_ENABLE_MASK		0x40
#define ADC_CHAN_0_1_HPF_MASK				0x00
#define ADC_CHAN_0_1_NO_HPF_MASK			0x01
#define ADC_CHAN_2_3_HPF_MASK				0x00
#define ADC_CHAN_2_3_NO_HPF_MASK			0x02
#define ADC_CHAN_4_5_HPF_MASK				0x00
#define ADC_CHAN_4_5_NO_HPF_MASK			0x04

/* ADCInputConfig */
#define ADC_CHAN_0_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_0_SINGLE_ENDED_INPUT_MASK		0x01
#define ADC_CHAN_1_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_1_SINGLE_ENDED_INPUT_MASK		0x02
#define ADC_CHAN_2_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_2_SINGLE_ENDED_INPUT_MASK		0x04
#define ADC_CHAN_3_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_3_SINGLE_ENDED_INPUT_MASK		0x08
#define ADC_CHAN_4_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_4_SINGLE_ENDED_INPUT_MASK		0x10
#define ADC_CHAN_5_DIFF_INPUT_MASK			0x00
#define ADC_CHAN_5_SINGLE_ENDED_INPUT_MASK		0x20

/* ADCInputPhase */
#define ADC_CHAN_0_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_0_PHASE_INVERT_MASK			0x01
#define ADC_CHAN_1_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_1_PHASE_INVERT_MASK			0x02
#define ADC_CHAN_2_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_2_PHASE_INVERT_MASK			0x04
#define ADC_CHAN_3_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_3_PHASE_INVERT_MASK			0x08
#define ADC_CHAN_4_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_4_PHASE_INVERT_MASK			0x10
#define ADC_CHAN_5_PHASE_NO_INVERT_MASK			0x00
#define ADC_CHAN_5_PHASE_INVERT_MASK			0x20

/* ADCSoftMute */
#define ADC_CHAN_0_NO_MUTE_MASK				0x00
#define ADC_CHAN_0_MUTE_MASK				0x00
#define ADC_CHAN_1_NO_MUTE_MASK				0x00
#define ADC_CHAN_1_MUTE_MASK				0x00
#define ADC_CHAN_2_NO_MUTE_MASK				0x00
#define ADC_CHAN_2_MUTE_MASK				0x00
#define ADC_CHAN_3_NO_MUTE_MASK				0x00
#define ADC_CHAN_3_MUTE_MASK				0x08
#define ADC_CHAN_4_NO_MUTE_MASK				0x00
#define ADC_CHAN_4_MUTE_MASK				0x10
#define ADC_CHAN_5_NO_MUTE_MASK				0x00
#define ADC_CHAN_5_MUTE_MASK				0x20

/* ADCControl3 */
#define ADC_SINGLE_CHANNEL_VOLUME_CONTROL_MODE_MASK	0x00
#define ADC_MASTER_VOLUME_CONTROL_MODE_MASK		0x80
#define ADC_ATTEN_SPEED_FAST_MASK			0x00
#define ADC_ATTEN_SPEED_SLOW_MASK			0x40

extern int pcm3168a_codec_init(void);
extern void pcm3168a_codec_exit(void);

#endif