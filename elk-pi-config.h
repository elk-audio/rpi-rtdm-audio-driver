// SPDX-License-Identifier: GPL-2.0
/*
* @brief config file to keep board specific configs (e.g. num of CV gates)
* @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
* Stockholm
*/
#ifndef ELK_PI_CONFIG_H
#define ELK_PI_CONFIG_H

#define ELK_PI_NUM_INPUT_CHANNELS	6
#define ELK_PI_NUM_OUTPUT_CHANNELS	8

// num channels sent by the codec
#define ELK_PI_NUM_CODEC_CHANNELS	8

#define ELK_PI_CODEC_FORMAT		INT24_LJ

#define ELK_PI_SAMPLING_RATE		48000

/* BCM2835_I2S_CVGATES_SUPPORT should be defined (through Make file or here or a Kconfig if this module is part of kernel tree) to enable cv gates support */

/**
 * Cv gate gpio pin definitions
 */
#define 	NUM_OF_CVGATE_OUTS	4
#define		NUM_OF_CVGATE_INS	2
#define		CVGATE_OUT1_PIN	17
#define		CVGATE_OUT2_PIN	27
#define		CVGATE_OUT3_PIN	22
#define		CVGATE_OUT4_PIN	23
#define		CVGATE_IN1_PIN		24
#define		CVGATE_IN2_PIN		25

#define CVGATE_OUTS_LIST CVGATE_OUT1_PIN, \
						CVGATE_OUT2_PIN, \
						CVGATE_OUT3_PIN, \
						CVGATE_OUT4_PIN

#define CVGATE_INS_LIST CVGATE_IN1_PIN, CVGATE_IN2_PIN

#endif