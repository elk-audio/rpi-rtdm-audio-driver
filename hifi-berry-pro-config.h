// SPDX-License-Identifier: GPL-2.0
/*
* @brief config file to keep board specific configs (e.g. num of CV gates)
* @copyright 2017-2020 Modern Ancient Instruments Networked AB, dba Elk,
* Stockholm
*/
#ifndef HIFI_BERRY_PRO_CONFIG_H_
#define HIFI_BERRY_PRO_CONFIG_H_

#define HIFI_BERRY_PRO_NUM_INPUT_CHANNELS	2
#define HIFI_BERRY_PRO_NUM_OUTPUT_CHANNELS	2

// num channels sent by the codec
#define HIFI_BERRY_PRO_NUM_CODEC_CHANNELS	2

#define HIFI_BERRY_PRO_CODEC_FORMAT		INT24_LJ

#define HIFI_BERRY_PRO_SAMPLING_RATE		48000

#define HIFI_BERRY_PRO_DAC_MODE			PCM5122_MASTER_MODE

#endif