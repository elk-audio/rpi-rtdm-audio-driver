// SPDX-License-Identifier: GPL-2.0
/*
* @brief config file to keep board specific configs (e.g. num of CV gates)
* @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk,
* Stockholm
*/
#ifndef ELK_PI_CONFIG_H
#define ELK_PI_CONFIG_H

/* Disable this if you want to build without cv gates support */
#define CONFIG_CVGATES_SUPPORT

/**
 * Cv gate gpio pin definitions
 */
#define 	NUM_OF_CVGATE_OUTS	4
#define		NUM_OF_CVGATE_INS	2
#define		CV_GATE_OUT1_PIN	17
#define		CV_GATE_OUT2_PIN	27
#define		CV_GATE_OUT3_PIN	22
#define		CV_GATE_OUT4_PIN	23
#define		CV_GATE_IN1_PIN		24
#define		CV_GATE_IN2_PIN		25

#define CVGATE_OUTS_LIST CV_GATE_OUT1_PIN, \
                        CV_GATE_OUT2_PIN, \
                        CV_GATE_OUT3_PIN, \
                        CV_GATE_OUT4_PIN

#define CVGATE_INS_LIST CV_GATE_IN1_PIN, CV_GATE_IN2_PIN

#endif