/**
 * @file rpi-rtdm-audio.h
 * @author Nitin Kulkarni (nitin.kulkarni@mindmusiclabs.com)
 * @brief 
 * @version 0.1
 * @date 2019-06-11
 * 
 * @copyright MIND music labs (c) 2019
 * 
 */
#ifndef RPI_RTDM_AUDIO_H
#define RPI_RTDM_AUDIO_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define RTDM_SUBCLASS_GPIO	0
#define DEVICE_NAME		"audio_rtdm"
#define RTAUDIO_PROFILE_VER	1
#define AUDIO_RTDM_VERSION_MAJ 	0
#define AUDIO_RTDM_VERSION_MIN	2
#define AUDIO_RTDM_VERSION_VER	0

#define DEFAULT_AUDIO_N_CHANNELS			8
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		64
#define NUM_OF_WORDS (DEFAULT_AUDIO_N_CHANNELS * \
DEFAULT_AUDIO_N_FRAMES_PER_BUFFER)
#define NUM_OF_PAGES					20

#define AUDIO_IOC_MAGIC		'r'


#define AUDIO_IRQ_WAIT			_IO(AUDIO_IOC_MAGIC, 1)
#define AUDIO_IMMEDIATE_SEND		_IOW(AUDIO_IOC_MAGIC, 2, int)
#define AUDIO_PROC_START		_IO(AUDIO_IOC_MAGIC, 3)
#define AUDIO_USERPROC_FINISHED		_IOW(AUDIO_IOC_MAGIC, 4, int)
#define AUDIO_PROC_STOP			_IO(AUDIO_IOC_MAGIC, 5)
#endif