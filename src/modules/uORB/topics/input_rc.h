
#pragma once

#include <stdint.h>
#ifdef __cplusplus
#include <cstring>
#else
#include <string.h>
#endif

#include <uORB/uORB.h>


#ifndef __cplusplus
#define RC_INPUT_SOURCE_UNKNOWN 0
#define RC_INPUT_SOURCE_PX4FMU_PPM 1
#define RC_INPUT_SOURCE_PX4IO_PPM 2
#define RC_INPUT_SOURCE_PX4IO_SPEKTRUM 3
#define RC_INPUT_SOURCE_PX4IO_SBUS 4
#define RC_INPUT_SOURCE_PX4IO_ST24 5
#define RC_INPUT_SOURCE_MAVLINK 6
#define RC_INPUT_SOURCE_QURT 7
#define RC_INPUT_SOURCE_PX4FMU_SPEKTRUM 8
#define RC_INPUT_SOURCE_PX4FMU_SBUS 9
#define RC_INPUT_SOURCE_PX4FMU_ST24 10
#define RC_INPUT_SOURCE_PX4FMU_SUMD 11
#define RC_INPUT_SOURCE_PX4FMU_DSM 12
#define RC_INPUT_SOURCE_PX4IO_SUMD 13
#define RC_INPUT_MAX_CHANNELS 18

#endif


#ifdef __cplusplus
struct __EXPORT input_rc_s {
#else
struct input_rc_s {
#endif
	uint64_t timestamp; // required for logger
	uint64_t timestamp_publication;
	uint64_t timestamp_last_signal;
	uint32_t channel_count;
	int32_t rssi;
	uint16_t rc_lost_frame_count;
	uint16_t rc_total_frame_count;
	uint16_t rc_ppm_frame_length;
	uint16_t values[18];
	bool rc_failsafe;
	bool rc_lost;
	uint8_t input_source;
	uint8_t _padding0[3]; // required for logger

#ifdef __cplusplus
	static const uint8_t RC_INPUT_SOURCE_UNKNOWN = 0;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_PPM = 1;
	static const uint8_t RC_INPUT_SOURCE_PX4IO_PPM = 2;
	static const uint8_t RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3;
	static const uint8_t RC_INPUT_SOURCE_PX4IO_SBUS = 4;
	static const uint8_t RC_INPUT_SOURCE_PX4IO_ST24 = 5;
	static const uint8_t RC_INPUT_SOURCE_MAVLINK = 6;
	static const uint8_t RC_INPUT_SOURCE_QURT = 7;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_SBUS = 9;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_ST24 = 10;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_SUMD = 11;
	static const uint8_t RC_INPUT_SOURCE_PX4FMU_DSM = 12;
	static const uint8_t RC_INPUT_SOURCE_PX4IO_SUMD = 13;
	static const uint8_t RC_INPUT_MAX_CHANNELS = 18;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(input_rc);
