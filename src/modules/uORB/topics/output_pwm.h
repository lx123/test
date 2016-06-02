

#pragma once

#include <stdint.h>
#ifdef __cplusplus
#include <cstring>
#else
#include <string.h>
#endif

#include <uORB/uORB.h>


#ifndef __cplusplus
#define PWM_OUTPUT_MAX_CHANNELS 16

#endif


#ifdef __cplusplus
struct __EXPORT output_pwm_s {
#else
struct output_pwm_s {
#endif
	uint64_t timestamp; // required for logger
	uint32_t channel_count;
	uint16_t values[16];
	uint8_t _padding0[4]; // required for logger

#ifdef __cplusplus
	static const uint8_t PWM_OUTPUT_MAX_CHANNELS = 16;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(output_pwm);
