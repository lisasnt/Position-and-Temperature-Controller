/*
 * ptc.c
 *
 * 	Created on: 22 mar 2022
 * 		Author: Lisa Santarossa
 */

#ifndef __PTC_H
#define __PTC_H

#include "adc.h"
#include <inttypes.h>
#include "temp.h"

#define PTC_CHANNEL (ADC_CHANNEL_11)


#define PTC_20_DEG_VOLTAGE_VALUE 	2.167381
#define PTC_40_DEG_VOLTAGE_VALUE 	2.300531
#define PTC_60_DEG_VOLTAGE_VALUE 	2.430341
#define PTC_MIN_DEG_VOLTAGE_VALUE	1.746563
#define PTC_MAX_DEG_VOLTAGE_VALUE 	2.830541

double get_ptc_volt(ADC_HandleTypeDef* adc, uint32_t timeout);
temperature_level  get_ptc_temp_zone(double ptc_volt);

#endif /* __PTC_H */
