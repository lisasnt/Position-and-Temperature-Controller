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

#define PTC_CHANNEL (ADC_CHANNEL_4)//F4 Version: (ADC_CHANNEL_11)


#ifndef CUSTOM_BOARD
#define PTC_20_DEG_VOLTAGE_VALUE 	2.431966789
#define PTC_40_DEG_VOLTAGE_VALUE 	2.581370676
#define PTC_60_DEG_VOLTAGE_VALUE 	2.727027328
#define PTC_MIN_DEG_VOLTAGE_VALUE	1.959776885
#define PTC_MAX_DEG_VOLTAGE_VALUE 	3.176082018
#else
#define PTC_20_DEG_VOLTAGE_VALUE 	2.167
#define PTC_40_DEG_VOLTAGE_VALUE 	2.3
#define PTC_60_DEG_VOLTAGE_VALUE 	2.43
#define PTC_MIN_DEG_VOLTAGE_VALUE	1.747
#define PTC_MAX_DEG_VOLTAGE_VALUE 	2.8
#endif

double get_ptc_volt(ADC_HandleTypeDef* adc, uint32_t timeout);
temperature_level  get_ptc_temp_zone(double ptc_volt);

#endif /* __PTC_H */
