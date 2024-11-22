/*
 * ntc.h
 *
 *  Created on: 22 mar 2022
 *      Author: Tommaso Canova
 */

#ifndef INC_NTC_H_
#define INC_NTC_H_

#include "adc.h"
#include "usart.h"
#include "temp.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define NTC_CHANNEL (ADC_CHANNEL_9)//F4 Version :(ADC_CHANNEL_8)

#ifndef CUSTOM_BOARD
#define NTC_20_DEG_VOLTAGE_VALUE 		2.203456
#define NTC_40_DEG_VOLTAGE_VALUE 		1.482085
#define NTC_60_DEG_VOLTAGE_VALUE 		0.933597
#define NTC_MINUS40_DEG_VOLTAGE_VALUE 	3.823179
#define NTC_125_DEG_VOLTAGE_VALUE 		0.204090
#else
#define NTC_20_DEG_VOLTAGE_VALUE 		2.36
#define NTC_40_DEG_VOLTAGE_VALUE 		1.482085
#define NTC_60_DEG_VOLTAGE_VALUE 		0.933597
#define NTC_MINUS40_DEG_VOLTAGE_VALUE 	3.823179
#define NTC_125_DEG_VOLTAGE_VALUE 		0.204090
#endif


double get_ntc_volt(ADC_HandleTypeDef* adc, uint32_t timeout);

temperature_level get_ntc_temp_zone(double ntc_volt);

#endif /* INC_NTC_H_ */
