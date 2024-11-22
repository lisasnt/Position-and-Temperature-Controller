/*
 * ptc.c
 *
 * Created on: 22 mar 2022
 * Author: Lisa Santarossa
 */
#include "ptc.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"

double ptc_volt, ptc_adc_value;
char ptc_buff[BUFF_SIZE];

double get_ptc_volt(ADC_HandleTypeDef* adc, uint32_t timeout){

	ptc_adc_value = 0.0;

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = PTC_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1; //F4 Version: 1;

	sConfig.SingleDiff = ADC_SINGLE_ENDED; 				// Needed on H7
	sConfig.OffsetNumber = ADC_OFFSET_NONE;				// Needed on H7
	sConfig.Offset = 0;                      			// Needed on H7
	sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;		// Needed on H7
	sConfig.OffsetSaturation = DISABLE;					// Needed on H7

	sConfig.SamplingTime = ADC3_SAMPLETIME_6CYCLES_5; //F4 Version: ADC_SAMPLETIME_144CYCLES;
	if(HAL_ADC_ConfigChannel(adc, &sConfig) != HAL_OK){
		Error_Handler();
	}

	HAL_ADC_Start(adc);
	HAL_ADC_PollForConversion(adc, timeout);

	for (uint8_t i = 0; i < SAMPLES; i++){
			ptc_adc_value += HAL_ADC_GetValue(adc);
	}

	ptc_volt = (ptc_adc_value / SAMPLES) * GPIO_MAX_VOLTAGE / ADC_BIT_RESOLUTION;

//	sprintf(ptc_buff, "PTC ADC  %f [bit] \r\n", (ptc_adc_value / SAMPLES));
//	HAL_UART_Transmit(&huart3, (uint8_t *)ptc_buff, strlen(ptc_buff), 100);

	HAL_ADC_Stop(adc);

	sprintf(ptc_buff, "PTC %f [V] \r\n", ptc_volt);
	HAL_UART_Transmit(&huart3, (uint8_t*)ptc_buff, strlen(ptc_buff), 100);
	return ptc_volt;
}

temperature_level  get_ptc_temp_zone(double ptc_volt){
	temperature_level temp_zone = TEMP_ERROR;
	if( ptc_volt >= PTC_MIN_DEG_VOLTAGE_VALUE && ptc_volt < PTC_20_DEG_VOLTAGE_VALUE){
		temp_zone = COLD;
		sprintf(ptc_buff, "PT1000 TEMP ZONE: COLD  \r\n\r\n");
	} else if(ptc_volt >= PTC_20_DEG_VOLTAGE_VALUE && ptc_volt < PTC_40_DEG_VOLTAGE_VALUE){
		temp_zone = NORMAL;
		sprintf(ptc_buff, "PT1000 TEMP ZONE: NORMAL  \r\n\r\n");
	} else if(ptc_volt >= PTC_40_DEG_VOLTAGE_VALUE && ptc_volt < PTC_60_DEG_VOLTAGE_VALUE){
		temp_zone = HOT;
		sprintf(ptc_buff, "PT1000 TEMP ZONE: HOT  \r\n\r\n");
	} else if(ptc_volt >= PTC_60_DEG_VOLTAGE_VALUE && ptc_volt <= PTC_MAX_DEG_VOLTAGE_VALUE){
		temp_zone = DANGER;
		sprintf(ptc_buff, "PT1000 TEMP ZONE: DANGER  \r\n\r\n");
	} else {
		sprintf(ptc_buff, "PT1000 TEMP ZONE: ERROR  \r\n\r\n");
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)ptc_buff, strlen(ptc_buff), 100);
	return temp_zone;
}
