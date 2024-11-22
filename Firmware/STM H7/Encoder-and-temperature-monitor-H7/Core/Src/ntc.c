/*
 * ntc.c
 *
 *  Created on: 22 mar 2022
 *      Author: Tommaso Canova
 */

#include "ntc.h"

char ntc_buff [BUFF_SIZE];
double ntc_volt, ntc_adc_value;

double get_ntc_volt(ADC_HandleTypeDef* adc, uint32_t timeout){

	ntc_adc_value = 0.0;

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = NTC_CHANNEL;
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
		ntc_adc_value += HAL_ADC_GetValue(adc);
	}

//	sprintf(ntc_buff, "NTC ADC %f [bit] \r\n", ntc_adc_value / SAMPLES );
//	HAL_UART_Transmit(&huart3, (uint8_t *)ntc_buff, strlen(ntc_buff), 100);

	ntc_volt = (ntc_adc_value / SAMPLES) * GPIO_MAX_VOLTAGE / ADC_BIT_RESOLUTION;

	sprintf(ntc_buff, "NTC %f [V] \r\n",ntc_volt);
	HAL_UART_Transmit(&huart3, (uint8_t *)ntc_buff, strlen(ntc_buff), 100);
	HAL_ADC_Stop(adc);

	return ntc_volt;
}

temperature_level get_ntc_temp_zone(double ntc_volt){

	temperature_level temp_zone = TEMP_ERROR;

	if(ntc_volt >= NTC_20_DEG_VOLTAGE_VALUE && ntc_volt <= GPIO_MAX_VOLTAGE){
		temp_zone = COLD;
		sprintf(ntc_buff, "NTC TEMP ZONE: COLD \r\n\r\n");
	}else if(ntc_volt >= NTC_40_DEG_VOLTAGE_VALUE && ntc_volt < NTC_20_DEG_VOLTAGE_VALUE){
		temp_zone = NORMAL;
		sprintf(ntc_buff, "NTC TEMP ZONE: NORMAL \r\n\r\n");
	}else if(ntc_volt >= NTC_60_DEG_VOLTAGE_VALUE && ntc_volt < NTC_40_DEG_VOLTAGE_VALUE){
		temp_zone = HOT;
		sprintf(ntc_buff, "NTC TEMP ZONE: HOT \r\n\r\n");
	}else if(ntc_volt >= NTC_125_DEG_VOLTAGE_VALUE && ntc_volt < NTC_60_DEG_VOLTAGE_VALUE){
		temp_zone = DANGER;
		sprintf(ntc_buff, "NTC TEMP ZONE: DANGER \r\n\r\n");
	}else{
		sprintf(ntc_buff, "NTC TEMP ZONE: NTC ERROR \r\n\r\n");
	}

	HAL_UART_Transmit(&huart3, (uint8_t *)ntc_buff, strlen(ntc_buff), 100);

	return temp_zone;
}


