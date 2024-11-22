/*
 * temp.h
 *
 *  Created on: Mar 23, 2022
 *      Author: tommasocanova
 */

#ifndef INC_TEMP_H_
#define INC_TEMP_H_

#define BUFF_SIZE 300

#define GPIO_MAX_VOLTAGE 3.3
#define ADC_BIT_RESOLUTION 4096.0
#define SAMPLES 30.0

typedef enum{
	COLD = 0U,
	NORMAL,
	HOT,
	DANGER,
	TEMP_ERROR
} temperature_level;

#endif /* INC_TEMP_H_ */
