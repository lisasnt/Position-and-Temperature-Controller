/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "ntc.h"
#include "ptc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TRUE   		1
#define FALSE   	0
#define ENC_BITs    27  // 13 multiturn, 12 angle, 2 stop-bits
#define AVG_MEAS    3   // Number of samples
#define BUF_SIZE   80
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint32_t enc_clk_count = 0;
uint32_t enc_bits[AVG_MEAS];
uint8_t ping = TRUE;
uint8_t sampl_cycles = 3;
uint8_t i = 0;
uint32_t angle[AVG_MEAS];
uint16_t turn[AVG_MEAS];
uint8_t tx_buff[BUF_SIZE];
uint8_t tx_buff_size;

uint8_t clk_front_count = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void getTemperature();
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_HS_USB_Init();
	MX_TIM7_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */

	memset(tx_buff, (uint8_t)0x0, sizeof(uint8_t)*BUF_SIZE);
	memset(enc_bits,(uint8_t)0x0, sizeof(uint32_t)*AVG_MEAS);
	tx_buff_size = (uint8_t) sprintf((char *) tx_buff, "Hello\n\r");
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, tx_buff_size, 500);

	// Set high preset pin for 2.5s to reset the encoder counter
	HAL_Delay(1000);
	HAL_GPIO_WritePin(ENC_PRESET_GPIO_Port, ENC_PRESET_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(ENC_PRESET_GPIO_Port, ENC_PRESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		memset(enc_bits,(uint8_t)0x0, sizeof(uint32_t)*AVG_MEAS);

		for (i = 0; i < AVG_MEAS; i++)
		{
			ping = TRUE;
			enc_clk_count = 0;
			enc_clk_count = 0;
			HAL_TIM_Base_Start_IT(&htim7);

			while (enc_clk_count < (ENC_BITs * 4) )
			{
				while (ping){}

				if ((enc_clk_count % 4) == 0) // Every data bit it's 4 timer interrupts
				{
					HAL_GPIO_TogglePin(GPIO_DEBUG_GPIO_Port, GPIO_DEBUG_Pin);
					enc_bits[i] <<= 1;
					enc_bits[i] |= ( ((uint32_t) HAL_GPIO_ReadPin(SSI_ENC_DATA_GPIO_Port, SSI_ENC_DATA_Pin)) & 0x00000001);
					HAL_GPIO_TogglePin(GPIO_DEBUG_GPIO_Port, GPIO_DEBUG_Pin);
				}
				else if ((enc_clk_count % 2) == 0)
				{}
				else
				{
					HAL_GPIO_TogglePin(SSI_ENC_CLK_GPIO_Port, SSI_ENC_CLK_Pin);
					HAL_GPIO_TogglePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin);
					clk_front_count++;
				}
				ping = TRUE;
			}

			HAL_GPIO_WritePin(SSI_ENC_CLK_GPIO_Port, SSI_ENC_CLK_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_Delay(5);
		}

		for (i = 0; i < AVG_MEAS; i++)
		{
			enc_bits[i] >>= 2;
			//13 bits angle
			angle[i] = (int32_t) ((enc_bits[i] & 0x00001FFF) * 44); // (360 / 2^13) * 1000 = 44 [mDeg]
			//12 bits multi-turn
			turn[i]  = (int16_t)((enc_bits[i] & 0x01FFE000) >> 13);
		}
		tx_buff_size = (uint8_t) sprintf((char *) tx_buff, "------\r\nENCODER:\r\n%lu|%lu|%lu mDeg\n\r%u|%u|%u turns\n\r\n\r",
				angle[0], angle[1], angle[2], turn[0], turn[1], turn[2]);
		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, tx_buff_size, 1000);
		getTemperature();
		HAL_Delay(500);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
			|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 275;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
	PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void getTemperature(){

	HAL_UART_Transmit(&huart3, (uint8_t*)"Temperatures: \r\n", strlen("Temperatures: \r\n"), 1000);
	double ptc_volt = get_ptc_volt(&hadc3, 200);
	get_ptc_temp_zone(ptc_volt);
	HAL_Delay(1000);

	double ntc_volt = get_ntc_volt(&hadc3, 200);
	get_ntc_temp_zone(ntc_volt);
	HAL_Delay(1000);
	// println(&huart2, "------------------------------");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM7)
	{
		enc_clk_count++;
		ping = FALSE;
	}
	/* USER CODE END Callback 1 */
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
