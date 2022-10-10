/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// timer0 and timerRun are created in timer.c and timer.h

// display led 7seg function
void display7SEG(int num) {
	switch (num) {
//		using output data register to set value for port B
	case 0: {
//			using operator & with 0xFF00 to reset 8bits low
		GPIOB->ODR &= 0xFF00;
//			set value to 8bits low by using operator & to retain 8bits high
		GPIOB->ODR |= 0x0040;
		break;
	}
	case 1: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0079;
		break;
	}
	case 2: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0024;
		break;
	}
	case 3: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0030;
		break;
	}
	case 4: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0019;
		break;
	}
	case 5: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0012;
		break;
	}
	case 6: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0002;
		break;
	}
	case 7: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0078;
		break;
	}
	case 8: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0000;
		break;
	}
	case 9: {
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x0010;
		break;
	}
	default:
		GPIOB->ODR &= 0xFF00;
		GPIOB->ODR |= 0x00FF;
		break;
	}
}

const int MAX_LED = 4; // number of max led
int index_led = 0;
int led_buffer[4] = { 1, 2, 3, 4 }; //led number buffer
// update led 7 seg function
void update7SEG(int index) {
	switch (index) {
	case 0:
		//Display the first 7SEG with led_buffer[0]
		display7SEG(led_buffer[0]);
		break;
	case 1:
		//Display the second 7SEG with led_buffer[1]
		display7SEG(led_buffer[1]);
		break;
	case 2:
		//Display the third 7SEG with led_buffer[2]
		display7SEG(led_buffer[2]);
		break;
	case 3:
		//Display the forth 7SEG with led_buffer[3]
		display7SEG(led_buffer[3]);
		break;
	default:
		break;
	}
}

void updateClockBuffer(int hour, int minute) {
	led_buffer[0] = hour / 10;
	led_buffer[1] = hour % 10;
	led_buffer[2] = minute / 10;
	led_buffer[3] = minute % 10;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_TIM2_Init();
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, SET);
	HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
	HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
	HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
//	set time to begin clock
	int hour = 15, minute = 8, second = 50;
	updateClockBuffer(hour, minute);
	setTimer0(1000);

	while (1) {
//		execute toggle led red, dot and update clock buffer by using timer0
		if (timer0_flag == 1) {
			setTimer0(1000);
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

//			logic to calculate clock
			second++;
			if (second >= 60) {
				second = 0;
				minute++;
			}
			if (minute >= 60) {
				minute = 0;
				hour++;
			}
			if (hour >= 24) {
				hour = 0;
			}
			//generate number for 4 led7segs
			updateClockBuffer(hour, minute);

//			Toggle value DOT
			HAL_GPIO_TogglePin(DOT_GPIO_Port, DOT_Pin);
		}
	}
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

/** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
Error_Handler();
}
/** Initializes the CPU, AHB and APB buses clocks
 */
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
Error_Handler();
}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

/* USER CODE BEGIN TIM2_Init 0 */

/* USER CODE END TIM2_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
TIM_MasterConfigTypeDef sMasterConfig = { 0 };

/* USER CODE BEGIN TIM2_Init 1 */

/* USER CODE END TIM2_Init 1 */
htim2.Instance = TIM2;
htim2.Init.Prescaler = 7999;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 10;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
Error_Handler();
}
/* USER CODE BEGIN TIM2_Init 2 */

/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
GPIO_InitTypeDef GPIO_InitStruct = { 0 };

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA,
DOT_Pin | LED_RED_Pin | EN0_Pin | EN1_Pin | EN2_Pin | EN3_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB,
SEG0_Pin | SEG1_Pin | SEG2_Pin | SEG3_Pin | SEG4_Pin | SEG5_Pin | SEG6_Pin,
	GPIO_PIN_RESET);

/*Configure GPIO pins : DOT_Pin LED_RED_Pin EN0_Pin EN1_Pin
 EN2_Pin EN3_Pin */
GPIO_InitStruct.Pin = DOT_Pin | LED_RED_Pin | EN0_Pin | EN1_Pin | EN2_Pin
	| EN3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin SEG3_Pin
 SEG4_Pin SEG5_Pin SEG6_Pin */
GPIO_InitStruct.Pin = SEG0_Pin | SEG1_Pin | SEG2_Pin | SEG3_Pin | SEG4_Pin
	| SEG5_Pin | SEG6_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int counter_7seg = 25; // counter for 4 led7seg
int seg7_flag = 0; // flag for 4 led7seg
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	timerRun(); // TIMER0 AND FUNCTION TIMERRUN IS GREARATED IN timer.h and timer.c
//	timer for led 7 segment begin
	if (counter_7seg > 0) {
		counter_7seg--;
		if (counter_7seg <= 0) {
			counter_7seg = 25;
			if (seg7_flag ==  0) {
//				set enable signal to turn on led7seg 1
				HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, RESET);
				HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
				HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
				HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
				seg7_flag = 1;
			} else if (seg7_flag == 1) {
				//	set enable signal to turn on led7seg 2
				HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, SET);
				HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, RESET);
				HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
				HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
				seg7_flag = 2;
			} else if (seg7_flag == 2) {
				//	set enable signal to turn on led7seg 3
				HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, SET);
				HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
				HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, RESET);
				HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
				seg7_flag = 3;
			} else if (seg7_flag == 3) {
				//	set enable signal to turn on led7seg 4
				HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, SET);
				HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
				HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
				HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, RESET);
				seg7_flag = 0;
			}
			if(index_led > MAX_LED - 1) index_led = 0;
//			update led 7seg
			update7SEG(index_led++);
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
__disable_irq();
while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
