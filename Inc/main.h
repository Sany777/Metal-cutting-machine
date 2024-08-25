/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights fixed.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */


//-------------------- IN
#define in_emerg_Pin GPIO_PIN_0
#define in_emerg_Port GPIOB

#define in_help_sw_Pin GPIO_PIN_1
#define in_help_sw_Port GPIOB

#define in_pomp_Pin GPIO_PIN_2
#define in_pomp_Port GPIOB

#define in_start_Pin GPIO_PIN_10
#define in_start_Port GPIOB

//-------------------- OUT

#define out_pneumatic_act_Pin GPIO_PIN_8
#define out_pneumatic_act_Port GPIOB

#define out_fix_act_Pin GPIO_PIN_7
#define out_fix_act_Port GPIOB

#define out_cut_down_act_Pin GPIO_PIN_6
#define out_cut_down_act_Port GPIOB

#define out_cut_act_Pin GPIO_PIN_6
#define out_cut_act_Port GPIOB

#define out_help_ON_Pin GPIO_PIN_5
#define out_help_ON_Port GPIOB

#define out_help_reverse_Pin GPIO_PIN_4
#define out_help_reverse_Port GPIOB

#define out_led_Pin GPIO_PIN_3
#define out_led_Port GPIOB

#define out_none_Pin GPIO_PIN_15
#define out_none_Port GPIOA

#define debug_Pin GPIO_PIN_13
#define debug_GPIO_Port GPIOC

#define servo_tx_Pin GPIO_PIN_2
#define servo_tx_GPIO_Port GPIOA
#define servo_rx_Pin GPIO_PIN_3
#define servo_rx_GPIO_Port GPIOA

#define op320_tx_Pin GPIO_PIN_9
#define op320_tx_GPIO_Port GPIOA
#define op320_rx_Pin GPIO_PIN_10
#define op320_rx_GPIO_Port GPIOA

#define  out_GPIO_Port GPIOB

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define huartServo 							(&huart2)
#define huartOP320 							(&huart1)


#define DELAY_READ_BUT 				275
#define DELAY_START_TASK 			2000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
