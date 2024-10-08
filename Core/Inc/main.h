/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TAM_CLAVE 10
#define TAM_BOTON 10
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{
	NO_PRESION,
	PRESION,
}flag_enum;

typedef enum{
	FALSE,
	TRUE,
}boolean_enum;
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
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOA
#define S1_EXTI_IRQn EXTI1_IRQn
#define S2_Pin GPIO_PIN_4
#define S2_GPIO_Port GPIOA
#define S2_EXTI_IRQn EXTI4_IRQn
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOA
#define ROW_1_Pin GPIO_PIN_6
#define ROW_1_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_0
#define S3_GPIO_Port GPIOB
#define S3_EXTI_IRQn EXTI0_IRQn
#define COLUMN_3_Pin GPIO_PIN_13
#define COLUMN_3_GPIO_Port GPIOB
#define COLUMN_3_EXTI_IRQn EXTI15_10_IRQn
#define COLUMN_4_Pin GPIO_PIN_14
#define COLUMN_4_GPIO_Port GPIOB
#define COLUMN_4_EXTI_IRQn EXTI15_10_IRQn
#define ROW_2_Pin GPIO_PIN_7
#define ROW_2_GPIO_Port GPIOC
#define ROW_4_Pin GPIO_PIN_8
#define ROW_4_GPIO_Port GPIOA
#define ROW_3_Pin GPIO_PIN_9
#define ROW_3_GPIO_Port GPIOA
#define COLUMN_2_Pin GPIO_PIN_11
#define COLUMN_2_GPIO_Port GPIOA
#define COLUMN_2_EXTI_IRQn EXTI15_10_IRQn
#define COLUMN_1_Pin GPIO_PIN_5
#define COLUMN_1_GPIO_Port GPIOB
#define COLUMN_1_EXTI_IRQn EXTI9_5_IRQn
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
