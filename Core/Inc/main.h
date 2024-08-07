/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LED_Green_Pin GPIO_PIN_13
#define LED_Green_GPIO_Port GPIOC
#define NRF24_IRQ_Pin GPIO_PIN_10
#define NRF24_IRQ_GPIO_Port GPIOB
#define NRF24_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_13
#define LCD_RW_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_14
#define LCD_EN_GPIO_Port GPIOB
#define LCD_DB4_Pin GPIO_PIN_15
#define LCD_DB4_GPIO_Port GPIOB
#define LCD_DB5_Pin GPIO_PIN_8
#define LCD_DB5_GPIO_Port GPIOA
#define LCD_DB6_Pin GPIO_PIN_9
#define LCD_DB6_GPIO_Port GPIOA
#define LCD_DB7_Pin GPIO_PIN_10
#define LCD_DB7_GPIO_Port GPIOA
#define NRF24_CE_Pin GPIO_PIN_6
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_CSN_Pin GPIO_PIN_7
#define NRF24_CSN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
