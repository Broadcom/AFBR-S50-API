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
#include "stm32f4xx_hal.h"

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
#define PWR1_Pin GPIO_PIN_0
#define PWR1_GPIO_Port GPIOA
#define PWR2_Pin GPIO_PIN_1
#define PWR2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define PWR3_Pin GPIO_PIN_4
#define PWR3_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_5
#define CLK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define PWR4_Pin GPIO_PIN_0
#define PWR4_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_10
#define CS4_GPIO_Port GPIOB
#define IRQ1_Pin GPIO_PIN_7
#define IRQ1_GPIO_Port GPIOC
#define IRQ1_EXTI_IRQn EXTI9_5_IRQn
#define CS3_Pin GPIO_PIN_8
#define CS3_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_9
#define CS2_GPIO_Port GPIOA
#define IRQ2_Pin GPIO_PIN_10
#define IRQ2_GPIO_Port GPIOA
#define IRQ2_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IRQ3_Pin GPIO_PIN_3
#define IRQ3_GPIO_Port GPIOB
#define IRQ3_EXTI_IRQn EXTI3_IRQn
#define IRQ4_Pin GPIO_PIN_5
#define IRQ4_GPIO_Port GPIOB
#define IRQ4_EXTI_IRQn EXTI9_5_IRQn
#define CS1_Pin GPIO_PIN_6
#define CS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
