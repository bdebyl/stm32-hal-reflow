/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  int16_t StartTemperature;
  int16_t TargetTemperature;
  uint8_t SecondsToTarget;
} ReflowProfile_TypeDef;
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
#define GPIO_CS0_Pin          GPIO_PIN_4
#define GPIO_CS0_GPIO_Port    GPIOC
#define GPIO_CS1_Pin          GPIO_PIN_5
#define GPIO_CS1_GPIO_Port    GPIOC
#define GPIO_LCD_RS_Pin       GPIO_PIN_10
#define GPIO_LCD_RS_GPIO_Port GPIOB
#define GPIO_BTN_Pin          GPIO_PIN_8
#define GPIO_BTN_GPIO_Port    GPIOC
#define GPIO_BTN_EXTI_IRQn    EXTI4_15_IRQn
#define GPIO_ZX_EN_Pin        GPIO_PIN_9
#define GPIO_ZX_EN_GPIO_Port  GPIOA
#define GPIO_ZX_DET_Pin       GPIO_PIN_10
#define GPIO_ZX_DET_GPIO_Port GPIOA
#define GPIO_ZX_DET_EXTI_IRQn EXTI4_15_IRQn
#define GPIO_R1E_Pin          GPIO_PIN_10
#define GPIO_R1E_GPIO_Port    GPIOC
#define GPIO_R2E_Pin          GPIO_PIN_11
#define GPIO_R2E_GPIO_Port    GPIOC
#define GPIO_LCD_E_Pin        GPIO_PIN_8
#define GPIO_LCD_E_GPIO_Port  GPIOB
#define GPIO_LCD_RW_Pin       GPIO_PIN_9
#define GPIO_LCD_RW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ARM_MATH_CM0
#define GPIO_LCD_PORT (GPIOB)
#define GPIO_LCD_WritePort(x)                                                  \
  (GPIO_LCD_PORT->ODR = (GPIO_LCD_PORT->ODR & 0xFF00) | x)
#define ZX_COUNT_MAX (60)
#define PID_MAX      (ZX_COUNT_MAX)
#define PID_MIN      (0)
#define PID_T        (PID_MAX * (1 / 60))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
