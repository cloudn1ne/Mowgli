/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle
UART_HandleTypeDef DRIVEMOTORS_USART_Handler; // UART  Handle
UART_HandleTypeDef BLADEMOTOR_USART_Handler; // UART  Handle

I2C_HandleTypeDef I2C_Handle;

ADC_HandleTypeDef ADC_Handle;

TIM_HandleTypeDef TIM1_Handle;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

void ADC_Test();
float ADC_BatteryVoltage();
float ADC_ChargeVoltage();
float ADC_ChargeCurrent();

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void LED_Init();
void TF4_Init();
void PAC5223RESET_Init();
void PAC5210RESET_Init();
void MASTER_USART_Init();
void DRIVEMOTORS_USART_Init();
void BLADEMOTOR_USART_Init();
void SystemClock_Config();
void I2C_Init(void);
void ADC1_Init(void);
void TIM1_Init(void);

void debug_printf(const char *fmt, ...);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
