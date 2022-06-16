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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

void ADC_Test();
float ADC_BatteryVoltage();
float ADC_ChargeVoltage();
float ADC_ChargeCurrent();
void ChargeController(void);
void StatusLEDUpdate(void);
void setDriveMotors(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir);
void setBladeMotor(uint8_t on_off);
uint8_t crcCalc(uint8_t *msg, uint8_t msg_len);
void msgPrint(uint8_t *msg, uint8_t msg_len);
void chirp();

extern float battery_voltage;
extern float charge_voltage;
extern float charge_current;

extern uint16_t chargecontrol_pwm_val;
extern uint8_t  chargecontrol_is_charging;
extern int16_t  right_wheel_speed_val;
extern int16_t  left_wheel_speed_val;
extern uint32_t right_encoder_ticks;
extern uint32_t left_encoder_ticks;

// uart statistics
extern uint16_t cnt_uart4_overrun;      // master
extern uint16_t cnt_usart2_overrun;     // drive motors


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
void TIM3_Init(void);
void MX_DMA_Init(void);


// UART Wrapper functions to hide HAL bullshit ...
void MASTER_Transmit(uint8_t *buffer, uint8_t len);
void DRIVEMOTORS_Transmit(uint8_t *buffer, uint8_t len);


void debug_printf(const char *fmt, ...);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
