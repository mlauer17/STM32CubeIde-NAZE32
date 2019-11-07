/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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



// variable used to run code at fixed frequency
int main_loop_trigger;

//
int prox_status;

// raw sensor measurements. acc XYZ, gyro XYZ
int16_t IMU[6];

// roll pitch yaw and altitude estimate
float RPYA[4];

// Setpoints
float RPYA_SP[4];

float MOTOR_CMD[4];


// Buffers used by UART1 in DMA mode
uint8_t buffer_tx[16];
uint8_t buffer_rx[16];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void stop_motors(void);
void pwm_init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Code_Timing_Pin GPIO_PIN_0
#define Code_Timing_GPIO_Port GPIOA
#define prox_echo_Pin GPIO_PIN_1
#define prox_echo_GPIO_Port GPIOA
#define prox_echo_EXTI_IRQn EXTI1_IRQn
#define prox_trig_Pin GPIO_PIN_12
#define prox_trig_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
