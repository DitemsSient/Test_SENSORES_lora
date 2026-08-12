/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPOUT_Pin GPIO_PIN_13
#define GPOUT_GPIO_Port GPIOC
#define XIN32_Pin GPIO_PIN_14
#define XIN32_GPIO_Port GPIOC
#define XOUT32_Pin GPIO_PIN_15
#define XOUT32_GPIO_Port GPIOC
#define Motovibrador_Pin GPIO_PIN_0
#define Motovibrador_GPIO_Port GPIOH
#define INT_IMU_Pin GPIO_PIN_1
#define INT_IMU_GPIO_Port GPIOH
#define MCU_485_TX_Pin GPIO_PIN_0
#define MCU_485_TX_GPIO_Port GPIOA
#define MCU_485_RX_Pin GPIO_PIN_1
#define MCU_485_RX_GPIO_Port GPIOA
#define MCU_LORA_TX_Pin GPIO_PIN_2
#define MCU_LORA_TX_GPIO_Port GPIOA
#define MCU_LORA_RX_Pin GPIO_PIN_3
#define MCU_LORA_RX_GPIO_Port GPIOA
#define ADC_BATTERY_Pin GPIO_PIN_4
#define ADC_BATTERY_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_5
#define USB_ID_GPIO_Port GPIOA
#define BLUETOOTH_MCU_Pin GPIO_PIN_6
#define BLUETOOTH_MCU_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOA
#define SENSOR_IR2_Pin GPIO_PIN_0
#define SENSOR_IR2_GPIO_Port GPIOB
#define SEL_PROG_LORA_O_BLE_Pin GPIO_PIN_1
#define SEL_PROG_LORA_O_BLE_GPIO_Port GPIOB
#define SEL_PROG_MCU_O_ModulosFTDI_Pin GPIO_PIN_2
#define SEL_PROG_MCU_O_ModulosFTDI_GPIO_Port GPIOB
#define MCU_BLE_TX_Pin GPIO_PIN_10
#define MCU_BLE_TX_GPIO_Port GPIOB
#define MCU_BLE_RX_Pin GPIO_PIN_11
#define MCU_BLE_RX_GPIO_Port GPIOB
#define LORA_BOOT_Pin GPIO_PIN_12
#define LORA_BOOT_GPIO_Port GPIOB
#define GPS_FORCE_Pin GPIO_PIN_8
#define GPS_FORCE_GPIO_Port GPIOA
#define MCU_GPS_TX_Pin GPIO_PIN_9
#define MCU_GPS_TX_GPIO_Port GPIOA
#define MCU_GPS_RX_Pin GPIO_PIN_10
#define MCU_GPS_RX_GPIO_Port GPIOA
#define RS485_CONTROL_Pin GPIO_PIN_15
#define RS485_CONTROL_GPIO_Port GPIOA
#define BLU_AUTORUN_Pin GPIO_PIN_3
#define BLU_AUTORUN_GPIO_Port GPIOB
#define BLUETOOTH_VSP_Pin GPIO_PIN_4
#define BLUETOOTH_VSP_GPIO_Port GPIOB
#define CS_FLASH_Pin GPIO_PIN_5
#define CS_FLASH_GPIO_Port GPIOB
#define SENSOR_IR1_Pin GPIO_PIN_3
#define SENSOR_IR1_GPIO_Port GPIOH
#define LORA_RESET_Pin GPIO_PIN_8
#define LORA_RESET_GPIO_Port GPIOB
#define BLUETOOTH_RESET_Pin GPIO_PIN_9
#define BLUETOOTH_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
