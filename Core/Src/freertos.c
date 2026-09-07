/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* Guardados para inspeccionar con el debugger tras un stack overflow — el
 * HardFault_Handler default es un while(1) mudo, sin esto un overflow se ve
 * exactamente como el sistema muerto/sin logs, sin pista de la causa. */
static volatile char        s_overflow_task_name[configMAX_TASK_NAME_LEN];

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  FreeRTOS llama esto cuando detecta que una tarea se paso de su
 *         stack (configCHECK_FOR_STACK_OVERFLOW=2, ver FreeRTOSConfig.h).
 * @note   No se llama Log_Print aqui: el stack de la tarea que reboso ya
 *         esta corrupto, cualquier trabajo extra (mutex, printf) puede
 *         terminar de tronar todo. Solo guardamos el nombre y nos quedamos
 *         en un while(1) para poder pausar con el debugger y ver
 *         s_overflow_task_name / pxCurrentTCB.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    for (uint8_t i = 0U; i < (configMAX_TASK_NAME_LEN - 1U) && pcTaskName[i] != '\0'; i++) {
        s_overflow_task_name[i] = pcTaskName[i];
    }
    __disable_irq();
    for (;;) {
        __NOP();
    }
}

/* USER CODE END Application */

