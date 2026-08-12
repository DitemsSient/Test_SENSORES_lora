/**
 * @file    Logger.h
 * @brief   Serial logging, temporalmente compartido con el UART de RS485 (UART4, PA0/PA1).
 *
 * @details Provides Log_Print() for any driver or module to emit
 *          timestamped text messages to a terminal.
 *          Thread-safety: none — a RTOS mutex should be added later.
 *
 *          NOTA: esta tarjeta no tiene un UART libre para logging dedicado
 *          (UART4=RS485, USART1=GPS, USART2=LoRa, USART3=Bluetooth). De
 *          momento se comparte con el UART de RS485 (huart4) mientras se
 *          decide una solución definitiva (p. ej. SWO/ITM), ya que las
 *          pruebas de RS485 se harán por separado más adelante. Transmitir
 *          logs por este UART interferirá con la comunicación RS485 real
 *          en cuanto ese módulo esté bajo prueba.
 *
 *          Usage:
 *            main.c  → call Log_Init() once after MX_UART4_Init().
 *            others  → #include "Logger.h" and call Log_Print(TAG, msg).
 *
 * @date    July 03, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* ========================  CONFIGURATION  ================================= */

#define LOG_UART    (&huart4)   /**< Compartido con RS485 (UART4, PA0/PA1) — ver nota arriba */
#define LOG_TX_TIMEOUT_MS  100U /**< Blocking transmit timeout               */

/* ========================  API  =========================================== */

/**
 * @brief  Initializes the logger and binds it to the log UART.
 * @note   Call once after MX_UART4_Init() in main.c.
 */
void Log_Init(void);

/**
 * @brief  Prints a tagged log message over the log UART.
 * @param  tag  Short module identifier, e.g. "BT", "GPS", "LORA".
 * @param  msg  Message string (NUL-terminated).
 *
 * Output format:  [TAG] msg\r\n
 */
void Log_Print(const char *tag, const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
