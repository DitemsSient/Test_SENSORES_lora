/**
 * @file    Logger.h
 * @brief   Serial logging, temporalmente compartido con el UART de Bluetooth (USART3, PB10/PB11).
 *
 * @details Provides Log_Print() for any driver or module to emit
 *          timestamped text messages to a terminal.
 *          Thread-safety: none — a RTOS mutex should be added later.
 *
 *          NOTA (migración a STM32L433CCUx): este MCU no tiene UART4, así
 *          que RS485 se quedó sin UART real — los pines MCU_485_TX/RX
 *          (PA0/PA1) se configuraron como GPIO_Output simple en el .ioc a
 *          petición del equipo, solo para dejarlos reservados, sin
 *          funcionalidad de comunicación por ahora (decisión de hardware
 *          pendiente, fuera del alcance de este driver).
 *
 *          Esta tarjeta tampoco tiene un UART libre dedicado para logging
 *          (USART1=GPS, USART2=LoRa, USART3=Bluetooth). De momento se
 *          comparte con el UART de Bluetooth (huart3) mientras se decide
 *          una solución definitiva (p. ej. SWO/ITM). Transmitir logs por
 *          este UART interferirá con las pruebas reales de Bluetooth — no
 *          correr ambas a la vez.
 *
 *          Usage:
 *            main.c  → call Log_Init() once after MX_USART3_UART_Init().
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

#define LOG_UART    (&huart3)   /**< Compartido con Bluetooth (USART3, PB10/PB11) — ver nota arriba */
#define LOG_TX_TIMEOUT_MS  100U /**< Blocking transmit timeout               */

/* ========================  API  =========================================== */

/**
 * @brief  Initializes the logger and binds it to the log UART.
 * @note   Call once after MX_USART3_UART_Init() in main.c.
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

/**
 * @brief  Prints a tagged, printf-style formatted log message over the log UART.
 * @param  tag  Short module identifier, e.g. "BT", "GPS", "LORA".
 * @param  fmt  printf-style format string.
 * @note   Formats into a 128-byte stack buffer via vsnprintf() before
 *         delegating to Log_Print() — truncates if the result is longer.
 *
 * Output format:  [TAG] msg\r\n
 */
void Log_Printf(const char *tag, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
