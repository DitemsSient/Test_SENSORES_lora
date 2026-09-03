/**
 * @file    Logger.h
 * @brief   Serial logging over the USB CDC virtual COM port.
 *
 * @details Provides Log_Print()/Log_Printf() for any driver o modulo emitir
 *          mensajes de texto por el puerto USB CDC (ver
 *          USB_DEVICE/App/usbd_cdc_if.c, CDC_Transmit_FS).
 *
 *          Thread-safety: Log_InitMutex() crea un mutex opcional. Mientras
 *          no se llame (arranque bare-metal, antes de osKernelInitialize())
 *          Log_Print()/Log_Printf() funcionan sin lock — un solo hilo de
 *          ejecucion en ese punto, no hace falta. Una vez creado el mutex
 *          (despues de osKernelInitialize(), antes de osKernelStart(), ver
 *          Inicializacion_Run()), cada llamada lo toma/suelta — necesario
 *          en cuanto haya mas de una tarea usando el Logger a la vez.
 *
 *          Usage:
 *            main.c  → Log_Init() dentro de Inicializacion_Run() (antes del
 *                      kernel). Log_InitMutex() despues de
 *                      osKernelInitialize(), antes de osKernelStart().
 *            others  → #include "Logger.h" y llamar Log_Print(TAG, msg).
 *
 * @date    July 03, 2026
 * @author  César Pérez
 * @version 2.0.0
 */

#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* ========================  CONFIGURATION  ================================= */

#define LOG_TX_TIMEOUT_MS   100U   /**< Max wait while CDC_Transmit_FS is busy */
#define LOG_MAX_MSG_LEN      160U  /**< "[TAG] msg\r\n" buffer size, truncates if longer */

/* ========================  API  =========================================== */

/**
 * @brief  Initializes the logger.
 * @note   Call once after MX_USB_DEVICE_Init() in main.c — and only on the
 *         path where the app did NOT jump to the bootloader.
 */
void Log_Init(void);

/**
 * @brief  Crea el mutex del Logger.
 * @note   Llamar UNA vez, despues de osKernelInitialize() y antes de
 *         osKernelStart() (kernel inicializado pero scheduler sin correr
 *         todavia — es seguro crear el mutex ahi, NO es seguro tomarlo).
 */
void Log_InitMutex(void);

/**
 * @brief  Prints a tagged log message over USB CDC.
 * @param  tag  Short module identifier, e.g. "BT", "I2C", "TEST".
 * @param  msg  Message string (NUL-terminated).
 *
 * Output format:  [TAG] msg\r\n
 */
void Log_Print(const char *tag, const char *msg);

/**
 * @brief  Prints a tagged, printf-style formatted log message over USB CDC.
 * @param  tag  Short module identifier, e.g. "BT", "I2C", "TEST".
 * @param  fmt  printf-style format string.
 * @note   Message is built in an internal fixed-size buffer via vsnprintf —
 *         truncates silently if the formatted message is longer.
 *
 * Output format:  [TAG] formatted message\r\n
 */
void Log_Printf(const char *tag, const char *fmt, ...);

/**
 * @brief  Imprime una linea en blanco (sin tag), util para separar bloques
 *         de log de distintos modulos.
 */
void Log_Blank(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
