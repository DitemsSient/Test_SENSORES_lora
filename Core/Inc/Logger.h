/**
 * @file    Logger.h
 * @brief   Serial logging over the USB CDC virtual COM port.
 *
 * @details Provides Log_Print()/Log_Printf() for any driver o modulo emitir
 *          mensajes de texto por el puerto USB CDC (ver
 *          USB_DEVICE/App/usbd_cdc_if.c, CDC_Transmit_FS).
 *
 *          Arquitectura por cola (2026-09-04, reemplaza el mutex anterior):
 *          antes, cada Log_Print() bloqueaba a quien llamaba hasta que el
 *          USB terminaba de transmitir — un cuello de botella real en
 *          cuanto varias tareas (sensores, disparo, GPS, etc.) compiten por
 *          el mismo Logger. Ahora Log_Print()/Log_Printf() SOLO copian la
 *          linea ya formateada a una cola de FreeRTOS (osMessageQueuePut,
 *          no bloqueante — timeout 0) y regresan de inmediato. La unica
 *          tarea que de verdad espera al USB es Log_Task() (ver
 *          Tareas_CrearTareas() en Tareas.c, que la crea con
 *          osThreadNew(Log_Task, ...)).
 *
 *          Mientras la cola no exista (arranque bare-metal, antes de
 *          Log_InitQueue()) Log_Print()/Log_Printf() mandan directo y
 *          bloqueante por USB, igual que antes — un solo hilo de ejecucion
 *          en ese punto (Inicializacion_Run(), previo al kernel), no hace
 *          falta la cola ahi.
 *
 *          Si la cola se llena (productor mas rapido que Log_Task
 *          imprimiendo, rafaga), el mensaje se DESCARTA — nunca se bloquea
 *          a una tarea de tiempo real por un log. Se cuenta internamente
 *          (ver contador privado en Logger.c, inspeccionable con el
 *          debugger si hace falta).
 *
 *          Usage:
 *            main.c  → Log_Init() dentro de Inicializacion_Run() (antes del
 *                      kernel). Log_InitQueue() despues de
 *                      osKernelInitialize(), antes de osKernelStart()
 *                      (Tareas_InicializarMutex() en Tareas.c).
 *            others  → #include "Logger.h" y llamar Log_Print(TAG, msg).
 *
 * @date    July 03, 2026
 * @author  César Pérez
 * @version 3.0.0
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
#define LOG_QUEUE_DEPTH       16U  /**< Lineas en vuelo antes de empezar a descartar */

/* ========================  API  =========================================== */

/**
 * @brief  Initializes the logger.
 * @note   Call once after MX_USB_DEVICE_Init() in main.c — and only on the
 *         path where the app did NOT jump to the bootloader.
 */
void Log_Init(void);

/**
 * @brief  Crea la cola del Logger.
 * @note   Llamar UNA vez, despues de osKernelInitialize() y antes de
 *         osKernelStart() (kernel inicializado pero scheduler sin correr
 *         todavia — es seguro crear la cola ahi). A diferencia del mutex
 *         anterior, encolar (osMessageQueuePut, timeout 0) SI es seguro
 *         entre este punto y osKernelStart() — no bloquea nunca, solo se
 *         quedan en la cola hasta que Log_Task() arranque y las drene.
 */
void Log_InitQueue(void);

/**
 * @brief  Cuerpo de la tarea consumidora del Logger — drena la cola y hace
 *         la transmision USB real (la unica bloqueante del sistema).
 * @note   Crear con osThreadNew(Log_Task, NULL, &attrs) en
 *         Tareas_CrearTareas(), despues de Log_InitQueue().
 * @param  argument  Sin uso (firma requerida por osThreadFunc_t).
 */
void Log_Task(void *argument);

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
