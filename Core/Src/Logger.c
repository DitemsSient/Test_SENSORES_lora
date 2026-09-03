/**
 * @file    Logger.c
 * @brief   Serial logging implementation over USB CDC.
 *
 * @date    July 03, 2026
 * @author  César Pérez
 * @version 2.0.0
 */

#include "Logger.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

/* ========================  EXTERNAL HAL HANDLES  =========================== */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ========================  PRIVATE STATE  ================================= */

static bool        s_ready = false;
static osMutexId_t s_mutex = NULL;   /* NULL hasta Log_InitMutex() — ver Logger.h */

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Sends a buffer over USB CDC and waits for the transfer to actually
 *         finish before returning.
 * @param  data  Buffer to transmit.
 * @param  len   Number of bytes.
 * @note   CDC_Transmit_FS() only HANDS OFF the pointer to the USB hardware
 *         (USBD_CDC_SetTxBuffer() just stores it, no copy) and returns
 *         immediately — the actual transfer keeps reading from `data`
 *         asynchronously until the IN endpoint completes. If we return as
 *         soon as the submit succeeds, the caller's buffer (out[]/msg[] on
 *         Log_Print()'s stack) can get reused by the NEXT call before the
 *         USB hardware finishes reading it — that showed up as garbled text
 *         mid-message. So after a successful submit we also wait for
 *         hcdc->TxState to clear back to 0 (set by CDC_TransmitCplt_FS()).
 *         Gives up silently after LOG_TX_TIMEOUT_MS either way — logging
 *         must never hang the app if nothing is connected on the other end.
 */
static void Log_TransmitUSB(uint8_t *data, uint16_t len) {
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS(data, len) == USBD_BUSY) {
        if ((HAL_GetTick() - start) > LOG_TX_TIMEOUT_MS) {
            return;
        }
    }

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    while (hcdc != NULL && hcdc->TxState != 0U) {
        if ((HAL_GetTick() - start) > LOG_TX_TIMEOUT_MS) {
            return;
        }
    }
}

/* ========================  PUBLIC FUNCTIONS  =============================== */

void Log_Init(void)
{
    s_ready = true;
}

void Log_InitMutex(void)
{
    s_mutex = osMutexNew(NULL);
}

void Log_Print(const char *tag, const char *msg)
{
    if (!s_ready || tag == NULL || msg == NULL) {
        return;
    }

    char out[LOG_MAX_MSG_LEN];
    int len = snprintf(out, sizeof(out), "[%s] %s\r\n", tag, msg);

    if (len <= 0) {
        return;
    }
    if ((size_t)len >= sizeof(out)) {
        len = (int)sizeof(out) - 1;
    }

    if (s_mutex != NULL) osMutexAcquire(s_mutex, osWaitForever);
    Log_TransmitUSB((uint8_t *)out, (uint16_t)len);
    if (s_mutex != NULL) osMutexRelease(s_mutex);
}

void Log_Blank(void)
{
    if (!s_ready) {
        return;
    }

    if (s_mutex != NULL) osMutexAcquire(s_mutex, osWaitForever);
    Log_TransmitUSB((uint8_t *)"\r\n", 2U);
    if (s_mutex != NULL) osMutexRelease(s_mutex);
}

void Log_Printf(const char *tag, const char *fmt, ...)
{
    if (!s_ready || tag == NULL || fmt == NULL) {
        return;
    }

    char msg[LOG_MAX_MSG_LEN];

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    Log_Print(tag, msg);
}
