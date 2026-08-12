/**
 * @file    Logger.c
 * @brief   Serial logging implementation, compartido con el UART de RS485 (UART4).
 *
 * @date    July 03, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Logger.h"
#include <string.h>

/* ========================  EXTERNAL HAL HANDLES  ========================== */

extern UART_HandleTypeDef huart4;

/* ========================  PRIVATE STATE  ================================= */

static UART_HandleTypeDef *s_huart = NULL;

/* ========================  PUBLIC FUNCTIONS  =============================== */

void Log_Init(void)
{
    s_huart = LOG_UART;
}

void Log_Print(const char *tag, const char *msg)
{
    if (s_huart == NULL || tag == NULL || msg == NULL) {
        return;
    }

    /* "[TAG] msg\r\n" — built in two transmit calls to avoid a stack buffer */
    const char *open  = "[";
    const char *close = "] ";
    const char *crlf  = "\r\n";

    HAL_UART_Transmit(s_huart, (uint8_t *)open,  1U,                   LOG_TX_TIMEOUT_MS);
    HAL_UART_Transmit(s_huart, (uint8_t *)tag,   (uint16_t)strlen(tag), LOG_TX_TIMEOUT_MS);
    HAL_UART_Transmit(s_huart, (uint8_t *)close, 2U,                   LOG_TX_TIMEOUT_MS);
    HAL_UART_Transmit(s_huart, (uint8_t *)msg,   (uint16_t)strlen(msg), LOG_TX_TIMEOUT_MS);
    HAL_UART_Transmit(s_huart, (uint8_t *)crlf,  2U,                   LOG_TX_TIMEOUT_MS);
}
