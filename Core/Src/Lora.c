/**
 * @file    Lora.c
 * @brief   LoRa UART module driver implementation for STM32F4xx.
 *
 * @details Uses HAL_UART_Transmit / HAL_UART_Receive for blocking transfers.
 *          Interrupt-driven reception is supported via Lora_StoreByte().
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Lora.h"
#include "Logger.h"
#include <string.h>

/* ========================  EXTERNAL HAL HANDLES  ========================== */

extern UART_HandleTypeDef huart2;

/* ========================  PUBLIC FUNCTIONS  =============================== */

LoraStatus_e Lora_Init(Lora_Handle_t *h)
{
    if (h == NULL) {
        Log_Print("LORA", "Error: handle NULL en Init");
        return LORA_ERR_PARAM;
    }

    Log_Print("LORA", "Iniciando modulo LoRa...");

    h->huart    = LORA_UART;
    h->rx_count = 0U;
    h->rx_byte  = 0U;
    h->rx_ready = false;
    memset(h->tx_buffer, 0, LORA_TX_BUFFER_SIZE);
    memset(h->rx_buffer, 0, LORA_RX_BUFFER_SIZE);

    /* Arm interrupt reception for the first byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);

    Log_Print("LORA", "LoRa inicializado correctamente");
    return LORA_OK;
}

LoraStatus_e Lora_Transmit(Lora_Handle_t *h, const uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return LORA_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, data, len, LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

LoraStatus_e Lora_Receive(Lora_Handle_t *h, uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return LORA_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Receive(h->huart, data, len, LORA_RX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

void Lora_StoreByte(Lora_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    if (h->rx_count < LORA_RX_BUFFER_SIZE) {
        h->rx_buffer[h->rx_count] = h->rx_byte;
        h->rx_count++;
    }

    /* Re-arm interrupt for next byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);
}

void Lora_ResetRx(Lora_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    h->rx_count = 0U;
    h->rx_ready = false;
    memset(h->rx_buffer, 0, LORA_RX_BUFFER_SIZE);
}

/* ========================  LOW POWER  ====================================== */

LoraStatus_e Lora_Sleep(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* Primary command for RM1262 — verify with hardware if module does not
     * respond. Alternative commands to try (uncomment one at a time):
     *
     *   "AT+SLPM\r\n"       — common Semtech / RAK variant
     *   "AT+SLEEP=1\r\n"    — some modules require a parameter
     *   "AT+LOWPOWER\r\n"   — used by certain LoRaWAN stacks
     *   "AT+PSLP\r\n"       — Murata / STM32WL variant
     */
    const uint8_t cmd[] = "AT+SLEEP\r\n";
    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, cmd,
                                              sizeof(cmd) - 1U,
                                              LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

LoraStatus_e Lora_WakeUp(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* Any UART byte wakes the RM1262 from sleep */
    const uint8_t wake = 0xFFU;
    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, &wake, 1U,
                                              LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    HAL_Delay(LORA_WAKE_DELAY_MS);
    return LORA_OK;
}
