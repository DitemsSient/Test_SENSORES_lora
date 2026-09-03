/**
 * @file    Bluetooth.c
 * @brief   Bluetooth UART module driver implementation for STM32F4xx.
 *
 * @details Uses HAL_UART_Transmit / HAL_UART_Receive for blocking transfers.
 *          Interrupt-driven reception is supported via Bt_StoreByte().
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Bluetooth.h"
#include "Logger.h"
#include <string.h>

/* ========================  EXTERNAL HAL HANDLES  ========================== */

extern UART_HandleTypeDef huart3;


/* ========================  PUBLIC FUNCTIONS  =============================== */

BtStatus_e Bt_Init(Bt_Handle_t *h)
{
    if (h == NULL) {
        Log_Print("BT", "Error: handle NULL en Init");
        return BT_ERR_PARAM;
    }

    Log_Print("BT", "Iniciando modulo Bluetooth BL654...");

    h->huart    = BT_UART;
    h->rx_count = 0U;
    h->rx_byte  = 0U;
    h->rx_ready = false;
    memset(h->tx_buffer, 0, BT_TX_BUFFER_SIZE);
    memset(h->rx_buffer, 0, BT_RX_BUFFER_SIZE);

    /* Arm interrupt reception for the first byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);

    Log_Print("BT", "Bluetooth inicializado correctamente");
    return BT_OK;
}

BtStatus_e Bt_Transmit(Bt_Handle_t *h, const uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return BT_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, data, len, BT_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? BT_ERR_TIMEOUT : BT_ERR_UART;
    }

    return BT_OK;
}

BtStatus_e Bt_Receive(Bt_Handle_t *h, uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return BT_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Receive(h->huart, data, len, BT_RX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? BT_ERR_TIMEOUT : BT_ERR_UART;
    }

    return BT_OK;
}

void Bt_StoreByte(Bt_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    if (h->rx_byte == '$') {
        h->rx_count = 0U;
        h->rx_ready = false;
        h->rx_buffer[0] = '\0';
    } else if (h->rx_byte == '\r') {
        if (h->rx_count > 0U) {
            h->rx_ready = true;
        }
    } else if (h->rx_count < BT_RX_BUFFER_SIZE - 1U) {
        h->rx_buffer[h->rx_count] = h->rx_byte;
        h->rx_count++;
        h->rx_buffer[h->rx_count] = '\0';
    }

    /* Re-arm interrupt for next byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);
}

void Bt_ResetRx(Bt_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    h->rx_count = 0U;
    h->rx_ready = false;
    memset(h->rx_buffer, 0, BT_RX_BUFFER_SIZE);
}

/* ========================  ADVERTISE API  ================================ */

BtStatus_e Bt_SendAdvertise(Bt_Handle_t *h)
{
    if (h == NULL) { return BT_ERR_PARAM; }

    Log_Print("BT", "Iniciando modo advertising ($CON)...");
    static const uint8_t cmd[] = "$CON\r";
    Bt_ResetRx(h);
    return Bt_Transmit(h, cmd, sizeof(cmd) - 1U);
}

/* ========================  SELF-TEST  ==================================== */

uint8_t Bt_Test(void)
{
    extern Bt_Handle_t hbt;

    static const uint8_t cmd[]  = "$AT\r";
    static       uint8_t resp[16];

    Bt_Init(&hbt);

    if (Bt_Transmit(&hbt, cmd, sizeof(cmd) - 1U) != BT_OK) { return 0U; }

    memset(resp, 0, sizeof(resp));
    HAL_UART_Receive(hbt.huart, resp, sizeof(resp) - 1U, BT_RX_TIMEOUT_MS);

    /* Busca "OK" en la respuesta */
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(resp) - 1U); i++) {
        if (resp[i] == 'O' && resp[i + 1U] == 'K') { return 1U; }
    }

    return 0U;
}
