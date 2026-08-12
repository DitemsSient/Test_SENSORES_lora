/**
 * @file    Lora.h
 * @brief   Driver for LoRa UART module on STM32L4xx.
 *
 * @details Provides basic UART transmit/receive interface for a LoRa module.
 *          CubeMX configuration:
 *          - UART peripheral in Asynchronous mode at the desired baud rate
 *            (typically 9600 or 115200).
 *          - No hardware flow control required.
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* UART handle — update to match CubeMX .ioc */

#define LORA_UART               (&huart2)

/* Timeouts */

#define LORA_TX_TIMEOUT_MS      500U    /**< Transmit timeout in ms          */
#define LORA_RX_TIMEOUT_MS      500U    /**< Receive timeout in ms           */
#define LORA_WAKE_DELAY_MS      100U    /**< Wait after wakeup byte (ms)     */

/* Buffer sizes */

#define LORA_TX_BUFFER_SIZE     256U    /**< Max transmit payload bytes      */
#define LORA_RX_BUFFER_SIZE     256U    /**< Max receive payload bytes       */

/* ========================  ENUMERATIONS  ================================== */

/* Driver status / error codes */

typedef enum {
    LORA_OK             = 0,    /**< Operation successful                    */
    LORA_ERR_PARAM,             /**< Invalid or NULL parameter               */
    LORA_ERR_UART,              /**< UART communication error                */
    LORA_ERR_TIMEOUT,           /**< Operation timed out                     */
    LORA_ERR_BUSY,              /**< Module busy                             */
    LORA_ERR_OVERFLOW           /**< Buffer overflow                         */
} LoraStatus_e;

/* ============================  STRUCTURES  ================================ */

/* LoRa driver control handle */

typedef struct {
    UART_HandleTypeDef *huart;          /**< CubeMX-generated UART handle    */
    uint8_t             tx_buffer[LORA_TX_BUFFER_SIZE]; /**< Transmit buffer */
    uint8_t             rx_buffer[LORA_RX_BUFFER_SIZE]; /**< Receive buffer  */
    uint16_t            rx_count;       /**< Bytes accumulated in rx_buffer  */
    uint8_t             rx_byte;        /**< Last byte from UART interrupt   */
    bool                rx_ready;       /**< true when data is available     */
} Lora_Handle_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the LoRa handle and binds it to the configured UART.
 * @param  h  Pointer to the LoRa handle.
 */
LoraStatus_e Lora_Init(Lora_Handle_t *h);

/**
 * @brief  Transmits a data buffer through the LoRa module.
 * @param  h     Pointer to the LoRa handle.
 * @param  data  Pointer to the data to send.
 * @param  len   Number of bytes to send.
 */
LoraStatus_e Lora_Transmit(Lora_Handle_t *h, const uint8_t *data, uint16_t len);

/**
 * @brief  Receives data from the LoRa module (blocking).
 * @param  h     Pointer to the LoRa handle.
 * @param  data  Destination buffer.
 * @param  len   Number of bytes to receive.
 */
LoraStatus_e Lora_Receive(Lora_Handle_t *h, uint8_t *data, uint16_t len);

/**
 * @brief  Accumulates one received byte into the internal buffer.
 * @param  h  Pointer to the LoRa handle.
 * @note   Call from the UART receive ISR (HAL_UART_RxCpltCallback).
 */
void Lora_StoreByte(Lora_Handle_t *h);

/**
 * @brief  Resets the receive buffer and byte counter.
 * @param  h  Pointer to the LoRa handle.
 */
void Lora_ResetRx(Lora_Handle_t *h);

/**
 * @brief  Sends the sleep AT command to the RM1262 module.
 * @param  h  Pointer to the LoRa handle.
 * @note   Primary command: AT+SLEEP. Wake up by sending any byte on UART.
 *         If the module does not respond, try the alternative commands
 *         listed in the source file.
 */
LoraStatus_e Lora_Sleep(Lora_Handle_t *h);

/**
 * @brief  Wakes the RM1262 from sleep by sending a dummy byte on UART.
 * @param  h  Pointer to the LoRa handle.
 * @note   Waits LORA_WAKE_DELAY_MS for the module to become ready before
 *         returning. Call before any transmit or receive operation.
 */
LoraStatus_e Lora_WakeUp(Lora_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* LORA_H */
