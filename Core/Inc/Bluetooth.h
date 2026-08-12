/**
 * @file    Bluetooth.h
 * @brief   Driver for UART Bluetooth module on STM32L4xx.
 *
 * @details Provides basic UART transmit/receive interface for a Bluetooth
 *          module (e.g., HC-05, HC-06, HM-10, or similar).
 *          CubeMX configuration:
 *          - UART peripheral in Asynchronous mode (commonly 9600 or 38400 baud).
 *          - No hardware flow control required.
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* UART handle — update to match CubeMX .ioc */

#define BT_UART                 (&huart1)

/* Timeouts */

#define BT_TX_TIMEOUT_MS        500U    /**< Transmit timeout in ms          */
#define BT_RX_TIMEOUT_MS        500U    /**< Receive timeout in ms           */

/* Buffer sizes */

#define BT_TX_BUFFER_SIZE       256U    /**< Max transmit payload bytes      */
#define BT_RX_BUFFER_SIZE       256U    /**< Max receive payload bytes       */

/* Advertise protocol */

#define BT_ADVERTISE_TIMEOUT_MS 15000U  /**< Timeout waiting for $OK after $CON  */

/* ========================  ENUMERATIONS  ================================== */

/* Driver status / error codes */

typedef enum {
    BT_OK               = 0,    /**< Operation successful                    */
    BT_ERR_PARAM,               /**< Invalid or NULL parameter               */
    BT_ERR_UART,                /**< UART communication error                */
    BT_ERR_TIMEOUT,             /**< Operation timed out                     */
    BT_ERR_BUSY,                /**< Module busy                             */
    BT_ERR_OVERFLOW             /**< Buffer overflow                         */
} BtStatus_e;

/* ============================  STRUCTURES  ================================ */

/* Bluetooth driver control handle */

typedef struct {
    UART_HandleTypeDef *huart;          /**< CubeMX-generated UART handle    */
    uint8_t             tx_buffer[BT_TX_BUFFER_SIZE];   /**< Transmit buffer */
    uint8_t             rx_buffer[BT_RX_BUFFER_SIZE];   /**< Receive buffer  */
    uint16_t            rx_count;       /**< Bytes accumulated in rx_buffer  */
    uint8_t             rx_byte;        /**< Last byte from UART interrupt   */
    bool                rx_ready;       /**< true when data is available     */
} Bt_Handle_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the Bluetooth handle and binds it to the configured UART.
 * @param  h  Pointer to the Bluetooth handle.
 */
BtStatus_e Bt_Init(Bt_Handle_t *h);

/**
 * @brief  Transmits a data buffer through the Bluetooth module.
 * @param  h     Pointer to the Bluetooth handle.
 * @param  data  Pointer to the data to send.
 * @param  len   Number of bytes to send.
 */
BtStatus_e Bt_Transmit(Bt_Handle_t *h, const uint8_t *data, uint16_t len);

/**
 * @brief  Receives data from the Bluetooth module (blocking).
 * @param  h     Pointer to the Bluetooth handle.
 * @param  data  Destination buffer.
 * @param  len   Number of bytes to receive.
 */
BtStatus_e Bt_Receive(Bt_Handle_t *h, uint8_t *data, uint16_t len);

/**
 * @brief  Accumulates one received byte into the internal buffer.
 * @param  h  Pointer to the Bluetooth handle.
 * @note   Call from the UART receive ISR (HAL_UART_RxCpltCallback).
 */
void Bt_StoreByte(Bt_Handle_t *h);

/**
 * @brief  Resets the receive buffer and byte counter.
 * @param  h  Pointer to the Bluetooth handle.
 */
void Bt_ResetRx(Bt_Handle_t *h);

/* ========================  ADVERTISE API  ================================ */

/**
 * @brief  Sends "CON\r" to enter advertising mode and resets the rx buffer.
 * @param  h  Pointer to the Bluetooth handle.
 */
BtStatus_e Bt_SendAdvertise(Bt_Handle_t *h);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Sends "AT\r\n" and checks for "OK" in the response within timeout.
 * @return 1 if the module replies with "OK", 0 on timeout or error.
 */
uint8_t Bt_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */
