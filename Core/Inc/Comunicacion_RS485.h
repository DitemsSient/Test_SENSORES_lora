/**
 * @file    Comunicacion_RS485.h
 * @brief   Driver for RS485 half-duplex UART communication on STM32L4xx.
 *
 * @details Frame structure:
 *          [ SOF1 | SOF2 | ...payload... | CHK | EOF1 | EOF2 ]
 *          CubeMX configuration:
 *          - UART peripheral in Asynchronous mode at the desired baud rate.
 *          - RS485_DE_PORT/PIN configured as GPIO output for DE/RE control.
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef COMUNICACION_RS485_H
#define COMUNICACION_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* GPIO port and pin for the DE/RE direction control line */

#define RS485_DE_PORT               GPIOA
#define RS485_DE_PIN                GPIO_PIN_15

/* ====================  DEVICE CONSTANTS  ================================== */

/* Frame delimiters */

#define RS485_SOF1                  0x2DU   /**< Start of Frame byte 1 ('-') */
#define RS485_SOF2                  0x2AU   /**< Start of Frame byte 2 ('*') */
#define RS485_EOF1                  0x2DU   /**< End of Frame byte 1   ('-') */
#define RS485_EOF2                  0x2AU   /**< End of Frame byte 2   ('*') */

/* Frame field lengths and timeouts */

#define RS485_TEXT_LEN              10U     /**< Fixed text field length     */
#define RS485_CHECKSUM_LEN          1U      /**< Checksum field length       */
#define RS485_SOF_LEN               2U      /**< Start-of-frame length       */
#define RS485_EOF_LEN               2U      /**< End-of-frame length         */
#define RS485_TX_TIMEOUT_MS         200U    /**< Transmit timeout            */
#define RS485_RX_BYTE_TIMEOUT_MS    5U      /**< Per-byte receive timeout    */

/* ========================  ENUMERATIONS  ================================== */

/* Driver status / error codes */

typedef enum {
    RS485_OK            = 0,    /**< Operation successful                    */
    RS485_ERR_PARAM,            /**< Invalid or NULL parameter               */
    RS485_ERR_UART,             /**< UART communication error                */
    RS485_ERR_TIMEOUT,          /**< Operation timed out                     */
    RS485_ERR_CHECKSUM,         /**< Invalid frame checksum                  */
    RS485_ERR_FRAMING,          /**< Frame structure error                   */
    RS485_ERR_FRAME,            /**< SOF or EOF not found                    */
    RS485_WAITING_FRAME         /**< Frame not yet complete                  */
} RS485_Status_t;

/* ============================  STRUCTURES  ================================ */

/**
 * @brief Payload to transmit or receive.
 *
 * @note  Uses __attribute__((packed)) to eliminate compiler padding and
 *        guarantee the same byte layout in memory as in the serialized frame.
 *        Without it, the compiler may insert padding bytes that would corrupt
 *        the manual packing done in RS485_PackPayload().
 */
typedef struct __attribute__((packed)) {
    uint8_t  cmd;           /**< Command or message identifier              */
    uint16_t var1;          /**< Variable 1 (little-endian in frame)        */
    uint16_t var2;          /**< Variable 2 (little-endian in frame)        */
    bool     flag;          /**< Boolean flag                               */
    char    *text;          /**< Text pointer — no '\0' guarantee on receive */
    uint8_t  len_text;      /**< Actual text length                         */
} RS485_Data_t;

/* Byte-by-byte receive context */

typedef struct {
    uint8_t  cont_frame;    /**< Bytes accumulated in the buffer            */
    bool     frame_ready;   /**< true when a complete frame is available    */
    uint8_t  rx_byte;       /**< Last byte received from UART               */
    uint8_t *buff_frame;    /**< Pointer to the frame buffer                */
    uint8_t  len_frame;     /**< Expected total frame length                */
} RS485_frame_t;

/* UART handle wrapper for the RS485 bus */

typedef struct {
    UART_HandleTypeDef *huart;  /**< CubeMX-generated UART handle           */
} RS485_Handle_t;

/* ================================  API  =================================== */

/**
 * @brief  Drives the DE pin HIGH to put the bus in transmit mode.
 */
void RS485_SetTx(void);

/**
 * @brief  Drives the DE pin LOW to put the bus in receive mode.
 */
void RS485_SetRx(void);

/**
 * @brief  Computes the total frame size including SOF, payload, checksum, and EOF.
 * @param  dt  Pointer to the data structure.
 */
uint8_t size_frame(RS485_Data_t *dt);

/**
 * @brief  Computes the serialized payload size.
 * @param  dt  Pointer to the data structure.
 */
uint8_t size_payload(RS485_Data_t *dt);

/**
 * @brief  Binds the RS485 handle to a UART peripheral.
 * @param  h      Pointer to the RS485 handle.
 * @param  huart  CubeMX-generated UART handle.
 */
void RS485_Init(RS485_Handle_t *h, UART_HandleTypeDef *huart);

/**
 * @brief  Serializes an RS485_Data_t structure into a byte buffer.
 * @param  in           Source data structure.
 * @param  payload      Destination buffer.
 * @param  payload_len  Destination buffer length.
 */
void RS485_PackPayload(const RS485_Data_t *in, uint8_t *payload, size_t payload_len);

/**
 * @brief  Deserializes a byte buffer into an RS485_Data_t structure.
 * @param  out          Destination data structure.
 * @param  payload      Source buffer.
 * @param  payload_len  Source buffer length.
 */
void RS485_UnpackPayload(RS485_Data_t *out, uint8_t *payload, size_t payload_len);

/**
 * @brief  Computes a modulo-256 byte-sum checksum over a buffer.
 * @param  data  Data buffer.
 * @param  len   Buffer length in bytes.
 */
uint8_t RS485_Checksum8(const uint8_t *data, size_t len);

/**
 * @brief  Packs and transmits a complete RS485 frame.
 * @param  h     RS485 handle.
 * @param  data  Data to transmit.
 * @note   Returns HAL_OK if the transmission succeeded, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef RS485_Send(RS485_Handle_t *h, RS485_Data_t *data);

/**
 * @brief  Parses a complete accumulated frame and extracts the data payload.
 * @param  in   Receive context with the accumulated frame.
 * @param  out  Destination structure for the deserialized data.
 * @note   Call from the main loop when frame_ready == true.
 *         Returns RS485_OK if the frame is valid.
 */
RS485_Status_t RS485_Parse(RS485_frame_t *in, RS485_Data_t *out);

/**
 * @brief  Accumulates one received byte into the frame buffer.
 * @param  frame  Receive context.
 * @note   Call from the UART receive ISR.
 */
void RS485_StoreByte(RS485_frame_t *frame);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Builds a minimal test frame and transmits it via RS485_Send().
 * @return 1 if HAL_UART_Transmit returns HAL_OK, 0 otherwise.
 * @note   TX-only test — loopback verification is a future step.
 */
uint8_t RS485_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* COMUNICACION_RS485_H */
