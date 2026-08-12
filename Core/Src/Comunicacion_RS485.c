/**
 * @file    Comunicacion_RS485.c
 * @brief   Driver implementation for RS485 half-duplex UART communication.
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Comunicacion_RS485.h"
#include <string.h>

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Writes a uint16_t value in little-endian order to a byte buffer.
 * @param  dst  Destination buffer (at least 2 bytes).
 * @param  v    Value to write.
 */
static void put_u16_le(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8) & 0xFFU);
}

/**
 * @brief  Reads a little-endian uint16_t from a byte buffer.
 * @param  src  Source buffer (at least 2 bytes).
 */
static uint16_t get_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Sets the DE/RE pin HIGH to enable the bus driver for transmission.
 */
void RS485_SetTx(void) {
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Clears the DE/RE pin LOW to enable the bus receiver for reception.
 */
void RS485_SetRx(void) {
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Returns size_payload(dt) + SOF + checksum + EOF lengths.
 */
uint8_t size_frame(RS485_Data_t *dt) {
    return size_payload(dt) + RS485_EOF_LEN + RS485_SOF_LEN + RS485_CHECKSUM_LEN;
}

/**
 * @brief  Returns the serialized payload size: sizeof(RS485_Data_t) minus the
 *         pointer and length fields, plus the fixed RS485_TEXT_LEN.
 */
uint8_t size_payload(RS485_Data_t *dt) {
    return sizeof(RS485_Data_t) - sizeof(dt->text) - sizeof(dt->len_text)
           + RS485_TEXT_LEN;
}

/**
 * @brief  Stores the UART handle pointer into the RS485 handle struct.
 */
void RS485_Init(RS485_Handle_t *h, UART_HandleTypeDef *huart) {
    h->huart = huart;
}

/**
 * @brief  Sums all bytes modulo 256 and returns the result.
 */
uint8_t RS485_Checksum8(const uint8_t *data, size_t len) {
    uint32_t sum = 0U;
    for (size_t i = 0U; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFFU);
}

/**
 * @brief  Zeros the payload buffer, then packs cmd, var1 (LE), var2 (LE),
 *         flag, and text bytes into contiguous positions.
 */
void RS485_PackPayload(const RS485_Data_t *in, uint8_t *payload, size_t payload_len) {
    memset(payload, 0, payload_len);
    payload[0] = in->cmd;
    put_u16_le(&payload[1], in->var1);
    put_u16_le(&payload[3], in->var2);
    payload[5] = in->flag ? 1U : 0U;
    memcpy(&payload[6], in->text, in->len_text);
}

/**
 * @brief  Extracts cmd, var1, var2, flag, and text from a packed byte buffer
 *         into the destination structure.
 */
void RS485_UnpackPayload(RS485_Data_t *out, uint8_t *payload, size_t payload_len) {
    (void)payload_len;
    out->cmd  = payload[0];
    out->var1 = get_u16_le(&payload[1]);
    out->var2 = get_u16_le(&payload[3]);
    out->flag = (payload[5] != 0U);
    memcpy(out->text, &payload[6], out->len_text);
}

/**
 * @brief  Serializes the payload, appends the checksum, wraps with SOF/EOF,
 *         enables TX, transmits via HAL_UART_Transmit(), waits for TC flag,
 *         then switches back to RX mode.
 */
HAL_StatusTypeDef RS485_Send(RS485_Handle_t *h, RS485_Data_t *data) {
    const uint8_t payload_len = size_payload(data);
    const uint8_t frame_len   = size_frame(data);

    uint8_t payload[payload_len];
    RS485_PackPayload(data, payload, payload_len);

    uint8_t chk = RS485_Checksum8(payload, payload_len);

    uint8_t frame[frame_len];
    size_t  idx = 0U;

    frame[idx++] = RS485_SOF1;
    frame[idx++] = RS485_SOF2;
    memcpy(&frame[idx], payload, payload_len);
    idx += payload_len;
    frame[idx++] = chk;
    frame[idx++] = RS485_EOF1;
    frame[idx++] = RS485_EOF2;

    HAL_StatusTypeDef st;
    RS485_SetTx();
    st = HAL_UART_Transmit(h->huart, frame, (uint16_t)frame_len, RS485_TX_TIMEOUT_MS);
    while (__HAL_UART_GET_FLAG(h->huart, UART_FLAG_TC) == RESET) {}
    RS485_SetRx();

    return st;
}

/**
 * @brief  Validates SOF, validates EOF, extracts and verifies the checksum,
 *         then unpacks the payload into out. Resets frame_ready and clears
 *         the buffer on any error or successful parse.
 */
RS485_Status_t RS485_Parse(RS485_frame_t *in, RS485_Data_t *out) {
    const uint8_t payload_len = size_payload(out);
    const uint8_t frame_len   = size_frame(out);

    if (!in->frame_ready) return RS485_WAITING_FRAME;

    const uint8_t *frame = in->buff_frame;

    if (frame[0] != RS485_SOF1 || frame[1] != RS485_SOF2) {
        memset(in->buff_frame, 0, frame_len);
        in->frame_ready = false;
        return RS485_ERR_FRAME;
    }

    if (frame[frame_len - 2U] != RS485_EOF1 || frame[frame_len - 1U] != RS485_EOF2) {
        memset(in->buff_frame, 0, frame_len);
        in->frame_ready = false;
        return RS485_ERR_FRAME;
    }

    uint8_t payload[payload_len];
    memcpy(payload, &frame[2], payload_len);

    const uint8_t chk_rx   = frame[2U + payload_len];
    const uint8_t chk_calc = RS485_Checksum8(payload, payload_len);

    if (chk_calc != chk_rx) {
        memset(in->buff_frame, 0, frame_len);
        in->frame_ready = false;
        return RS485_ERR_CHECKSUM;
    }

    RS485_UnpackPayload(out, payload, payload_len);
    memset(in->buff_frame, 0, frame_len);
    in->frame_ready = false;

    return RS485_OK;
}

/**
 * @brief  Appends rx_byte to buff_frame and sets frame_ready when the expected
 *         frame length is reached. Does nothing if frame_ready is already set.
 */
void RS485_StoreByte(RS485_frame_t *frame) {
    RS485_Data_t aux;
    const uint8_t frame_len = size_frame(&aux);

    if (!frame) return;
    if (frame->frame_ready) return;

    if (frame->cont_frame < frame_len) {
        frame->buff_frame[frame->cont_frame++] = frame->rx_byte;
        if (frame->cont_frame >= frame_len) {
            frame->frame_ready = true;
            frame->cont_frame  = 0U;
        }
    }
}

/* ========================  SELF-TEST  ==================================== */

uint8_t RS485_Test(void)
{
    extern RS485_Handle_t hrs485;

    static char test_text[] = "TEST";

    RS485_Data_t data = {0};
    data.cmd      = 0x54U;     /* 'T' */
    data.text     = test_text;
    data.len_text = 4U;

    HAL_StatusTypeDef result = RS485_Send(&hrs485, &data);

    return (result == HAL_OK) ? 1U : 0U;
}
