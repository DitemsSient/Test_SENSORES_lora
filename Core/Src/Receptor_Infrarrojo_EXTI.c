/**
 * @file    Receptor_Infrarrojo_EXTI.c
 * @brief   Driver implementation for TSOP IR receiver via GPIO EXTI + DWT.
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Receptor_Infrarrojo_EXTI.h"
#include <string.h>

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Computes the cycle delta, correctly handling DWT->CYCCNT wrap-around
 *         (32-bit unsigned subtraction wraps safely on its own).
 * @param  current   DWT->CYCCNT at the current edge.
 * @param  previous  DWT->CYCCNT at the previous edge.
 */
static uint32_t IR_GetCycleDelta(uint32_t current, uint32_t previous) {
    return (uint32_t)(current - previous);
}

/**
 * @brief  Appends one decoded bit (MSB-first) into the byte being built; on
 *         the 8th bit, closes it into frame_buf and starts the next byte.
 * @param  h    Pointer to the driver handle.
 * @param  bit  0 or 1, already classified by the caller.
 * @note   Called only from IR_EXTI_Callback() — no NULL check, hot path.
 */
static void IR_PushBit(Ir_Handle_t *h, uint8_t bit) {
    h->current_byte = (uint8_t)((h->current_byte << 1U) | (bit & 0x01U));
    h->bit_count++;

    if (h->bit_count >= IR_BITS_PER_BYTE) {
        if (h->frame_len < IR_FRAME_MAX_BYTES) {
            h->frame_buf[h->frame_len] = h->current_byte;
            h->frame_len++;
        } else {
            h->frame_overflow = true;
        }
        h->current_byte = 0U;
        h->bit_count    = 0U;
    }
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

IrStatus_e IR_Init(Ir_Handle_t *h) {
    if (h == NULL) return IR_ERR_PARAM;

    /* Habilita el bloque de trace/debug del nucleo y arranca DWT->CYCCNT. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    memset((void *)h, 0, sizeof(Ir_Handle_t));
    h->first_edge     = true;
    h->last_edge_tick = HAL_GetTick();

    return IR_OK;
}

/**
 * @brief  Reads DWT->CYCCNT, converts the delta to microseconds, and — only
 *         on the edge where the pin ends up LOW (end of SPACE) — classifies
 *         it as bit 0 / bit 1 / byte separator and updates the frame buffer.
 *         The edge where the pin ends up HIGH (end of MARK) only updates
 *         the timing reference for the next delta.
 */
void IR_EXTI_Callback(Ir_Handle_t *h) {
    if (h == NULL) return;

    uint32_t now_cycles = DWT->CYCCNT;
    h->last_edge_tick = HAL_GetTick();

    if (h->first_edge) {
        h->last_edge_cycles = now_cycles;
        h->first_edge        = false;
        return;
    }

    uint32_t delta_cycles  = IR_GetCycleDelta(now_cycles, h->last_edge_cycles);
    h->last_edge_cycles    = now_cycles;

    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t delta_us      = delta_cycles / cycles_per_us;

    if (HAL_GPIO_ReadPin(IR_TSOP_PORT, IR_TSOP_PIN) != GPIO_PIN_RESET) {
        /* Fin de MARK — se descarta, solo era referencia de tiempo. */
        return;
    }

    /* Fin de SPACE — guarda el delta crudo (diagnostico) y clasifica. */
    if (h->raw_count < IR_RAW_MAX) {
        h->raw_dt[h->raw_count] = (delta_us < 0xFFFFU) ? (uint16_t)delta_us : 0xFFFFU;
        h->raw_count++;
    } else {
        h->raw_overflow = true;
    }

    if (delta_us < IR_BIT0_MAX_US) {
        IR_PushBit(h, 0U);
    } else if (delta_us < IR_BIT1_MAX_US) {
        IR_PushBit(h, 1U);
    } else {
        /* Separador de byte/trama: si habia bits sueltos sin completar un
         * byte, se descartan (trama mal alineada, no hay forma de saber a
         * que byte pertenecian). */
        h->current_byte = 0U;
        h->bit_count     = 0U;
    }
}

/**
 * @brief  Detects end-of-frame by silence (IR_SILENCE_MS with no new edges)
 *         and, if bytes were captured, marks the frame ready for the caller.
 */
IrStatus_e IR_Process(Ir_Handle_t *h) {
    if (h == NULL) return IR_ERR_PARAM;
    if (h->frame_ready) return IR_OK;
    if (h->raw_count == 0U) return IR_ERR_NO_FRAME;

    uint32_t elapsed_ms = HAL_GetTick() - h->last_edge_tick;
    if (elapsed_ms < IR_SILENCE_MS) return IR_ERR_NO_FRAME;

    h->frame_ready = true;
    return IR_OK;
}

void IR_Reset(Ir_Handle_t *h) {
    if (h == NULL) return;

    h->frame_len      = 0U;
    h->frame_overflow = false;
    h->current_byte   = 0U;
    h->bit_count      = 0U;
    h->raw_count      = 0U;
    h->raw_overflow   = false;
    h->frame_ready    = false;
    h->first_edge     = true;
}

uint8_t IR_Test(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    uint32_t after = DWT->CYCCNT;

    return (after != before) ? 1U : 0U;
}
