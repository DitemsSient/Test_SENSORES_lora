/**
 * @file    Receptor_Infrarrojo_TSOP.c
 * @brief   Driver implementation for TSOP IR receiver via Timer Input Capture.
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Receptor_Infrarrojo_TSOP.h"

/* ======================  STATIC FUNCTIONS  ================================ */

/* Forward declarations */

static void     ResetTesterContext(ir_tester_t *tester);
static uint32_t GetTimerDelta(uint32_t current, uint32_t previous);
static bool     InRange(uint32_t value, uint32_t low, uint32_t high);

/**
 * @brief  Clears all tester context fields and sets first_edge = true.
 * @param  tester  Pointer to the tester context.
 */
static void ResetTesterContext(ir_tester_t *tester) {
    tester->last_edge_ts = 0U;
    tester->first_edge   = true;
    tester->raw_idx      = 0U;
    tester->buffer_full  = false;
    tester->edge_count   = 0U;
    memset((void *)tester->raw_dt, 0, sizeof(tester->raw_dt));
}

/**
 * @brief  Computes the timer count delta, correctly handling counter wrap-around.
 * @param  current   Current captured counter value.
 * @param  previous  Previous captured counter value.
 * @note   Unsigned subtraction handles both 16-bit and 32-bit timer overflow.
 */
static uint32_t GetTimerDelta(uint32_t current, uint32_t previous) {
    return (uint32_t)(current - previous);
}

/**
 * @brief  Checks whether a value falls within [low, high] inclusive.
 * @param  value  Value to test.
 * @param  low    Lower bound.
 * @param  high   Upper bound.
 */
static bool InRange(uint32_t value, uint32_t low, uint32_t high) {
    return (value >= low) && (value <= high);
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Compares space_us sequentially against all defined timing ranges
 *         and returns the matching ir_space_type_t classification.
 */
ir_space_type_t IR_ClassifySpace(uint16_t space_us) {
    if (space_us < IR_NOISE_THRESHOLD_US)                           return IR_SPACE_NOISE;
    if (InRange(space_us, IR_SPACE0_MIN_US, IR_SPACE0_MAX_US))     return IR_SPACE_BIT0;
    if (InRange(space_us, IR_SPACE1_MIN_US, IR_SPACE1_MAX_US))     return IR_SPACE_BIT1;
    if (InRange(space_us, IR_INTER_MIN_US,  IR_INTER_MAX_US))      return IR_SPACE_INTER;
    if (InRange(space_us, IR_SYNC_MIN_US,   IR_SYNC_MAX_US))       return IR_SPACE_SYNC;
    return IR_SPACE_UNKNOWN;
}

/**
 * @brief  XOR-folds all bytes in the buffer and returns the result.
 */
uint8_t IR_ComputeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t chk = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

/**
 * @brief  Copies the config into the receiver context, zeroes all state fields,
 *         calls ResetParser(), and starts Input Capture interrupts.
 */
ir_status_t IR_Init(ir_receiver_t *receiver, const ir_config_t *config) {
    if (receiver == NULL || config == NULL || config->htim == NULL) {
        return IR_ERR_NULL_PARAM;
    }

    receiver->config      = *config;
    receiver->initialized = false;
    receiver->last_error  = IR_OK;

    receiver->last_capture  = 0U;
    receiver->last_edge_ts  = 0U;
    receiver->first_edge    = true;
    receiver->data_pending  = false;
    receiver->pending_space = 0U;
    receiver->checksum      = false;

    ResetParser(receiver);

    HAL_StatusTypeDef st = HAL_TIM_IC_Start_IT(config->htim, config->tim_channel);
    if (st != HAL_OK) {
        receiver->last_error = IR_ERR_NOT_INIT;
        return IR_ERR_NOT_INIT;
    }

    receiver->initialized = true;
    return IR_OK;
}

/**
 * @brief  Stops Input Capture interrupts and zeros the entire receiver context.
 */
ir_status_t IR_DeInit(ir_receiver_t *receiver) {
    if (receiver == NULL) {
        return IR_ERR_NULL_PARAM;
    }

    HAL_TIM_IC_Stop_IT(receiver->config.htim, receiver->config.tim_channel);
    memset(receiver, 0, sizeof(ir_receiver_t));

    return IR_OK;
}

/**
 * @brief  Reads the captured timer value, determines whether the edge was a
 *         rising (end of MARK) or falling (end of SPACE) edge via GPIO state,
 *         and stores the SPACE duration in pending_space for IR_Process().
 *
 *         With the TSOP (inverted signal):
 *         - GPIO_PIN_SET  (RISING)  = end of MARK  → discard delta
 *         - GPIO_PIN_RESET (FALLING) = end of SPACE → store delta
 */
void IR_CaptureCallback(ir_receiver_t *receiver, TIM_HandleTypeDef *htim) {
    if (receiver == NULL || !receiver->initialized) return;
    if (htim->Instance != receiver->config.htim->Instance) return;

    uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, receiver->config.tim_channel);
    GPIO_PinState pin_state  = HAL_GPIO_ReadPin(receiver->config.tsop_port, receiver->config.tsop_pin);

    if (receiver->first_edge) {
        receiver->last_capture = current_capture;
        receiver->first_edge   = false;
        return;
    }

    uint32_t delta_us       = GetTimerDelta(current_capture, receiver->last_capture);
    receiver->last_capture  = current_capture;

    if (pin_state == GPIO_PIN_RESET) {
        /* End of SPACE — this carries the protocol information */
        if (delta_us < 0xFFFFU) {
            receiver->pending_space = (uint16_t)delta_us;
            receiver->data_pending  = true;
        }
    }
}

/**
 * @brief  Atomically reads pending_space, classifies it with IR_ClassifySpace(),
 *         and feeds it through the state machine (WAIT_SYNC → READ_BITS → WAIT_INTER).
 *         Calls IR_FrameReady_Callback() when a fully valid frame is assembled.
 */
ir_status_t IR_Process(ir_receiver_t *receiver) {
    if (receiver == NULL)          return IR_ERR_NULL_PARAM;
    if (!receiver->initialized)    return IR_ERR_NOT_INIT;
    if (!receiver->data_pending)   return IR_OK;

    uint16_t space_us       = receiver->pending_space;
    receiver->data_pending  = false;

    ir_space_type_t space_type = IR_ClassifySpace(space_us);

    if (space_type == IR_SPACE_NOISE)   return IR_OK;

    if (space_type == IR_SPACE_UNKNOWN) {
        receiver->last_error = IR_ERR_INVALID_BIT;
        ResetParser(receiver);
        return IR_ERR_INVALID_BIT;
    }

    switch (receiver->parser_state) {

        case IR_STATE_WAIT_SYNC: {
            if (space_type == IR_SPACE_SYNC) {
                ResetParser(receiver);
                receiver->parser_state = IR_STATE_READ_BITS;
            }
            break;
        }

        case IR_STATE_READ_BITS: {
            if (space_type == IR_SPACE_SYNC) {
                ResetParser(receiver);
                receiver->parser_state = IR_STATE_READ_BITS;
                break;
            }
            if (space_type == IR_SPACE_BIT0 || space_type == IR_SPACE_BIT1) {
                receiver->bit_buffer <<= 1U;
                if (space_type == IR_SPACE_BIT1) {
                    receiver->bit_buffer |= 0x01U;
                }
                receiver->bit_count++;

                if (receiver->bit_count == IR_BITS_PER_BYTE) {
                    if (receiver->frame_idx >= IR_MAX_FRAME_BYTES) {
                        receiver->last_error = IR_ERR_FRAME_TOO_LONG;
                        ResetParser(receiver);
                        return IR_ERR_FRAME_TOO_LONG;
                    }

                    receiver->frame_buf[receiver->frame_idx++] = receiver->bit_buffer;
                    receiver->bit_buffer   = 0U;
                    receiver->bit_count    = 0U;
                    receiver->parser_state = IR_STATE_WAIT_INTER;
                }
            } else {
                receiver->last_error = IR_ERR_INVALID_BIT;
                ResetParser(receiver);
                return IR_ERR_INVALID_BIT;
            }
            break;
        }

        case IR_STATE_WAIT_INTER: {
            if (space_type == IR_SPACE_INTER) {
                receiver->parser_state = IR_STATE_READ_BITS;
            } else if (space_type == IR_SPACE_SYNC) {
                uint8_t total_bytes = receiver->frame_idx;

                if (total_bytes < 3U) {
                    receiver->last_error = IR_ERR_FRAME_TOO_SHORT;
                    ResetParser(receiver);
                    return IR_ERR_FRAME_TOO_SHORT;
                }

                uint8_t data_len      = total_bytes - 1U;
                uint8_t rx_checksum   = receiver->frame_buf[data_len];
                uint8_t calc_checksum = IR_ComputeChecksum(receiver->frame_buf, data_len);

                if (rx_checksum != calc_checksum) {
                    receiver->last_error = IR_ERR_CHECKSUM;
                    ResetParser(receiver);
                    return IR_ERR_CHECKSUM;
                }

                /* Valid frame — deliver to application */
                receiver->checksum = true;
                //IR_FrameReady_Callback(receiver->frame_buf, data_len);

                return IR_OK;
            } else {
                receiver->last_error = IR_ERR_INVALID_BIT;
                ResetParser(receiver);
                return IR_ERR_INVALID_BIT;
            }
            break;
        }

        default: {
            ResetParser(receiver);
            break;
        }
    }

    return IR_OK;
}

/**
 * @brief  Returns receiver->last_error, or IR_ERR_NULL_PARAM if the pointer is NULL.
 */
ir_status_t IR_GetLastError(const ir_receiver_t *receiver) {
    if (receiver == NULL) return IR_ERR_NULL_PARAM;
    return receiver->last_error;
}

/**
 * @brief  Clears the tester context via ResetTesterContext().
 */
ir_status_t IR_Tester_Init(ir_tester_t *tester) {
    if (tester == NULL) return IR_ERR_NULL_PARAM;
    ResetTesterContext(tester);
    return IR_OK;
}

/**
 * @brief  Reads the captured value, computes the delta via GetTimerDelta(),
 *         and stores it in raw_dt[]. Sets buffer_full when the array is exhausted.
 */
void IR_Tester_CaptureCallback(ir_tester_t *tester, TIM_HandleTypeDef *htim,
                               const ir_config_t *config) {
    if (tester == NULL || htim == NULL || config == NULL) return;
    if (htim->Instance != config->htim->Instance)         return;
    if (tester->buffer_full)                              return;

    uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, config->tim_channel);
    tester->edge_count++;

    if (tester->first_edge) {
        tester->last_edge_ts = current_capture;
        tester->first_edge   = false;
        return;
    }

    uint32_t delta_us        = GetTimerDelta(current_capture, tester->last_edge_ts);
    tester->last_edge_ts     = current_capture;

    if (tester->raw_idx < IR_RAW_BUFFER_SIZE) {
        tester->raw_dt[tester->raw_idx++] = (delta_us < 0xFFFFU) ? (uint16_t)delta_us : 0xFFFFU;
    } else {
        tester->buffer_full = true;
    }
}

/**
 * @brief  Copies raw_dt[] into analyzed_dt[] and classifies each entry with
 *         IR_ClassifySpace(). Results are stored in static arrays for inspection
 *         in the debugger. Set a breakpoint after the loop to inspect them.
 */
void IR_Tester_Analyze(const ir_tester_t *tester) {
    if (tester == NULL) return;

    static ir_space_type_t space_type[IR_RAW_BUFFER_SIZE];
    static uint16_t        analyzed_dt[IR_RAW_BUFFER_SIZE];
    static uint32_t        analysis_count = 0U;

    analysis_count = tester->raw_idx;

    for (uint32_t i = 0U; i < tester->raw_idx; i++) {
        analyzed_dt[i] = tester->raw_dt[i];
        space_type[i]  = IR_ClassifySpace(tester->raw_dt[i]);
    }

    /* Suppress unused-variable warnings in Release build */
    (void)analyzed_dt;
    (void)space_type;
    (void)analysis_count;
}

/**
 * @brief  Delegates to ResetTesterContext() to prepare for a new capture.
 */
void IR_Tester_Reset(ir_tester_t *tester) {
    if (tester == NULL) return;
    ResetTesterContext(tester);
}

/**
 * @brief  Resets the parser to IR_STATE_WAIT_SYNC and clears all bit/frame
 *         accumulation fields without touching the configuration or error state.
 */
void ResetParser(ir_receiver_t *receiver) {
    receiver->parser_state = IR_STATE_WAIT_SYNC;
    receiver->bit_buffer   = 0U;
    receiver->bit_count    = 0U;
    receiver->frame_idx    = 0U;
    memset(receiver->frame_buf, 0, sizeof(receiver->frame_buf));
}
