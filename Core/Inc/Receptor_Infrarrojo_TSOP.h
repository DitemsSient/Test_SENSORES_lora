/**
 * @file    Receptor_Infrarrojo_TSOP.h
 * @brief   IR receiver driver for TSOP module via Timer Input Capture on STM32L4xx.
 *
 * @details Supports frames of 2–8 data bytes plus 1 XOR checksum byte.
 *          Scalable to any timer with Input Capture capability.
 *
 *          Frame structure:
 *          SYNC → [B0] → INTER → [B1] → INTER → … → [BN-1] → INTER → [CHK] → SYNC
 *
 *          Protocol timing (HIGH spaces between MARKs):
 *          MARK   = 500  µs  (LOW burst)
 *          SPACE0 = 600  µs  (HIGH → bit 0)
 *          SPACE1 = 1400 µs  (HIGH → bit 1)
 *          INTER  = 2500 µs  (HIGH → inter-byte separator)
 *          SYNC   = 4000 µs  (HIGH → frame start/end)
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef RECEPTOR_INFRARROJO_TSOP_H
#define RECEPTOR_INFRARROJO_TSOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ========================  CONFIGURATION  ================================= */

/* GPIO port and pin connected to each TSOP output (2 sensores en esta tarjeta) */

#define TSOP1_GPIO_Port          GPIOH
#define TSOP1_Pin                GPIO_PIN_3   /**< SENSOR_IR1 (PH3 / BOOT0) */

#define TSOP2_GPIO_Port          GPIOB
#define TSOP2_Pin                GPIO_PIN_0   /**< SENSOR_IR2 (PB0)         */

/* ====================  DEVICE CONSTANTS  ================================== */

/* Protocol timing constants in microseconds */

#define IR_MARK_US              400U    /**< LOW burst duration              */

/* Protocol timing — threshold-based (cascading "less than") ranges.
 * STM32 transmits: BIT0=400, BIT1=800, INTER=1200, SYNC=1600 µs.
 * Thresholds add +300 µs headroom above each nominal value.           */

#define IR_SPACE0_MIN_US        100U    /**< Noise floor                     */
#define IR_SPACE0_MAX_US        699U    /**< < 700  → BIT 0  (nom 400 µs)   */
#define IR_SPACE1_MIN_US        700U    /**< >= 700                          */
#define IR_SPACE1_MAX_US        1099U   /**< < 1100 → BIT 1  (nom 800 µs)   */
#define IR_INTER_MIN_US         1100U   /**< >= 1100                         */
#define IR_INTER_MAX_US         1499U   /**< < 1500 → INTER  (nom 1200 µs)  */
#define IR_SYNC_MIN_US          1500U   /**< >= 1500 → SYNC  (nom 1600 µs)  */
#define IR_SYNC_MAX_US          65000U  /**< Any large value                 */
#define IR_NOISE_THRESHOLD_US   100U    /**< Times below this are discarded  */

/* Calibration frame code — 3 data bytes, no CRC, sent by the transmitter */
#define IR_CAL_CODE             0xABCDEFUL

/* Buffer sizing — increase IR_MAX_DATA_BYTES to support longer frames */

#define IR_MAX_DATA_BYTES       8U                              /**< Max data bytes (excl. checksum) */
#define IR_MAX_FRAME_BYTES      (IR_MAX_DATA_BYTES + 1U)       /**< +1 for checksum                 */
#define IR_BITS_PER_BYTE        8U
#define IR_RAW_BUFFER_SIZE      128U                           /**< Raw buffer size (Tester mode)   */

/* ========================  ENUMERATIONS  ================================== */

/* Driver status / error codes */

typedef enum {
    IR_OK                   = 0x00U, /**< Operation successful              */
    IR_ERR_NULL_PARAM       = 0x01U, /**< NULL pointer received             */
    IR_ERR_NOT_INIT         = 0x02U, /**< Driver not initialized            */
    IR_ERR_BUFFER_FULL      = 0x03U, /**< Capture buffer full               */
    IR_ERR_CHECKSUM         = 0x04U, /**< Invalid frame checksum            */
    IR_ERR_INVALID_BIT      = 0x05U, /**< Time cannot be classified as bit  */
    IR_ERR_FRAME_TOO_LONG   = 0x06U, /**< Frame exceeds IR_MAX_DATA_BYTES   */
    IR_ERR_FRAME_TOO_SHORT  = 0x07U, /**< Frame has fewer than 2 data bytes */
    IR_BUSY                 = 0x08U, /**< Parser is currently processing    */
} ir_status_t;

/* Classification of a received HIGH space */

typedef enum {
    IR_SPACE_NOISE  = 0,    /**< Below noise threshold, discarded          */
    IR_SPACE_BIT0   = 1,    /**< Classified as bit 0                       */
    IR_SPACE_BIT1   = 2,    /**< Classified as bit 1                       */
    IR_SPACE_INTER  = 3,    /**< Inter-byte separator                      */
    IR_SPACE_SYNC   = 4,    /**< Frame start or end SYNC                   */
    IR_SPACE_UNKNOWN= 5,    /**< Time falls outside all defined ranges     */
} ir_space_type_t;

/* Internal parser state machine states */

typedef enum {
    IR_STATE_WAIT_SYNC  = 0,    /**< Waiting for start SYNC                */
    IR_STATE_READ_BITS  = 1,    /**< Reading 8 bits of the current byte    */
    IR_STATE_WAIT_INTER = 2,    /**< Waiting for INTER or end SYNC         */
} ir_parser_state_t;

/* ============================  STRUCTURES  ================================ */

/* Hardware configuration — filled by the user and passed to IR_Init() */

typedef struct {
    TIM_HandleTypeDef *htim;        /**< Timer handle with Input Capture   */
    uint32_t           tim_channel; /**< Timer channel (TIM_CHANNEL_x)     */
    GPIO_TypeDef       *tsop_port;  /**< GPIO port of this TSOP output (TSOP1_GPIO_Port / TSOP2_GPIO_Port) */
    uint16_t            tsop_pin;   /**< GPIO pin of this TSOP output (TSOP1_Pin / TSOP2_Pin)              */
} ir_config_t;

/**
 * @brief Internal receiver context.
 * @note  Do not manipulate directly from the application.
 */
typedef struct {
    /* Peripheral configuration */
    ir_config_t         config;
    bool                initialized;

    /* Raw capture state */
    volatile uint32_t   last_capture;
    volatile uint32_t   last_edge_ts;
    volatile bool       first_edge;
    volatile bool       last_was_falling;

    /* Parser state */
    ir_parser_state_t   parser_state;
    uint8_t             bit_buffer;
    uint8_t             bit_count;
    uint8_t             frame_buf[IR_MAX_FRAME_BYTES];
    uint8_t             frame_idx;
    volatile bool       data_pending;
    uint16_t            pending_space;

    /* Error tracking */
    volatile bool       checksum;       /**< true if last checksum was valid */
    ir_status_t         last_error;
} ir_receiver_t;

/* Tester mode context — captures raw timings for calibration */

typedef struct {
    volatile uint32_t   last_edge_ts;
    volatile bool       first_edge;
    volatile uint16_t   raw_dt[IR_RAW_BUFFER_SIZE]; /**< Space durations in µs */
    volatile uint32_t   raw_idx;
    volatile bool       buffer_full;
    uint32_t            edge_count;
} ir_tester_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the IR receiver with the given peripheral configuration.
 * @param  receiver  Pointer to the receiver context.
 * @param  config    Pointer to the hardware configuration structure.
 * @note   Returns IR_OK on success, error code otherwise.
 */
ir_status_t IR_Init(ir_receiver_t *receiver, const ir_config_t *config);

/**
 * @brief  Deinitializes and clears the receiver context.
 * @param  receiver  Pointer to the receiver context.
 */
ir_status_t IR_DeInit(ir_receiver_t *receiver);

/**
 * @brief  Input capture callback — call from HAL_TIM_IC_CaptureCallback().
 * @param  receiver  Pointer to the receiver context.
 * @param  htim      Timer handle that fired the interrupt.
 */
void IR_CaptureCallback(ir_receiver_t *receiver, TIM_HandleTypeDef *htim);

/**
 * @brief  Processes any pending capture data from the buffer.
 * @param  receiver  Pointer to the receiver context.
 * @note   Call from the main loop. Returns IR_OK if no errors occurred.
 */
ir_status_t IR_Process(ir_receiver_t *receiver);

/**
 * @brief  User callback invoked when a valid frame is fully received.
 * @param  data  Pointer to the data bytes (checksum excluded).
 * @param  len   Number of valid data bytes.
 * @note   Implement this function in main.c.
 */
void IR_FrameReady_Callback(const uint8_t *data, uint8_t len);

/**
 * @brief  Returns the last recorded error code.
 * @param  receiver  Pointer to the receiver context.
 */
ir_status_t IR_GetLastError(const ir_receiver_t *receiver);

/* Tester mode API */

/**
 * @brief  Initializes the Tester mode context.
 * @param  tester  Pointer to the tester context.
 */
ir_status_t IR_Tester_Init(ir_tester_t *tester);

/**
 * @brief  Tester capture callback — call instead of IR_CaptureCallback() in Tester mode.
 * @param  tester  Pointer to the tester context.
 * @param  htim    Timer handle that fired the interrupt.
 * @param  config  Peripheral configuration (used to validate the channel).
 */
void IR_Tester_CaptureCallback(ir_tester_t *tester, TIM_HandleTypeDef *htim,
                               const ir_config_t *config);

/**
 * @brief  Classifies all times stored in raw_dt[] and prints the detected space type.
 * @param  tester  Pointer to the tester context.
 * @note   Useful for calibrating tolerances via UART/SWO.
 */
void IR_Tester_Analyze(const ir_tester_t *tester);

/**
 * @brief  Resets the raw buffer for a new capture session.
 * @param  tester  Pointer to the tester context.
 */
void IR_Tester_Reset(ir_tester_t *tester);

/* Internal utility functions (accessible for testing) */

/**
 * @brief  Classifies a space duration according to the protocol.
 * @param  space_us  Space duration in microseconds.
 */
ir_space_type_t IR_ClassifySpace(uint16_t space_us);

/**
 * @brief  Computes the XOR checksum of a byte buffer.
 * @param  data  Pointer to the data buffer.
 * @param  len   Number of bytes to process.
 */
uint8_t IR_ComputeChecksum(const uint8_t *data, uint8_t len);

/**
 * @brief  Resets the parser state machine to wait for the next SYNC.
 * @param  receiver  Pointer to the receiver context.
 */
void ResetParser(ir_receiver_t *receiver);

#ifdef __cplusplus
}
#endif

#endif /* RECEPTOR_INFRARROJO_TSOP_H */
