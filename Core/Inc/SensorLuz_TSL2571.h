/**
 * @file    SensorLuz_TSL2571.h
 * @brief   Driver for TSL2571 Ambient Light Sensor (ALS) via I2C on STM32L4xx.
 *
 * @details Provides register-level access and a high-level Lux calculation API.
 *          The I2C handle is passed at init, making the driver bus-agnostic.
 *          CubeMX configuration:
 *          - Enable an I2C peripheral in Standard or Fast mode.
 *          - TSL2571 default 7-bit address: 0x39.
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef SENSORLUZ_TSL2571_H
#define SENSORLUZ_TSL2571_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* I2C address — change if ADDR pin is reconfigured */

#define TSL2571_ADDR_7BIT       (0x39U)
#define TSL2571_ADDR_HAL        (TSL2571_ADDR_7BIT << 1)   /**< HAL expects 8-bit */

/* ====================  DEVICE CONSTANTS  ================================== */

/* Register map */

#define TSL2571_REG_ENABLE      0x00U   /**< Power / ALS enable                  */
#define TSL2571_REG_ATIME       0x01U   /**< ALS integration time                */
#define TSL2571_REG_CONTROL     0x0FU   /**< ALS gain control                    */
#define TSL2571_REG_ID          0x12U   /**< Device ID (read-only)               */
#define TSL2571_REG_STATUS      0x13U   /**< ALS status                          */
#define TSL2571_REG_C0DATAL     0x14U   /**< CH0 low byte (auto-inc: 0x14–0x17)  */
#define TSL2571_REG_C0DATAH     0x15U
#define TSL2571_REG_C1DATAL     0x16U
#define TSL2571_REG_C1DATAH     0x17U

/* Register bit masks */

#define TSL2571_ENABLE_PON      0x01U   /**< Power ON                            */
#define TSL2571_ENABLE_AEN      0x02U   /**< ALS Enable                          */

/* Command byte prefixes (datasheet Table 1) */

#define TSL2571_CMD_BYTE        0x80U   /**< Single register  : 100x xxxx        */
#define TSL2571_CMD_AUTO        0xA0U   /**< Auto-increment   : 101x xxxx        */

/* ========================  ENUMERATIONS  ================================== */

/* CONTROL register [1:0] gain options */

typedef enum {
    TSL2571_GAIN_1X   = 0x00U,
    TSL2571_GAIN_8X   = 0x01U,
    TSL2571_GAIN_16X  = 0x02U,
    TSL2571_GAIN_120X = 0x03U
} TSL2571_Gain;

/* ============================  STRUCTURES  ================================ */

/* Device handle — one instance per TSL2571 on the bus */

typedef struct {
    I2C_HandleTypeDef *hi2c;        /**< Pointer to HAL I2C handle             */
    uint8_t            addr;        /**< 7-bit I2C address (e.g. 0x39)         */
    uint32_t           timeout_ms;  /**< HAL timeout for I2C operations         */
    uint8_t            atime;       /**< Current ATIME register value           */
    TSL2571_Gain       gain;        /**< Current gain setting                   */
} TSL2571_t;

/* Raw channel readings from a single conversion */

typedef struct {
    uint16_t ch0;           /**< Broadband channel (visible + IR)          */
    uint16_t ch1;           /**< Infrared-only channel                     */
    bool     saturated;     /**< true if either channel saturated          */
} TSL2571_RawData_t;

/* ================================  API  =================================== */

/**
 * @brief  Binds the driver handle to an I2C bus and stores configuration.
 * @param  dev         Pointer to device handle.
 * @param  hi2c        I2C peripheral handle.
 * @param  addr        7-bit I2C address.
 * @param  timeout_ms  HAL timeout in milliseconds.
 * @note   Call before any other function.
 */
void TSL2571_Attach(TSL2571_t *dev, I2C_HandleTypeDef *hi2c,
                    uint8_t addr, uint32_t timeout_ms);

/**
 * @brief  Powers on the sensor, applies ATIME and gain, and enables ALS.
 * @param  dev    Pointer to device handle.
 * @param  atime  ATIME register value (integration time).
 * @param  gain   Gain selection.
 * @note   Blocks for one integration cycle before returning.
 */
HAL_StatusTypeDef TSL2571_Begin(TSL2571_t *dev, uint8_t atime, TSL2571_Gain gain);

/**
 * @brief  Writes a single register via I2C.
 * @param  dev  Pointer to device handle.
 * @param  reg  Register address.
 * @param  val  Value to write.
 */
HAL_StatusTypeDef TSL2571_WriteReg(TSL2571_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief  Reads a single register via I2C.
 * @param  dev  Pointer to device handle.
 * @param  reg  Register address.
 * @param  val  Pointer to store the read value.
 */
HAL_StatusTypeDef TSL2571_ReadReg(TSL2571_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief  Enables ALS (sets PON and AEN bits).
 * @param  dev  Pointer to device handle.
 */
HAL_StatusTypeDef TSL2571_Enable(TSL2571_t *dev);

/**
 * @brief  Disables ALS (clears PON and AEN bits).
 * @param  dev  Pointer to device handle.
 */
HAL_StatusTypeDef TSL2571_Disable(TSL2571_t *dev);

/**
 * @brief  Updates the gain register and stores the new value in the handle.
 * @param  dev   Pointer to device handle.
 * @param  gain  New gain setting.
 */
HAL_StatusTypeDef TSL2571_SetGain(TSL2571_t *dev, TSL2571_Gain gain);

/**
 * @brief  Updates the integration time register and stores it in the handle.
 * @param  dev    Pointer to device handle.
 * @param  atime  New ATIME register value.
 */
HAL_StatusTypeDef TSL2571_SetATime(TSL2571_t *dev, uint8_t atime);

/**
 * @brief  Reads both raw ADC channels in a single auto-increment I2C transaction.
 * @param  dev   Pointer to device handle.
 * @param  data  Pointer to TSL2571_RawData_t to fill.
 */
HAL_StatusTypeDef TSL2571_ReadRawChannels(TSL2571_t *dev, TSL2571_RawData_t *data);

/**
 * @brief  Averages N readings and converts the result to Lux.
 * @param  dev       Pointer to device handle.
 * @param  nSamples  Number of readings to average (1–255).
 * @param  gapMs     Delay between samples in ms (should be >= integration time).
 * @param  lux       Output: calculated lux value.
 * @param  raw       Output: raw channel data from the last reading.
 */
HAL_StatusTypeDef TSL2571_ReadLux(TSL2571_t *dev, uint8_t nSamples,
                                  uint16_t gapMs, float *lux,
                                  TSL2571_RawData_t *raw);

/**
 * @brief  Calculates the integration time in ms from an ATIME register value.
 * @param  atime  ATIME register value.
 * @note   Formula: Tint = 2.7296 × (256 − ATIME) ms.
 */
float TSL2571_IntegrationMs(uint8_t atime);

/**
 * @brief  Returns the maximum valid ADC count for a given ATIME, capped at 65535.
 * @param  atime  ATIME register value.
 */
uint16_t TSL2571_MaxCount(uint8_t atime);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Reads one raw-channel sample and checks I2C ACK + ch0 > 0.
 * @return 1 if the sensor responds and returns a non-zero reading, 0 otherwise.
 * @note   Sensor must already be initialised with TSL2571_Begin().
 */
uint8_t TSL2571_Test(void);

#endif /* SENSORLUZ_TSL2571_H */
