/**
 * @file    MMC5983MA.h
 * @brief   3-axis magnetometer driver for MEMSIC MMC5983MA over I2C on STM32F4xx.
 *
 * @details Configures the MMC5983MA for continuous measurement at 10 Hz with
 *          automatic SET/RESET enabled (auto-demagnetization every measurement).
 *          A manual SET pulse is also issued during Init to guarantee a clean
 *          initial state and after saturation is detected.
 *
 *          18-bit resolution per axis: the driver assembles the two high bytes
 *          plus the two-bit LSB from the shared register into a single uint32_t
 *          and then converts to µT using the device sensitivity (16384 counts/G,
 *          i.e. 163.84 counts/µT).
 *
 *          CubeMX / .ioc requirements:
 *          - I2C1 enabled, Fast Mode 400 kHz recommended (Standard 100 kHz also
 *            works).
 *          - No additional GPIO required (address pin SA0 tied to GND → 0x30).
 *
 * @date    June 12, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* Handle, I2C address and timeout. Must match the .ioc configuration */

#define MMC_I2C_HANDLE          (&hi2c1)
#define MMC_I2C_ADDR            (0x30U << 1U)   /**< SA0=GND → 7-bit 0x30 */
#define MMC_I2C_TIMEOUT         100U

/* Continuous measurement ODR selection (written to CTRL1 ODR[2:0]) */

#define MMC_ODR_1HZ             0x01U
#define MMC_ODR_10HZ            0x02U
#define MMC_ODR_20HZ            0x03U
#define MMC_ODR_50HZ            0x04U
#define MMC_ODR_100HZ           0x05U

/* Active ODR for this application */

#define MMC_ODR_ACTIVE          MMC_ODR_10HZ

/* Saturation threshold: if any axis raw count is within this margin of 0 or   */
/* 2^18-1 (262143) the sensor is considered saturated and a SET is issued.      */

#define MMC_SAT_MARGIN          1000U

/* ====================  DEVICE CONSTANTS  ================================== */

/* Register map */

#define MMC_REG_XOUT0           0x00U   /**< X-axis output bits[17:10]       */
#define MMC_REG_XOUT1           0x01U   /**< X-axis output bits[9:2]         */
#define MMC_REG_YOUT0           0x02U   /**< Y-axis output bits[17:10]       */
#define MMC_REG_YOUT1           0x03U   /**< Y-axis output bits[9:2]         */
#define MMC_REG_ZOUT0           0x04U   /**< Z-axis output bits[17:10]       */
#define MMC_REG_ZOUT1           0x05U   /**< Z-axis output bits[9:2]         */
#define MMC_REG_XYZ_OUT2        0x06U   /**< XYZ bits[1:0] packed            */
#define MMC_REG_STATUS          0x08U   /**< Device status                   */
#define MMC_REG_CTRL0           0x09U   /**< Control 0: SET/RESET/measure    */
#define MMC_REG_CTRL1           0x0AU   /**< Control 1: BW, ODR, CMM         */
#define MMC_REG_CTRL2           0x0BU   /**< Control 2: CMM_EN, auto SR      */
#define MMC_REG_PRODUCT_ID      0x2FU   /**< Product ID, expected 0x30       */

/* Bit masks — CTRL0 */

#define MMC_CTRL0_TM_M          0x01U   /**< Trigger single measurement      */
#define MMC_CTRL0_SET           0x08U   /**< Perform SET pulse               */
#define MMC_CTRL0_RESET         0x10U   /**< Perform RESET pulse             */
#define MMC_CTRL0_AUTO_SR       0x20U   /**< Auto SET/RESET each measurement */

/* Bit masks — CTRL2 */

#define MMC_CTRL2_CMM_EN        0x10U   /**< Enable continuous measurement   */

/* Bit masks — STATUS */

#define MMC_STATUS_MEAS_DONE    0x01U   /**< New data ready                  */

/* Device identification */

#define MMC_PRODUCT_ID          0x30U

/* Sensitivity: 16384 counts per Gauss = 163.84 counts per µT               */

#define MMC_COUNTS_PER_UT       163.84f

/* Zero-field output code (midscale for 18-bit signed representation)         */

#define MMC_ZERO_CODE           131072U /**< 2^17, zero-field reference       */

/* ====================  ENUMERATIONS  ====================================== */

/* Return codes for all MMC5983MA functions */

typedef enum {
    MMC_OK              = 0,
    MMC_ERR_COM         = 1,    /**< I2C transaction failed              */
    MMC_ERR_ID          = 2,    /**< Product ID mismatch                 */
    MMC_ERR_TIMEOUT     = 3,    /**< Measurement not ready in time       */
    MMC_ERR_PARAM       = 4     /**< Invalid parameter                   */
} MMC_Status_e;

/* ============================  STRUCTURES  ================================ */

/* Structure to hold one magnetometer reading */

typedef struct {
    float   x_uT;       /**< Magnetic field X axis in µT             */
    float   y_uT;       /**< Magnetic field Y axis in µT             */
    float   z_uT;       /**< Magnetic field Z axis in µT             */
    uint32_t x_raw;     /**< Raw 18-bit count X (0–262143)           */
    uint32_t y_raw;     /**< Raw 18-bit count Y (0–262143)           */
    uint32_t z_raw;     /**< Raw 18-bit count Z (0–262143)           */
} MMC_Data_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the MMC5983MA.
 * @note   Issues a SET pulse, verifies Product ID, then enables continuous
 *         measurement at MMC_ODR_ACTIVE with auto SET/RESET per measurement.
 *         Call after MX_I2C1_Init().
 */
MMC_Status_e MMC5983MA_Init(void);

/**
 * @brief  Reads the latest magnetic field data from the sensor.
 * @param  out  Pointer to MMC_Data_t to fill with raw counts and µT values.
 * @note   In continuous mode data is always ready; the function reads the
 *         7-byte output burst (XOUT0–XYZ_OUT2) in a single I2C transaction.
 *         If any axis is saturated a SET pulse is issued automatically before
 *         returning the data.
 */
MMC_Status_e MMC5983MA_ReadAll(MMC_Data_t *out);

/**
 * @brief  Issues a SET pulse to demagnetize the sensing element.
 * @note   Blocks ~1 ms. Call after exposure to a strong external field or
 *         when saturation is detected. Automatically called by Init.
 */
MMC_Status_e MMC5983MA_Set(void);

/**
 * @brief  Reads the Product ID register and verifies it equals 0x30.
 * @param  id  Pointer to receive the raw ID byte.
 */
MMC_Status_e MMC5983MA_WhoAmI(uint8_t *id);

/**
 * @brief  Basic self-test: verifies WHO_AM_I and confirms data registers
 *         change between two consecutive readings.
 * @note   Called by the TestHW menu. Returns 1 on pass, 0 on fail.
 */
uint8_t MMC5983MA_Test(void);

#endif /* MMC5983MA_H */
