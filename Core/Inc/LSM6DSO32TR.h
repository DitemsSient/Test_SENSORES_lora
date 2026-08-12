/**
 * @file    LSM6DSO32TR.h
 * @brief   6-axis IMU driver for ST LSM6DSO32TR (accel + gyro) over I2C on STM32L4xx.
 *
 * @details Configures the LSM6DSO32TR accelerometer and gyroscope with a fixed
 *          ODR of 104 Hz (high-performance, low-noise), delivers all six axes
 *          in a single 12-byte burst read, and applies a gyroscope bias
 *          calibration computed at startup.
 *
 *          Full-scale ranges used:
 *          - Accelerometer: ±16 g  (CTRL1_XL FS = 01)
 *          - Gyroscope    : ±500 dps (CTRL2_G  FS = 010)
 *
 *          CubeMX / .ioc requirements:
 *          - I2C1 enabled, Fast Mode 400 kHz recommended.
 *          - SDO/SA0 pin tied to GND → I2C address 0x6A.
 *          - No additional GPIO required.
 *
 * @date    June 12, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef LSM6DSO32TR_H
#define LSM6DSO32TR_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* Handle, I2C address and timeout. Must match the .ioc configuration */

#define LSM_I2C_HANDLE          (&hi2c1)
#define LSM_I2C_ADDR            (0x6AU << 1U)  /**< SA0=GND → 7-bit 0x6A   */
#define LSM_I2C_TIMEOUT         100U

/* Gyroscope bias calibration: samples averaged at rest */

#define LSM_CAL_SAMPLES         200U    /**< 200 samples × ~10 ms ≈ 2 s     */
#define LSM_CAL_DELAY_MS        10U

/* ====================  DEVICE CONSTANTS  ================================== */

/* Register map (relevant subset) */

#define LSM_REG_WHO_AM_I        0x0FU   /**< Device ID, expected 0x6C        */
#define LSM_REG_CTRL1_XL        0x10U   /**< Accelerometer control           */
#define LSM_REG_CTRL2_G         0x11U   /**< Gyroscope control               */
#define LSM_REG_CTRL3_C         0x12U   /**< General control (SW reset, BDU) */
#define LSM_REG_CTRL6_C         0x15U   /**< Accel high-performance mode     */
#define LSM_REG_CTRL7_G         0x16U   /**< Gyro high-performance mode      */
#define LSM_REG_STATUS          0x1EU   /**< Data-ready flags                */
#define LSM_REG_OUT_TEMP_L      0x20U   /**< Temperature output LSB          */
#define LSM_REG_OUT_TEMP_H      0x21U   /**< Temperature output MSB          */
#define LSM_REG_OUTX_L_G        0x22U   /**< Gyro X LSB (burst start)        */
#define LSM_REG_OUTX_L_XL       0x28U   /**< Accel X LSB (burst start)       */

/* CTRL1_XL: ODR_XL=0100 (104 Hz HP), FS_XL=01 (±16 g), LPF2 off           */

#define LSM_CTRL1_XL_VAL        0x44U

/* CTRL2_G:  ODR_G=0100 (104 Hz HP), FS_G=010 (±500 dps)                    */

#define LSM_CTRL2_G_VAL         0x44U

/* CTRL3_C:  BDU=1 (block data update), IF_INC=1 (auto address increment)   */

#define LSM_CTRL3_C_VAL         0x44U

/* Device identification */

#define LSM_WHO_AM_I_VAL        0x6CU

/* Sensitivity constants */

#define LSM_ACCEL_SENS_16G      0.488f  /**< mg/LSB for ±16 g FS             */
#define LSM_GYRO_SENS_500DPS    17.50f  /**< mdps/LSB for ±500 dps FS        */

/* Conversion factors */

#define LSM_MG_TO_G             0.001f
#define LSM_MDPS_TO_DPS         0.001f

/* Status register bits */

#define LSM_STATUS_XLDA         0x01U   /**< Accel data available            */
#define LSM_STATUS_GDA          0x02U   /**< Gyro data available             */

/* ====================  ENUMERATIONS  ====================================== */

/* Return codes for all LSM6DSO32TR functions */

typedef enum {
    LSM_OK              = 0,
    LSM_ERR_COM         = 1,    /**< I2C transaction failed              */
    LSM_ERR_ID          = 2,    /**< WHO_AM_I mismatch                   */
    LSM_ERR_NOT_READY   = 3,    /**< Data not ready within timeout       */
    LSM_ERR_PARAM       = 4     /**< Invalid parameter                   */
} LSM_Status_e;

/* ============================  STRUCTURES  ================================ */

/* Gyroscope calibration biases (applied automatically in ReadAll) */

typedef struct {
    float   gx_bias_dps;    /**< Gyro X bias in dps */
    float   gy_bias_dps;    /**< Gyro Y bias in dps */
    float   gz_bias_dps;    /**< Gyro Z bias in dps */
    bool    calibrated;     /**< True after CalibrateGyroBias() */
} LSM_Cal_t;

/* Structure to hold one full IMU reading */

typedef struct {
    /* Raw 16-bit signed counts */
    int16_t ax_raw;         /**< Accel X raw count                   */
    int16_t ay_raw;         /**< Accel Y raw count                   */
    int16_t az_raw;         /**< Accel Z raw count                   */
    int16_t gx_raw;         /**< Gyro  X raw count                   */
    int16_t gy_raw;         /**< Gyro  Y raw count                   */
    int16_t gz_raw;         /**< Gyro  Z raw count                   */

    /* Converted physical units */
    float   ax_g;           /**< Accel X in g                        */
    float   ay_g;           /**< Accel Y in g                        */
    float   az_g;           /**< Accel Z in g                        */
    float   gx_dps;         /**< Gyro  X in dps (bias-corrected)     */
    float   gy_dps;         /**< Gyro  Y in dps (bias-corrected)     */
    float   gz_dps;         /**< Gyro  Z in dps (bias-corrected)     */
    float   temp_c;         /**< Die temperature in °C               */
} LSM_Data_t;

/* Driver handle */

typedef struct {
    LSM_Cal_t   cal;                /**< Calibration state               */
    bool        initialized;        /**< True after successful Init()    */
    uint8_t     consecutive_errors; /**< I2C error counter for recovery  */
    uint32_t    total_recoveries;   /**< Total SW-reset recoveries       */
} LSM6DSO32TR_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the LSM6DSO32TR.
 * @param  dev  Pointer to driver handle; must persist for the lifetime of use.
 * @note   Performs SW reset, verifies WHO_AM_I (0x6C), then programs CTRL1_XL,
 *         CTRL2_G and CTRL3_C. Call after MX_I2C1_Init().
 */
LSM_Status_e LSM6DSO32TR_Init(LSM6DSO32TR_t *dev);

/**
 * @brief  Reads WHO_AM_I register.
 * @param  dev  Pointer to driver handle.
 * @param  id   Pointer to receive the raw ID byte (expected 0x6C).
 */
LSM_Status_e LSM6DSO32TR_WhoAmI(LSM6DSO32TR_t *dev, uint8_t *id);

/**
 * @brief  Computes gyroscope bias by averaging LSM_CAL_SAMPLES readings at rest.
 * @param  dev  Pointer to driver handle.
 * @note   Keep the device completely still during calibration (~2 s).
 *         Bias is stored in dev->cal and applied automatically in ReadAll.
 */
LSM_Status_e LSM6DSO32TR_CalibrateGyroBias(LSM6DSO32TR_t *dev);

/**
 * @brief  Reads accelerometer, gyroscope and temperature in two burst transactions.
 * @param  dev  Pointer to driver handle.
 * @param  out  Pointer to LSM_Data_t to fill with raw and converted values.
 * @note   Gyro bias is subtracted automatically if dev->cal.calibrated is true.
 *         On I2C error, triggers LSM6DSO32TR_Recover() and returns LSM_ERR_COM.
 */
LSM_Status_e LSM6DSO32TR_ReadAll(LSM6DSO32TR_t *dev, LSM_Data_t *out);

/**
 * @brief  Performs a SW reset and re-initializes the sensor, preserving calibration.
 * @param  dev  Pointer to driver handle.
 * @note   Called automatically by ReadAll after consecutive I2C failures.
 */
LSM_Status_e LSM6DSO32TR_Recover(LSM6DSO32TR_t *dev);

/**
 * @brief  Puts both accelerometer and gyroscope into power-down mode (ODR=0).
 * @param  dev  Pointer to driver handle.
 * @note   Typical current drops to ~5 µA. Call LSM6DSO32TR_PowerOn() to resume.
 *         Calibration data is preserved in the handle.
 */
LSM_Status_e LSM6DSO32TR_PowerDown(LSM6DSO32TR_t *dev);

/**
 * @brief  Restores accelerometer and gyroscope to the active ODR (104 Hz HP).
 * @param  dev  Pointer to driver handle.
 * @note   Mirrors the ODR configuration set by Init. Wait ~15 ms after calling
 *         before reading data to allow the first sample to settle.
 */
LSM_Status_e LSM6DSO32TR_PowerOn(LSM6DSO32TR_t *dev);

/**
 * @brief  Basic self-test: verifies WHO_AM_I and reads one data sample.
 * @note   Called by the TestHW menu. Returns 1 on pass, 0 on fail.
 */
uint8_t LSM6DSO32TR_Test(LSM6DSO32TR_t *dev);

#endif /* LSM6DSO32TR_H */
