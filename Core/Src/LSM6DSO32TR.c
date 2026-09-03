/**
 * @file    LSM6DSO32TR.c
 * @brief   Driver implementation for ST LSM6DS3 6-axis IMU (accel + gyro).
 *          Nombre de archivo/API conservan "LSM6DSO32TR" por compatibilidad
 *          con el resto del proyecto; el chip real es LSM6DS3 (WHO_AM_I 0x69).
 *
 * @date    June 12, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "LSM6DSO32TR.h"
#include "I2C1_Bus.h"
#include <string.h>

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern I2C_HandleTypeDef hi2c1;

/* ======================  STATIC FUNCTIONS  ================================ */

/* Low-level register access helpers */

/**
 * @brief  Writes a single byte to a register.
 * @param  reg    Register address.
 * @param  value  Byte to write.
 */
static LSM_Status_e LSM_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    I2C1Bus_Lock();
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(LSM_I2C_HANDLE, LSM_I2C_ADDR,
                                                    buf, 2U, LSM_I2C_TIMEOUT);
    I2C1Bus_Unlock();

    return (st != HAL_OK) ? LSM_ERR_COM : LSM_OK;
}

/**
 * @brief  Reads one or more consecutive registers starting at reg.
 * @param  reg   First register address (IF_INC must be set in CTRL3_C).
 * @param  data  Output buffer.
 * @param  len   Number of bytes to read.
 */
static LSM_Status_e LSM_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    I2C1Bus_Lock();
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(LSM_I2C_HANDLE, LSM_I2C_ADDR,
                                                    &reg, 1U, LSM_I2C_TIMEOUT);
    if (st == HAL_OK) {
        st = HAL_I2C_Master_Receive(LSM_I2C_HANDLE, LSM_I2C_ADDR,
                                    data, len, LSM_I2C_TIMEOUT);
    }
    I2C1Bus_Unlock();

    return (st != HAL_OK) ? LSM_ERR_COM : LSM_OK;
}

/**
 * @brief  Applies the configuration registers common to Init and Recover.
 * @param  dev  Pointer to driver handle.
 * @note   Called after every SW reset so configuration is in one place.
 */
static LSM_Status_e LSM_Configure(LSM6DSO32TR_t *dev)
{
    (void)dev;  /* Reserved for future per-handle configuration */

    LSM_Status_e s;

    /* CTRL3_C: BDU=1, IF_INC=1 — set first so burst reads work immediately */
    s = LSM_WriteReg(LSM_REG_CTRL3_C, LSM_CTRL3_C_VAL);
    if (s != LSM_OK) return s;

    HAL_Delay(5U);

    /* CTRL2_G: gyro ODR 104 Hz HP, ±500 dps */
    s = LSM_WriteReg(LSM_REG_CTRL2_G, LSM_CTRL2_G_VAL);
    if (s != LSM_OK) return s;

    /* CTRL1_XL: accel ODR 104 Hz HP, ±16 g */
    s = LSM_WriteReg(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_VAL);
    if (s != LSM_OK) return s;

    /* Wait for first sample to be ready at 104 Hz (~10 ms) */
    HAL_Delay(15U);

    return LSM_OK;
}

/**
 * @brief  Converts a 12-byte raw burst (gyro + accel) into physical units.
 * @param  dev  Pointer to driver handle (for calibration access).
 * @param  buf  Raw buffer: bytes 0-5 = gyro XYZ, bytes 6-11 = accel XYZ.
 * @param  out  Pointer to LSM_Data_t to fill.
 */
static void LSM_ConvertBurst(const LSM6DSO32TR_t *dev,
                             const uint8_t       *buf,
                             LSM_Data_t          *out)
{
    /* Gyroscope (little-endian signed 16-bit) */
    out->gx_raw = (int16_t)((uint16_t)buf[1]  << 8U | buf[0]);
    out->gy_raw = (int16_t)((uint16_t)buf[3]  << 8U | buf[2]);
    out->gz_raw = (int16_t)((uint16_t)buf[5]  << 8U | buf[4]);

    /* Accelerometer */
    out->ax_raw = (int16_t)((uint16_t)buf[7]  << 8U | buf[6]);
    out->ay_raw = (int16_t)((uint16_t)buf[9]  << 8U | buf[8]);
    out->az_raw = (int16_t)((uint16_t)buf[11] << 8U | buf[10]);

    /* Convert to physical units */
    out->gx_dps = (float)out->gx_raw * LSM_GYRO_SENS_500DPS  * LSM_MDPS_TO_DPS;
    out->gy_dps = (float)out->gy_raw * LSM_GYRO_SENS_500DPS  * LSM_MDPS_TO_DPS;
    out->gz_dps = (float)out->gz_raw * LSM_GYRO_SENS_500DPS  * LSM_MDPS_TO_DPS;

    out->ax_g   = (float)out->ax_raw * LSM_ACCEL_SENS_16G * LSM_MG_TO_G;
    out->ay_g   = (float)out->ay_raw * LSM_ACCEL_SENS_16G * LSM_MG_TO_G;
    out->az_g   = (float)out->az_raw * LSM_ACCEL_SENS_16G * LSM_MG_TO_G;

    /* Apply gyro bias if calibrated */
    if (dev->cal.calibrated)
    {
        out->gx_dps -= dev->cal.gx_bias_dps;
        out->gy_dps -= dev->cal.gy_bias_dps;
        out->gz_dps -= dev->cal.gz_bias_dps;
    }
}

/* ================================  API  =================================== */

/* Public functions declared in LSM6DSO32TR.h */

/**
 * @brief  SW reset, WHO_AM_I check, then full sensor configuration.
 *         handle is zeroed before init so previous calibration is lost.
 *         Call LSM6DSO32TR_CalibrateGyroBias() after Init if needed.
 */
LSM_Status_e LSM6DSO32TR_Init(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return LSM_ERR_PARAM;

    memset(dev, 0, sizeof(LSM6DSO32TR_t));

    /* SW reset: CTRL3_C bit 0 */
    LSM_Status_e s = LSM_WriteReg(LSM_REG_CTRL3_C, 0x01U);
    if (s != LSM_OK) return s;
    HAL_Delay(10U);

    /* Verify device identity */
    uint8_t id = 0U;
    s = LSM6DSO32TR_WhoAmI(dev, &id);
    if (s != LSM_OK) return s;
    if (id != LSM_WHO_AM_I_VAL) return LSM_ERR_ID;

    s = LSM_Configure(dev);
    if (s != LSM_OK) return s;

    dev->initialized = true;
    return LSM_OK;
}

/**
 * @brief  Reads WHO_AM_I register (0x0F). Expected value is 0x69 (LSM6DS3).
 */
LSM_Status_e LSM6DSO32TR_WhoAmI(LSM6DSO32TR_t *dev, uint8_t *id)
{
    if (dev == NULL || id == NULL) return LSM_ERR_PARAM;
    return LSM_ReadRegs(LSM_REG_WHO_AM_I, id, 1U);
}

/**
 * @brief  Averages LSM_CAL_SAMPLES gyro readings with the device at rest.
 *         Bias values are stored in dev->cal and subtracted in ReadAll.
 */
LSM_Status_e LSM6DSO32TR_CalibrateGyroBias(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return LSM_ERR_PARAM;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint8_t buf[6];

    for (uint32_t i = 0U; i < LSM_CAL_SAMPLES; i++)
    {
        LSM_Status_e s = LSM_ReadRegs(LSM_REG_OUTX_L_G, buf, 6U);
        if (s != LSM_OK) return s;

        int16_t gx = (int16_t)((uint16_t)buf[1] << 8U | buf[0]);
        int16_t gy = (int16_t)((uint16_t)buf[3] << 8U | buf[2]);
        int16_t gz = (int16_t)((uint16_t)buf[5] << 8U | buf[4]);

        sum_x += (float)gx * LSM_GYRO_SENS_500DPS * LSM_MDPS_TO_DPS;
        sum_y += (float)gy * LSM_GYRO_SENS_500DPS * LSM_MDPS_TO_DPS;
        sum_z += (float)gz * LSM_GYRO_SENS_500DPS * LSM_MDPS_TO_DPS;

        HAL_Delay(LSM_CAL_DELAY_MS);
    }

    float n = (float)LSM_CAL_SAMPLES;
    dev->cal.gx_bias_dps = sum_x / n;
    dev->cal.gy_bias_dps = sum_y / n;
    dev->cal.gz_bias_dps = sum_z / n;
    dev->cal.calibrated  = true;

    return LSM_OK;
}

/**
 * @brief  Reads gyro (6 bytes from OUTX_L_G) and accel (6 bytes from OUTX_L_XL)
 *         in two consecutive burst reads, converts to physical units, and
 *         reads the temperature register pair separately.
 *         On I2C failure increments dev->consecutive_errors; at 3 consecutive
 *         errors triggers Recover().
 */
LSM_Status_e LSM6DSO32TR_ReadAll(LSM6DSO32TR_t *dev, LSM_Data_t *out)
{
    if (dev == NULL || out == NULL) return LSM_ERR_PARAM;

    uint8_t buf[12];

    /* Read gyro (6 bytes) then accel (6 bytes) back-to-back into buf */
    LSM_Status_e s = LSM_ReadRegs(LSM_REG_OUTX_L_G, buf, 6U);
    if (s != LSM_OK) goto handle_error;

    s = LSM_ReadRegs(LSM_REG_OUTX_L_XL, &buf[6], 6U);
    if (s != LSM_OK) goto handle_error;

    dev->consecutive_errors = 0U;
    LSM_ConvertBurst(dev, buf, out);

    /* Temperature: 16-bit, sensitivity 256 LSB/°C, offset 25 °C */
    uint8_t tbuf[2];
    s = LSM_ReadRegs(LSM_REG_OUT_TEMP_L, tbuf, 2U);
    if (s == LSM_OK)
    {
        int16_t t_raw = (int16_t)((uint16_t)tbuf[1] << 8U | tbuf[0]);
        out->temp_c = 25.0f + (float)t_raw / 256.0f;
    }

    return LSM_OK;

handle_error:
    dev->consecutive_errors++;
    if (dev->consecutive_errors >= 3U)
    {
        LSM6DSO32TR_Recover(dev);
    }
    return LSM_ERR_COM;
}

/**
 * @brief  SW reset + re-configure, preserving dev->cal so calibration survives.
 *         Resets consecutive_errors and increments total_recoveries.
 */
LSM_Status_e LSM6DSO32TR_Recover(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return LSM_ERR_PARAM;

    /* Preserve calibration across recovery */
    LSM_Cal_t saved_cal = dev->cal;
    uint32_t  saved_recoveries = dev->total_recoveries + 1U;

    LSM_WriteReg(LSM_REG_CTRL3_C, 0x01U);
    HAL_Delay(15U);

    LSM_Status_e s = LSM_Configure(dev);

    dev->cal              = saved_cal;
    dev->total_recoveries = saved_recoveries;
    dev->consecutive_errors = 0U;
    dev->initialized      = (s == LSM_OK);

    return s;
}

/**
 * @brief  Writes ODR=0000 to CTRL1_XL and CTRL2_G, disabling both sensors.
 *         Current drops to ~5 µA. Handle state and calibration are untouched.
 */
LSM_Status_e LSM6DSO32TR_PowerDown(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return LSM_ERR_PARAM;

    LSM_Status_e s;
    /* ODR=0000 in bits [7:4] → power-down; keep FS bits unchanged */
    s = LSM_WriteReg(LSM_REG_CTRL2_G,  0x00U);
    if (s != LSM_OK) return s;
    s = LSM_WriteReg(LSM_REG_CTRL1_XL, 0x00U);
    return s;
}

/**
 * @brief  Restores CTRL1_XL and CTRL2_G to the active configuration (104 Hz HP).
 *         Waits 15 ms for the first valid sample before returning.
 */
LSM_Status_e LSM6DSO32TR_PowerOn(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return LSM_ERR_PARAM;

    LSM_Status_e s;
    s = LSM_WriteReg(LSM_REG_CTRL2_G,  LSM_CTRL2_G_VAL);
    if (s != LSM_OK) return s;
    s = LSM_WriteReg(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_VAL);
    if (s != LSM_OK) return s;

    HAL_Delay(15U);
    return LSM_OK;
}

/**
 * @brief  Checks WHO_AM_I and reads one data sample to confirm sensor is alive.
 *         Returns 1 on pass, 0 on fail.
 */
uint8_t LSM6DSO32TR_Test(LSM6DSO32TR_t *dev)
{
    if (dev == NULL) return 0U;

    uint8_t id = 0U;
    if (LSM6DSO32TR_WhoAmI(dev, &id) != LSM_OK) return 0U;
    if (id != LSM_WHO_AM_I_VAL)                  return 0U;

    LSM_Data_t data = {0};
    if (LSM6DSO32TR_ReadAll(dev, &data) != LSM_OK) return 0U;

    return 1U;
}
