/**
 * @file    MMC5983MA.c
 * @brief   Driver implementation for MEMSIC MMC5983MA 3-axis magnetometer.
 *
 * @date    June 12, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "MMC5983MA.h"
#include "I2C1_Bus.h"

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern I2C_HandleTypeDef hi2c1;

/* ======================  STATIC FUNCTIONS  ================================ */

/* Low-level register access helpers */

/**
 * @brief  Writes a single byte to a register.
 * @param  reg    Register address.
 * @param  value  Byte to write.
 */
static MMC_Status_e MMC_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    I2C1Bus_Lock();
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(MMC_I2C_HANDLE, MMC_I2C_ADDR,
                                                    buf, 2U, MMC_I2C_TIMEOUT);
    I2C1Bus_Unlock();

    return (st != HAL_OK) ? MMC_ERR_COM : MMC_OK;
}

/**
 * @brief  Reads one or more consecutive registers starting at reg.
 * @param  reg   First register address.
 * @param  data  Output buffer.
 * @param  len   Number of bytes to read.
 */
static MMC_Status_e MMC_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    I2C1Bus_Lock();
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(MMC_I2C_HANDLE, MMC_I2C_ADDR,
                                                    &reg, 1U, MMC_I2C_TIMEOUT);
    if (st == HAL_OK) {
        st = HAL_I2C_Master_Receive(MMC_I2C_HANDLE, MMC_I2C_ADDR,
                                    data, len, MMC_I2C_TIMEOUT);
    }
    I2C1Bus_Unlock();

    return (st != HAL_OK) ? MMC_ERR_COM : MMC_OK;
}

/**
 * @brief  Returns true if the raw count indicates axis saturation.
 * @param  raw  18-bit raw count (0–262143).
 */
static bool MMC_IsSaturated(uint32_t raw)
{
    return (raw < MMC_SAT_MARGIN) || (raw > (262143U - MMC_SAT_MARGIN));
}

/**
 * @brief  Assembles the three 18-bit raw values from the 7-byte burst buffer.
 * @param  buf  Pointer to 7-byte buffer starting at MMC_REG_XOUT0.
 * @param  out  Pointer to MMC_Data_t to fill.
 */
static void MMC_AssembleRaw(const uint8_t *buf, MMC_Data_t *out)
{
    /* Each axis: high byte (bits 17:10) | low byte (bits 9:2) | 2 LSBs packed */
    out->x_raw = ((uint32_t)buf[0] << 10U) |
                 ((uint32_t)buf[1] <<  2U) |
                 ((buf[6] >> 6U) & 0x03U);

    out->y_raw = ((uint32_t)buf[2] << 10U) |
                 ((uint32_t)buf[3] <<  2U) |
                 ((buf[6] >> 4U) & 0x03U);

    out->z_raw = ((uint32_t)buf[4] << 10U) |
                 ((uint32_t)buf[5] <<  2U) |
                 ((buf[6] >> 2U) & 0x03U);
}

/**
 * @brief  Converts raw 18-bit counts to µT and stores them in out.
 * @param  out  Pointer to MMC_Data_t (raw fields must already be filled).
 */
static void MMC_ConvertToUT(MMC_Data_t *out)
{
    out->x_uT = ((float)out->x_raw - (float)MMC_ZERO_CODE) / MMC_COUNTS_PER_UT;
    out->y_uT = ((float)out->y_raw - (float)MMC_ZERO_CODE) / MMC_COUNTS_PER_UT;
    out->z_uT = ((float)out->z_raw - (float)MMC_ZERO_CODE) / MMC_COUNTS_PER_UT;
}

/* ================================  API  =================================== */

/* Public functions declared in MMC5983MA.h */

/**
 * @brief  Issues a SET pulse, checks WHO_AM_I, then starts continuous mode
 *         at MMC_ODR_ACTIVE with auto SET/RESET enabled on every measurement.
 */
MMC_Status_e MMC5983MA_Init(void)
{
    MMC_Status_e status;
    uint8_t id = 0U;

    /* Force a clean magnetic state before any configuration */
    status = MMC5983MA_Set();
    if (status != MMC_OK) return status;

    HAL_Delay(10U);

    /* Verify device identity */
    status = MMC5983MA_WhoAmI(&id);
    if (status != MMC_OK) return status;
    if (id != MMC_PRODUCT_ID) return MMC_ERR_ID;

    /* CTRL0: enable auto SET/RESET on every sample */
    status = MMC_WriteReg(MMC_REG_CTRL0, MMC_CTRL0_AUTO_SR);
    if (status != MMC_OK) return status;

    /* CTRL1: set bandwidth to 100 Hz (BW=11) — ODR is set via CTRL2        */
    status = MMC_WriteReg(MMC_REG_CTRL1, 0x03U);
    if (status != MMC_OK) return status;

    /* CTRL2: enable continuous measurement mode + target ODR */
    status = MMC_WriteReg(MMC_REG_CTRL2, MMC_CTRL2_CMM_EN | MMC_ODR_ACTIVE);
    if (status != MMC_OK) return status;

    /* El modo continuo solo arma el auto-repetido; hace falta un TM_M para
     * disparar la primera medicion (las siguientes ya se repiten solas). */
    status = MMC_WriteReg(MMC_REG_CTRL0, MMC_CTRL0_TM_M);
    if (status != MMC_OK) return status;

    return MMC_OK;
}

/**
 * @brief  Reads the 7-byte output burst, assembles 18-bit values, converts to µT.
 *         Issues a SET pulse automatically if saturation is detected on any axis.
 */
MMC_Status_e MMC5983MA_ReadAll(MMC_Data_t *out)
{
    if (out == NULL) return MMC_ERR_PARAM;

    uint8_t buf[7];
    MMC_Status_e status = MMC_ReadRegs(MMC_REG_XOUT0, buf, 7U);
    if (status != MMC_OK) return status;

    MMC_AssembleRaw(buf, out);

    /* Recover from saturation before converting */
    if (MMC_IsSaturated(out->x_raw) ||
        MMC_IsSaturated(out->y_raw) ||
        MMC_IsSaturated(out->z_raw))
    {
        MMC5983MA_Set();
        HAL_Delay(5U);

        status = MMC_ReadRegs(MMC_REG_XOUT0, buf, 7U);
        if (status != MMC_OK) return status;
        MMC_AssembleRaw(buf, out);
    }

    MMC_ConvertToUT(out);
    return MMC_OK;
}

/**
 * @brief  Writes the SET bit to CTRL0 and waits 1 ms for the pulse to complete.
 */
MMC_Status_e MMC5983MA_Set(void)
{
    MMC_Status_e status = MMC_WriteReg(MMC_REG_CTRL0, MMC_CTRL0_SET);
    if (status != MMC_OK) return status;
    HAL_Delay(1U);
    return MMC_OK;
}

/**
 * @brief  Reads Product ID register and returns it via the id pointer.
 */
MMC_Status_e MMC5983MA_WhoAmI(uint8_t *id)
{
    if (id == NULL) return MMC_ERR_PARAM;
    return MMC_ReadRegs(MMC_REG_PRODUCT_ID, id, 1U);
}

/**
 * @brief  Verifies WHO_AM_I, reads one sample and checks that the values are
 *         within the physical range expected (±800 µT Earth field ± margin).
 *         Returns 1 on pass, 0 on fail.
 */
uint8_t MMC5983MA_Test(void)
{
    uint8_t id = 0U;
    if (MMC5983MA_WhoAmI(&id) != MMC_OK) return 0U;
    if (id != MMC_PRODUCT_ID)            return 0U;

    MMC_Data_t data = {0};
    if (MMC5983MA_ReadAll(&data) != MMC_OK) return 0U;

    /* Sanity: Earth's field is roughly ±80 µT; allow generous ±800 µT margin */
    if (data.x_uT < -800.0f || data.x_uT > 800.0f) return 0U;
    if (data.y_uT < -800.0f || data.y_uT > 800.0f) return 0U;
    if (data.z_uT < -800.0f || data.z_uT > 800.0f) return 0U;

    return 1U;
}
