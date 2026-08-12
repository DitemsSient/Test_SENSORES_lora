/**
 * @file    SensorLuz_TSL2571.c
 * @brief   Driver implementation for TSL2571 Ambient Light Sensor.
 *
 * @date    March 11, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "SensorLuz_TSL2571.h"

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Builds a TSL2571 command byte for single-register access.
 * @param  reg  Target register address.
 */
static uint8_t prv_CmdByte(uint8_t reg) {
    return TSL2571_CMD_BYTE | (reg & 0x1FU);
}

/**
 * @brief  Builds a TSL2571 command byte for auto-increment block access.
 * @param  reg  Starting register address.
 */
static uint8_t prv_CmdAuto(uint8_t reg) {
    return TSL2571_CMD_AUTO | (reg & 0x1FU);
}

/**
 * @brief  Converts a TSL2571_Gain enum value to its float multiplier.
 * @param  gain  Gain setting.
 */
static float prv_GainMultiplier(TSL2571_Gain gain) {
    switch (gain) {
        case TSL2571_GAIN_1X:   return 1.0f;
        case TSL2571_GAIN_8X:   return 8.0f;
        case TSL2571_GAIN_16X:  return 16.0f;
        case TSL2571_GAIN_120X: return 120.0f;
        default:                return 1.0f;
    }
}

/**
 * @brief  Calculates lux from averaged CH0 and CH1 counts using the device formula.
 * @param  ch0    Averaged broadband count.
 * @param  ch1    Averaged IR-only count.
 * @param  atime  ATIME register value used during the measurement.
 * @param  gain   Gain setting used during the measurement.
 * @note   Uses the two-formula approach from the TSL2571 datasheet.
 *         Glass attenuation factor GA = 1.0 (no enclosure). Adjust if needed.
 *         Returns 0.0 if the result is negative.
 */
static float prv_CalculateLux(uint16_t ch0, uint16_t ch1,
                               uint8_t atime, TSL2571_Gain gain) {
    const float tint  = TSL2571_IntegrationMs(atime);
    const float gainX = prv_GainMultiplier(gain);
    const float GA    = 1.0f;   /* Glass attenuation — adjust for enclosure */

    const float cpl = (tint * gainX) / (GA * 53.0f);
    if (cpl <= 0.0f) return 0.0f;

    const float fCh0 = (float)ch0;
    const float fCh1 = (float)ch1;

    const float lux1 = (fCh0 - 2.0f * fCh1)        / cpl;
    const float lux2 = (0.6f  * fCh0 - fCh1)        / cpl;

    const float lux = (lux2 > lux1) ? lux2 : lux1;
    return (lux < 0.0f) ? 0.0f : lux;
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Calculates integration time in ms using: Tint = 2.7296 × (256 − ATIME).
 */
float TSL2571_IntegrationMs(uint8_t atime) {
    return 2.7296f * (float)(256 - (int)atime);
}

/**
 * @brief  Computes max ADC count as (256 − ATIME) × 1024, capped at 65535.
 */
uint16_t TSL2571_MaxCount(uint8_t atime) {
    const uint32_t raw = (uint32_t)(256U - atime) * 1024UL;
    return (raw > 65535UL) ? (uint16_t)65535U : (uint16_t)raw;
}

/**
 * @brief  Stores hi2c, addr, timeout_ms in the handle; initializes atime and gain defaults.
 */
void TSL2571_Attach(TSL2571_t *dev, I2C_HandleTypeDef *hi2c,
                    uint8_t addr, uint32_t timeout_ms) {
    dev->hi2c       = hi2c;
    dev->addr       = addr;
    dev->timeout_ms = timeout_ms;
    dev->atime      = 0xFFU;    /* Undefined until TSL2571_Begin() is called */
    dev->gain       = TSL2571_GAIN_1X;
}

/**
 * @brief  Sends a 2-byte I2C transaction: command byte + value byte.
 */
HAL_StatusTypeDef TSL2571_WriteReg(TSL2571_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2];
    buf[0] = prv_CmdByte(reg);
    buf[1] = val;
    return HAL_I2C_Master_Transmit(dev->hi2c, (uint16_t)(dev->addr << 1),
                                   buf, 2U, dev->timeout_ms);
}

/**
 * @brief  Sends the command byte then reads one byte back in two I2C transactions.
 */
HAL_StatusTypeDef TSL2571_ReadReg(TSL2571_t *dev, uint8_t reg, uint8_t *val) {
    uint8_t cmd = prv_CmdByte(reg);
    HAL_StatusTypeDef st;

    st = HAL_I2C_Master_Transmit(dev->hi2c, (uint16_t)(dev->addr << 1),
                                  &cmd, 1U, dev->timeout_ms);
    if (st != HAL_OK) return st;

    return HAL_I2C_Master_Receive(dev->hi2c, (uint16_t)(dev->addr << 1),
                                  val, 1U, dev->timeout_ms);
}

/**
 * @brief  Writes PON | AEN to the ENABLE register.
 */
HAL_StatusTypeDef TSL2571_Enable(TSL2571_t *dev) {
    return TSL2571_WriteReg(dev, TSL2571_REG_ENABLE,
                            TSL2571_ENABLE_PON | TSL2571_ENABLE_AEN);
}

/**
 * @brief  Writes 0x00 to the ENABLE register.
 */
HAL_StatusTypeDef TSL2571_Disable(TSL2571_t *dev) {
    return TSL2571_WriteReg(dev, TSL2571_REG_ENABLE, 0x00U);
}

/**
 * @brief  Writes the gain value to CONTROL and updates dev->gain on success.
 */
HAL_StatusTypeDef TSL2571_SetGain(TSL2571_t *dev, TSL2571_Gain gain) {
    HAL_StatusTypeDef st = TSL2571_WriteReg(dev, TSL2571_REG_CONTROL, (uint8_t)gain);
    if (st == HAL_OK) dev->gain = gain;
    return st;
}

/**
 * @brief  Writes atime to ATIME register and updates dev->atime on success.
 */
HAL_StatusTypeDef TSL2571_SetATime(TSL2571_t *dev, uint8_t atime) {
    HAL_StatusTypeDef st = TSL2571_WriteReg(dev, TSL2571_REG_ATIME, atime);
    if (st == HAL_OK) dev->atime = atime;
    return st;
}

/**
 * @brief  Powers on the device, waits 3 ms for oscillator startup, sets ATIME
 *         and gain, enables ALS, then blocks for one full integration cycle.
 */
HAL_StatusTypeDef TSL2571_Begin(TSL2571_t *dev, uint8_t atime, TSL2571_Gain gain) {
    HAL_StatusTypeDef st;

    st = TSL2571_WriteReg(dev, TSL2571_REG_ENABLE, TSL2571_ENABLE_PON);
    if (st != HAL_OK) return st;

    HAL_Delay(3U);  /* Datasheet: ≥ 2.72 ms oscillator wake-up */

    st = TSL2571_SetATime(dev, atime);
    if (st != HAL_OK) return st;

    st = TSL2571_SetGain(dev, gain);
    if (st != HAL_OK) return st;

    st = TSL2571_Enable(dev);
    if (st != HAL_OK) return st;

    HAL_Delay((uint32_t)(TSL2571_IntegrationMs(atime) + 2.0f));

    return HAL_OK;
}

/**
 * @brief  Sends an auto-increment command starting at C0DATAL, reads 4 bytes
 *         (C0L, C0H, C1L, C1H), assembles the 16-bit values, and checks for
 *         saturation against TSL2571_MaxCount().
 */
HAL_StatusTypeDef TSL2571_ReadRawChannels(TSL2571_t *dev, TSL2571_RawData_t *data) {
    uint8_t cmd = prv_CmdAuto(TSL2571_REG_C0DATAL);
    uint8_t buf[4];
    HAL_StatusTypeDef st;

    st = HAL_I2C_Master_Transmit(dev->hi2c, (uint16_t)(dev->addr << 1),
                                  &cmd, 1U, dev->timeout_ms);
    if (st != HAL_OK) return st;

    st = HAL_I2C_Master_Receive(dev->hi2c, (uint16_t)(dev->addr << 1),
                                 buf, 4U, dev->timeout_ms);
    if (st != HAL_OK) return st;

    data->ch0 = ((uint16_t)buf[1] << 8) | buf[0];
    data->ch1 = ((uint16_t)buf[3] << 8) | buf[2];

    const uint16_t maxCnt = TSL2571_MaxCount(dev->atime);
    data->saturated = (data->ch0 >= maxCnt - 2U) || (data->ch1 >= maxCnt - 2U);

    return HAL_OK;
}

/**
 * @brief  Averages nSamples readings of both channels with gapMs between each,
 *         then passes the averaged counts to prv_CalculateLux().
 */
HAL_StatusTypeDef TSL2571_ReadLux(TSL2571_t *dev, uint8_t nSamples,
                                   uint16_t gapMs, float *lux,
                                   TSL2571_RawData_t *raw) {
    uint32_t sum0 = 0U;
    uint32_t sum1 = 0U;

    for (uint8_t i = 0U; i < nSamples; i++) {
        HAL_StatusTypeDef st = TSL2571_ReadRawChannels(dev, raw);
        if (st != HAL_OK) return st;

        sum0 += raw->ch0;
        sum1 += raw->ch1;

        if (i < (uint8_t)(nSamples - 1U)) {
            HAL_Delay(gapMs);
        }
    }

    const uint16_t ch0Avg = (uint16_t)(sum0 / nSamples);
    const uint16_t ch1Avg = (uint16_t)(sum1 / nSamples);

    *lux = prv_CalculateLux(ch0Avg, ch1Avg, dev->atime, dev->gain);
    return HAL_OK;
}

/* ========================  SELF-TEST  ==================================== */

uint8_t TSL2571_Test(void)
{
    extern TSL2571_t tsl;

    TSL2571_RawData_t data = {0};

    if (TSL2571_ReadRawChannels(&tsl, &data) != HAL_OK) { return 0U; }
    if (data.ch0 == 0U)                                  { return 0U; }

    return 1U;
}
