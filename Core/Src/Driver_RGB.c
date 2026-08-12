/**
 * @file    Driver_RGB.c
 * @brief   Driver implementation for LP55231 RGB LED controller.
 *
 * @date    March 09, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Driver_RGB.h"

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Stores the I2C handle, address, and timeout into the device struct.
 */
void LP55231_Attach(LP55231_t *dev, I2C_HandleTypeDef *hi2c,
                    uint8_t addr, uint32_t timeout_ms) {
    dev->hi2c       = hi2c;
    dev->addr       = addr;
    dev->timeout_ms = timeout_ms;
}

/**
 * @brief  Sends a single byte to the given register via I2C mem write.
 */
HAL_StatusTypeDef LP55231_WriteReg(LP55231_t *dev, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(dev->hi2c, LP55231_ADDR_HAL, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1U, dev->timeout_ms);
}

/**
 * @brief  Reads a single byte from the given register via I2C mem read.
 */
HAL_StatusTypeDef LP55231_ReadReg(LP55231_t *dev, uint8_t reg, uint8_t *val) {
    return HAL_I2C_Mem_Read(dev->hi2c, LP55231_ADDR_HAL, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1U, dev->timeout_ms);
}

/**
 * @brief  Writes 0xFF to the reset register to perform a full software reset.
 */
HAL_StatusTypeDef LP55231_Reset(LP55231_t *dev) {
    return LP55231_WriteReg(dev, REG_RESET, 0xFFU);
}

/**
 * @brief  Resets the device. I2C is already initialized by MX_I2Cx_Init().
 */
HAL_StatusTypeDef LP55231_Begin(LP55231_t *dev) {
    return LP55231_Reset(dev);
}

/**
 * @brief  Sets CNTRL1 enable bit (bit 6) and configures MISC register
 *         for internal clock, charge pump, and auto-increment.
 */
HAL_StatusTypeDef LP55231_Enable(LP55231_t *dev) {
    HAL_StatusTypeDef st;

    /* CNTRL1: enable bit6 -> 0x40 */
    st = LP55231_WriteReg(dev, REG_CNTRL1, 0x40U);
    if (st != HAL_OK) return st;

    /* MISC: internal clock + charge pump + auto increment -> 0x53 */
    return LP55231_WriteReg(dev, REG_MISC, 0x53U);
}

/**
 * @brief  Reads CNTRL1, clears the enable bit (bit 6), and writes it back.
 */
HAL_StatusTypeDef LP55231_Disable(LP55231_t *dev) {
    uint8_t val = 0U;
    if (LP55231_ReadReg(dev, REG_CNTRL1, &val) != HAL_OK) return HAL_ERROR;
    val &= (uint8_t)~0x40U;
    return LP55231_WriteReg(dev, REG_CNTRL1, val);
}

/**
 * @brief  Writes the PWM value to the channel's PWM register (REG_D1_PWM + offset).
 */
bool LP55231_SetChannelPWM(LP55231_t *dev, uint8_t channel, uint8_t value) {
    if (channel >= LP55231_NUM_CHANNELS) return false;
    return (LP55231_WriteReg(dev, (uint8_t)(REG_D1_PWM + channel), value) == HAL_OK);
}

/**
 * @brief  Writes the fader value to the master fader register (REG_MASTER_FADE_1 + offset).
 */
bool LP55231_SetMasterFader(LP55231_t *dev, uint8_t fader, uint8_t value) {
    if (fader >= LP55231_NUM_FADERS) return false;
    return (LP55231_WriteReg(dev, (uint8_t)(REG_MASTER_FADE_1 + fader), value) == HAL_OK);
}

/**
 * @brief  Reads the channel control register, sets or clears bit 5 (log brightness),
 *         and writes it back.
 */
bool LP55231_SetLogBrightness(LP55231_t *dev, uint8_t channel, bool enable) {
    if (channel >= LP55231_NUM_CHANNELS) return false;

    uint8_t reg_val = 0U;
    if (LP55231_ReadReg(dev, (uint8_t)(REG_D1_CTRL + channel), &reg_val) != HAL_OK) {
        return false;
    }

    reg_val &= (uint8_t)~0x20U;
    if (enable) reg_val |= 0x20U;

    return (LP55231_WriteReg(dev, (uint8_t)(REG_D1_CTRL + channel), reg_val) == HAL_OK);
}

/**
 * @brief  Writes the current control value to the channel's I_CTL register.
 */
bool LP55231_SetDriveCurrent(LP55231_t *dev, uint8_t channel, uint8_t value) {
    if (channel >= LP55231_NUM_CHANNELS) return false;
    return (LP55231_WriteReg(dev, (uint8_t)(REG_D1_I_CTL + channel), value) == HAL_OK);
}

/**
 * @brief  Reads the channel control register, encodes the fader index into
 *         bits [7:6], and writes it back to assign the channel to a fader group.
 */
bool LP55231_AssignChannelToMasterFader(LP55231_t *dev, uint8_t channel,
                                        uint8_t fader) {
    if (channel >= LP55231_NUM_CHANNELS) return false;
    if (fader >= LP55231_NUM_FADERS) return false;

    uint8_t reg_val = 0U;
    if (LP55231_ReadReg(dev, (uint8_t)(REG_D1_CTRL + channel), &reg_val) != HAL_OK) {
        return false;
    }

    uint8_t bit_val = (uint8_t)(((fader + 1U) & 0x03U) << 6);

    reg_val &= (uint8_t)~0xC0U;
    reg_val |= bit_val;

    return (LP55231_WriteReg(dev, (uint8_t)(REG_D1_CTRL + channel), reg_val) == HAL_OK);
}

/* ========================  SELF-TEST  ==================================== */

#define RGB_TEST_STEP_MS    1500U
#define RGB_TEST_CYCLES     2U

static const uint8_t ch_red[]   = { 0U, 3U, 6U };  /* LP_CH1, LP_CH4, LP_CH7 */
static const uint8_t ch_green[] = { 1U, 4U, 7U };  /* LP_CH2, LP_CH5, LP_CH8 */
static const uint8_t ch_blue[]  = { 2U, 5U, 8U };  /* LP_CH3, LP_CH6, LP_CH9 */

void DriverRGB_Test(void)
{
    extern LP55231_t rgb;

    for (uint8_t cycle = 0U; cycle < RGB_TEST_CYCLES; cycle++) {
        /* Rojo */
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_red[i],   0xFFU); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_green[i], 0x00U); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_blue[i],  0x00U); }
        HAL_Delay(RGB_TEST_STEP_MS);

        /* Verde */
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_red[i],   0x00U); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_green[i], 0xFFU); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_blue[i],  0x00U); }
        HAL_Delay(RGB_TEST_STEP_MS);

        /* Azul */
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_red[i],   0x00U); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_green[i], 0x00U); }
        for (uint8_t i = 0U; i < 3U; i++) { LP55231_SetChannelPWM(&rgb, ch_blue[i],  0xFFU); }
        HAL_Delay(RGB_TEST_STEP_MS);
    }

    /* Apagar todos los canales */
    for (uint8_t i = 0U; i < LP55231_NUM_CHANNELS; i++) {
        LP55231_SetChannelPWM(&rgb, i, 0x00U);
    }
}
