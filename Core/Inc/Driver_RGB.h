/**
 * @file    Driver_RGB.h
 * @brief   Driver for LP55231 9-channel RGB LED controller over I2C.
 *
 * @details CubeMX configuration:
 *          - I2C peripheral (e.g. I2C1) in Standard or Fast mode.
 *          - LP55231 default 7-bit address: 0x32.
 *          - Adjust LP55231_ADDR_7BIT if AD pin changes the address.
 *
 * @date    March 09, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef DRIVER_RGB_H
#define DRIVER_RGB_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* I2C address — change LP55231_ADDR_7BIT if AD pin is reconfigured */

#define LP55231_ADDR_7BIT       0x32U
#define LP55231_ADDR_HAL        (LP55231_ADDR_7BIT << 1)

/* ====================  DEVICE CONSTANTS  ================================== */

/* LP55231 register map */

#define REG_CNTRL1              0x00U
#define REG_CNTRL2              0x01U
#define REG_RATIO_MSB           0x02U
#define REG_RATIO_LSB           0x03U
#define REG_OUTPUT_ONOFF_MSB    0x04U
#define REG_OUTPUT_ONOFF_LSB    0x05U
#define REG_D1_CTRL             0x06U
#define REG_D1_PWM              0x16U
#define REG_D1_I_CTL            0x26U
#define REG_MISC                0x36U
#define REG_RESET               0x3DU
#define REG_MASTER_FADE_1       0x48U

#define LP55231_NUM_CHANNELS    9U
#define LP55231_NUM_FADERS      3U

/* ========================  ENUMERATIONS  ================================== */

/* LED channel identifiers (LP_CH1–LP_CH9) */

typedef enum {
    LP_CH1 = 0, LP_CH2, LP_CH3,
    LP_CH4,     LP_CH5, LP_CH6,
    LP_CH7,     LP_CH8, LP_CH9
} LP55231_Led_e;

/* ============================  STRUCTURES  ================================ */

/* Device handle — one instance per LP55231 on the bus */

typedef struct {
    I2C_HandleTypeDef   *hi2c;          /**< I2C peripheral handle      */
    uint8_t              addr;          /**< 7-bit I2C address          */
    uint32_t             timeout_ms;    /**< HAL timeout in ms          */
} LP55231_t;

/* ================================  API  =================================== */

/**
 * @brief  Attaches the device handle to an I2C peripheral.
 * @param  dev         Pointer to LP55231 handle.
 * @param  hi2c        I2C peripheral handle.
 * @param  addr        7-bit I2C address.
 * @param  timeout_ms  HAL timeout in milliseconds.
 */
void LP55231_Attach(LP55231_t *dev, I2C_HandleTypeDef *hi2c,
                    uint8_t addr, uint32_t timeout_ms);

/**
 * @brief  Writes a single register.
 * @param  dev  Pointer to LP55231 handle.
 * @param  reg  Register address.
 * @param  val  Value to write.
 */
HAL_StatusTypeDef LP55231_WriteReg(LP55231_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief  Reads a single register.
 * @param  dev  Pointer to LP55231 handle.
 * @param  reg  Register address.
 * @param  val  Pointer to store the read value.
 */
HAL_StatusTypeDef LP55231_ReadReg(LP55231_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief  Resets the device and prepares it for use.
 * @param  dev  Pointer to LP55231 handle.
 * @note   I2C must already be initialized by MX_I2Cx_Init().
 */
HAL_StatusTypeDef LP55231_Begin(LP55231_t *dev);

/**
 * @brief  Enables the chip (internal oscillator + charge pump).
 * @param  dev  Pointer to LP55231 handle.
 */
HAL_StatusTypeDef LP55231_Enable(LP55231_t *dev);

/**
 * @brief  Disables the chip (clears enable bit in CNTRL1).
 * @param  dev  Pointer to LP55231 handle.
 */
HAL_StatusTypeDef LP55231_Disable(LP55231_t *dev);

/**
 * @brief  Performs a software reset.
 * @param  dev  Pointer to LP55231 handle.
 */
HAL_StatusTypeDef LP55231_Reset(LP55231_t *dev);

/**
 * @brief  Sets the PWM duty cycle for a single channel.
 * @param  dev      Pointer to LP55231 handle.
 * @param  channel  Channel index (0–8).
 * @param  value    PWM value (0–255).
 */
bool LP55231_SetChannelPWM(LP55231_t *dev, uint8_t channel, uint8_t value);

/**
 * @brief  Sets a master fader level.
 * @param  dev    Pointer to LP55231 handle.
 * @param  fader  Fader index (0–2).
 * @param  value  Fader value (0–255).
 */
bool LP55231_SetMasterFader(LP55231_t *dev, uint8_t fader, uint8_t value);

/**
 * @brief  Enables or disables logarithmic brightness on a channel.
 * @param  dev      Pointer to LP55231 handle.
 * @param  channel  Channel index (0–8).
 * @param  enable   true = logarithmic, false = linear.
 */
bool LP55231_SetLogBrightness(LP55231_t *dev, uint8_t channel, bool enable);

/**
 * @brief  Sets the drive current for a channel.
 * @param  dev      Pointer to LP55231 handle.
 * @param  channel  Channel index (0–8).
 * @param  value    Current value (device-specific, see datasheet).
 */
bool LP55231_SetDriveCurrent(LP55231_t *dev, uint8_t channel, uint8_t value);

/**
 * @brief  Assigns a channel to a master fader group.
 * @param  dev      Pointer to LP55231 handle.
 * @param  channel  Channel index (0–8).
 * @param  fader    Fader index (0–2).
 */
bool LP55231_AssignChannelToMasterFader(LP55231_t *dev, uint8_t channel,
                                        uint8_t fader);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Cicla todos los LEDs en Rojo → Verde → Azul, 2 veces (1.5 s por color = 9 s).
 * @note   Bloqueante. Mapeo asumido: R=LP_CH1/LP_CH4/LP_CH7, G=LP_CH2/LP_CH5/LP_CH8, B=LP_CH3/LP_CH6/LP_CH9.
 *         Ajustar ch_red/ch_green/ch_blue si el layout de la PCB es diferente.
 *         Todos los canales quedan en 0 al finalizar.
 */
void DriverRGB_Test(void);

#endif /* DRIVER_RGB_H */
