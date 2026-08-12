/**
 * @file    PowerManager.h
 * @brief   System-level low-power coordinator for all peripheral drivers.
 *
 * @details Provides two high-level functions to suspend or resume every
 *          peripheral that supports a low-power state:
 *
 *          - Flash MX25L6445E  : Deep Power-Down (~1 µA)
 *          - GPS L86-M33       : Standby via PMTK161 + FORCE_ON LOW
 *          - LSM6DSO32TR       : LSM6DSO32TR_PowerDown/PowerOn (ODR=0 → ~5 µA)
 *          - TSL2571           : ALS disabled (PON + AEN cleared)
 *          - BQ27441 fuel gauge: Hibernate mode
 *          - LoRa RM1262       : AT+SLEEP command via UART
 *          - LP55231 RGB       : Disable/Enable via I2C
 *          - Buzzer            : PWM duty cycle 0
 *          - Vibration motor   : Output LOW
 *          - BL654 Bluetooth   : Auto-sleep (no command needed)
 *
 *          Nota: esta tarjeta (Sensores) no tiene OLED ni LED RGB discreto
 *          por GPIO — ambos son componentes de la PCB 1 (Mira) y fueron
 *          removidos de este coordinador.
 *
 *          Handles for drivers that require a context struct must be
 *          registered once with PowerManager_Init() before calling
 *          Suspend or Wake.
 *
 *          STM32 low-power mode (Stop / Standby) is declared here but
 *          not yet implemented — see PowerManager_MCUSleep().
 *
 * @date    May 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef POWERMANAGER_H
#define POWERMANAGER_H

#include "stm32l4xx_hal.h"
#include "GPS.h"
#include "LSM6DSO32TR.h"
#include "SensorLuz_TSL2571.h"
#include "Driver_RGB.h"
#include "Lora.h"
#include <stdbool.h>

/* ========================  ENUMERATIONS  ================================= */

/* PowerManager return codes */

typedef enum {
    PM_OK               = 0,    /**< All operations succeeded                */
    PM_ERR_NOT_INIT     = 1,    /**< PowerManager_Init() was not called       */
    PM_ERR_PARTIAL      = 2,    /**< One or more drivers reported an error    */
    PM_ERR_PARAM        = 3     /**< NULL pointer passed to Init             */
} PM_Status_e;

/* ============================  STRUCTURES  ================================ */

/* Result detail from the last Suspend or Wake call */

typedef struct {
    bool flash_ok;      /**< Flash entered / exited power-down correctly     */
    bool gps_ok;        /**< GPS entered / exited standby correctly          */
    bool imu_ok;        /**< LSM6DSO32TR entered / exited low-power correctly */
    bool light_ok;      /**< TSL2571 disabled / enabled correctly            */
    bool gauge_ok;      /**< BQ27441 entered / exited hibernate correctly    */
    bool rgb_ok;        /**< LP55231 disabled / enabled correctly            */
    bool lora_ok;       /**< RM1262 entered / exited sleep correctly         */
} PM_Result_t;

/* ================================  API  =================================== */

/**
 * @brief  Registers the driver handles required by PowerManager.
 * @param  hgps    Pointer to the initialised GPS handle (Gps_Handle_t).
 * @param  himu    Pointer to the initialised LSM6DSO32TR handle (LSM6DSO32TR_t).
 * @param  hlight  Pointer to the initialised TSL2571 handle (TSL2571_t).
 * @note   Call once after all driver Init() functions have succeeded.
 *         Flash, SSD1306, BQ27441, LedRGB, Buzzer and Vibrator use
 *         module-level handles and do not need to be passed here.
 */
PM_Status_e PowerManager_Init(Gps_Handle_t    *hgps,
                               LSM6DSO32TR_t   *himu,
                               TSL2571_t       *hlight,
                               LP55231_t       *hrgb,
                               Lora_Handle_t   *hlora);

/**
 * @brief  Puts every supported peripheral into its lowest-power state.
 * @param  result  Optional pointer to a PM_Result_t struct that receives
 *                 the per-driver outcome. Pass NULL to ignore.
 * @note   Actuators (LED, Buzzer, Vibrator) are turned off unconditionally.
 *         ICs (Flash, GPS, IMU, light sensor, display, gauge) are commanded
 *         via their respective low-power APIs.
 *         Returns PM_OK only if every driver succeeded.
 */
PM_Status_e PowerManager_SuspendAll(PM_Result_t *result);

/**
 * @brief  Restores every peripheral to normal operation.
 * @param  result  Optional pointer to a PM_Result_t struct that receives
 *                 the per-driver outcome. Pass NULL to ignore.
 * @note   Mirrors PowerManager_SuspendAll() in reverse order.
 *         Returns PM_OK only if every driver succeeded.
 */
PM_Status_e PowerManager_WakeAll(PM_Result_t *result);

/**
 * @brief  Sends the STM32 into a low-power mode after suspending peripherals.
 * @note   NOT YET IMPLEMENTED — deferred to a future sprint.
 *         Agreed approach: Stop mode with UART wakeup (EXTI on USART1 RX pin).
 *         Requires .ioc change before implementation.
 *         See PowerManager.c stub for full details and pending items.
 */
void PowerManager_MCUSleep(void);

#endif /* POWERMANAGER_H */
