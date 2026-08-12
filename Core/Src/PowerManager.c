/**
 * @file    PowerManager.c
 * @brief   Driver implementation for the system-level low-power coordinator.
 *
 * @date    May 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "PowerManager.h"
#include "Flash.h"
#include "GPS.h"
#include "LSM6DSO32TR.h"
#include "SensorLuz_TSL2571.h"
#include "BatteryMonitor.h"
#include "Buzzer.h"
#include "Motovibrador.h"
#include "Lora.h"

/* ======================  STATIC VARIABLES  ================================ */

/* Registered driver handles */

static Gps_Handle_t  *pm_gps   = NULL;
static LSM6DSO32TR_t *pm_imu   = NULL;
static TSL2571_t     *pm_light = NULL;
static LP55231_t     *pm_rgb   = NULL;
static Lora_Handle_t *pm_lora  = NULL;
static bool           pm_ready = false;

/* ================================  API  =================================== */

PM_Status_e PowerManager_Init(Gps_Handle_t    *hgps,
                               LSM6DSO32TR_t   *himu,
                               TSL2571_t       *hlight,
                               LP55231_t       *hrgb,
                               Lora_Handle_t   *hlora)
{
    if (hgps == NULL || himu == NULL || hlight == NULL ||
        hrgb == NULL || hlora == NULL) return PM_ERR_PARAM;

    pm_gps   = hgps;
    pm_imu   = himu;
    pm_light = hlight;
    pm_rgb   = hrgb;
    pm_lora  = hlora;
    pm_ready = true;

    return PM_OK;
}

PM_Status_e PowerManager_SuspendAll(PM_Result_t *result)
{
    if (!pm_ready) return PM_ERR_NOT_INIT;

    PM_Result_t res = { true, true, true, true, true, true, true };
    bool all_ok = true;

    /* --- Actuators off (no error possible) --- */
    Buzzer_Stop();
    Vibrator_Off();

    /* --- ICs with low-power commands --- */

    res.flash_ok   = (Flash_PowerDown()                        == FLASH_OK);
    res.gps_ok     = (Gps_SendMTK(pm_gps, GPS_CMD_STANDBY)    == GPS_OK);
    res.imu_ok     = (LSM6DSO32TR_PowerDown(pm_imu) == LSM_OK);
    res.light_ok   = (TSL2571_Disable(pm_light)                == HAL_OK);
    res.gauge_ok   = (BatGauge_Hibernate()                     == HAL_OK);
    res.rgb_ok     = (LP55231_Disable(pm_rgb)                  == HAL_OK);
    res.lora_ok    = (Lora_Sleep(pm_lora)                      == LORA_OK);

    /* GPS FORCE_ON pin LOW — allows module to enter standby */
    Gps_ForceOff();

    /* BL654 Bluetooth enters auto-sleep when UART is idle — no command needed */

    if (!res.flash_ok || !res.gps_ok  || !res.imu_ok   ||
        !res.light_ok || !res.gauge_ok || !res.rgb_ok   ||
        !res.lora_ok) {
        all_ok = false;
    }

    if (result != NULL) *result = res;
    return all_ok ? PM_OK : PM_ERR_PARTIAL;
}

PM_Status_e PowerManager_WakeAll(PM_Result_t *result)
{
    if (!pm_ready) return PM_ERR_NOT_INIT;

    PM_Result_t res = { true, true, true, true, true, true, true };
    bool all_ok = true;

    /* --- ICs: restore from low-power --- */

    res.flash_ok  = (Flash_WakeUp()               == FLASH_OK);
    res.gauge_ok  = (BatGauge_WakeUp()             == HAL_OK);
    res.imu_ok    = (LSM6DSO32TR_PowerOn(pm_imu) == LSM_OK);
    res.light_ok  = (TSL2571_Enable(pm_light)      == HAL_OK);
    res.rgb_ok    = (LP55231_Enable(pm_rgb)        == HAL_OK);
    res.lora_ok   = (Lora_WakeUp(pm_lora)          == LORA_OK);

    /* GPS: assert FORCE_ON before sending wake byte */
    Gps_ForceOn();
    HAL_Delay(10U);
    /* Any byte on the UART wakes the L86-M33 from standby */
    res.gps_ok = (Gps_SendMTK(pm_gps, GPS_CMD_HOT_RESTART) == GPS_OK);

    /* BL654 Bluetooth wakes automatically when UART traffic resumes */

    if (!res.flash_ok || !res.gps_ok  || !res.imu_ok   ||
        !res.light_ok || !res.gauge_ok || !res.rgb_ok   ||
        !res.lora_ok) {
        all_ok = false;
    }

    if (result != NULL) *result = res;
    return all_ok ? PM_OK : PM_ERR_PARTIAL;
}

void PowerManager_MCUSleep(void)
{
    /* NOT YET IMPLEMENTED — deferred to a future sprint.
     *
     * Agreed implementation approach: STM32 Stop mode with UART wakeup.
     * The MCU wakes when any byte arrives on huart1 (EXTI on the RX pin).
     *
     * Pending before implementing:
     *   - Open BoardPruebas.ioc in CubeMX, configure the USART1 RX pin
     *     (PA10) as GPIO_EXTI10 with falling-edge detection and enable
     *     the EXTI15_10 interrupt in NVIC.
     *
     * Implementation steps once .ioc is updated:
     *   1. Call PowerManager_SuspendAll()
     *   2. Enable EXTI wakeup on USART1 RX pin
     *   3. HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI)
     *   4. On return: call SystemClock_Config() to restore PLL
     *   5. Re-arm UART interrupt: HAL_UART_Receive_IT(...)
     *   6. Call PowerManager_WakeAll()
     *
     * Also pending:
     *   - Validate RM1262 LoRa sleep command on hardware. Primary command
     *     is AT+SLEEP\r\n. If the module does not respond, try the
     *     alternatives commented in Lora_Sleep() (Lora.c).
     */
}
