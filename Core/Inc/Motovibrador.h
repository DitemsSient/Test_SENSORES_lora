/**
 * @file    Motovibrador.h
 * @brief   Driver for ERM/coin vibration motor on STM32L4xx.
 *
 * @details Supports two operation modes selected at compile time:
 *          - VIBRATOR_MODE_GPIO: digital ON/OFF via a single GPIO output.
 *          - VIBRATOR_MODE_PWM : variable intensity (0–100 %) via a timer
 *            PWM channel. Configure hardware defines to match the CubeMX
 *            .ioc settings for the chosen mode.
 *
 * @date    March 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef MOTOVIBRADOR_H
#define MOTOVIBRADOR_H

#include "stm32l4xx_hal.h"

/* ========================  CONFIGURATION  ================================= */

/* Select exactly one mode */

//#define VIBRATOR_MODE_PWM           /**< PWM mode: variable intensity 0–100 % */
#define VIBRATOR_MODE_GPIO          /**< GPIO mode: ON/OFF only                */

/* Hardware configuration — GPIO mode */

#ifdef VIBRATOR_MODE_GPIO

#define VIBRATOR_GPIO_PORT      GPIOH
#define VIBRATOR_GPIO_PIN       GPIO_PIN_0

#endif /* VIBRATOR_MODE_GPIO */

/* Hardware configuration — PWM mode */

#ifdef VIBRATOR_MODE_PWM

/**
 * @brief Timer handle assigned to the vibrator.
 * @note  Must match the timer configured in the CubeMX .ioc.
 *        Also update the extern declaration in Motovibrador.c.
 */
#define VIBRATOR_TIMER          (&htim3)
#define VIBRATOR_CHANNEL        TIM_CHANNEL_1

/**
 * @brief Auto-Reload Register value configured in CubeMX.
 * @note  Used to compute the duty cycle. If ARR = 999, then 50 % duty → CCR = 500.
 */
#define VIBRATOR_TIMER_ARR      999U

#endif /* VIBRATOR_MODE_PWM */

/**
 * @brief Gap between consecutive pattern steps (ms).
 * @note  Allows the motor to stop briefly between pulses.
 */
#define VIBRATOR_STEP_GAP_MS    10U

/* ====================  DEVICE CONSTANTS  ================================== */

/* Predefined intensity levels (0–100 %) */

#define VIB_OFF         0U      /**< Motor off                             */
#define VIB_SOFT        25U     /**< Soft vibration (25 %)                 */
#define VIB_MID         50U     /**< Medium vibration (50 %)               */
#define VIB_STRONG      75U     /**< Strong vibration (75 %)               */
#define VIB_MAX         100U    /**< Maximum vibration (100 %)             */

/* Calculates the number of steps in a static VibStep_t array */

#define VIB_PATTERN_LEN(arr)    (sizeof(arr) / sizeof((arr)[0]))

/* ============================  STRUCTURES  ================================ */

/* Represents one step in a vibration pattern */

typedef struct {
    uint8_t  intensity;     /**< Intensity 0–100 % (0 = off).
                                 GPIO mode: 0 = OFF, any > 0 = ON          */
    uint16_t duration_ms;   /**< Step duration in milliseconds             */
} VibStep_t;

/* ====================  PREDEFINED PATTERNS  =============================== */

/* Short pulse — confirmation / tap */

static const VibStep_t VIB_PATTERN_TICK[] = {
    { VIB_MAX,   50U },
};
#define VIB_PATTERN_TICK_LEN        1U

/* Double pulse — standard notification */

static const VibStep_t VIB_PATTERN_NOTIFICATION[] = {
    { VIB_MAX,  100U },
    { VIB_OFF,   80U },
    { VIB_MAX,  100U },
};
#define VIB_PATTERN_NOTIFICATION_LEN    3U

/* Triple fast pulse — alert / warning */

static const VibStep_t VIB_PATTERN_ALERT[] = {
    { VIB_MAX,   80U },
    { VIB_OFF,   60U },
    { VIB_MAX,   80U },
    { VIB_OFF,   60U },
    { VIB_MAX,   80U },
};
#define VIB_PATTERN_ALERT_LEN           5U

/* Long continuous vibration — incoming call / alarm */

static const VibStep_t VIB_PATTERN_CALL[] = {
    { VIB_MAX,  800U },
    { VIB_OFF,  400U },
    { VIB_MAX,  800U },
};
#define VIB_PATTERN_CALL_LEN            3U

/* Ascending ramp — soft start (PWM mode only for perceptible effect) */

static const VibStep_t VIB_PATTERN_RAMP[] = {
    { VIB_SOFT,    100U },
    { VIB_MID,     100U },
    { VIB_STRONG,  100U },
    { VIB_MAX,     200U },
};
#define VIB_PATTERN_RAMP_LEN            4U

/* Heartbeat — double-beat effect (PWM mode for best feel) */

static const VibStep_t VIB_PATTERN_HEARTBEAT[] = {
    { VIB_STRONG,  100U },
    { VIB_OFF,      80U },
    { VIB_MAX,     120U },
    { VIB_OFF,     400U },
};
#define VIB_PATTERN_HEARTBEAT_LEN       4U

/* SOS in Morse — · · · — — — · · · */

static const VibStep_t VIB_PATTERN_SOS[] = {
    /* S: · · · */
    { VIB_MAX,  100U },  { VIB_OFF,  100U },
    { VIB_MAX,  100U },  { VIB_OFF,  100U },
    { VIB_MAX,  100U },  { VIB_OFF,  200U },
    /* O: — — — */
    { VIB_MAX,  300U },  { VIB_OFF,  100U },
    { VIB_MAX,  300U },  { VIB_OFF,  100U },
    { VIB_MAX,  300U },  { VIB_OFF,  200U },
    /* S: · · · */
    { VIB_MAX,  100U },  { VIB_OFF,  100U },
    { VIB_MAX,  100U },  { VIB_OFF,  100U },
    { VIB_MAX,  100U },
};
#define VIB_PATTERN_SOS_LEN             19U

/* ================================  API  =================================== */

/**
 * @brief  Initializes the vibrator peripheral (GPIO or PWM, per config).
 * @note   Call once after MX_GPIOx_Init() or MX_TIMx_Init().
 */
void Vibrator_Init(void);

/**
 * @brief  Sets the vibrator intensity (0–100 %).
 * @param  intensity  Percentage (0 = off, 100 = maximum).
 * @note   GPIO mode: 0 → pin LOW, any other value → pin HIGH.
 *         PWM mode : value is mapped linearly to the duty cycle.
 */
void Vibrator_Set(uint8_t intensity);

/**
 * @brief  Turns the vibrator on at 100 % intensity.
 */
void Vibrator_On(void);

/**
 * @brief  Turns the vibrator off immediately.
 */
void Vibrator_Off(void);

/**
 * @brief  Plays a full VibStep_t pattern sequence.
 * @param  pattern  Pointer to the step array.
 * @param  length   Number of steps in the array.
 * @note   Blocking — uses HAL_Delay() internally. Motor is off when finished.
 */
void Vibrator_PlayPattern(const VibStep_t *pattern, uint16_t length);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Activa el motor a intensidad máxima durante 5 segundos.
 * @note   Bloqueante. El resultado lo confirma el usuario visualmente.
 */
void Vibrator_Test(void);

#endif /* MOTOVIBRADOR_H */
