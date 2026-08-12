/**
 * @file    Motovibrador.c
 * @brief   Driver implementation for ERM/coin vibration motor.
 *
 * @date    March 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Motovibrador.h"

/* ======================  EXTERNAL HAL HANDLES  ============================ */

#ifdef VIBRATOR_MODE_PWM
extern TIM_HandleTypeDef htim3;
#endif

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Starts the PWM channel (PWM mode only) and turns the motor off.
 */
void Vibrator_Init(void) {
#ifdef VIBRATOR_MODE_PWM
    HAL_TIM_PWM_Start(VIBRATOR_TIMER, VIBRATOR_CHANNEL);
#endif
    Vibrator_Off();
}

/**
 * @brief  Clamps intensity to 100, then either drives the GPIO high/low
 *         (GPIO mode) or maps the percentage to a CCR value (PWM mode).
 *
 *         PWM mapping: ccr = (intensity * ARR) / 100
 */
void Vibrator_Set(uint8_t intensity) {
    if (intensity > 100U) {
        intensity = 100U;
    }

#ifdef VIBRATOR_MODE_GPIO
    HAL_GPIO_WritePin(VIBRATOR_GPIO_PORT, VIBRATOR_GPIO_PIN,
                      (intensity > 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

#ifdef VIBRATOR_MODE_PWM
    uint32_t ccr = ((uint32_t)intensity * VIBRATOR_TIMER_ARR) / 100U;
    __HAL_TIM_SET_COMPARE(VIBRATOR_TIMER, VIBRATOR_CHANNEL, ccr);
#endif
}

/**
 * @brief  Delegates to Vibrator_Set(100).
 */
void Vibrator_On(void) {
    Vibrator_Set(100U);
}

/**
 * @brief  Delegates to Vibrator_Set(0).
 */
void Vibrator_Off(void) {
    Vibrator_Set(0U);
}

/**
 * @brief  Iterates over the step array calling Vibrator_Set() for each step's
 *         intensity, waits duration_ms, then turns the motor off when done.
 */
void Vibrator_PlayPattern(const VibStep_t *pattern, uint16_t length) {
    for (uint16_t i = 0U; i < length; i++) {
        Vibrator_Set(pattern[i].intensity);
        HAL_Delay(pattern[i].duration_ms);
    }
    Vibrator_Off();
}

/* ========================  SELF-TEST  ==================================== */

void Vibrator_Test(void)
{
    Vibrator_On();
    HAL_Delay(5000U);
    Vibrator_Off();
}
