/**
 * @file    Buzzer.c
 * @brief   Driver implementation for passive buzzer via PWM.
 *
 * @date    March 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Buzzer.h"

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern TIM_HandleTypeDef htim3;

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Starts the timer PWM channel and leaves the buzzer silent.
 */
void Buzzer_Init(void) {
    HAL_TIM_PWM_Start(BUZZER_TIMER, BUZZER_CHANNEL);
    Buzzer_Stop();
}

/**
 * @brief  Computes the period from the desired frequency and sets ARR and CCR
 *         so the output is a 50 % square wave at that frequency.
 */
void Buzzer_PlayTone(uint16_t freq) {
    if (freq == 0U) {
        Buzzer_Stop();
        return;
    }

    uint32_t period = BUZZER_TIMER_CLK / (uint32_t)freq;

    __HAL_TIM_SET_AUTORELOAD(BUZZER_TIMER, period - 1U);
    __HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CHANNEL, period / 2U);  /* 50 % duty */
}

/**
 * @brief  Sets CCR to 0, which disables the PWM output without stopping the timer.
 */
void Buzzer_Stop(void) {
    __HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CHANNEL, 0U);
}

/**
 * @brief  Iterates over the note array, converting each rhythmic value to ms
 *         using the BPM, plays the tone, waits, then inserts a short gap.
 *
 *         Whole-note duration: whole_ms = (60000 / bpm) * 4
 *         Note duration:       note_ms  = whole_ms / abs(duration)
 *         Dotted note (negative duration): note_ms *= 1.5
 */
void Buzzer_PlayMelody(const BuzzerNote_t *melody, uint16_t length, uint16_t bpm) {
    uint32_t whole_ms = (60000UL / (uint32_t)bpm) * 4UL;

    for (uint16_t i = 0U; i < length; i++) {
        int16_t  dur_raw = melody[i].duration;
        uint16_t dur_abs = (dur_raw < 0) ? (uint16_t)(-dur_raw) : (uint16_t)dur_raw;

        uint32_t note_ms = whole_ms / (uint32_t)dur_abs;

        /* Dotted note: duration × 1.5 */
        if (dur_raw < 0) {
            note_ms = note_ms + (note_ms / 2U);
        }

        Buzzer_PlayTone((uint16_t)melody[i].note);
        HAL_Delay(note_ms);
        Buzzer_Stop();
        HAL_Delay(BUZZER_NOTE_GAP_MS);
    }
}

/* ========================  SELF-TEST  ==================================== */

void Buzzer_Test(void)
{
    Buzzer_PlayTone(A4);
    HAL_Delay(5000U);
    Buzzer_Stop();
}
