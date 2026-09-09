/**
 * @file    Secuencia_Leds.c
 * @brief   Driver implementation for the named LED blink sequences.
 *
 * @date    September 07, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Secuencia_Leds.h"
#include "Driver_RGB.h"

/* ======================  EXTERNAL HAL HANDLES  ============================ */

/* Handle del LP55231 y arreglos de canales — definidos NO estaticos en
 * Inicializacion.c (ver comentarios ahi), reusados aqui tal cual. */
extern LP55231_t     rgb;
extern const uint8_t rgb_red_ch[3];
extern const uint8_t rgb_green_ch[3];
extern const uint8_t rgb_blue_ch[3];

/* ======================  STATIC FUNCTIONS  ================================ */

static void Leds_SetCanal(const uint8_t ch[3], bool on)
{
    uint8_t val = on ? 0xFFU : 0x00U;
    for (uint8_t j = 0U; j < 3U; j++) {
        LP55231_SetChannelPWM(&rgb, ch[j], val);
    }
}

/* ================================  API  =================================== */

void Leds_Apagar(void)
{
    Leds_SetCanal(rgb_red_ch,   false);
    Leds_SetCanal(rgb_green_ch, false);
    Leds_SetCanal(rgb_blue_ch,  false);
}

void Leds_SetRojo(bool on)
{
    Leds_SetCanal(rgb_red_ch, on);
}

void Leds_SetVerde(bool on)
{
    Leds_SetCanal(rgb_green_ch, on);
}

void Leds_SetAzul(bool on)
{
    Leds_SetCanal(rgb_blue_ch, on);
}

void Leds_SetMagenta(bool on)
{
    Leds_SetCanal(rgb_red_ch,  on);
    Leds_SetCanal(rgb_blue_ch, on);
}

void Leds_ParpadeoFinInit(void)
{
    for (uint8_t i = 0U; i < LEDS_FIN_INIT_COUNT; i++) {
        Leds_SetVerde(true);
        HAL_Delay(LEDS_FIN_INIT_MS);
        Leds_SetVerde(false);
        HAL_Delay(LEDS_FIN_INIT_MS);
    }
}

void Leds_ParpadeoCalibracionOk(void)
{
    Leds_SetMagenta(true);
    HAL_Delay(LEDS_CALIBRACION_OK_MS);
    Leds_SetMagenta(false);
}

void Leds_ParpadeoRojo(uint8_t veces, uint32_t periodo_ms)
{
    for (uint8_t i = 0U; i < veces; i++) {
        Leds_SetRojo(true);
        HAL_Delay(periodo_ms);
        Leds_SetRojo(false);
        HAL_Delay(periodo_ms);
    }
}

void Leds_ParpadeoMagenta(uint8_t veces, uint32_t periodo_ms)
{
    for (uint8_t i = 0U; i < veces; i++) {
        Leds_SetMagenta(true);
        HAL_Delay(periodo_ms);
        Leds_SetMagenta(false);
        HAL_Delay(periodo_ms);
    }
}

void Leds_ParpadeoFinJuego(void)
{
    for (uint8_t vuelta = 0U; vuelta < LEDS_FIN_JUEGO_VUELTAS; vuelta++) {
        Leds_SetRojo(true);
        HAL_Delay(LEDS_FIN_JUEGO_MS);
        Leds_SetRojo(false);

        Leds_SetVerde(true);
        HAL_Delay(LEDS_FIN_JUEGO_MS);
        Leds_SetVerde(false);

        Leds_SetAzul(true);
        HAL_Delay(LEDS_FIN_JUEGO_MS);
        Leds_SetAzul(false);

        Leds_SetMagenta(true);
        HAL_Delay(LEDS_FIN_JUEGO_MS);
        Leds_SetMagenta(false);
    }
}
