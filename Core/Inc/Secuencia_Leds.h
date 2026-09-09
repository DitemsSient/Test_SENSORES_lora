/**
 * @file    Secuencia_Leds.h
 * @brief   Secuencias de parpadeo del RGB (LP55231) con nombre semantico —
 *          reune en un solo lugar los patrones ya usados por todo el
 *          proyecto (init, BluetoothTask, CalibrateTask) para que el
 *          significado de cada uno se lea en el nombre de la funcion, en
 *          vez de tener que interpretar bloques sueltos de
 *          LP55231_SetChannelPWM repetidos por varios archivos.
 *
 * @details Dos tipos de funciones:
 *          - "Set" (Leds_SetRojo/Verde/Azul/Magenta/Apagar): instantaneas,
 *            sin delay — encienden/apagan un color en los 3 pares fisicos
 *            (D1/D3/D5, D2/D4/D6, D7/D8/D9) y regresan de inmediato. Seguras
 *            de llamar tanto antes del kernel (Inicializacion_Run()) como
 *            dentro de una tarea.
 *          - "Parpadeo" (Leds_ParpadeoXxx): bloqueantes, usan HAL_Delay()
 *            internamente (funciona igual antes del kernel y dentro de una
 *            tarea — ver "HAL patterns" en CLAUDE.md, blocking intencional).
 *            NO usarlas donde haga falta hacer algo mas mientras se
 *            parpadea (ej. drenar UART) — para eso seguir armando el loop a
 *            mano con las funciones "Set" (ver BT_Blink en Tareas.c).
 *
 *          El bloque de deteccion de modo bootloader (Inicializacion_Run())
 *          NO usa esta libreria a proposito — ver memoria de proyecto
 *          "NUNCA quitar el bloque de bootloader", ese bloque no se toca.
 *
 * @date    September 07, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef SECUENCIA_LEDS_H
#define SECUENCIA_LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

#define LEDS_FIN_INIT_MS            400U   /**< Verde de cierre de Inicializacion_Run() */
#define LEDS_FIN_INIT_COUNT           3U

#define LEDS_CALIBRACION_OK_MS      500U   /**< Magenta al validar un impacto de calibracion (una sola vez) */

#define LEDS_BT_ERROR_MS            400U   /**< Rojo al recibir algo distinto de lo esperado en BluetoothTask */
#define LEDS_BT_ERROR_COUNT           3U

#define LEDS_BT_DSCON_MS            400U   /**< Cian (verde+azul) al recibir $DSCON — homologado con Mira */
#define LEDS_BT_DSCON_COUNT           5U

#define LEDS_RUN_CUENTA_MS           500U   /**< Magenta: cuenta regresiva de 10s tras ACKRUN (10 x 500ms on/off = 10s) */
#define LEDS_RUN_CUENTA_COUNT         10U

#define LEDS_RUN_INICIO_MS           400U   /**< Magenta: confirmacion de arranque de MODO_EJERCICIO, tras la cuenta regresiva (un solo parpadeo) */
#define LEDS_RUN_INICIO_COUNT          1U

#define LEDS_END_S_MS                400U   /**< Rojo al confirmarse ACKEND_S (fin de ejercicio por orden del admin) — homologado con Mira */
#define LEDS_END_S_COUNT               5U

#define LEDS_FIN_JUEGO_MS            300U   /**< Parpadeo colorido (rojo/verde/azul/magenta) al llegar $END_A — fin normal por tiempo agotado */
#define LEDS_FIN_JUEGO_VUELTAS         3U    /**< Cuantas veces se repite el ciclo completo de 4 colores — homologado con Mira */

/* ================================  API  =================================== */

/**
 * @brief  Apaga rojo+verde+azul en los 3 pares fisicos.
 */
void Leds_Apagar(void);

/**
 * @brief  Enciende/apaga SOLO el canal rojo en los 3 pares fisicos.
 * @param  on  true enciende (0xFF), false apaga (0x00).
 */
void Leds_SetRojo(bool on);

/**
 * @brief  Enciende/apaga SOLO el canal verde en los 3 pares fisicos.
 * @param  on  true enciende (0xFF), false apaga (0x00).
 */
void Leds_SetVerde(bool on);

/**
 * @brief  Enciende/apaga SOLO el canal azul en los 3 pares fisicos.
 * @param  on  true enciende (0xFF), false apaga (0x00).
 */
void Leds_SetAzul(bool on);

/**
 * @brief  Enciende/apaga rojo+azul juntos (magenta) en los 3 pares fisicos.
 * @param  on  true enciende (0xFF), false apaga (0x00).
 */
void Leds_SetMagenta(bool on);

/**
 * @brief  Parpadeo verde de cierre de Inicializacion_Run() — LEDS_FIN_INIT_COUNT
 *         veces, LEDS_FIN_INIT_MS on/off. Bloqueante (HAL_Delay), se llama
 *         antes de osKernelStart().
 */
void Leds_ParpadeoFinInit(void);

/**
 * @brief  Parpadeo magenta al validar un impacto de calibracion — una sola
 *         vez, LEDS_CALIBRACION_OK_MS on/off. Bloqueante (HAL_Delay).
 */
void Leds_ParpadeoCalibracionOk(void);

/**
 * @brief  Parpadeo rojo generico — veces repeticiones, periodo_ms on/off
 *         cada una. Bloqueante (HAL_Delay). Usado con
 *         (LEDS_BT_ERROR_COUNT, LEDS_BT_ERROR_MS) y con
 *         (LEDS_END_S_COUNT, LEDS_END_S_MS) al confirmarse ACKEND_S — ver
 *         BT_HandleEndS() en Tareas.c.
 * @note   Si hace falta drenar UART mientras se parpadea (como hace
 *         BluetoothTask al recibir $DSCON), no usar esta funcion — armar
 *         el loop a mano con Leds_SetRojo()/osDelay(), ver BT_Blink() en
 *         Tareas.c.
 */
void Leds_ParpadeoRojo(uint8_t veces, uint32_t periodo_ms);

/**
 * @brief  Parpadeo magenta generico — veces repeticiones, periodo_ms on/off
 *         cada una. Bloqueante (HAL_Delay). Usado con
 *         (LEDS_RUN_CUENTA_COUNT, LEDS_RUN_CUENTA_MS) para la cuenta
 *         regresiva visual de 10s tras ACKRUN, y con (LEDS_RUN_INICIO_COUNT,
 *         LEDS_RUN_INICIO_MS) para el parpadeo corto que confirma que ya se
 *         entro a MODO_EJERCICIO — ver BT_HandleRun() en Tareas.c.
 */
void Leds_ParpadeoMagenta(uint8_t veces, uint32_t periodo_ms);

/**
 * @brief  Secuencia colorida de fin de juego — recorre rojo/verde/azul/
 *         magenta LEDS_FIN_JUEGO_VUELTAS veces completas, LEDS_FIN_JUEGO_MS
 *         on/off cada color. Bloqueante (HAL_Delay). Se usa al llegar
 *         $END_A por Bluetooth, ver Tareas.c.
 */
void Leds_ParpadeoFinJuego(void);

#ifdef __cplusplus
}
#endif

#endif /* SECUENCIA_LEDS_H */
