/**
 * @file    Receptor_Infrarrojo_EXTI.h
 * @brief   IR receiver driver for TSOP module via GPIO EXTI + DWT cycle counter.
 *
 * @details Reemplaza al Timer Input Capture (ver Receptor_Infrarrojo_TSOP.h,
 *          deprecado): esta tarjeta no tiene ningun timer con canal Input
 *          Capture libre para el pin del TSOP. En su lugar:
 *          - EXTI en el pin del TSOP, ambos flancos, dispara
 *            IR_EXTI_Callback() desde HAL_GPIO_EXTI_Callback().
 *          - DWT->CYCCNT (contador de ciclos del nucleo Cortex-M4, libre,
 *            no consume ningun TIMx) mide el delta entre flancos.
 *
 *          Logica del TSOP (senal invertida): flanco de bajada (pin queda
 *          en bajo) = fin de SPACE, es el que codifica bit/separador. El
 *          flanco de subida (fin de MARK) se descarta.
 *
 *          Decodificacion (umbrales validados contra un transmisor real,
 *          ver IR_BIT0_MAX_US / IR_BIT1_MAX_US abajo):
 *          - SPACE corto   -> bit 0
 *          - SPACE mediano -> bit 1
 *          - SPACE largo   -> separador de byte / fin de trama
 *          Los bits se acumulan MSB-first hasta completar IR_BITS_PER_BYTE;
 *          cada separador cierra el byte actual y empieza el siguiente. El
 *          fin de trama se detecta por silencio (IR_SILENCE_MS sin flancos
 *          nuevos) desde el loop principal, vía IR_Process().
 *
 *          Este driver NO valida checksum ni impone una estructura de
 *          trama fija — solo entrega los bytes decodificados tal cual
 *          llegaron. La interpretacion del contenido (checksum, longitud
 *          esperada, etc.) es responsabilidad del caller.
 *
 *          CubeMX / .ioc requirements:
 *          - El pin del TSOP en modo "External Interrupt Mode with
 *            Rising/Falling edge trigger detection" (los DOS flancos).
 *          - El NVIC del EXTI correspondiente habilitado (EXTI0_IRQn para
 *            PB0/SENSOR_IR2 en esta tarjeta).
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef RECEPTOR_INFRARROJO_EXTI_H
#define RECEPTOR_INFRARROJO_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* GPIO del TSOP (SENSOR_IR2 en esta tarjeta) */

#define IR_TSOP_PORT             GPIOB
#define IR_TSOP_PIN              GPIO_PIN_0

/* Umbrales de clasificacion del ancho de SPACE, en microsegundos.
 * Validados contra un transmisor real ("A#" cada 3 s): cortos ~350-450us
 * (bit 0), medianos ~700-900us (bit 1), separador ~1150-1600us. Deja
 * margen entre bandas para tolerar jitter del transmisor y de la ISR. */

#define IR_BIT0_MAX_US           500U     /**< SPACE < esto        -> bit 0 */
#define IR_BIT1_MAX_US           900U     /**< 500 <= SPACE < esto -> bit 1 */
                                            /**< SPACE >= IR_BIT1_MAX_US -> separador */

/* Tiempo sin ningun flanco nuevo que se interpreta como "trama terminada" */

#define IR_SILENCE_MS            1000U

/* ====================  DEVICE CONSTANTS  ================================== */

#define IR_BITS_PER_BYTE         8U
#define IR_FRAME_MAX_BYTES       16U     /**< Cuantos bytes decodificados caben por trama */

/* ========================  ENUMERATIONS  ================================== */

typedef enum {
    IR_OK             = 0,
    IR_ERR_PARAM      = 1,
    IR_ERR_NO_FRAME   = 2    /**< IR_Process() llamado sin trama lista */
} IrStatus_e;

/* ============================  STRUCTURES  ================================ */

/** @brief Contexto del driver — todo lo que toca la ISR es volatile. */
typedef struct {
    volatile uint32_t last_edge_cycles;   /**< DWT->CYCCNT en el flanco anterior    */
    volatile bool     first_edge;         /**< true hasta el primer flanco recibido */
    volatile uint32_t last_edge_tick;     /**< HAL_GetTick() del ultimo flanco (ms) */

    volatile uint8_t  current_byte;       /**< Byte en construccion (bits MSB-first) */
    volatile uint8_t  bit_count;          /**< Bits acumulados en current_byte       */

    volatile uint8_t  frame_buf[IR_FRAME_MAX_BYTES]; /**< Bytes decodificados        */
    volatile uint8_t  frame_len;                     /**< Bytes validos en frame_buf */
    volatile bool     frame_overflow;                /**< true si no cupo un byte    */

    bool     frame_ready;                 /**< true cuando IR_Process() detecta silencio */
} Ir_Handle_t;

/* ================================  API  =================================== */

/**
 * @brief  Habilita DWT->CYCCNT y deja el contexto listo para la primera trama.
 * @param  h  Puntero al handle del driver.
 * @note   Llamar una sola vez, antes de que puedan llegar interrupciones del TSOP.
 */
IrStatus_e IR_Init(Ir_Handle_t *h);

/**
 * @brief  Callback de EXTI — llamar desde HAL_GPIO_EXTI_Callback() cuando el
 *         pin que dispare sea IR_TSOP_PIN.
 * @param  h  Puntero al handle del driver.
 * @note   Trabajo minimo pero no trivial: clasifica el SPACE y acumula bits/
 *         bytes. Nunca llama a Log_Print/Log_Printf desde aqui.
 */
void IR_EXTI_Callback(Ir_Handle_t *h);

/**
 * @brief  Revisa si ya paso IR_SILENCE_MS sin flancos nuevos; si es asi,
 *         marca la trama como lista (h->frame_ready) para que el caller la
 *         lea de h->frame_buf / h->frame_len y luego reinicie con IR_Reset().
 * @param  h  Puntero al handle del driver.
 * @note   Llamar en cada vuelta del while(1). No hace nada si no hay bytes
 *         acumulados o si todavia hay actividad.
 */
IrStatus_e IR_Process(Ir_Handle_t *h);

/**
 * @brief  Reinicia el buffer de trama para la siguiente captura.
 * @param  h  Puntero al handle del driver.
 * @note   Llamar despues de leer h->frame_buf/h->frame_len con frame_ready true.
 */
void IR_Reset(Ir_Handle_t *h);

/**
 * @brief  Self-test: confirma que DWT->CYCCNT esta corriendo (avanza con el
 *         tiempo). No prueba la recepcion IR en si — eso requiere un
 *         transmisor real, fuera del alcance de un self-test automatico.
 * @retval 1 si DWT->CYCCNT avanza, 0 si no.
 */
uint8_t IR_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* RECEPTOR_INFRARROJO_EXTI_H */
