/**
 * @file    Bootloader.h
 * @brief   Software jump to the STM32 factory USB DFU bootloader, gated by two pins held at boot.
 *
 * @details No BOOT0 pin is exposed on this board, so entering the ST system
 *          bootloader (System Memory, address 0x1FFF0000 on STM32L433) is
 *          done entirely in software: if BOTH configured pins read LOW when
 *          Bootloader_CheckAndEnter() runs, the MCU resets its clocks/
 *          peripherals to a clean state and jumps to the bootloader's reset
 *          vector — the CPU ends up running ST's code exactly as if BOOT0
 *          had been HIGH at reset. From there the bootloader brings up USB
 *          on its own (PA11/PA12) and enumerates as a DFU device; flash with
 *          STM32CubeProgrammer selecting "USB" instead of "ST-LINK".
 *          If only one of the two pins is LOW, it does NOT enter — both
 *          must agree.
 *
 *          LED feedback (via el driver RGB LP55231 de este proyecto — el
 *          handle &rgb debe estar atado/Begin/Enable antes de llamar):
 *          ROJO parpadeando = los 2 pines en bajo, a punto de saltar al
 *          bootloader (unico caso que maneja este driver).
 *          Si NO entra, este driver no toca el LED — main.c decide entre
 *          VERDE (modo Logger, PB2=0) o AZUL (modo FTDI, PB2=1) usando la
 *          misma lectura de PB2 que ya condiciona Log_Init(). Ver USER
 *          CODE 2 en main.c.
 *
 *          This driver intentionally has no Driver_Test() — the only way to
 *          "test" it is to actually jump, which ends normal execution.
 *
 *          CubeMX / .ioc requirements:
 *          - BOOTLOADER_BTN_PORT/PIN y BOOTLOADER_BTN2_PORT/PIN deben estar
 *            configurados como entrada GPIO (en esta tarjeta llegan a
 *            traves de un mux: PB2 y PB1).
 *          - El handle &rgb (LP55231) debe estar Attach/Begin/Enable ANTES
 *            de llamar a esta funcion para que el indicador visual funcione.
 *
 * @date    August 23, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* Los DOS pines deben leer bajo para entrar al bootloader — si solo uno
 * esta en bajo, no entra. En esta tarjeta llegan a traves de un mux
 * (SEL_PROG_MCU_FTDI = PB2, SEL_PROG_LORA_BLU = PB1, ya configurados como
 * entrada en el .ioc) — activos en bajo, misma convencion que el driver
 * original. Cambiar aqui si los pines se reasignan. */

#define BOOTLOADER_BTN_PORT         GPIOB
#define BOOTLOADER_BTN_PIN          GPIO_PIN_2

#define BOOTLOADER_BTN2_PORT        GPIOB
#define BOOTLOADER_BTN2_PIN         GPIO_PIN_1

/* System Memory base address for STM32L433 (AN2606, STM32L43xxx/L44xxx table).
 * Confirmed via ST documentation — do not reuse this value on a different
 * STM32 family/line without checking AN2606 again. */

#define BOOTLOADER_SYSMEM_ADDR      0x1FFF0000UL

/* Parpadeo (rojo antes de saltar, o verde en arranque normal): periodo
 * on/off y numero de ciclos, mismo patron para los dos casos. */

#define BOOTLOADER_LED_BLINK_MS     400U
#define BOOTLOADER_LED_BLINK_COUNT  3U

/* ========================  ENUMERATIONS  ================================== */

/* Return code — only the "not entered" path actually returns; the "enter"
 * path jumps away and never comes back to the caller. */

typedef enum {
    BOOTLOADER_NOT_ENTERED = 0    /**< Button not held, continue normal boot */
} Bootloader_Status_e;

/* ================================  API  =================================== */

/**
 * @brief  Checks both boot pins and, if BOTH are held low, jumps to the USB
 *         DFU bootloader.
 * @note   Call after the RGB driver (&rgb) is Attach/Begin/Enable, right at
 *         the start of USER CODE 2 — before any other peripheral/driver
 *         init that you would not want left half-configured if this jumps
 *         away. Blinks the RGB LED red (par1/D2) BOOTLOADER_LED_BLINK_COUNT
 *         times and jumps (never returns) if both pins read LOW; otherwise
 *         leaves the LED untouched and returns BOOTLOADER_NOT_ENTERED — the
 *         caller (main.c) is responsible for the green/blue indication in
 *         that case.
 */
Bootloader_Status_e Bootloader_CheckAndEnter(void);

#endif /* BOOTLOADER_H */
