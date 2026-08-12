/**
 * @file    ModoProgramacion.h
 * @brief   Programming mode selector — switches the analog mux between
 *          USB/MCU and Bluetooth programming paths.
 *
 * @details Controls a single GPIO connected to a CD4051B (or equivalent)
 *          analog mux that routes the programming bus:
 *            - LOW  (default) → USB / MCU path
 *            - HIGH           → Bluetooth path
 *
 *          CubeMX: configure MODOPROG_PIN as GPIO_Output, no pull,
 *          initial state LOW. Pin label: PROG_MUX_SEL.
 *
 * @date    June 01, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef MODOPROGRAMACION_H
#define MODOPROGRAMACION_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* GPIO connected to the mux select pin. Update to match the .ioc. */
/* Default: D8 on Nucleo-144 (PF12) — change port/pin as needed.  */

#define MODOPROG_PORT           GPIOB
#define MODOPROG_PIN            GPIO_PIN_1

/* ========================  ENUMERATIONS  ================================== */

/* Active programming path — extend values if a wider mux is added */

typedef enum {
    MODOPROG_MODE_MCU    = 0,   /**< USB / MCU path (default) */
    MODOPROG_MODE_BT     = 1    /**< Bluetooth path           */
} ModoProg_Mode_e;

/* Return codes for driver functions */

typedef enum {
    MODOPROG_OK          = 0,
    MODOPROG_ERR         = 1
} ModoProg_Status_e;

/* ========================  GLOBAL STATE  ================================== */

/* Current programming mode. Read from anywhere with:                        */
/*   #include "ModoProgramacion.h"  →  ModoProg_CurrentMode                  */

extern ModoProg_Mode_e ModoProg_CurrentMode;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the mux select pin to MCU mode (LOW).
 * @note   Call once after MX_GPIO_Init().
 */
ModoProg_Status_e ModoProgramacion_Init(void);

/**
 * @brief  Switches the mux to the Bluetooth programming path (HIGH).
 */
void ModoProgramacion_SetBT(void);

/**
 * @brief  Switches the mux to the USB/MCU programming path (LOW).
 */
void ModoProgramacion_SetMCU(void);

/**
 * @brief  Returns true if the Bluetooth path is currently selected.
 */
bool ModoProgramacion_IsBluetoothMode(void);

#endif /* MODOPROGRAMACION_H */
