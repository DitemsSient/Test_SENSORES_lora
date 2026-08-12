/**
 * @file    ModoProgramacion.c
 * @brief   Driver implementation for the programming mux selector.
 *
 * @date    June 01, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "ModoProgramacion.h"

/* ========================  GLOBAL STATE  ================================== */

/* Initialized to MCU mode; updated by SetBT() / SetMCU() */

ModoProg_Mode_e ModoProg_CurrentMode = MODOPROG_MODE_MCU;

/* ================================  API  =================================== */

/**
 * @brief  Sets the mux pin LOW to guarantee MCU path on startup.
 */
ModoProg_Status_e ModoProgramacion_Init(void)
{
    HAL_GPIO_WritePin(MODOPROG_PORT, MODOPROG_PIN, GPIO_PIN_RESET);
    ModoProg_CurrentMode = MODOPROG_MODE_MCU;
    return MODOPROG_OK;
}

/**
 * @brief  Drives the pin HIGH, routing the bus to the Bluetooth module.
 */
void ModoProgramacion_SetBT(void)
{
    HAL_GPIO_WritePin(MODOPROG_PORT, MODOPROG_PIN, GPIO_PIN_SET);
    ModoProg_CurrentMode = MODOPROG_MODE_BT;
}

/**
 * @brief  Drives the pin LOW, routing the bus back to USB/MCU.
 */
void ModoProgramacion_SetMCU(void)
{
    HAL_GPIO_WritePin(MODOPROG_PORT, MODOPROG_PIN, GPIO_PIN_RESET);
    ModoProg_CurrentMode = MODOPROG_MODE_MCU;
}

/**
 * @brief  Returns true if the Bluetooth path is currently selected.
 */
bool ModoProgramacion_IsBluetoothMode(void)
{
    return (ModoProg_CurrentMode == MODOPROG_MODE_BT);
}
