/**
 * @file    I2C1_Bus.h
 * @brief   Mutex de bus compartido para I2C1 (BatteryMonitor, LSM6DSO32TR,
 *          MMC5983MA, SensorLuz_TSL2571, Driver_RGB — todos en el mismo bus).
 *
 * @details HAL_I2Cx no es reentrante entre tareas — dos llamadas concurrentes
 *          al mismo bus pueden hacer que una falle en silencio. Cada driver
 *          que toque hi2c1 debe envolver TODA llamada HAL directa (Transmit,
 *          Receive, Mem_Write, Mem_Read, IsDeviceReady — todas, no solo las
 *          obvias) entre I2C1Bus_Lock()/I2C1Bus_Unlock().
 *
 *          Mismo patron que el mutex del Logger: mientras no se llame
 *          I2C1Bus_InitMutex() (arranque bare-metal, antes de
 *          osKernelInitialize()) Lock()/Unlock() no hacen nada — un solo
 *          hilo de ejecucion en ese punto, no hace falta. Una vez creado el
 *          mutex (despues de osKernelInitialize(), antes de
 *          osKernelStart()), cada Lock()/Unlock() toma/suelta de verdad.
 *
 * @date    August 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef I2C1_BUS_H
#define I2C1_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================  API  =================================== */

/**
 * @brief  Crea el mutex del bus I2C1.
 * @note   Llamar UNA vez, despues de osKernelInitialize() y antes de
 *         osKernelStart() — igual que Log_InitMutex().
 */
void I2C1Bus_InitMutex(void);

/**
 * @brief  Toma el mutex del bus I2C1. No-op si el mutex aun no existe.
 */
void I2C1Bus_Lock(void);

/**
 * @brief  Suelta el mutex del bus I2C1. No-op si el mutex aun no existe.
 */
void I2C1Bus_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C1_BUS_H */
