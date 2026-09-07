/**
 * @file    Tareas.h
 * @brief   Creacion y coordinacion de las tareas FreeRTOS de la tarjeta.
 *
 * @details Mismo patron que el proyecto hermano (Mira):
 *          main() -> Inicializacion_Run() (bare-metal, antes del kernel)
 *                 -> osKernelInitialize()
 *                 -> Tareas_InicializarMutex() (mutex de recursos
 *                    compartidos — Logger, bus I2C1 — con
 *                    osMutexPrioInherit, ver Logger.c/I2C1_Bus.c)
 *                 -> Tareas_CrearTareas() (todas las tareas de una vez,
 *                    algunas pueden nacer suspendidas si no deben correr
 *                    todavia)
 *                 -> osKernelStart()
 *
 *          Regla dura: ni Tareas_InicializarMutex() ni Tareas_CrearTareas()
 *          deben llamar Log_Print/Log_Printf — corren DESPUES de que el
 *          mutex del Logger ya existe pero ANTES de que el scheduler
 *          arranque, asi que cualquier intento de tomar ese mutex ahi se
 *          cuelga para siempre en silencio. El primer log (banner de heap
 *          libre, fallas de creacion de tareas, etc.) va hasta la primera
 *          linea de la primera tarea que corre.
 *
 * @date    September 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef TAREAS_H
#define TAREAS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Crea los mutex de recursos compartidos entre tareas (Logger, bus
 *         I2C1), con herencia de prioridad activada.
 * @note   Llamar UNA vez, despues de osKernelInitialize() y antes de
 *         osKernelStart(). No llamar Log_Print/Log_Printf aqui ni despues,
 *         hasta que el scheduler este corriendo.
 */
void Tareas_InicializarMutex(void);

/**
 * @brief  Crea todas las tareas de la aplicacion.
 * @note   Llamar UNA vez, despues de Tareas_InicializarMutex() y antes de
 *         osKernelStart(). Mismo cuidado que arriba: nada de Log_Print aqui.
 */
void Tareas_CrearTareas(void);

#ifdef __cplusplus
}
#endif

#endif /* TAREAS_H */
