/**
 * @file    I2C1_Bus.c
 * @brief   Driver implementation for the shared I2C1 bus mutex.
 *
 * @date    August 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "I2C1_Bus.h"
#include "cmsis_os.h"
#include <stddef.h>

/* ======================  STATIC VARIABLES  ================================ */

static osMutexId_t s_mutex = NULL;   /* NULL hasta I2C1Bus_InitMutex() */

/* ================================  API  =================================== */

/* Public functions declared in the .h */

void I2C1Bus_InitMutex(void)
{
    /* osMutexPrioInherit: evita
     * inversion de prioridad si LoraTask (AboveNormal) llega a esperar
     * este mutex. */
    static const osMutexAttr_t attr = { .attr_bits = osMutexPrioInherit };
    s_mutex = osMutexNew(&attr);
}

void I2C1Bus_Lock(void)
{
    if (s_mutex != NULL) {
        osMutexAcquire(s_mutex, osWaitForever);
    }
}

void I2C1Bus_Unlock(void)
{
    if (s_mutex != NULL) {
        osMutexRelease(s_mutex);
    }
}
