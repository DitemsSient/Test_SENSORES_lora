# Migración a STM32L433CCUx

Este proyecto (`SIENT_SENSORES_DITEMS`) se configuró originalmente para
`STM32L452CEUx`, pero el MCU real en la protoboard es `STM32L433CCUx`. Se creó
un proyecto CubeIDE nuevo apuntando al chip correcto. Esta es la guía de qué
mover y cómo, para no perder el trabajo de drivers/documentación/pruebas ya
hecho aquí.

---

## 1. Copiar TAL CUAL al proyecto nuevo

Estos archivos no dependen del modelo exacto de MCU (son wrappers HAL
genéricos de familia STM32L4), cópialos completos, reemplazando lo que haya
en el proyecto nuevo:

### `Core/Inc/` — 20 headers de drivers
```
BatteryMonitor.h        Lora.h                   Receptor_Infrarrojo_TSOP.h
Bluetooth.h              MMC5983MA.h              SensorLuz_TSL2571.h
Buzzer.h                 ModoProgramacion.h       HWTest_Status.h
Buzzer_Melodias.h        Motovibrador.h
Comunicacion_RS485.h     PowerManager.h
Driver_RGB.h             Flash.h
GPS.h                    Logger.h
LSM6DSO32TR.h
```
**NO copies** `main.h`, `stm32l4xx_it.h`, `stm32l4xx_hal_conf.h` — esos los
genera CubeMX específicos para el `.ioc` nuevo.

### `Core/Src/` — 14 fuentes de drivers
```
BatteryMonitor.c   Comunicacion_RS485.c   Lora.c              PowerManager.c
Bluetooth.c        Driver_RGB.c           MMC5983MA.c         Receptor_Infrarrojo_TSOP.c
Buzzer.c           Flash.c                ModoProgramacion.c  SensorLuz_TSL2571.c
                   GPS.c                   Motovibrador.c      Logger.c
```
**NO copies** `main.c`, `stm32l4xx_it.c`, `system_stm32l4xx.c`,
`stm32l4xx_hal_msp.c`, `syscalls.c`, `sysmem.c` — generados/específicos del
proyecto nuevo. El contenido de `main.c` (tus 8 tareas) se migra a mano —
ver sección 3.

### Todo `Core/Doc/`
Cópialo completo — es documentación, no depende del MCU.

### `.claude/` y `CLAUDE.md`
Cópialos completos. **Después de la migración hay que actualizar el
`CLAUDE.md`** (MCU real, tabla de periféricos) una vez que confirmes qué
quedó configurado en el `.ioc` nuevo — sobre todo si `UART4` no existe en el
L433 y hay que reasignar RS485 a otro periférico.

---

## 2. NO copiar (se regenera en el proyecto nuevo)

- `main.c`, `main.h`
- `stm32l4xx_it.c/h`
- `stm32l4xx_hal_conf.h`
- `stm32l4xx_hal_msp.c`
- `system_stm32l4xx.c`
- `syscalls.c`, `sysmem.c`
- `*.ioc`, `.mxproject`, `.project`, `.cproject`
- Carpeta `Drivers/` completa (HAL/CMSIS — CubeMX la genera para el chip correcto)

---

## 3. Contenido de `main.c` a re-pegar a mano

El `main.c` nuevo lo genera CubeMX vacío (solo con los `MX_..._Init()` que
configures en el `.ioc` nuevo). Una vez que el `.ioc` nuevo tenga los
periféricos armados (y sepamos si hay que mover RS485 de UART4 a otro
periférico), pega esto dentro de los bloques `USER CODE` correspondientes
del `main.c` nuevo:

### `USER CODE BEGIN Includes`
```c
#include "Logger.h"
#include "Buzzer.h"
#include "Buzzer_Melodias.h"
#include "Motovibrador.h"
#include "Flash.h"
#include "Lora.h"
#include "GPS.h"
#include <string.h>
```

### `USER CODE BEGIN PD`
```c
/* Tarea 5: direccion de prueba para Flash — ultimo sector, igual que Flash_Test() */
#define FLASH_TASK5_TEST_ADDR   0x7FF000U
```

### `USER CODE BEGIN PV`
```c
/* Tarea 6: handle de LoRa enlazado a mano (sin Lora_Init(), ver nota abajo) */
static Lora_Handle_t hlora;
```

### `USER CODE BEGIN 2` (antes del `while(1)`)
```c
  /* ===================  TAREA 1: UART de RS485 (Logger)  =================
   * Objetivo: confirmar que se puede escribir por el UART de RS485 (UART4,
   * PA0/PA1) usando Log_Print/Log_Printf. huart4 a 115200 8N1. Manda un
   * mensaje cada 1 s en el loop principal.
   * NOTA: ModoProgramacion y el mux de selección son un pendiente aparte,
   * no forman parte de esta tarea.
   *
   * RESULTADO EN EL PROYECTO VIEJO (MCU equivocado): no se pudo leer nada,
   * ni siquiera con jumper directo en el pin del micro — causa raiz: el
   * .ioc estaba armado para STM32L452CEUx en vez del STM32L433CCUx real.
   * Repetir esta prueba de cero en el proyecto nuevo.
   */
//  Log_Init();
//  Log_Print("TEST", "Boot OK - Tarea 1: UART RS485 (Logger)");
  /* ======================================================================= */

  /* ===================  TAREA 2: Escaneo de bus I2C1  ====================
   * EN PAUSA: el escaneo de I2C en la tarjeta Mira dio problema de bus.
   * No se corre ninguna prueba de I2C en esta tarjeta hasta validar eso.
   *
   * Log_Print("TEST", "Boot OK - Tarea 2: Escaneo de bus I2C1");
   */
  /* ======================================================================= */

  /* ===================  TAREA 3: Buzzer (melodia)  =======================
   * Objetivo: confirmar el buzzer tocando una melodia completa una vez al
   * boot. TIM3 CH2 / PA7 (confirmar que este mapeo siga igual en L433).
   */
//  Buzzer_Init();
//  Log_Print("TEST", "Boot OK - Tarea 3: Buzzer - tocando melodia");
//  Buzzer_PlayMelody(ode_to_joy, MELODY_LEN(ode_to_joy), 120U);
//  Log_Print("TEST", "Tarea 3: Buzzer - fin de melodia");
  /* ======================================================================= */

  /* ===================  TAREA 4: Motovibrador  ============================
   * Objetivo: confirmar el motor ERM encendiendo 5 s al boot. GPIO PH0,
   * modo VIBRATOR_MODE_GPIO (on/off).
   */
//  Vibrator_Init();
//  Log_Print("TEST", "Boot OK - Tarea 4: Motovibrador - encendiendo 5s");
//  Vibrator_On();
//  HAL_Delay(5000U);
//  Vibrator_Off();
//  Log_Print("TEST", "Tarea 4: Motovibrador - apagado");
  /* ======================================================================= */

  /* ===================  TAREA 5: Flash SPI (MX25L6445E)  ==================
   * Objetivo: escribir "MENSAJE EN LA FLASH DE PRUEBA" una vez al boot y
   * despues, cada 3 s en el loop, releerlo y mandarlo por el Logger.
   * SPI2, CS manual PB5 (confirmar mapeo en L433).
   */
//  {
//      const char *flash_test_msg = "MENSAJE EN LA FLASH DE PRUEBA";
//
//      FlashStatus_e st_init  = Flash_Init();
//      FlashStatus_e st_erase = Flash_EraseSector(FLASH_TASK5_TEST_ADDR);
//      FlashStatus_e st_write = Flash_Write(FLASH_TASK5_TEST_ADDR,
//                                            (const uint8_t *)flash_test_msg,
//                                            (uint32_t)(strlen(flash_test_msg) + 1U));
//
//      Log_Printf("FLASH", "Init:%d Erase:%d Write:%d",
//                 (int)st_init, (int)st_erase, (int)st_write);
//  }
  /* ======================================================================= */

  /* ===================  TAREA 6: LoRa (RM1262) - AT por poleo  ============
   * Objetivo: mandar "AT\r\n" cada 3 s y confirmar que el modulo responde
   * "OK". Por POLEO (bloqueante), USART2/huart2 sin interrupcion.
   * NO llamamos Lora_Init() porque esa arma HAL_UART_Receive_IT() y deja el
   * huart en estado Busy_Rx, lo que bloquearia nuestras llamadas
   * bloqueantes de abajo. Solo enlazamos el handle a mano.
   */
//  hlora.huart = LORA_UART;
//  Log_Print("TEST", "Boot OK - Tarea 6: LoRa (AT) - por poleo");
  /* ======================================================================= */

  /* ===================  TAREA 7: GPS (L86-M33) - lectura cruda por poleo ===
   * El GPS no usa comandos AT: en cuanto FORCE_ON esta en HIGH transmite
   * solo tramas NMEA ($GPRMC, $GPGGA, ...) sin que se le pida nada. Aqui
   * solo confirmamos comunicacion leyendo bytes crudos del UART, sin
   * parsear todavia. Por POLEO, USART1/huart1 sin interrupcion.
   * NO llamamos Gps_Init() porque esa arma HAL_UART_Receive_IT() y deja el
   * huart en estado Busy_Rx, lo que bloquearia nuestras llamadas
   * bloqueantes de abajo. Gps_ForceOn() si se puede llamar: es solo un
   * HAL_GPIO_WritePin, no toca el UART.
   */
//  Gps_ForceOn();
//  Log_Print("TEST", "Boot OK - Tarea 7: GPS - lectura cruda por poleo");
  /* ======================================================================= */
```

### `USER CODE BEGIN WHILE` (dentro del `while(1)`)
```c
    /* ===================  TAREA 1: UART de RS485 (Logger)  =================
//    Log_Print("TEST", "Escribiendo por UART4 (RS485)");
//    HAL_Delay(1000U);
     ======================================================================= */

    /* ===================  TAREA 1C: Prueba directa por HAL en los 4 UART  ===
     * Manda una cadena distinta por cada UART, directo por HAL, sin pasar
     * por el Logger — util para repetir en el proyecto nuevo si algun UART
     * sigue sin responder. Ajustar huartX segun como queden mapeados los
     * perifericos reales en el L433 (puede que ya no sean 4 UART, ver
     * pendiente de UART4 abajo).
     */
//    const uint8_t msg1[] = "PRUEBA UART1\r\n";
//    const uint8_t msg2[] = "PRUEBA UART2\r\n";
//    const uint8_t msg3[] = "PRUEBA UART3\r\n";
//    const uint8_t msg4[] = "PRUEBA UART4\r\n";
//
//    HAL_UART_Transmit(&huart1, msg1, sizeof(msg1) - 1U, 100U);
//    HAL_Delay(500U);
//    HAL_UART_Transmit(&huart2, msg2, sizeof(msg2) - 1U, 100U);
//    HAL_Delay(500U);
//    HAL_UART_Transmit(&huart3, msg3, sizeof(msg3) - 1U, 100U);
//    HAL_Delay(500U);
//    HAL_UART_Transmit(&huart4, msg4, sizeof(msg4) - 1U, 100U);
    /* ======================================================================= */

    /* ===================  DIAGNOSTICO: Toggle GPIO puro (PH0, Motovibrador)
     * Confirma que el firmware SI esta corriendo en el micro, sin usar
     * ningun UART. Prende/apaga PH0 cada 4 s. Medir con multimetro: debe
     * alternar 3.3V / 0V. Util para repetir primero en el proyecto nuevo
     * antes que cualquier prueba de UART.
     */
//    HAL_GPIO_TogglePin(Motovibrador_GPIO_Port, Motovibrador_Pin);
//    HAL_Delay(4000U);
    /* ======================================================================= */

    /* ===================  TAREA 2: Escaneo de bus I2C1  ====================
     * EN PAUSA: el escaneo de I2C en la tarjeta Mira dio problema de bus.
     * No se corre hasta validar eso.
     */
//    {
//        uint8_t found      = 0U;
//        uint8_t busy_count = 0U;
//        uint8_t err_count  = 0U;
//
//        for (uint8_t addr = 1U; addr < 127U; addr++) {
//            HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1U, 10U);
//
//            if (st == HAL_OK) {
//                Log_Printf("I2C", "Dispositivo encontrado en 0x%02X", addr);
//                found++;
//            } else if (st == HAL_BUSY) {
//                busy_count++;
//            } else {
//                err_count++;
//            }
//        }
//
//        if (found > 0U) {
//            Log_Printf("I2C", "Total encontrados: %u", found);
//        } else if (busy_count == 126U) {
//            Log_Print("I2C", "Escaneo fallo: bus ocupado/atascado en todas las direcciones");
//        } else {
//            Log_Printf("I2C", "Ningun dispositivo encontrado (busy:%u err:%u de 126)",
//                       busy_count, err_count);
//        }
//
//        HAL_Delay(2000U);
//    }
    /* ======================================================================= */

    /* ===================  TAREA 3: Buzzer (melodia)  =======================
     * Sin loop propio — la melodia se reproducia una vez en USER CODE 2.
//    HAL_Delay(2000U);
     ======================================================================= */

    /* ===================  TAREA 4: Motovibrador  ============================
//    Log_Print("TEST", "Boot OK - Tarea 4: Motovibrador - encendiendo 5s");
//    Vibrator_On();
//    HAL_Delay(4000U);
//    Vibrator_Off();
//    Log_Print("TEST", "Tarea 4: Motovibrador - apagado");
//    HAL_Delay(4000U);
     ======================================================================= */

    /* ===================  TAREA 5: Flash SPI (MX25L6445E)  ==================
//    char read_buf[64] = {0};
//
//    FlashStatus_e st_read = Flash_Read(FLASH_TASK5_TEST_ADDR,
//                                        (uint8_t *)read_buf,
//                                        sizeof(read_buf) - 1U);
//
//    Log_Printf("FLASH", "Read:%d Msg:\"%s\"", (int)st_read, read_buf);
//
//    HAL_Delay(3000U);
     ======================================================================= */

    /* ===================  TAREA 6: LoRa (RM1262) - AT por poleo  ============
//    const uint8_t cmd[] = "AT\r\n";
//    uint8_t       resp[64] = {0};
//
//    HAL_UART_Transmit(hlora.huart, cmd, sizeof(cmd) - 1U, LORA_TX_TIMEOUT_MS);
//
//    HAL_StatusTypeDef st = HAL_UART_Receive(hlora.huart, resp, sizeof(resp) - 1U, LORA_RX_TIMEOUT_MS);
//
//    if (strstr((const char *)resp, "OK") != NULL) {
//        Log_Printf("LORA", "Respuesta OK: \"%s\"", (const char *)resp);
//    } else {
//        Log_Printf("LORA", "Sin OK (st:%d) resp:\"%s\"", (int)st, (const char *)resp);
//    }
//
//    HAL_Delay(3000U);
     ======================================================================= */

    /* ===================  TAREA 7: GPS (L86-M33) - lectura cruda por poleo ===
     * EN PAUSA: usa el mismo huart1 que la Tarea 1B (Logger). No correr
     * ambas a la vez.
//    uint8_t raw[64] = {0};
//
//    HAL_StatusTypeDef st = HAL_UART_Receive(GPS_UART, raw, sizeof(raw) - 1U, 3000U);
//
//    if (st == HAL_OK || raw[0] != 0U) {
//        Log_Printf("GPS", "Crudo recibido: \"%s\"", (const char *)raw);
//    } else {
//        Log_Print("GPS", "Sin datos (timeout) - revisar FORCE_ON/cableado");
//    }
     ======================================================================= */
```

---

## 4. Orden recomendado de trabajo en el proyecto nuevo

1. Terminar de configurar el `.ioc` nuevo (periféricos + pines) contra el
   datasheet real del `STM32L433CCUx` — **confirmar primero si existe
   UART4**; si no existe, decidir a qué periférico se mueve RS485 antes de
   generar código.
2. Copiar los archivos de la sección 1.
3. Generar código desde el `.ioc` nuevo.
4. Pegar el contenido de la sección 3 en el `main.c` nuevo.
5. Repetir primero el **DIAGNOSTICO de toggle GPIO puro** (PH0) antes que
   cualquier prueba de UART — así confirmamos de entrada que el firmware
   corre en el chip correcto antes de retomar las 8 tareas.
6. Actualizar `CLAUDE.md` con el MCU real y la tabla de periféricos una vez
   que el `.ioc` nuevo esté cerrado.
