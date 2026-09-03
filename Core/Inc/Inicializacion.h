/**
 * @file    Inicializacion.h
 * @brief   Inicializacion secuencial de todos los perifericos/modulos de la
 *          tarjeta Sensores — punto de entrada unico llamado desde main().
 *
 * @details Mismo patron que la libreria hermana de la tarjeta Mira: un
 *          INIT_<PERIFERICO>_ENABLE por cada driver del proyecto. En 0U el
 *          .h de ese driver ni siquiera se incluye y su init ni se compila
 *          (ahorro real de flash/RAM por preprocesador, no un if en
 *          runtime). INIT_BOOTLOADER_ENABLE, INIT_USB_LOGGER_ENABLE e
 *          INIT_RGB_ENABLE son infraestructura base y van siempre en 1U —
 *          el RGB entra en ese grupo (a diferencia del LedRGB de Mira, que
 *          no tenia bandera propia) porque el indicador visual del
 *          bootloader de ESTA tarjeta depende de el (LP55231, no LEDs
 *          discretos).
 *
 *          Uso: main.c solo necesita llamar Inicializacion_Run() una vez,
 *          en USER CODE 2. Si el usuario mantiene presionados los pines
 *          del bootloader, esa llamada NUNCA regresa (salta al bootloader
 *          USB DFU de fabrica).
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef INICIALIZACION_H
#define INICIALIZACION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* ========================  CONFIGURATION  ================================= */

/* Infraestructura base — siempre 1U, no se apagan. */

#define INIT_BOOTLOADER_ENABLE          1U   /**< Salto por software al bootloader USB DFU (PB1+PB2) */
#define INIT_USB_LOGGER_ENABLE          1U   /**< USB CDC (MX_USB_DEVICE_Init) + Logger              */
#define INIT_RGB_ENABLE                 1U   /**< LP55231 — feedback visual del bootloader/arranque  */

/* Resto de drivers del proyecto — arrancan en 0U. Poner en 1U conforme se
 * vayan integrando a la secuencia de arranque real. */

#define INIT_BUZZER_ENABLE              1U   /**< Buzzer.h (+ Buzzer_Melodias.h)      */
#define INIT_MOTOVIBRADOR_ENABLE        1U   /**< Motovibrador.h                      */
#define INIT_FLASH_ENABLE               1U   /**< Flash.h — MX25L6445E SPI NOR        */
#define INIT_LORA_ENABLE                1U   /**< Lora.h                              */
#define INIT_GPS_ENABLE                 1U   /**< GPS.h — L86-M33 / L76-L             */
#define INIT_RX_IR_ENABLE               1U   /**< Receptor_Infrarrojo_EXTI.h (recepcion IR) */
#define INIT_BATTERY_ENABLE             1U   /**< BatteryMonitor.h — BQ27441-G1       */
#define INIT_IMU_ENABLE                 1U   /**< LSM6DSO32TR.h                       */
#define INIT_MAGNETOMETER_ENABLE        1U   /**< MMC5983MA.h                         */
#define INIT_LIGHT_SENSOR_ENABLE        1U   /**< SensorLuz_TSL2571.h                 */
#define INIT_BLUETOOTH_ENABLE           1U   /**< Bluetooth.h — BL654/BL652           */
/* RS485: comentado por completo — pendiente hasta tener el MCU/tarjeta con
 * los pines correctos para probarlo (ver Core/Doc/Pendientes.md). */
// #define INIT_RS485_ENABLE               0U   /**< Comunicacion_RS485.h             */
#define INIT_POWERMANAGER_ENABLE        0U   /**< PowerManager.h — pendiente, se prueba despues */

/* ================================  API  =================================== */

/**
 * @brief  Corre toda la secuencia de inicializacion, en orden. Bootloader
 *         primero (siempre), USB CDC + Logger despues (solo si no se
 *         entro al bootloader), y el resto de drivers habilitados.
 * @note   Llamar UNA vez en main(), dentro de USER CODE 2. No regresa si
 *         el usuario mantiene presionados los pines del bootloader.
 */
void Inicializacion_Run(void);

/**
 * @brief  Imprime el banner inicial por el Logger. Publica para poder
 *         re-imprimirla desde main() si hace falta diagnosticar, pero
 *         Inicializacion_Run() ya la llama una sola vez en el arranque
 *         normal — no repetirla en el while(1).
 */
void Inicializacion_PrintBanner(void);

/**
 * @brief  Escanea el bus I2C1 y compara contra las direcciones esperadas
 *         de esta tarjeta, reportando por el Logger cual si aparecio y
 *         cual no.
 * @note   Bloqueante (~1-2 s). Requiere I2C1 ya inicializado (MX_I2C1_Init,
 *         siempre corre) y el Logger listo si se quiere ver el reporte.
 */
void Search_Disp_I2C(void);

#ifdef __cplusplus
}
#endif

#endif /* INICIALIZACION_H */
