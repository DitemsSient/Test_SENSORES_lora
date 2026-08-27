/**
 * @file    Test.h
 * @brief   Funciones de prueba por periferico, para bring-up y verificacion
 *          manual de hardware — no forman parte del firmware final.
 *
 * @details Cada Test_X() valida un periferico de forma aislada, normalmente
 *          llamando al Driver_Init() + Driver_Test() correspondiente (ver el
 *          driver de cada componente para el detalle de que hace su propio
 *          self-test). Los que necesitan actividad continua (GPS, receptor
 *          IR) se dividen en un _Init() (llamar una vez) y un _Poll()
 *          (llamar en cada vuelta del while(1)).
 *
 *          Uso tipico: en un main() de bring-up, llamar UNA de estas
 *          funciones a la vez (no todas juntas — varias comparten UART/pines
 *          y se van a estorbar). Para las de _Init()/_Poll(), llamar el
 *          _Init() una vez en la inicializacion y el _Poll() en el while(1).
 *
 *          Test_GPS_Poll() y Test_IR_Poll() dependen de callbacks HAL
 *          (HAL_UART_RxCpltCallback, HAL_GPIO_EXTI_Callback) definidos en
 *          Test.c — no declarar esos mismos callbacks en ningun otro
 *          archivo del proyecto mientras esta libreria este en uso.
 *
 *          Pruebas historicas SIN codigo funcional que mover (ya cumplieron
 *          su objetivo — bring-up de hardware, hardware ya confirmado — y
 *          no se reconstruyeron porque no aportan nada nuevo): quedan
 *          documentadas como comentario al inicio de Test.c, no como
 *          funciones aqui.
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* ========================  CONFIGURATION  ================================= */

/* Direccion de prueba para Flash — ultimo sector, igual que Flash_Test() */

#define TEST_FLASH_ADDR          0x7FF000U

/* ================================  API  =================================== */

/* --- Un solo disparo, bloqueante --- */

/**
 * @brief  Prueba el Buzzer: Buzzer_Init() + una melodia corta (ode_to_joy).
 * @note   Bloqueante (~10 s). Confirmacion visual/auditiva por el usuario.
 */
void Test_Buzzer(void);

/**
 * @brief  Prueba el motovibrador: Vibrator_Init() + Vibrator_Test() (5 s a maxima intensidad).
 * @note   Bloqueante. Confirmacion fisica por el usuario.
 */
void Test_Motovibrador(void);

/**
 * @brief  Prueba el Flash SPI (MX25L6445E): Flash_Init() + Flash_Test()
 *         (escribe/lee/compara un patron en el ultimo sector).
 * @retval 1 si paso, 0 si fallo el Init o el self-test.
 */
uint8_t Test_Flash(void);

/**
 * @brief  Prueba el RGB (LP55231): Attach/Begin/Enable propios (no usa el
 *         handle global de Inicializacion) + DriverRGB_Test() (ciclo
 *         Rojo->Verde->Azul x2).
 * @note   Bloqueante (~9 s). Confirmacion visual por el usuario.
 */
void Test_RGB(void);

/**
 * @brief  Genera un pulso de reset real por software en LORA_RESET — la
 *         secuencia confirmada que fuerza al KG200Z a entrar a su propio
 *         bootloader de sistema en el flanco de subida.
 * @note   Requiere LORA_BOOT ya en 1 (pasivo). Ver Core/Doc/Pendientes.md /
 *         notas de programacion del LoRa para el resto de la secuencia
 *         (CubeProgrammer: Under Reset + Hardware reset).
 */
void Test_LoRa_ResetPulse(void);

/* --- Continuas: _Init() una vez, _Poll() en cada vuelta del while(1) --- */

/**
 * @brief  Arranca el GPS (L86-M33 / L76-L) por interrupcion: Gps_Init(),
 *         y manda GPS_OUTPUT_RMC_GGA + GPS_FIX_1HZ.
 * @note   Requiere Log_Init() ya llamado si se quiere ver algo en el Logger.
 */
void Test_GPS_Init(void);

/**
 * @brief  Procesa la ultima trama NMEA recibida e imprime por el Logger:
 *         la trama cruda ([GPS-RAW]) y, cada 2 s, el resumen de fix/sats.
 * @note   Llamar en cada vuelta del while(1) despues de Test_GPS_Init().
 */
void Test_GPS_Poll(void);

/**
 * @brief  Arranca la captura del receptor IR (TSOP, SENSOR_IR2/PB0) por
 *         EXTI + DWT->CYCCNT — ver Receptor_Infrarrojo_EXTI.h.
 */
void Test_IR_Init(void);

/**
 * @brief  Revisa si hay una trama IR completa (silencio > IR_SILENCE_MS) y,
 *         si es asi, la imprime por el Logger como bytes hexadecimales.
 * @note   Llamar en cada vuelta del while(1) despues de Test_IR_Init().
 */
void Test_IR_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_H */
