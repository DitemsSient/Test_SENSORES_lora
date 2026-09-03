/**
 * @file    Test.c
 * @brief   Driver implementation for the per-peripheral bring-up tests.
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Test.h"
#include "main.h"
#include "Logger.h"
#include "Buzzer.h"
#include "Buzzer_Melodias.h"
#include "Motovibrador.h"
#include "Flash.h"
#include "Driver_RGB.h"
#include "Lora.h"
#include "GPS.h"
#include "Receptor_Infrarrojo_EXTI.h"
#include <stdio.h>

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;   /* GPS_UART = &huart1, ver GPS.h */

/* ============================================================================
 * HISTORIAL — pruebas de bring-up ya concluidas, SIN codigo funcional que
 * mover (se fue depurando en main.c a lo largo del proyecto hasta quedar
 * solo la nota). Documentadas aqui para no perder el contexto de que se
 * probo y como. Hardware ya confirmado, no se reconstruyen.
 *
 * - Tarea 1 — UART de RS485 (Logger): este MCU (STM32L433CCUx) no tiene
 *   UART4 — MCU_485_TX/RX (PA0/PA1) quedaron como GPIO_Output simple,
 *   reservados, sin funcionalidad UART. El Logger se reasigno temporalmente
 *   a huart3 (USART3, Bluetooth) y luego a USB CDC (ver Logger.h actual).
 *
 * - Tarea 2 — Escaneo de bus I2C1: reimplementada como Test_I2C_Scan() (SI
 *   tiene codigo funcional, no es solo historial). Direcciones encontradas
 *   la ultima vez: 0x30 (MMC5983MA), 0x32 (LP55231 RGB), 0x39 (SensorLuz
 *   TSL2571), 0x55 (BatteryMonitor BQ27441), 0x6A (LSM6DSO32TR IMU).
 *
 * - Tarea 6 — LoRa (RM1262/KG200Z) AT por poleo: se mandaba "AT\r\n" cada
 *   3 s por huart2 esperando "OK". Superado por la programacion directa
 *   del KG200Z por SWD (ver Test_LoRa_ResetPulse() y Pendientes.md).
 *
 * - Tarea 7 — GPS lectura cruda por poleo (sin parser): confirmo que el
 *   modulo transmite NMEA en huart1 en cuanto FORCE_ON esta en alto, sin
 *   pedir nada. Superado por Test_GPS_Init()/Test_GPS_Poll() (con parser e
 *   interrupcion real).
 * ==========================================================================*/

/* ======================  STATIC VARIABLES  ================================ */

/* Handle de GPS para Test_GPS_Init()/Test_GPS_Poll() — separado del handle
 * que use Inicializacion.c en el firmware final. */
static Gps_Handle_t test_gps;

/* Handle del receptor IR para Test_IR_Init()/Test_IR_Poll(). */
static Ir_Handle_t test_ir;

/* ================================  API  =================================== */

/* Public functions declared in the .h */

void Test_I2C_Scan(void) {
    Log_Print("TEST", "I2C: escaneando bus 0x08-0x77...");

    uint8_t found = 0U;
    for (uint8_t addr7 = 0x08U; addr7 <= 0x77U; addr7++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1U), 2U, 5U) == HAL_OK) {
            Log_Printf("TEST", "I2C: dispositivo en 0x%02X", addr7);
            found++;
        }
    }

    Log_Printf("TEST", "I2C: escaneo terminado, %u dispositivo(s)", found);
}

void Test_Buzzer(void) {
    Buzzer_Init();
    Log_Print("TEST", "Buzzer: tocando melodia (ode_to_joy)");
    Buzzer_PlayMelody(ode_to_joy, MELODY_LEN(ode_to_joy), 120U);
    Log_Print("TEST", "Buzzer: fin de melodia");
}

void Test_Motovibrador(void) {
    Vibrator_Init();
    Log_Print("TEST", "Motovibrador: encendiendo 5 s");
    Vibrator_Test();
    Log_Print("TEST", "Motovibrador: fin");
}

uint8_t Test_Flash(void) {
    if (Flash_Init() != FLASH_OK) {
        Log_Print("TEST", "Flash: fallo Flash_Init (ID no coincide)");
        return 0U;
    }

    uint8_t pass = Flash_Test();
    Log_Printf("TEST", "Flash: self-test %s", pass ? "OK" : "FALLO");
    return pass;
}

void Test_RGB(void) {
    LP55231_t test_rgb;

    LP55231_Attach(&test_rgb, &hi2c1, LP55231_ADDR_7BIT, 100U);
    LP55231_Begin(&test_rgb);
    HAL_Delay(2U);
    LP55231_Enable(&test_rgb);

    Log_Print("TEST", "RGB: ciclo Rojo->Verde->Azul x2");
    DriverRGB_Test();
    Log_Print("TEST", "RGB: fin de ciclo");

    LP55231_Disable(&test_rgb);
}

void Test_LoRa_ResetPulse(void) {
    Log_Print("TEST", "LoRa: pulso de reset por software (LORA_RESET)");
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100U);
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
    Log_Print("TEST", "LoRa: pulso de reset enviado");
}

void Test_GPS_Init(void) {
    Gps_Init(&test_gps);
    HAL_Delay(200U);
    Gps_SendMTK(&test_gps, GPS_OUTPUT_RMC_GGA);
    Gps_SendMTK(&test_gps, GPS_FIX_1HZ);
    Log_Print("TEST", "GPS: configurado - RMC+GGA a 1Hz");
}

void Test_GPS_Poll(void) {
    /* Imprime CADA trama NMEA cruda tal como llega, antes de que
     * Gps_Process() la consuma — util para confirmar a ojo si el modulo
     * aplico bien la configuracion (solo deberian verse $GPRMC/$GPGGA). */
    if (test_gps.sentence_ready) {
        Log_Print("GPS-RAW", test_gps.sentence);
    }
    Gps_Process(&test_gps);

    static uint32_t last_print_tick = 0U;
    uint32_t now = HAL_GetTick();
    if ((now - last_print_tick) >= 2000U) {
        last_print_tick = now;
        if (test_gps.data.position_valid) {
            Log_Printf("GPS", "FIX sats=%u lat=%s alt=%.1fm",
                       test_gps.data.satellites, test_gps.data.position_str,
                       test_gps.data.altitude);
        } else {
            Log_Printf("GPS", "sin fix (sats=%u)", test_gps.data.satellites);
        }
    }
}

void Test_IR_Init(void) {
    IR_Init(&test_ir);
    Log_Print("TEST", "IR: captura EXTI+DWT lista");
}

void Test_IR_Poll(void) {
    if (IR_Process(&test_ir) != IR_OK) return;

    /* Solo tiempos crudos, tal como llegan — sin traducir nada todavia. */
    Log_Printf("IR", "----- trama: %u deltas -----", test_ir.raw_count);
    for (uint16_t i = 0U; i < test_ir.raw_count; i++) {
        Log_Printf("IR", "[%u] %u us", i, test_ir.raw_dt[i]);
    }
    if (test_ir.raw_overflow) {
        Log_Print("IR", "buffer de tiempos lleno — la trama pudo haberse cortado");
    }

    IR_Reset(&test_ir);
}

/* ======================  HAL WEAK CALLBACKS  =============================== */

/* HAL_UART_RxCpltCallback() y HAL_GPIO_EXTI_Callback() se movieron a
 * Inicializacion.c — GPS y el receptor IR ya son parte del arranque real
 * (INIT_GPS_ENABLE / INIT_RX_IR_ENABLE), no solo de pruebas. Un solo weak
 * override por funcion en todo el link, no redefinirlos aqui. */
