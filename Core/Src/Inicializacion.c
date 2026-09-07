/**
 * @file    Inicializacion.c
 * @brief   Driver implementation for the sequential system initialization.
 *
 * @date    August 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Inicializacion.h"
#include <stdbool.h>
#include <string.h>

/* ==========================================================================
 * Un bloque #if INIT_XXX_ENABLE por cada driver del proyecto. En 0U el .h
 * ni se incluye — el driver completo queda fuera de la compilacion.
 * ========================================================================*/

#if INIT_BOOTLOADER_ENABLE
#include "Bootloader.h"
#endif

#if INIT_USB_LOGGER_ENABLE
#include "usb_device.h"
#include "Logger.h"
#endif

#if INIT_RGB_ENABLE
#include "Driver_RGB.h"
#endif

#if INIT_BUZZER_ENABLE
#include "Buzzer.h"
#include "Buzzer_Melodias.h"
#endif

#if INIT_MOTOVIBRADOR_ENABLE
#include "Motovibrador.h"
#endif

#if INIT_FLASH_ENABLE
#include "Flash.h"
#endif

#if INIT_LORA_ENABLE
#include "Lora.h"
#endif

#if INIT_GPS_ENABLE
#include "GPS.h"
#endif

#if INIT_RX_IR_ENABLE
#include "Receptor_Infrarrojo_EXTI.h"
#endif

#if INIT_BATTERY_ENABLE
#include "BatteryMonitor.h"
#endif

#if INIT_IMU_ENABLE
#include "LSM6DSO32TR.h"
#endif

#if INIT_MAGNETOMETER_ENABLE
#include "MMC5983MA.h"
#endif

#if INIT_LIGHT_SENSOR_ENABLE
#include "SensorLuz_TSL2571.h"
#endif

#if INIT_BLUETOOTH_ENABLE
#include "Bluetooth.h"
#endif

/* ModoProgramacion.h se elimino del proyecto — la deteccion de modo
 * (Logger/LoRa/Bluetooth) ya la resuelven los switches fisicos PB1/PB2,
 * ver el bloque de Bootloader mas abajo. No aplica aqui. */

#if INIT_RS485_ENABLE
#include "Comunicacion_RS485.h"
/* TODO: agregar RS485_Init(&hrs485, huart) cuando se integre. */
#endif

#if INIT_POWERMANAGER_ENABLE
#include "PowerManager.h"
/* TODO: agregar PowerManager_Init(...) cuando se integre (requiere los
 * handles de GPS/IMU/etc. ya inicializados). */
#endif

/* ======================  CONFIGURATION  ==================================== */

/* Margen fijo tras MX_USB_DEVICE_Init() antes de mandar cualquier mensaje —
 * el host tarda en enumerar el dispositivo. */
#define INIT_USB_ENUM_DELAY_MS       5000U

/* Pausa entre un paso de inicializacion y el siguiente, para poder leer el
 * log con calma. */
#define INIT_STEP_DELAY_MS           500U

/* Cuanto esperamos tramas NMEA del GPS antes de darlo por sin respuesta. */
#define INIT_GPS_WAIT_MS             4000U

#if INIT_RGB_ENABLE
/* Par1 del mapeo fisico confirmado de esta tarjeta: D1=verde, D2=rojo, D7=azul. */
#define INIT_LED_CH_GREEN            0U   /* D1 — modo Logger            */
#define INIT_LED_CH_RED              1U   /* D2 — mezclado con azul = Bluetooth */
#define INIT_LED_CH_BLUE             6U   /* D7 — modo LoRa               */
#define INIT_LED_BLINK_MS            400U
#define INIT_LED_BLINK_COUNT         3U

/* Prueba de arcoiris: los 3 pares fisicos (D1/D2/D7, D3/D4/D8, D5/D6/D9) a
 * la vez — mismo rol por posicion (verde/rojo/azul) en los tres. NO
 * estaticos — Tareas.c los referencia con "extern" para el parpadeo
 * magenta de CalibrateTask (mismo patron que "rgb" un poco mas abajo). */
#define INIT_RAINBOW_STEP_MS         600U
const uint8_t rgb_green_ch[3] = { 0U, 2U, 4U };  /* D1, D3, D5 */
const uint8_t rgb_red_ch[3]   = { 1U, 3U, 5U };  /* D2, D4, D6 */
const uint8_t rgb_blue_ch[3]  = { 6U, 7U, 8U };  /* D7, D8, D9 */

typedef struct {
    uint8_t     r, g, b;
    const char *name;
} InitRgbColor_t;

static const InitRgbColor_t s_rainbow[] = {
    { 1U, 0U, 0U, "Rojo"     },
    { 1U, 1U, 0U, "Amarillo" },
    { 0U, 1U, 0U, "Verde"    },
    { 0U, 1U, 1U, "Cyan"     },
    { 0U, 0U, 1U, "Azul"     },
    { 1U, 0U, 1U, "Magenta"  },
};
#define INIT_RAINBOW_COLOR_COUNT   (sizeof(s_rainbow) / sizeof(s_rainbow[0]))
#endif

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern I2C_HandleTypeDef hi2c1;   /* I2C1 — siempre inicializado por MX_I2C1_Init() */

#if INIT_BLUETOOTH_ENABLE
extern UART_HandleTypeDef huart3;   /* BT_UART = &huart3, ver Bluetooth.h */
/* Handle real de BluetoothTask (Tareas.c, no estatico ahi) — el callback
 * de abajo despacha aqui los bytes de huart3 mientras esa tarea vive. */
extern Bt_Handle_t s_bt_task;
#endif

#if INIT_GPS_ENABLE
extern UART_HandleTypeDef huart1;   /* GPS_UART = &huart1, ver GPS.h */
#endif

#if INIT_LORA_ENABLE
extern UART_HandleTypeDef huart2;   /* LORA_UART = &huart2, ver Lora.h */
#endif

/* ======================  STATIC VARIABLES  ================================ */

#if INIT_RGB_ENABLE
/* Handle del LP55231. NO estatico — Bootloader.c lo referencia con
 * "extern LP55231_t rgb;". */
LP55231_t rgb;
#endif

#if INIT_IMU_ENABLE
static LSM6DSO32TR_t s_imu;
#endif

#if INIT_LIGHT_SENSOR_ENABLE
static TSL2571_t s_light;
#endif

#if INIT_BLUETOOTH_ENABLE
static Bt_Handle_t s_bt;
#endif

#if INIT_RX_IR_ENABLE
/* Handle de IR — NO estatico — CalibrateTask (Tareas.c) lo consume via
 * "extern Ir_Handle_t ir_handle;". IR_Init() ya corre aqui, antes del
 * RTOS, asi que para cuando CalibrateTask arranca el handle ya esta listo
 * — a diferencia de GPS/Sensores no hace falta reinicializarlo en la tarea. */
Ir_Handle_t ir_handle;
#endif

#if INIT_GPS_ENABLE
static Gps_Handle_t s_gps;
#endif

#if INIT_LORA_ENABLE
static Lora_Handle_t s_lora;
#endif

/* Direcciones I2C esperadas en esta tarjeta (7 bits). */
typedef struct {
    uint8_t     addr;
    const char *name;
} I2cDevice_t;

static const I2cDevice_t s_i2c_expected[] = {
    { 0x30U, "MMC5983MA (magnetometro)" },
    { 0x32U, "LP55231 (RGB)"            },
    { 0x39U, "TSL2571 (luz)"            },
    { 0x55U, "BQ27441 (bateria)"        },
    { 0x6AU, "LSM6DSO32TR (IMU)"        },
};
#define I2C_EXPECTED_COUNT   (sizeof(s_i2c_expected) / sizeof(s_i2c_expected[0]))
#define I2C_FOUND_MAX         16U

/* Diagnostico, modo de operacion y datos de partida — declarados extern en
 * Inicializacion.h, definidos aqui (no estaticos: SensorsTask en Tareas.c
 * escribe en g_exercise_data, y BluetoothTask escribira en
 * g_modo_operacion mas adelante). */
Diagnostico_t Diagnostico = {0};

/* Nace siempre en MODO_CONFIGURACION — ver Modo_SetOperacion(). */
ModoOperacion_e g_modo_operacion = MODO_CONFIGURACION;

/* [PRUEBA] Simula haber recibido ya "$CONF1,3,EQUIPO7,PETRAA,10,100,60,
 * 01F51DDE41B911\r" por LoRa — LoraTask todavia no existe, asi que
 * precargamos la estructura tal cual quedaria despues de parsear ese
 * mensaje, para poder probar el intercambio con Bluetooth de forma
 * aislada. Quitar/reemplazar cuando exista LoraTask real. */
ExerciseGameData_t g_exercise_data = {
    .orden       = 1U,
    .lora        = 3U,
    .team_name   = "EQUIPO7",
    .player_name = "PETRAA",
    .lives       = 10U,
    .ammo        = 100U,
    .tiempo      = 60U,
    .mac         = "01F51DDE41B911",
    .lvBatery    = 100U,
};

/* Prioridades NVIC — FIJAS, no se intercambian en runtime (decision
 * 2026-09-04, ver Modo_SetOperacion()):
 *   EXTI0_IRQn  (recepcion IR de disparos) = 0  — fuera del rango que le
 *     importa a FreeRTOS (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5,
 *     ver FreeRTOSConfig.h). Valido SOLO porque IR_EXTI_Callback no toca
 *     nada de la API de FreeRTOS (ni Log_Print, ni mutex/colas) — si algun
 *     dia esa ISR necesita tocar RTOS, esto deja de ser seguro.
 *   USART2_IRQn (recepcion LoRa) = 5 — la prioridad mas alta DENTRO del
 *     rango seguro para RTOS. Se queda ahi siempre: la IR en 0 la
 *     preemptea de todos modos cuando hace falta, pero su ISR es de
 *     microsegundos (EXTI+DWT sin bloqueos) — el costo de esa preemption
 *     es insignificante contra el tiempo de un byte UART, no hay riesgo
 *     real de overrun. No hace falta bajarla en MODO_EJERCICIO.
 *   Todo lo demas (Bluetooth USART3, RS485 UART4, etc.) = 6.
 * Configuradas fijas en el NVIC (main.c / stm32l4xx_hal_msp.c via CubeMX,
 * EXTI0 en 0 hay que ponerlo tambien en el panel NVIC del .ioc para que
 * sobreviva la regeneracion). */

/* ================================  API  =================================== */

/* Public functions declared in the .h */

void Inicializacion_Run(void) {
#if INIT_RGB_ENABLE
    /* El RGB debe quedar listo ANTES de la decision de bootloader — es lo
     * que da el feedback visual (rojo/verde), tanto si se va a saltar
     * como si sigue el arranque normal. */
    LP55231_Attach(&rgb, &hi2c1, LP55231_ADDR_7BIT, 100U);
    LP55231_Begin(&rgb);
    HAL_Delay(2U);
    LP55231_Enable(&rgb);
#endif

#if INIT_BOOTLOADER_ENABLE
    /* Bootloader SIEMPRE primero. Si PB1 Y PB2 estan en bajo, esta llamada
     * NUNCA regresa: parpadea rojo y salta al bootloader USB DFU (logica
     * en Bootloader.c). El USB no se ha tocado todavia en ningun punto de
     * esta funcion — se queda completamente libre hasta despues de esta
     * decision, para no dejarlo a medio inicializar si se salta. */
    Bootloader_CheckAndEnter();

    /* No se entro al bootloader: PB1/PB2 indican el modo del mux de
     * programacion (SEL_PROG_LORA_BLU / SEL_PROG_MCU_FTDI) —
     *   PB2 bajo           -> Logger  (USB-C directo al MCU)
     *   PB2 alto, PB1 alto -> LoRa    (FTDI -> modulo LoRa)
     *   PB2 alto, PB1 bajo -> Bluetooth (FTDI -> modulo BLE)
     */
    bool pb2_low = (HAL_GPIO_ReadPin(BOOTLOADER_BTN_PORT, BOOTLOADER_BTN_PIN)   == GPIO_PIN_RESET);
    bool pb1_low = (HAL_GPIO_ReadPin(BOOTLOADER_BTN2_PORT, BOOTLOADER_BTN2_PIN) == GPIO_PIN_RESET);

    const char *mode_name;
    if (pb2_low) {
        mode_name = "LOGGER (USB-C al MCU)";
    } else if (!pb1_low) {
        mode_name = "LORA (FTDI -> modulo LoRa)";
    } else {
        mode_name = "BLUETOOTH (FTDI -> modulo BLE)";
    }

#if INIT_RGB_ENABLE
    for (uint8_t i = 0U; i < INIT_LED_BLINK_COUNT; i++) {
        if (pb2_low) {
            LP55231_SetChannelPWM(&rgb, INIT_LED_CH_GREEN, 0xFFU);
        } else if (!pb1_low) {
            LP55231_SetChannelPWM(&rgb, INIT_LED_CH_BLUE, 0xFFU);
        } else {
            LP55231_SetChannelPWM(&rgb, INIT_LED_CH_RED, 0xFFU);
            LP55231_SetChannelPWM(&rgb, INIT_LED_CH_BLUE, 0xFFU);
        }
        HAL_Delay(INIT_LED_BLINK_MS);
        LP55231_SetChannelPWM(&rgb, INIT_LED_CH_GREEN, 0x00U);
        LP55231_SetChannelPWM(&rgb, INIT_LED_CH_RED, 0x00U);
        LP55231_SetChannelPWM(&rgb, INIT_LED_CH_BLUE, 0x00U);
        HAL_Delay(INIT_LED_BLINK_MS);
    }
#endif
#endif

#if INIT_USB_LOGGER_ENABLE
    /* Recien AQUI se toca el USB — ya se sabe que no se salto al
     * bootloader. MX_USB_DEVICE_Init() se saco de su posicion generada
     * por CubeMX (ver USER CODE 2 en main.c) para que corra despues de la
     * decision de arriba, no antes. */
    MX_USB_DEVICE_Init();
    HAL_Delay(INIT_USB_ENUM_DELAY_MS);

    Log_Init();
    Log_Blank();   /* separa el arranque de lo que haya quedado en la terminal */
    Inicializacion_PrintBanner();
#if INIT_BOOTLOADER_ENABLE
    Log_Printf("SIENT", "Modo detectado: %s", mode_name);
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif
#endif

#if INIT_USB_LOGGER_ENABLE
    Log_Blank();
    Diagnostico.i2c_completo = Search_Disp_I2C();
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_MOTOVIBRADOR_ENABLE
    Log_Blank();
    Log_Print("MOTOVIBRADOR", "Inicializando...");
    Vibrator_Init();
    Diagnostico.motovibrador = true;   /* Vibrator_Init() no regresa status */
    Log_Print("MOTOVIBRADOR", "Motovibrador inicializado correctamente.");
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_BUZZER_ENABLE
    Log_Blank();
    Log_Print("BUZZER", "Inicializando...");
    Buzzer_Init();
    Diagnostico.buzzer = true;   /* Buzzer_Init() no regresa status */
    Log_Print("BUZZER", "Buzzer inicializado correctamente.");
    Log_Print("BUZZER", "Sonando tono de alarma (2x)...");
    Buzzer_PlayMelody(alert, MELODY_LEN(alert), 160U);
    HAL_Delay(500U);
    Buzzer_PlayMelody(alert, MELODY_LEN(alert), 160U);
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_FLASH_ENABLE
    Log_Blank();
    Log_Print("FLASH", "Inicializando Flash SPI...");

    /* Migas de pan: leemos el JEDEC ID crudo ANTES de comparar, para ver
     * exactamente que esta contestando el chip (o si no contesta nada). */
    FlashID_t flash_id = {0U, 0U};
    FlashStatus_e flash_id_st = Flash_ReadID(&flash_id);

    if (flash_id_st != FLASH_OK) {
        Log_Print("FLASH", "ERROR: Flash_ReadID fallo (problema de SPI, no de ID) — revisar SPI2/CS PB5.");
    } else {
        Log_Printf("FLASH", "JEDEC ID leido: manufacturer=0x%02X device_id=0x%04X",
                   flash_id.manufacturer, flash_id.device_id);
        Log_Printf("FLASH", "JEDEC ID esperado: manufacturer=0x%02X device_id=0x%04X",
                   FLASH_MANUFACTURER_ID, FLASH_DEVICE_ID);

        if (flash_id.manufacturer == 0x00U && flash_id.device_id == 0x0000U) {
            Log_Print("FLASH", "ID en 0x00 0x0000 -> el chip no esta respondiendo (MISO siempre bajo: revisar alimentacion, CS o soldadura).");
        } else if (flash_id.manufacturer == 0xFFU && flash_id.device_id == 0xFFFFU) {
            Log_Print("FLASH", "ID en 0xFF 0xFFFF -> bus flotante (MISO sin pull, o chip no responde: revisar MISO/MOSI/SCK).");
        }
    }

    if (Flash_Init() == FLASH_OK) {
        Diagnostico.flash = true;
        Log_Print("FLASH", "Flash inicializado correctamente.");
    } else {
        Log_Print("FLASH", "ERROR: fallo la inicializacion del Flash.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_MAGNETOMETER_ENABLE
    Log_Blank();
    Log_Print("MAG", "Inicializando magnetometro MMC5983MA...");
    if (MMC5983MA_Init() == MMC_OK) {
        Diagnostico.magnetometro = true;
        Log_Print("MAG", "Magnetometro inicializado correctamente.");

        /* Primera lectura, para confirmar a ojo que los datos tienen sentido. */
        MMC_Data_t mag_data;
        if (MMC5983MA_ReadAll(&mag_data) == MMC_OK) {
            Log_Printf("MAG", "X=%.1fuT Y=%.1fuT Z=%.1fuT",
                       mag_data.x_uT, mag_data.y_uT, mag_data.z_uT);
        }
    } else {
        Log_Print("MAG", "ERROR: fallo la inicializacion del magnetometro.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_IMU_ENABLE
    Log_Blank();
    Log_Print("IMU", "Inicializando IMU LSM6DSO32TR...");
    if (LSM6DSO32TR_Init(&s_imu) == LSM_OK) {
        Diagnostico.imu = true;
        Log_Print("IMU", "IMU inicializado correctamente.");

        LSM_Data_t imu_data;
        if (LSM6DSO32TR_ReadAll(&s_imu, &imu_data) == LSM_OK) {
            Log_Printf("IMU", "accel(g)=%.2f,%.2f,%.2f gyro(dps)=%.2f,%.2f,%.2f",
                       imu_data.ax_g, imu_data.ay_g, imu_data.az_g,
                       imu_data.gx_dps, imu_data.gy_dps, imu_data.gz_dps);
        }
    } else {
        Log_Print("IMU", "ERROR: fallo la inicializacion del IMU.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_BATTERY_ENABLE
    Log_Blank();
    Log_Print("BAT", "Inicializando BatteryMonitor BQ27441...");
    if (BatGauge_Init() == HAL_OK) {
        Diagnostico.batterymonitor = true;
        Log_Print("BAT", "BatteryMonitor inicializado correctamente.");

        BatGauge_Data_t bat_data;
        BatGauge_Update(&bat_data);
        if (bat_data.is_ready) {
            Log_Printf("BAT", "V=%umV I=%dmA SOC=%u%%",
                       bat_data.voltage_mV, bat_data.avg_current_mA, bat_data.soc_pct);
        }
    } else {
        Log_Print("BAT", "ERROR: fallo la inicializacion del BatteryMonitor.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_LIGHT_SENSOR_ENABLE
    Log_Blank();
    Log_Print("LUZ", "Inicializando sensor de luz TSL2571...");
    TSL2571_Attach(&s_light, &hi2c1, TSL2571_ADDR_7BIT, 100U);
    if (TSL2571_Begin(&s_light, 0xC0U, TSL2571_GAIN_1X) == HAL_OK) {
        Diagnostico.sensorluz = true;
        Log_Print("LUZ", "Sensor de luz inicializado correctamente.");

        float lux = 0.0f;
        TSL2571_RawData_t light_data;
        if (TSL2571_ReadLux(&s_light, 1U, 200U, &lux, &light_data) == HAL_OK) {
            Log_Printf("LUZ", "CH0=%u CH1=%u Lux=%.1f",
                       light_data.ch0, light_data.ch1, lux);
        }
    } else {
        Log_Print("LUZ", "ERROR: fallo la inicializacion del sensor de luz.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_BLUETOOTH_ENABLE
    Log_Blank();
    Log_Print("BT", "Iniciando modulo Bluetooth BL654...");
    /* Enlace manual (sin Bt_Init()) — Bt_Init() arma HAL_UART_Receive_IT(),
     * lo que deja el huart en Busy_Rx y bloquearia las llamadas bloqueantes
     * de abajo (mismo motivo que GPS/LoRa en las pruebas de bring-up). */
    s_bt.huart = BT_UART;

    {
        uint8_t bt_resp[64];

        /* "00" en la respuesta = exito (protocolo AT Interface de Laird/Ezurio,
         * ver Core/Doc — "01\t<codigo>" seria error). Solo consultamos el
         * firmware si el modulo confirmo que esta sano. */
        Log_Print("BT", "Probando comunicacion AT...");
        static const uint8_t cmd_at[] = "AT\r";
        Bt_Transmit(&s_bt, cmd_at, sizeof(cmd_at) - 1U);
        memset(bt_resp, 0, sizeof(bt_resp));
        HAL_UART_Receive(s_bt.huart, bt_resp, sizeof(bt_resp) - 1U, BT_RX_TIMEOUT_MS);

        bool bt_ok = (strstr((char *)bt_resp, "00") != NULL);
        Diagnostico.bluetooth = bt_ok;
        if (bt_ok) {
            Log_Print("BT", "BLE responde OK (00) — modulo sano.");
        } else {
            Log_Print("BT", "ERROR: BLE no respondio 00 — revisar UART3/modulo.");
        }

        if (bt_ok) {
            Log_Print("BT", "Consultando firmware (AT I 3)...");
            static const uint8_t cmd_fw[] = "AT I 3\r";
            Bt_Transmit(&s_bt, cmd_fw, sizeof(cmd_fw) - 1U);
            memset(bt_resp, 0, sizeof(bt_resp));
            HAL_UART_Receive(s_bt.huart, bt_resp, sizeof(bt_resp) - 1U, BT_RX_TIMEOUT_MS);
            Log_Printf("BT", "Firmware: %s", (char *)bt_resp);

            /* Arranca nuestro programa/script en el modulo — solo si el BT
             * salio sano arriba. Antes de crear las tareas, para que
             * BluetoothTask ya se encuentre el modulo corriendo. */
            Log_Print("BT", "Arrancando programa SensoresM (AT+RUN)...");
            static const uint8_t cmd_run[] = "AT+RUN \"SensoresM\"\r";
            Bt_Transmit(&s_bt, cmd_run, sizeof(cmd_run) - 1U);
            memset(bt_resp, 0, sizeof(bt_resp));
            HAL_UART_Receive(s_bt.huart, bt_resp, sizeof(bt_resp) - 1U, BT_RX_TIMEOUT_MS);
            if (strstr((char *)bt_resp, "00") != NULL) {
                Log_Print("BT", "SensoresM arrancado (00).");
            } else {
                Log_Printf("BT", "ERROR: AT+RUN no respondio 00: %s", (char *)bt_resp);
            }
        }
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_LORA_ENABLE
    Log_Blank();
    Lora_Init(&s_lora);   /* ya imprime "Iniciando modulo LoRa..." internamente */

    Log_Print("LORA", "Mandando ATQ...");
    Lora_ResetRx(&s_lora);
    static const uint8_t lora_cmd_atq[] = "ATQ\r\n";
    Lora_Transmit(&s_lora, lora_cmd_atq, sizeof(lora_cmd_atq) - 1U);
    HAL_Delay(600U);

    if ((s_lora.rx_count > 0U) && (strstr((char *)s_lora.rx_buffer, "OK") != NULL)) {
        Diagnostico.lora = true;
        Log_Print("LORA", "Comunicacion exitosa, se recibio OK.");
    } else {
        Log_Print("LORA", "ERROR: no se recibio OK — revisar USART2/modulo.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_RX_IR_ENABLE
    Log_Blank();
    Log_Print("RX_IR", "Inicializando receptor IR (EXTI+DWT)...");
    IR_Init(&ir_handle);
    Diagnostico.rx_ir = true;   /* Solo init — sin transmisor externo no hay forma de autoverificar */
    Log_Print("RX_IR", "Receptor IR listo — no se hace poll aqui, requiere un transmisor externo disparando.");
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_RGB_ENABLE
    Log_Blank();
    Log_Print("RGB", "Probando arcoiris...");
    for (uint8_t i = 0U; i < INIT_RAINBOW_COLOR_COUNT; i++) {
        const InitRgbColor_t *c = &s_rainbow[i];
        for (uint8_t j = 0U; j < 3U; j++) {
            LP55231_SetChannelPWM(&rgb, rgb_red_ch[j],   c->r ? 0xFFU : 0x00U);
            LP55231_SetChannelPWM(&rgb, rgb_green_ch[j], c->g ? 0xFFU : 0x00U);
            LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j],  c->b ? 0xFFU : 0x00U);
        }
        HAL_Delay(INIT_RAINBOW_STEP_MS);
    }
    for (uint8_t ch = 0U; ch < LP55231_NUM_CHANNELS; ch++) {
        LP55231_SetChannelPWM(&rgb, ch, 0x00U);
    }
    Diagnostico.rgb = true;
    Log_Print("RGB", "Prueba de arcoiris terminada.");
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

#if INIT_GPS_ENABLE
    Log_Blank();
    Log_Print("GPS", "Inicializando GPS (L86-M33/L76-L)...");
    Gps_Init(&s_gps);
    HAL_Delay(200U);
    Gps_SendMTK(&s_gps, GPS_OUTPUT_RMC_GGA);
    Gps_SendMTK(&s_gps, GPS_FIX_0_5HZ);
    Log_Print("GPS", "Configurado (RMC+GGA a 0.5Hz, cada 2s). Esperando tramas...");

    /* No exigimos fix (dificil en interiores) — solo confirmamos que estan
     * llegando bytes reales del modulo, imprimiendo un par de tramas NMEA
     * crudas tal como llegan. */
    uint32_t gps_wait_start    = HAL_GetTick();
    uint8_t  gps_lines_printed = 0U;
    while (((HAL_GetTick() - gps_wait_start) < INIT_GPS_WAIT_MS) && (gps_lines_printed < 2U)) {
        if (s_gps.sentence_ready) {
            Log_Printf("GPS-RAW", "%s", s_gps.sentence);
            gps_lines_printed++;
        }
        Gps_Process(&s_gps);
    }

    if (gps_lines_printed == 0U) {
        Log_Print("GPS", "ERROR: no se recibio ninguna trama NMEA — revisar USART1/modulo.");
    } else {
        Diagnostico.gps = true;
        Log_Print("GPS", "Comunicacion confirmada.");
    }
    HAL_Delay(INIT_STEP_DELAY_MS);
#endif

    /* Mas pasos de inicializacion secuencial se agregan aqui conforme se
     * vayan habilitando mas drivers (ver los INIT_*_ENABLE arriba). */

    /* Arranca siempre en MODO_CONFIGURACION. Las prioridades del NVIC
     * (IR=0, LoRa=5, resto=6) son FIJAS y ya se configuran en el .ioc/NVIC
     * de CubeMX — Modo_SetOperacion() ya no las toca, ver su comentario. */
    Modo_SetOperacion(MODO_CONFIGURACION);

#if INIT_USB_LOGGER_ENABLE
    Log_Blank();
    Diagnostico_Print();
#endif

#if INIT_RGB_ENABLE
    /* Cierre del init: parpadeo en verde de los 3 pares fisicos a la vez
     * (D1/D3/D5), a diferencia del parpadeo de bootloader que solo usa un
     * par (D1). */
    for (uint8_t i = 0U; i < INIT_LED_BLINK_COUNT; i++) {
        for (uint8_t j = 0U; j < 3U; j++) {
            LP55231_SetChannelPWM(&rgb, rgb_green_ch[j], 0xFFU);
        }
        HAL_Delay(INIT_LED_BLINK_MS);
        for (uint8_t j = 0U; j < 3U; j++) {
            LP55231_SetChannelPWM(&rgb, rgb_green_ch[j], 0x00U);
        }
        HAL_Delay(INIT_LED_BLINK_MS);
    }
#endif
}

void Inicializacion_PrintBanner(void) {
#if INIT_USB_LOGGER_ENABLE
    Log_Print("SIENT", "========================================");
    Log_Print("SIENT", "  Proyecto SIENT_SENSORES v1");
    Log_Print("SIENT", "========================================");
    /* El modo (Logger/LoRa/Bluetooth) lo imprime Inicializacion_Run() justo
     * despues de esta llamada — ver deteccion de PB1/PB2 ahi. */
#endif
}

bool Search_Disp_I2C(void) {
    uint8_t found[I2C_FOUND_MAX];
    uint8_t found_count = 0U;
    uint8_t matched_count = 0U;

#if INIT_USB_LOGGER_ENABLE
    Log_Print("I2C", "Escaneando bus I2C1...");
#endif

    /* 1. Escanea el bus y guarda lo que responda. */
    for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1U), 2U, 5U) == HAL_OK) {
            if (found_count < I2C_FOUND_MAX) {
                found[found_count++] = addr;
            }
#if INIT_USB_LOGGER_ENABLE
            Log_Printf("I2C", "0x%02X responde", addr);
#endif
        }
    }

#if INIT_USB_LOGGER_ENABLE
    Log_Print("I2C", "Comparando contra dispositivos esperados...");
#endif

    /* 2. Compara cada direccion esperada contra lo encontrado y reporta. */
    for (uint8_t i = 0U; i < I2C_EXPECTED_COUNT; i++) {
        uint8_t expected_addr = s_i2c_expected[i].addr;
        bool    match         = false;

        for (uint8_t j = 0U; j < found_count; j++) {
            if (found[j] == expected_addr) {
                match = true;
                break;
            }
        }

        if (match) {
            matched_count++;
        }

#if INIT_USB_LOGGER_ENABLE
        if (match) {
            Log_Printf("I2C", "0x%02X %s -> encontrado", expected_addr, s_i2c_expected[i].name);
        } else {
            Log_Printf("I2C", "0x%02X %s -> NO encontrado", expected_addr, s_i2c_expected[i].name);
        }
#endif
    }

    return (matched_count == I2C_EXPECTED_COUNT);
}

void Diagnostico_Print(void) {
#if INIT_USB_LOGGER_ENABLE
    Log_Print("DIAG", "---- Diagnostico de inicializacion ----");
    Log_Printf("DIAG", "I2C completo:   %s", Diagnostico.i2c_completo   ? "OK" : "FALLO");
    Log_Printf("DIAG", "Motovibrador:   %s", Diagnostico.motovibrador   ? "OK" : "FALLO");
    Log_Printf("DIAG", "Buzzer:         %s", Diagnostico.buzzer         ? "OK" : "FALLO");
    Log_Printf("DIAG", "Flash:          %s", Diagnostico.flash          ? "OK" : "FALLO");
    Log_Printf("DIAG", "Magnetometro:   %s", Diagnostico.magnetometro   ? "OK" : "FALLO");
    Log_Printf("DIAG", "IMU:            %s", Diagnostico.imu            ? "OK" : "FALLO");
    Log_Printf("DIAG", "BatteryMonitor: %s", Diagnostico.batterymonitor ? "OK" : "FALLO");
    Log_Printf("DIAG", "Sensor de luz:  %s", Diagnostico.sensorluz      ? "OK" : "FALLO");
    Log_Printf("DIAG", "Bluetooth:      %s", Diagnostico.bluetooth      ? "OK" : "FALLO");
    Log_Printf("DIAG", "LoRa:           %s", Diagnostico.lora           ? "OK" : "FALLO");
    Log_Printf("DIAG", "RX_IR:          %s", Diagnostico.rx_ir          ? "OK" : "FALLO");
    Log_Printf("DIAG", "RGB:            %s", Diagnostico.rgb            ? "OK" : "FALLO");
    Log_Printf("DIAG", "GPS:            %s", Diagnostico.gps            ? "OK" : "FALLO");
    Log_Print("DIAG", "----------------------------------------");
#endif
}

void Inicializacion_PrintExerciseData(void) {
#if INIT_USB_LOGGER_ENABLE
    Log_Print("EXERCISE", "---- Datos de ejercicio ----");
    Log_Printf("EXERCISE", "Orden: %u", g_exercise_data.orden);
    Log_Printf("EXERCISE", "Lora: %u", g_exercise_data.lora);
    Log_Printf("EXERCISE", "Equipo: %s", g_exercise_data.team_name);
    Log_Printf("EXERCISE", "Alias: %s", g_exercise_data.player_name);
    Log_Printf("EXERCISE", "Vidas: %u", g_exercise_data.lives);
    Log_Printf("EXERCISE", "Balas: %u", g_exercise_data.ammo);
    Log_Printf("EXERCISE", "Tiempo: %lu", (unsigned long)g_exercise_data.tiempo);
    Log_Printf("EXERCISE", "MAC1: %s", g_exercise_data.mac);
    Log_Printf("EXERCISE", "MAC2: %s", g_exercise_data.mac2);
    Log_Print("EXERCISE", "-----------------------------");
#endif
}

void Inicializacion_PrintSensorsData(void) {
#if INIT_USB_LOGGER_ENABLE
    Log_Print("SENSORS", "---- Lecturas de sensores ----");
    Log_Printf("SENSORS", "Bateria: %u%%", g_exercise_data.lvBatery);
    Log_Printf("SENSORS", "Lux: %.1f", g_exercise_data.lux);
    Log_Printf("SENSORS", "Mag(uT): %.1f,%.1f,%.1f",
               g_exercise_data.mag_x_uT, g_exercise_data.mag_y_uT, g_exercise_data.mag_z_uT);
    Log_Printf("SENSORS", "Gyro(dps): %.1f,%.1f,%.1f",
               g_exercise_data.gyro_x_dps, g_exercise_data.gyro_y_dps, g_exercise_data.gyro_z_dps);
    Log_Print("SENSORS", "-------------------------------");
#endif
}

void Modo_SetOperacion(ModoOperacion_e modo) {
    g_modo_operacion = modo;

    /* Las prioridades del NVIC son fijas (IR=0, LoRa=5, resto=6 — fijadas
     * en el .ioc/NVIC de CubeMX, ver comentario mas arriba en este
     * archivo) — esta funcion ya NO las toca. Solo lleva el estado del
     * modo para la logica de negocio (que mensajes procesar, si el
     * disparo IR cuenta para el marcador, etc.). */
#if INIT_USB_LOGGER_ENABLE
    Log_Printf("MODO", "Modo de operacion: %s",
               (modo == MODO_EJERCICIO) ? "EJERCICIO" : "CONFIGURACION");
#endif
}

/* ======================  HAL WEAK CALLBACKS  =============================== */

/**
 * @brief  HAL weak callback — recepcion completa por UART. Solo hay UNA
 *         definicion de esta funcion en todo el link, asi que despacha por
 *         instancia a cada UART que este armado por interrupcion (huart1 =
 *         GPS, huart2 = LoRa).
 * @note   Trabajo minimo: Gps_StoreByte()/Lora_StoreByte() ya rearman el
 *         siguiente byte solos.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#if INIT_GPS_ENABLE
    if (huart->Instance == GPS_UART->Instance) {
        Gps_StoreByte(&s_gps);
        return;
    }
#endif
#if INIT_LORA_ENABLE
    if (huart->Instance == LORA_UART->Instance) {
        Lora_StoreByte(&s_lora);
        return;
    }
#endif
#if INIT_BLUETOOTH_ENABLE
    if (huart->Instance == BT_UART->Instance) {
        /* s_bt_task (Tareas.c) — handle real de BluetoothTask, arma su
         * propia recepcion IT con Bt_Init(). El "s_bt" de aqui arriba
         * (smoke test de bring-up) usa HAL_UART_Receive() bloqueante, sin
         * IT armada, asi que nunca compite con este despacho. */
        Bt_StoreByte(&s_bt_task);
    }
#endif
}

/**
 * @brief  HAL weak callback — error de UART (framing/overrun/ruido).
 * @note   CRITICO: en HAL_UART_Receive_IT(), un overrun (o un framing error
 *         si se arma la recepcion a mitad de un byte que ya viene en
 *         camino — muy facil con un modulo que transmite libre, sin
 *         sincronizarse con nuestro arranque) ABORTA la recepcion por
 *         completo y NO se vuelve a armar sola. Sin este callback, el
 *         primer error deja al UART sordo para siempre — exactamente el bug
 *         que vimos con el GPS (el TTL si veia tramas limpias en el pin,
 *         pero el MCU nunca capturaba nada). Limpiamos las banderas de
 *         error y re-armamos la recepcion de 1 byte.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
#if INIT_GPS_ENABLE
    if (huart->Instance == GPS_UART->Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(huart, &s_gps.rx_byte, 1U);
        return;
    }
#endif
#if INIT_LORA_ENABLE
    if (huart->Instance == LORA_UART->Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(huart, &s_lora.rx_byte, 1U);
        return;
    }
#endif
#if INIT_BLUETOOTH_ENABLE
    if (huart->Instance == BT_UART->Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(huart, &s_bt_task.rx_byte, 1U);
    }
#endif
}

#if INIT_RX_IR_ENABLE
/**
 * @brief  HAL weak callback — EXTI de cualquier pin armado. Solo el pin del
 *         TSOP (IR_TSOP_PIN) esta armado en este proyecto.
 * @note   Trabajo minimo: nada de Log_Print/Log_Printf aqui adentro.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IR_TSOP_PIN) {
        IR_EXTI_Callback(&ir_handle);
    }
}
#endif
