/**
 * @file    Tareas.c
 * @brief   Driver implementation for FreeRTOS task creation and coordination.
 *
 * @date    September 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Tareas.h"
#include "cmsis_os.h"
#include "Logger.h"
#include "I2C1_Bus.h"
#include "GPS.h"
#include "Inicializacion.h"
#include "SensorLuz_TSL2571.h"
#include "LSM6DSO32TR.h"
#include "MMC5983MA.h"
#include "BatteryMonitor.h"
#include "Receptor_Infrarrojo_EXTI.h"
#include "Driver_RGB.h"
#include "Bluetooth.h"
#include <string.h>
#include <stdio.h>

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern UART_HandleTypeDef huart1;   /* GPS_UART = &huart1, ver GPS.h */
extern I2C_HandleTypeDef  hi2c1;    /* I2C1 — bus compartido, protegido por mutex */

/* Handle IR y canales RGB — definidos en Inicializacion.c, NO estaticos
 * ahi para poder referenciarlos aqui (ver comentarios en Inicializacion.c). */
extern Ir_Handle_t ir_handle;
extern LP55231_t   rgb;
extern const uint8_t rgb_red_ch[3];
extern const uint8_t rgb_green_ch[3];
extern const uint8_t rgb_blue_ch[3];

/* ======================  CONFIGURATION  ==================================== */

#define GPS_DMA_BUF_SIZE     256U
#define GPS_HEARTBEAT_MS    8000U
#define SENSORS_PERIOD_MS  12000U
#define CALIB_PERIOD_MS      300U
#define CALIB_RAW_MIN         16U    /**< raw_count > esto para considerar el buffer valido (ver Test_IR: tramas reales de 2 bytes dan 17-19 deltas) */
#define CALIB_VALID_WORD  0xAA55U
#define CALIB_BLINK_MS       500U

/* ======================  STATIC VARIABLES  ================================ */

static osThreadId_t gpsTaskHandle;
static const osThreadAttr_t gpsTask_attributes = {
    .name       = "GpsTask",
    .stack_size = 1024U * 4U,   /* 4KB */
    .priority   = (osPriority_t)osPriorityNormal,
};

static osThreadId_t sensorsTaskHandle;
static const osThreadAttr_t sensorsTask_attributes = {
    .name       = "SensorsTask",
    .stack_size = 1024U * 4U,   /* 4KB — llamadas anidadas de HAL I2C (Begin/Init/ReadAll) */
    .priority   = (osPriority_t)osPriorityNormal,
};

static osThreadId_t calibrateTaskHandle;
static const osThreadAttr_t calibrateTask_attributes = {
    .name       = "CalibrateTask",
    .stack_size = 512U * 4U,   /* 2KB */
    .priority   = (osPriority_t)osPriorityNormal,
};

static osThreadId_t loggerTaskHandle;
static const osThreadAttr_t loggerTask_attributes = {
    .name       = "LoggerTask",
    .stack_size = 512U * 4U,   /* 2KB */
    .priority   = (osPriority_t)osPriorityNormal,
};

static osThreadId_t bluetoothTaskHandle;
static const osThreadAttr_t bluetoothTask_attributes = {
    .name       = "BluetoothTask",
    .stack_size = 512U * 4U,   /* 2KB */
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Handle de Bluetooth propio de esta tarea — separado del que usa el
 * smoke test de Inicializacion.c, mismo patron que GPS/Sensores. NO
 * estatico — el HAL_UART_RxCpltCallback compartido (Inicializacion.c) lo
 * referencia con "extern" para poder despachar bytes de huart3 aqui
 * mientras BluetoothTask esta viva (mismo patron que "rgb"/"ir_handle"). */
Bt_Handle_t s_bt_task;

/* Estado de BluetoothTask — mismo patron que el proyecto hermano (Mira):
 * un AckEstado_e global que se marca justo antes/despues de mandar un
 * comando, y que el propio loop de recepcion resuelve cuando llega el ACK
 * correspondiente (ver AckEstado_e en Bluetooth.h). volatile porque se lee
 * y escribe desde el mismo loop de la tarea en distintas iteraciones. */
static volatile AckEstado_e s_ack_pendiente = ACK_NINGUNO;
static volatile bool        s_bt_conectado  = false;

#define BT_BLINK_MS          500U    /**< Parpadeo azul mientras se espera ACKCON            */
#define BT_SIM_LORA_MS      5000U    /**< [PRUEBA] Cuanto "simulamos" la espera de LoRa (sin parpadeo, ya lo hace el verde de fin de Inicializacion_Run()) */
#define BT_DSCON_BLINK_MS   600U     /**< Parpadeo rojo al recibir DSCON                     */
#define BT_DSCON_BLINK_COUNT  5U     /**< Veces que parpadea rojo al recibir DSCON           */

/* Buffer circular del DMA y handle de GPS propios de esta tarea — separados
 * del handle que usa el smoke test de Inicializacion.c (ese ya cumplio su
 * proposito en el arranque, este es el que corre para siempre). */
static uint8_t       s_gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t      s_gps_dma_last_pos = 0U;
static Gps_Handle_t  s_gps_task;

/* Handles de sensores propios de SensorsTask — igual que GPS, separados de
 * los que usa el smoke test de Inicializacion.c. MMC5983MA no usa handle
 * propio (API a nivel de modulo, ver Inicializacion.c). */
static TSL2571_t     s_light_task;
static LSM6DSO32TR_t s_imu_task;

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Tarea del GPS: recepcion por DMA circular + deteccion de linea
 *         IDLE (HAL_UARTEx_ReceiveToIdle_DMA) — ver Core/Doc, arquitectura
 *         de tareas UART: se eligio DMA+IDLE en vez de IT byte-a-byte
 *         porque un solo error de framing/overrun sin HAL_UART_ErrorCallback
 *         deja la recepcion IT sorda para siempre (bug real, ya visto en
 *         el smoke test de Inicializacion.c).
 */
static void GpsTask(void *argument)
{
    (void)argument;

    /* Reporte de heap libre — primera linea de la primera tarea real que
     * corre, ya con el scheduler vivo. NUNCA mover esto a antes de
     * osKernelStart(). */
    Log_Printf("RTOS", "Heap libre tras crear tareas: %u bytes (de %u)",
               (unsigned)xPortGetFreeHeapSize(), (unsigned)configTOTAL_HEAP_SIZE);
    Log_Printf("RTOS", "Handles: sensors=%s calib=%s logger=%s bt=%s",
               (sensorsTaskHandle    != NULL) ? "OK" : "NULL",
               (calibrateTaskHandle  != NULL) ? "OK" : "NULL",
               (loggerTaskHandle     != NULL) ? "OK" : "NULL",
               (bluetoothTaskHandle  != NULL) ? "OK" : "NULL");

    /* El smoke test de Inicializacion.c dejo el UART armado en modo IT
     * (Gps_Init -> HAL_UART_Receive_IT, re-armado solo en cada byte). Lo
     * cancelamos limpio antes de pasar a modo DMA, que es el que esta
     * tarea usa de forma permanente. */
    HAL_UART_AbortReceive(&huart1);

    memset(&s_gps_task, 0, sizeof(s_gps_task));
    s_gps_task.huart = GPS_UART;
    s_gps_dma_last_pos = 0U;

    HAL_UARTEx_ReceiveToIdle_DMA(GPS_UART, s_gps_dma_buf, GPS_DMA_BUF_SIZE);
    Log_Print("GPS", "GpsTask arrancada (DMA circular + linea IDLE).");

    uint32_t sentence_count    = 0U;
    uint32_t last_status_tick  = HAL_GetTick();

    for (;;) {
        if (s_gps_task.sentence_ready) {
            if (Gps_Process(&s_gps_task) == GPS_OK) {
                sentence_count++;
                if (s_gps_task.data.position_valid) {
                    Log_Printf("GPS", "FIX sats=%u lat=%s alt=%.1fm",
                               s_gps_task.data.satellites,
                               s_gps_task.data.position_str,
                               s_gps_task.data.altitude);
                }
            }
        }

        /* Reporte de "sigo viva" cada 5s, con o sin fix — sin esto, si nunca
         * hay fix (antena mala, interiores, etc.) no hay forma de distinguir
         * "la tarea esta corriendo pero sin fix" de "la tarea se murio". */
        if ((HAL_GetTick() - last_status_tick) >= GPS_HEARTBEAT_MS) {
            last_status_tick = HAL_GetTick();
            Log_Printf("GPS", "GpsTask viva: %lu tramas procesadas, sats=%u, fix=%s",
                       (unsigned long)sentence_count, s_gps_task.data.satellites,
                       s_gps_task.data.position_valid ? "SI" : "NO");
        }

        osDelay(200U);
    }
}

/**
 * @brief  Tarea de lectura de sensores: cada SENSORS_PERIOD_MS lee luz,
 *         bateria, IMU (giroscopio) y magnetometro por I2C1 (protegido con
 *         mutex, ver I2C1Bus_Lock/Unlock), vacia los datos en
 *         g_exercise_data y los imprime con Inicializacion_PrintSensorsData().
 * @note   Handles propios de esta tarea, separados de los que usa el smoke
 *         test de Inicializacion.c — mismo patron que GpsTask.
 */
static void SensorsTask(void *argument)
{
    (void)argument;

    /* Log ANTES de tocar el I2C — si esto nunca aparece, la tarea ni
     * siquiera arranco (osThreadNew regreso NULL, heap insuficiente); si
     * aparece pero "SensorsTask lista" no, se colgo dentro de Begin/Init. */
    Log_Print("SENSORS", "SensorsTask arrancada, inicializando sensores...");

    /* NOTA: LSM6DSO32TR.c y MMC5983MA.c ya toman I2C1Bus_Lock()/Unlock()
     * ellos mismos en cada transaccion (ver LSM_WriteReg/LSM_ReadRegs y
     * MMC5983MA.c) — el mutex NO es recursivo, asi que envolverlos aqui con
     * otro Lock()/Unlock() por fuera es un auto-deadlock (la tarea se
     * bloquea esperando un mutex que ella misma ya tiene tomado). TSL2571 y
     * BatteryMonitor SI necesitan el Lock manual porque esos drivers no
     * bloquean nada internamente. */
    TSL2571_Attach(&s_light_task, &hi2c1, TSL2571_ADDR_7BIT, 100U);
    I2C1Bus_Lock();
    HAL_StatusTypeDef light_begin_st = TSL2571_Begin(&s_light_task, 0xC0U, TSL2571_GAIN_1X);
    I2C1Bus_Unlock();
    if (light_begin_st == HAL_OK) {
        Log_Print("SENSORS", "TSL2571 (luz) listo.");
    } else {
        Log_Print("SENSORS", "ERROR: TSL2571_Begin fallo.");
    }

    if (LSM6DSO32TR_Init(&s_imu_task) == LSM_OK) {
        Log_Print("SENSORS", "LSM6DSO32TR (IMU) listo.");
    } else {
        Log_Print("SENSORS", "ERROR: LSM6DSO32TR_Init fallo.");
    }

    Log_Print("SENSORS", "SensorsTask lista, arrancando ciclo de lectura.");

    for (;;) {
        float lux = 0.0f;
        TSL2571_RawData_t light_raw;
        I2C1Bus_Lock();
        HAL_StatusTypeDef lux_st = TSL2571_ReadLux(&s_light_task, 1U, 200U, &lux, &light_raw);
        I2C1Bus_Unlock();
        if (lux_st == HAL_OK) {
            g_exercise_data.lux = lux;
        }

        BatGauge_Data_t bat_data;
        I2C1Bus_Lock();
        BatGauge_Update(&bat_data);
        I2C1Bus_Unlock();
        if (bat_data.is_ready) {
            g_exercise_data.lvBatery = bat_data.soc_pct;
        }

        LSM_Data_t imu_data;
        if (LSM6DSO32TR_ReadAll(&s_imu_task, &imu_data) == LSM_OK) {
            g_exercise_data.gyro_x_dps = imu_data.gx_dps;
            g_exercise_data.gyro_y_dps = imu_data.gy_dps;
            g_exercise_data.gyro_z_dps = imu_data.gz_dps;
        }

        MMC_Data_t mag_data;
        if (MMC5983MA_ReadAll(&mag_data) == MMC_OK) {
            g_exercise_data.mag_x_uT = mag_data.x_uT;
            g_exercise_data.mag_y_uT = mag_data.y_uT;
            g_exercise_data.mag_z_uT = mag_data.z_uT;
        }

        Inicializacion_PrintSensorsData();

        osDelay(SENSORS_PERIOD_MS);
    }
}

/**
 * @brief  Tarea de calibracion (disparo IR de prueba): cada CALIB_PERIOD_MS
 *         consume lo que IR_EXTI_Callback (ISR de EXTI0, prioridad 0, fuera
 *         del control de FreeRTOS — ver Modo_SetOperacion()) fue acumulando
 *         en ir_handle.
 * @note   Trama valida = raw_count > CALIB_RAW_MIN (ver Test_IR: tramas
 *         reales de 2 bytes dan 17-19 deltas). Si no es valida, se
 *         descarta en silencio (ni se imprime). Si es valida:
 *           - MODO_CONFIGURACION: no se saca hash, se compara directo
 *             contra CALIB_VALID_WORD (0xAA55). Si coincide, parpadeo
 *             magenta 500ms en los 3 pares fisicos. Se imprime "Impacto:
 *             <dato>" siempre (coincida o no).
 *           - MODO_EJERCICIO: TODO — pendiente (restar vidas + retransmitir
 *             por Bluetooth). De momento solo se drena el buffer para que
 *             no se desborde.
 */
static void CalibrateTask(void *argument)
{
    (void)argument;

    Log_Print("CALIB", "CalibrateTask arrancada.");

    for (;;) {
        if (IR_Process(&ir_handle) == IR_OK && ir_handle.frame_ready) {
            uint16_t raw_count = ir_handle.raw_count;

            if (raw_count > CALIB_RAW_MIN) {
                if (g_modo_operacion == MODO_CONFIGURACION) {
                    uint16_t dato = 0U;
                    if (ir_handle.frame_len >= 2U) {
                        dato = ((uint16_t)ir_handle.frame_buf[0] << 8U) | ir_handle.frame_buf[1];
                    }

                    if (dato == CALIB_VALID_WORD) {
                        for (uint8_t j = 0U; j < 3U; j++) {
                            LP55231_SetChannelPWM(&rgb, rgb_red_ch[j],  0xFFU);
                            LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0xFFU);
                        }
                        osDelay(CALIB_BLINK_MS);
                        for (uint8_t j = 0U; j < 3U; j++) {
                            LP55231_SetChannelPWM(&rgb, rgb_red_ch[j],  0x00U);
                            LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0x00U);
                        }
                    }

                    Log_Printf("CALIB", "Impacto: 0x%04X", dato);
                } else {
                    /* MODO_EJERCICIO — pendiente, ver nota arriba. */
                    Log_Print("CALIB", "Impacto recibido en MODO_EJERCICIO — logica pendiente (restar vidas + Bluetooth).");
                }
            }

            IR_Reset(&ir_handle);
        }

        osDelay(CALIB_PERIOD_MS);
    }
}

/**
 * @brief  [PRUEBA] Duerme ms milisegundos en pedacitos de 20ms, imprimiendo
 *         por el Logger cualquier cosa que llegue por Bluetooth mientras
 *         tanto — util para ver que responde el modulo aunque no sea
 *         ninguno de los comandos que reconocemos ($...\r). Momentaneo,
 *         mientras hacemos pruebas — quitar cuando el protocolo este
 *         cerrado y ya no haga falta ver "todo" lo que llega.
 */
static void BT_SleepAndLog(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        if (s_bt_task.rx_ready) {
            Log_Printf("BT", "[PRUEBA] RX: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }
}

/**
 * @brief  Blinkea rojo/azul/magenta en los 3 pares fisicos N veces, con
 *         periodo on/off period_ms. Helper compartido de BluetoothTask.
 *         Usa BT_SleepAndLog() en vez de osDelay() para no perder de vista
 *         nada que llegue por Bluetooth mientras parpadea.
 */
static void BT_Blink(bool red, bool blue, uint32_t period_ms, uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++) {
        for (uint8_t j = 0U; j < 3U; j++) {
            if (red)  LP55231_SetChannelPWM(&rgb, rgb_red_ch[j],  0xFFU);
            if (blue) LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0xFFU);
        }
        BT_SleepAndLog(period_ms);
        for (uint8_t j = 0U; j < 3U; j++) {
            if (red)  LP55231_SetChannelPWM(&rgb, rgb_red_ch[j],  0x00U);
            if (blue) LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0x00U);
        }
        BT_SleepAndLog(period_ms);
    }
}

/**
 * @brief  Maneja "$DSCON" — llega en cualquier momento, sin importar que
 *         estabamos esperando. Responde $ACKDSCON, parpadea rojo
 *         BT_DSCON_BLINK_COUNT veces (BT_DSCON_BLINK_MS), y deja el estado
 *         listo para reintentar el enlace desde cero.
 * @note   El modulo Bluetooth es quien retransmite el ACKDSCON si hace
 *         falta — nosotros no reintentamos el envio.
 */
static void BT_HandleDSCON(void)
{
    static const uint8_t ackdscon[] = "$ACKDSCON\r";
    Bt_Transmit(&s_bt_task, ackdscon, sizeof(ackdscon) - 1U);
    Log_Print("BT", "DSCON recibido — desconectados.");

    s_bt_conectado  = false;
    s_ack_pendiente = ACK_NINGUNO;

    BT_Blink(true, false, BT_DSCON_BLINK_MS, BT_DSCON_BLINK_COUNT);
}

/**
 * @brief  Tarea de Bluetooth: enlace inicial con Mira, protocolo $...\r.
 * @note   Secuencia (2026-09-07):
 *         [PRUEBA] sin LoraTask todavia, g_exercise_data ya viene precargada
 *         (ver Inicializacion.c) simulando un $CONF ya recibido por LoRa —
 *         solo parpadeamos magenta 10s para simular esa espera.
 *         Luego: $CON<mac> -> parpadeo azul INDEFINIDO hasta "ACKCON" (si
 *         llega otra cosa, parpadeo rojo 3x400ms y se sigue esperando) ->
 *         azul fijo (enlazados) -> espera "READY" -> $CONF<datos> (solo
 *         mac1, mac2 no se manda) -> espera "ACKCONF" hasta
 *         BT_ACKCONF_TIMEOUT_MS -> loop de escucha para siempre.
 *         "$DSCON" se atiende SIEMPRE, en cualquier punto de la secuencia
 *         (ver BT_HandleDSCON()) y regresa al inicio (reintenta el enlace).
 * @note   TODO: por ahora la secuencia arranca sola al inicio de la tarea
 *         en vez de esperar a que LoraTask confirme un $CONF real — no
 *         existe LoraTask todavia (ver bloque [PRUEBA] arriba).
 * @note   TODO: que hacer tras un timeout de ACKCONF (reintentar, reportar
 *         error, etc.) sigue pendiente de definir — de momento solo se
 *         loguea y se sigue al loop de escucha.
 */
static void BluetoothTask(void *argument)
{
    (void)argument;

    Bt_Init(&s_bt_task);
    Log_Print("BT", "BluetoothTask arrancada.");

reconectar:
    s_bt_conectado  = false;
    s_ack_pendiente = ACK_NINGUNO;

    /* [PRUEBA] Simulacion de la llegada por LoRa — quitar cuando exista
     * LoraTask real (ver nota arriba y en Inicializacion.c). Ya no
     * parpadea, solo espera BT_SIM_LORA_MS (5s) despues del parpadeo verde
     * de fin de Inicializacion_Run(). */
    Log_Printf("BT", "[PRUEBA] Datos (simulados): orden=%u lora=%u equipo=%s alias=%s vidas=%u balas=%u tiempo=%lu mac=%s",
               g_exercise_data.orden, g_exercise_data.lora, g_exercise_data.team_name,
               g_exercise_data.player_name, g_exercise_data.lives, g_exercise_data.ammo,
               (unsigned long)g_exercise_data.tiempo, g_exercise_data.mac);
    Log_Print("BT", "[PRUEBA] Simulando espera de LoRa (5s)...");
    BT_SleepAndLog(BT_SIM_LORA_MS);

    Bt_SendAdvertise(&s_bt_task, g_exercise_data.mac);   /* $CON<mac>\r */
    s_ack_pendiente = ACK_CON;
    Log_Print("BT", "CON enviado, esperando ACKCON (parpadeo azul, sin limite de tiempo)...");

    bool     led_on     = false;
    uint32_t last_blink = HAL_GetTick();
    for (;;) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                BT_HandleDSCON();
                goto reconectar;
            }
            /* strcmp exacto, no strncmp de prefijo: "ACKCONF" empieza con
             * las mismas 6 letras que "ACKCON" y haria falso match aqui. */
            if (strcmp((char *)s_bt_task.rx_buffer, "ACKCON") == 0) {
                Bt_ResetRx(&s_bt_task);
                s_ack_pendiente = ACK_NINGUNO;
                break;
            }
            Log_Printf("BT", "Esperaba ACKCON, llego: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
            BT_Blink(true, false, 400U, 3U);
        }

        if ((HAL_GetTick() - last_blink) >= BT_BLINK_MS) {
            last_blink = HAL_GetTick();
            led_on     = !led_on;
            for (uint8_t j = 0U; j < 3U; j++) {
                LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], led_on ? 0xFFU : 0x00U);
            }
        }

        osDelay(20U);
    }

    /* Enlazados — LEDs azules fijos mientras Mira y nosotros intercambiamos
     * servicios/configuracion. Se apagan hasta que llegue ACKCONF. */
    for (uint8_t j = 0U; j < 3U; j++) {
        LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0xFFU);
    }
    s_bt_conectado = true;
    Log_Print("BT", "ACKCON recibido — enlazados, esperando READY...");

    for (;;) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                BT_HandleDSCON();
                goto reconectar;
            }
            if (strncmp((char *)s_bt_task.rx_buffer, "READY", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                break;
            }
            Log_Printf("BT", "RX no reconocido esperando READY: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    Log_Print("BT", "READY recibido — esperando 150ms antes de enviar CONF...");
    osDelay(150U);

    /* $CONF<datos>\r — homogeneo con lo que Mira espera: 8 campos, solo la
     * primera MAC (mac2 se guarda pero no se manda). */
    char    payload[160];
    int32_t n = snprintf(payload, sizeof(payload), "$CONF%u,%u,%s,%s,%u,%u,%lu,%s\r",
                          g_exercise_data.orden, g_exercise_data.lora,
                          g_exercise_data.team_name, g_exercise_data.player_name,
                          g_exercise_data.lives, g_exercise_data.ammo,
                          (unsigned long)g_exercise_data.tiempo, g_exercise_data.mac);

    if (n > 0 && (size_t)n < sizeof(payload)) {
        Bt_Transmit(&s_bt_task, (uint8_t *)payload, (uint16_t)n);
    } else {
        Log_Print("BT", "ERROR: payload de CONF demasiado grande.");
    }

    s_ack_pendiente = ACK_CONF;
    uint32_t ackconf_start = HAL_GetTick();
    while (s_ack_pendiente == ACK_CONF && (HAL_GetTick() - ackconf_start) < BT_ACKCONF_TIMEOUT_MS) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                BT_HandleDSCON();
                goto reconectar;
            }
            if (strncmp((char *)s_bt_task.rx_buffer, "ACKCONF", 7U) == 0) {
                s_ack_pendiente = ACK_NINGUNO;
            } else {
                Log_Printf("BT", "RX no reconocido esperando ACKCONF: %s", (char *)s_bt_task.rx_buffer);
            }
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    if (s_ack_pendiente == ACK_NINGUNO) {
        Log_Print("BT", "ACKCONF recibido.");
    } else {
        s_ack_pendiente = ACK_NINGUNO;
        Log_Print("BT", "ERROR: no llego ACKCONF a tiempo.");
        /* TODO: pendiente de definir que hacemos aqui (reintentar, etc.). */
    }

    /* Ya se confirmo el intercambio (o se agoto el tiempo) — no hace falta
     * seguir indicando "enlazados" con los LEDs. */
    for (uint8_t j = 0U; j < 3U; j++) {
        LP55231_SetChannelPWM(&rgb, rgb_blue_ch[j], 0x00U);
    }

    /* Loop de escucha para siempre — DSCON se atiende aqui tambien, en
     * cualquier momento de la partida. */
    Log_Print("BT", "BluetoothTask entra al loop de escucha.");
    for (;;) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                BT_HandleDSCON();
                goto reconectar;
            }
            Log_Printf("BT", "RX no reconocido: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

void Tareas_InicializarMutex(void)
{
    Log_InitQueue();
    I2C1Bus_InitMutex();
}

void Tareas_CrearTareas(void)
{
    gpsTaskHandle     = osThreadNew(GpsTask, NULL, &gpsTask_attributes);
    sensorsTaskHandle = osThreadNew(SensorsTask, NULL, &sensorsTask_attributes);
    calibrateTaskHandle = osThreadNew(CalibrateTask, NULL, &calibrateTask_attributes);
    loggerTaskHandle  = osThreadNew(Log_Task, NULL, &loggerTask_attributes);
    bluetoothTaskHandle = osThreadNew(BluetoothTask, NULL, &bluetoothTask_attributes);
    /* Si algun handle sale NULL (heap insuficiente), osThreadNew() no
     * truena, solo regresa NULL — no hay forma segura de loggearlo aqui
     * (scheduler todavia no corre, ver Tareas.h). Cuando haya mas de una
     * tarea, la primera que si arranque puede revisar los handles de las
     * demas y reportarlo. */
}

/* ======================  HAL WEAK CALLBACKS  =============================== */

/**
 * @brief  HAL weak callback — evento de recepcion por DMA con deteccion de
 *         linea IDLE (HAL_UARTEx_ReceiveToIdle_DMA). En modo circular,
 *         "Size" es la posicion absoluta dentro del buffer donde va la
 *         escritura del DMA, no la cantidad de bytes nuevos — hay que
 *         restarla contra la posicion anterior (con wrap-around si el DMA
 *         ya le dio la vuelta al buffer).
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != GPS_UART->Instance) {
        return;
    }

    if (Size >= s_gps_dma_last_pos) {
        Gps_StoreBytes(&s_gps_task, &s_gps_dma_buf[s_gps_dma_last_pos],
                       (uint16_t)(Size - s_gps_dma_last_pos));
    } else {
        /* El DMA le dio la vuelta al buffer entre el evento anterior y este. */
        Gps_StoreBytes(&s_gps_task, &s_gps_dma_buf[s_gps_dma_last_pos],
                       (uint16_t)(GPS_DMA_BUF_SIZE - s_gps_dma_last_pos));
        Gps_StoreBytes(&s_gps_task, &s_gps_dma_buf[0], Size);
    }

    s_gps_dma_last_pos = Size;
}
