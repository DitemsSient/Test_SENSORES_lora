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
#include "Secuencia_Leds.h"
#include "Bluetooth.h"
#include "Lora.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern UART_HandleTypeDef huart1;   /* GPS_UART = &huart1, ver GPS.h */
extern UART_HandleTypeDef huart2;   /* LORA_UART = &huart2, ver Lora.h */
extern I2C_HandleTypeDef  hi2c1;    /* I2C1 — bus compartido, protegido por mutex */

/* Handle IR — definido en Inicializacion.c, NO estatico ahi para poder
 * referenciarlo aqui (ver comentarios en Inicializacion.c). Los canales
 * RGB ya no se tocan directo aqui — ver Secuencia_Leds.h. */
extern Ir_Handle_t ir_handle;

/* ======================  CONFIGURATION  ==================================== */

#define GPS_DMA_BUF_SIZE     256U
#define GPS_HEARTBEAT_MS   20000U
#define SENSORS_PERIOD_MS  25000U
#define CALIB_PERIOD_MS      300U
#define CALIB_RAW_MIN         16U    /**< raw_count > esto para considerar el buffer valido (ver Test_IR: tramas reales de 2 bytes dan 17-19 deltas) */
#define CALIB_VALID_WORD  0xAA55U

#define LORA_DMA_BUF_SIZE    256U
#define LORA_EXERCISE_PERIOD_MS 10000U /**< Cada cuanto se manda telemetria completa en MODO_EJERCICIO */
#define TESTLORA_PERIOD_MS   30000U    /**< Cada cuanto se manda "TESTLORA" mientras estamos en MODO_CONFIGURACION */

/* Tarjeta de pruebas aislada de LoRa (2026-09-09): solo trae el modulo LoRa
 * y parte de la alimentacion, sin GPS ni sensores conectados — asi que
 * GpsTask/SensorsTask van a estar tronando en cada lectura sin aportar
 * nada util mientras se prueba el LoRa solo. En vez de una tarea aparte
 * (que competiria por el mismo huart2 que ya usa LoraTask para telemetria),
 * el envio periodico de "TESTLORA" se agrega dentro del loop de LoraTask —
 * mismo criterio ya usado para la telemetria de MODO_EJERCICIO: un solo
 * dueno del UART, sin mutex extra ni riesgo de dos tareas transmitiendo al
 * mismo tiempo. Regresar ambas a 1U cuando la tarjeta final con todos los
 * sensores este lista para probarse de nuevo. */
#define TASK_GPS_ENABLE          0U
#define TASK_SENSORS_ENABLE      0U

/* ======================  STATIC VARIABLES  ================================ */

#if TASK_GPS_ENABLE
static osThreadId_t gpsTaskHandle;
static const osThreadAttr_t gpsTask_attributes = {
    .name       = "GpsTask",
    .stack_size = 1024U * 4U,   /* 4KB */
    .priority   = (osPriority_t)osPriorityNormal,
};
#endif

#if TASK_SENSORS_ENABLE
static osThreadId_t sensorsTaskHandle;
static const osThreadAttr_t sensorsTask_attributes = {
    .name       = "SensorsTask",
    .stack_size = 1024U * 4U,   /* 4KB — llamadas anidadas de HAL I2C (Begin/Init/ReadAll) */
    .priority   = (osPriority_t)osPriorityNormal,
};
#endif

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

static osThreadId_t loraTaskHandle;
static const osThreadAttr_t loraTask_attributes = {
    .name       = "LoraTask",
    .stack_size = 1024U * 4U,   /* 4KB — parseo de CSV + hex encode/decode usan buffers locales */
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Buffer circular del DMA y handle de LoRa propios de esta tarea. */
static uint8_t        s_lora_dma_buf[LORA_DMA_BUF_SIZE];
static uint16_t       s_lora_dma_last_pos = 0U;
static Lora_Handle_t  s_lora_task;

/* NO estaticos — HAL_UART_ErrorCallback (Inicializacion.c) los revisa para
 * saber si ya debe dejar de re-armar HAL_UART_Receive_IT(): mientras estas
 * banderas esten en false (bring-up, antes del RTOS) SI hace falta
 * re-armar IT en cada error; en cuanto GpsTask/LoraTask arman su DMA
 * circular, re-armar IT encima del DMA lo rompe en silencio (bug real, ver
 * comentario en Inicializacion.c). */
bool g_gps_dma_activo  = false;
bool g_lora_dma_activo = false;

/* Bandera que le avisa a BluetoothTask que ya llego y se guardo un $CONF
 * por LoRa — reemplaza el delay fijo de [PRUEBA]. Bandera que LoraTask
 * revisa para saber cuando debe mandar la telemetria de vuelta (se marca
 * al llegar ACKCONF en BluetoothTask). Mismo patron volatile que
 * s_ack_pendiente/s_bt_conectado — un solo escritor, un solo lector cada
 * una, sin necesidad de mutex. */
static volatile bool s_lora_conf_listo       = false;
static volatile bool s_lora_enviar_telemetria = false;

/* Bandera que LoraTask marca al recibir $RUN por LoRa — BluetoothTask (que
 * para entonces ya esta en su loop de escucha, tras el ACKCONF) la revisa
 * para mandarle $RUN\r a la Mira y esperar ACKRUN antes de entrar de verdad
 * a MODO_EJERCICIO (ver BT_HandleRun()). Mismo patron volatile de un solo
 * escritor/un solo lector que s_lora_conf_listo. */
static volatile bool s_lora_run_recibido = false;

/* Mismo patron que s_lora_run_recibido, pero para $END_S — LoraTask la
 * marca al recibirlo, BluetoothTask (loop de escucha) le manda $END_S\r a
 * la Mira y espera ACKEND_S antes de dar por terminado el ejercicio, ver
 * BT_HandleEndS(). */
static volatile bool s_lora_end_s_recibido = false;

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
/* Parpadeo cian de DSCON / rojo de "respuesta inesperada" — ver
 * LEDS_BT_DSCON_MS, LEDS_BT_DSCON_COUNT, LEDS_BT_ERROR_MS y
 * LEDS_BT_ERROR_COUNT en Secuencia_Leds.h */

#if TASK_GPS_ENABLE
/* Buffer circular del DMA y handle de GPS propios de esta tarea — separados
 * del handle que usa el smoke test de Inicializacion.c (ese ya cumplio su
 * proposito en el arranque, este es el que corre para siempre). */
static uint8_t       s_gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t      s_gps_dma_last_pos = 0U;
static Gps_Handle_t  s_gps_task;
#endif

#if TASK_SENSORS_ENABLE
/* Handles de sensores propios de SensorsTask — igual que GPS, separados de
 * los que usa el smoke test de Inicializacion.c. MMC5983MA no usa handle
 * propio (API a nivel de modulo, ver Inicializacion.c). */
static TSL2571_t     s_light_task;
static LSM6DSO32TR_t s_imu_task;
#endif

/* ======================  STATIC FUNCTIONS  ================================ */

#if TASK_GPS_ENABLE
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

    /* El smoke test de Inicializacion.c dejo el UART armado en modo IT
     * (Gps_Init -> HAL_UART_Receive_IT, re-armado solo en cada byte). Lo
     * cancelamos limpio antes de pasar a modo DMA, que es el que esta
     * tarea usa de forma permanente. */
    HAL_UART_AbortReceive(&huart1);

    memset(&s_gps_task, 0, sizeof(s_gps_task));
    s_gps_task.huart = GPS_UART;
    s_gps_dma_last_pos = 0U;

    HAL_UARTEx_ReceiveToIdle_DMA(GPS_UART, s_gps_dma_buf, GPS_DMA_BUF_SIZE);
    g_gps_dma_activo = true;   /* HAL_UART_ErrorCallback ya no debe re-armar IT */
    Log_Print("GPS", "GpsTask arrancada (DMA circular + linea IDLE).");

    uint32_t sentence_count    = 0U;
    uint32_t last_status_tick  = HAL_GetTick();

    for (;;) {
        if (s_gps_task.sentence_ready) {
            if (Gps_Process(&s_gps_task) == GPS_OK) {
                sentence_count++;
                /* Detalle de FIX comentado a proposito mientras se depura
                 * BLE+LoRa — ver nota del heartbeat de abajo. */
                // if (s_gps_task.data.position_valid) {
                //     Log_Printf("GPS", "FIX sats=%u lat=%s alt=%.1fm",
                //                s_gps_task.data.satellites,
                //                s_gps_task.data.position_str,
                //                s_gps_task.data.altitude);
                // }
            }
        }

        /* Reporte de "sigo viva" cada 5s, con o sin fix — sin esto, si nunca
         * hay fix (antena mala, interiores, etc.) no hay forma de distinguir
         * "la tarea esta corriendo pero sin fix" de "la tarea se murio".
         * Detalle (tramas/sats/fix) comentado a proposito mientras se
         * depura BLE+LoRa — ensuciaba mucho el logger. Solo se deja esta
         * linea corta para confirmar que la tarea si esta viva. */
        if ((HAL_GetTick() - last_status_tick) >= GPS_HEARTBEAT_MS) {
            last_status_tick = HAL_GetTick();
            // Log_Printf("GPS", "GpsTask viva: %lu tramas procesadas, sats=%u, fix=%s",
            //            (unsigned long)sentence_count, s_gps_task.data.satellites,
            //            s_gps_task.data.position_valid ? "SI" : "NO");
            Log_Print("GPS", "TASKGPS");
        }

        osDelay(200U);
    }
}
#endif /* TASK_GPS_ENABLE */

/**
 * @brief  Procesa UNA linea AT completa que ya saco Lora_PopLine() de la
 *         cola (terminada en '\n' originalmente, ya sin el '\r'/'\n').
 *         Confirmado con hardware real 2026-09-09: el modulo NO manda el
 *         mensaje $...*** limpio y solo por el UART — manda su propio
 *         protocolo de lineas AT (ecos de comandos, "OK", eventos), y el
 *         payload real del gateway viaja adentro de una linea con forma
 *         "+QEVT:<puerto>:<lenHex>:<payloadHex>" (confirmado viendo un
 *         downlink real: "+QEVT:1:3E:24434F4E..."). Todo lo demas que
 *         manda el modulo (eco de nuestros propios AT+QXXX, "OK",
 *         "+QEVT:SEND_CONFIRMED" de nuestros propios uplinks, etc.) no
 *         trae datos del gateway — se loguea para depurar y ya, no hay
 *         nada que decodificar ahi.
 * @param  line  Buffer NUL-terminado con la linea AT ya recibida.
 */
static void Lora_ProcesarLinea(const char *line)
{
    Log_Printf("LORA", "[PRUEBA] Linea RX: %s", line);

    if (strncmp(line, "+QEVT:", 6U) != 0) {
        return;   /* eco de comando, "OK", "+QEVT:SEND_CONFIRMED", etc. — nada que hacer */
    }

    /* "+QEVT:<puerto>:<lenHex>:<payloadHex>" — se ignora el puerto y el
     * largo, el payload es todo lo que sigue del 2do ':' hasta el final
     * de la linea (mismo criterio que esperarDatoLora() del codigo de
     * referencia, CODIGO_LORA/lora_kg200z.c). */
    const char *p1 = strchr(line + 6, ':');
    const char *p2 = (p1 != NULL) ? strchr(p1 + 1, ':') : NULL;
    if (p2 == NULL) {
        return;   /* "+QEVT:SEND_CONFIRMED" y similares no traen ":" — no es dato */
    }
    const char *payload_hex = p2 + 1;

    /* El payload de la aplicacion (lo que manda quien controla el gateway,
     * no el modulo) SIGUE usando nuestro protocolo $...*** — por eso se
     * decodifica de hex y se busca CONF/RUN/END_S/END igual que antes,
     * solo que ahora sobre el campo payload_hex en vez de la linea entera. */
    uint8_t decoded[300];
    size_t  decoded_len = Lora_DecodeHexToBytes(payload_hex, decoded, sizeof(decoded) - 1U);
    decoded[decoded_len] = '\0';
    char *decoded_str = (char *)decoded;
    Log_Printf("LORA", "[PRUEBA] Payload decodificado: %s", decoded_str);

    char *conf_pos  = strstr(decoded_str, "CONF");
    char *run_pos   = strstr(decoded_str, "RUN");
    char *end_s_pos = strstr(decoded_str, "END_S");   /* revisar ANTES que "END" a secas */
    char *end_pos   = strstr(decoded_str, "END");     /* "END" generico, END_S ya se atendio arriba */

    if (conf_pos != NULL) {
        const char *contenido = conf_pos + 4;
        if (Lora_ParseDatos(contenido, &g_exercise_data)) {
            Log_Print("LORA", "CONF recibido y guardado en g_exercise_data.");
            /* [PRUEBA] Diagnostico de la MAC — sospecha de que la MAC real
             * (armada por el gateway/servidor de otro equipo, a diferencia
             * de las de prueba tecleadas a mano) pueda traer algun
             * caracter invisible de mas (espacio, salto de linea perdido,
             * etc.) que no se note nada mas viendo el %s en el log, pero
             * que si rompe el chequeo de largo exacto (17 = "CON"+14) que
             * hace SensoresM.sb al recibir $CON<mac> — con eso el modulo
             * rechaza la conexion en silencio, sin que nosotros veamos
             * ningun error. strlen aqui deberia dar EXACTAMENTE 14. */
            Log_Printf("LORA", "[PRUEBA] mac='%s' strlen=%u (debe ser 14)",
                       g_exercise_data.mac, (unsigned)strlen(g_exercise_data.mac));
            s_lora_conf_listo = true;
        } else {
            Log_Print("LORA", "ERROR: CONF no se pudo parsear.");
        }
    } else if (run_pos != NULL) {
        /* No se entra a MODO_EJERCICIO todavia aqui — falta que la
         * Mira confirme con ACKRUN (ver BT_HandleRun() en
         * BluetoothTask). Solo se avisa. */
        Log_Print("LORA", "RUN recibido — avisando a BluetoothTask para mandar $RUN a la Mira.");
        s_lora_run_recibido = true;
    } else if (end_s_pos != NULL) {
        /* No se termina el ejercicio todavia aqui — falta que la
         * Mira confirme con ACKEND_S (ver BT_HandleEndS() en
         * BluetoothTask). Solo se avisa. */
        Log_Print("LORA", "END_S recibido — avisando a BluetoothTask para mandar $END_S a la Mira.");
        s_lora_end_s_recibido = true;
    } else if (end_pos != NULL) {
        /* TODO: "END" a secas (sin "_S") sigue sin logica definida —
         * de momento solo se loguea, no se hace nada mas. */
        Log_Print("LORA", "END recibido — logica de fin de ejercicio pendiente de definir.");
    } else {
        Log_Printf("LORA", "Payload de +QEVT sin CONF/RUN/END reconocido: %s", decoded_str);
    }
}

/* [PRUEBA] Simulacion de movimiento para ver cambios en Unity mientras
 * GpsTask esta deshabilitada (TASK_GPS_ENABLE=0) — punto de partida las
 * coordenadas reales que dio el usuario (su companero de pruebas). Paso
 * chico a proposito (~5m por envio) para que se note un desplazamiento
 * pequeno en el mapa, no un salto brusco. */
#define SIM_GPS_LAT_INICIAL     19.436408
#define SIM_GPS_LON_INICIAL    -99.176603
#define SIM_GPS_STEP_DEG          0.00005   /* ~5m por envio a esta latitud */

/**
 * @brief  [PRUEBA] Avanza g_exercise_data.latitud/longitud/timestamp un
 *         poquito en cada llamada (arriba/derecha en el mapa = norte/este,
 *         +lat/+lon) y pone orientacion/pasos al azar — todo sin sensores
 *         reales, solo para validar que la app de Unity refleje cambios.
 *         Quitar/reemplazar por las lecturas reales de GPS/IMU cuando
 *         TASK_GPS_ENABLE vuelva a 1.
 */
static void Lora_SimularMovimiento(void)
{
    static bool     inicializado = false;
    static uint32_t ts_sim = 0U;

    if (!inicializado) {
        g_exercise_data.latitud  = SIM_GPS_LAT_INICIAL;
        g_exercise_data.longitud = SIM_GPS_LON_INICIAL;
        ts_sim = HAL_GetTick() / 1000U;
        srand(HAL_GetTick());
        inicializado = true;
    } else {
        g_exercise_data.latitud  += SIM_GPS_STEP_DEG;   /* un poco hacia arriba (norte) */
        g_exercise_data.longitud += SIM_GPS_STEP_DEG;   /* un poco hacia la derecha (este) */
        ts_sim++;
    }

    g_exercise_data.timestamp   = ts_sim;
    g_exercise_data.orientacion = (uint16_t)(rand() % 360);
    g_exercise_data.pasos       = (uint16_t)(rand() % 2000);
}

/**
 * @brief  Manda la telemetria de 12 campos de vuelta al gateway
 *         (ID,numOrden,vidas,municion,bateria,latitud,longitud,altitud,
 *         orientacion,pasos,ack,timestamp — latitud ANTES que longitud,
 *         ver g_exercise_data), codificada a hex sobre AT+QSEND=1:1:<hex>\r\n
 *         (igual que mandarPorLora() del codigo de referencia).
 * @note   El modulo LoRa no confirma nada por su cuenta — solo manda la
 *         trama y ya, no hay "SEND_CONFIRMED" que esperar. (Antes si se
 *         esperaba, bloqueando LoraTask hasta 10s cada vez porque ese
 *         timeout SIEMPRE se agotaba sin gateway real — con eso se tragaba
 *         en silencio cualquier $RUN/$END_S real que llegara justo en esa
 *         ventana. Ya no se espera nada, se manda y se regresa de
 *         inmediato.)
 */
static void Lora_EnviarTelemetria(void)
{
    Lora_SimularMovimiento();

    char csv[160];
    /* Solo bateria_ch (esta tarjeta) va en la telemetria por ahora —
     * bateria_ap (Mira) todavia no se agrega aqui, ver TODO en
     * Inicializacion.h (pendiente hasta actualizar la base de datos). */
    /* Orden lat/lon: LATITUD antes que LONGITUD — asi lo manda de verdad
     * generarCadena() del codigo de referencia del companero (CODIGO_LORA/
     * lora_kg200z.c), aunque su propio comentario diga "longitud,latitud"
     * (comentario desactualizado, el codigo real pasa latStr primero). */
    int  n = snprintf(csv, sizeof(csv), "%u,%u,%u,%u,%u,%.5f,%.5f,%.1f,%u,%u,%u,%lu",
                       g_exercise_data.lora, g_exercise_data.orden,
                       g_exercise_data.lives, g_exercise_data.ammo,
                       g_exercise_data.bateria_ch,
                       (double)g_exercise_data.latitud, (double)g_exercise_data.longitud,
                       (double)g_exercise_data.altitud,
                       g_exercise_data.orientacion, g_exercise_data.pasos,
                       g_exercise_data.ack, (unsigned long)g_exercise_data.timestamp);

    if (n <= 0 || (size_t)n >= sizeof(csv)) {
        Log_Print("LORA", "ERROR: CSV de telemetria demasiado grande.");
        return;
    }

    char hex_payload[(sizeof(csv) * 2U) + 1U];
    Lora_EncodeToHex((const uint8_t *)csv, (size_t)n, hex_payload);

    char cmd[sizeof(hex_payload) + 16U];
    int  cmd_len = snprintf(cmd, sizeof(cmd), "AT+QSEND=1:1:%s\r\n", hex_payload);
    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        Log_Print("LORA", "ERROR: comando AT+QSEND demasiado grande.");
        return;
    }

    Lora_Transmit(&s_lora_task, (const uint8_t *)cmd, (uint16_t)cmd_len);
    Log_Printf("LORA", "Telemetria enviada: %s", csv);
}

/**
 * @brief  Manda "TESTLORA" cada TESTLORA_PERIOD_MS mientras estamos en
 *         MODO_CONFIGURACION (nunca en MODO_EJERCICIO, ahi ya manda la
 *         telemetria real cada LORA_EXERCISE_PERIOD_MS y no hace falta
 *         duplicar transmisiones). Util para probar el modulo LoRa aislado
 *         en la tarjeta de pruebas (sin GPS ni sensores, ver
 *         TASK_GPS_ENABLE/TASK_SENSORS_ENABLE), confirmando en el gateway
 *         que el enlace sigue vivo aunque nadie haya mandado $CONF todavia.
 */
static void Lora_EnviarTestLora(void)
{
    static const char testlora_msg[] = "TESTLORA";

    char hex_payload[(sizeof(testlora_msg) * 2U) + 1U];
    Lora_EncodeToHex((const uint8_t *)testlora_msg, sizeof(testlora_msg) - 1U, hex_payload);

    char cmd[sizeof(hex_payload) + 16U];
    int  cmd_len = snprintf(cmd, sizeof(cmd), "AT+QSEND=1:1:%s\r\n", hex_payload);
    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        Log_Print("LORA", "ERROR: comando AT+QSEND de TESTLORA demasiado grande.");
        return;
    }

    Lora_Transmit(&s_lora_task, (const uint8_t *)cmd, (uint16_t)cmd_len);
    Log_Print("LORA", "TESTLORA enviado.");
}

/**
 * @brief  Tarea de LoRa: recepcion por DMA circular + linea IDLE (mismo
 *         patron que GpsTask), partida en lineas AT por Lora_StoreBytes()
 *         (ver su doc comment en Lora.h — el modulo manda su propio
 *         protocolo de lineas, no un $...*** limpio, confirmado con
 *         hardware real 2026-09-09).
 * @note   Cada vuelta del loop se drenan TODAS las lineas pendientes con
 *         Lora_PopLine() (puede haber varias juntas) y cada una se procesa
 *         con Lora_ProcesarLinea(): solo las que empiezan con "+QEVT:"
 *         traen datos, y ahi adentro se decodifica el payload de hex y se
 *         busca, en este orden: "CONF" -> Lora_ParseDatos() directo a
 *         g_exercise_data + marca s_lora_conf_listo (arranca BluetoothTask);
 *         "RUN" -> avisa a BluetoothTask (s_lora_run_recibido); "END_S" ->
 *         avisa a BluetoothTask (s_lora_end_s_recibido); "END" (a secas)
 *         -> TODO, logica de fin de ejercicio pendiente de definir.
 * @note   La telemetria de 12 campos (Lora_EnviarTelemetria()) se manda en
 *         dos casos: una vez cuando BluetoothTask marca
 *         s_lora_enviar_telemetria (tras ACKCONF), y periodica cada
 *         LORA_EXERCISE_PERIOD_MS mientras g_modo_operacion==MODO_EJERCICIO.
 */
static void LoraTask(void *argument)
{
    (void)argument;

    /* Reporte de heap libre — primera linea de la primera tarea real que
     * corre, ya con el scheduler vivo. NUNCA mover esto a antes de
     * osKernelStart(). Vivia en GpsTask, se movio aqui porque GpsTask esta
     * deshabilitada (TASK_GPS_ENABLE=0) mientras se prueba la tarjeta
     * aislada de LoRa — LoraTask es ahora la primera tarea que arranca. */
    Log_Printf("RTOS", "Heap libre tras crear tareas: %u bytes (de %u)",
               (unsigned)xPortGetFreeHeapSize(), (unsigned)configTOTAL_HEAP_SIZE);
    Log_Printf("RTOS", "Handles: calib=%s logger=%s bt=%s",
               (calibrateTaskHandle  != NULL) ? "OK" : "NULL",
               (loggerTaskHandle     != NULL) ? "OK" : "NULL",
               (bluetoothTaskHandle  != NULL) ? "OK" : "NULL");

    Log_Print("LORA", "LoraTask arrancada, esperando $CONF...");

    /* El bring-up de Inicializacion.c (Lora_Setup/Lora_Connect, si no esta
     * simulado) dejo el UART armado en modo IT — lo cancelamos limpio
     * antes de pasar a modo DMA, igual que GpsTask con huart1. */
    HAL_UART_AbortReceive(&huart2);

    memset(&s_lora_task, 0, sizeof(s_lora_task));
    s_lora_task.huart = LORA_UART;
    s_lora_dma_last_pos = 0U;

    HAL_UARTEx_ReceiveToIdle_DMA(LORA_UART, s_lora_dma_buf, LORA_DMA_BUF_SIZE);
    g_lora_dma_activo = true;   /* HAL_UART_ErrorCallback ya no debe re-armar IT */

    /* Cada LORA_EXERCISE_PERIOD_MS, mientras estemos en MODO_EJERCICIO, se
     * manda la telemetria completa — arranca al recibir $RUN (ver abajo). */
    uint32_t last_exercise_tx = 0U;

    /* Cada TESTLORA_PERIOD_MS, mientras estemos en MODO_CONFIGURACION (antes
     * de RUN), se manda "TESTLORA" — ver Lora_EnviarTestLora(). En cuanto
     * entra MODO_EJERCICIO esto se detiene solo (la condicion de abajo deja
     * de cumplirse) y la telemetria real de arriba toma el relevo. */
    uint32_t last_testlora_tx = 0U;

    for (;;) {
        if ((g_modo_operacion == MODO_EJERCICIO) &&
            ((HAL_GetTick() - last_exercise_tx) >= LORA_EXERCISE_PERIOD_MS)) {
            last_exercise_tx = HAL_GetTick();
            Lora_EnviarTelemetria();
        }

        if ((g_modo_operacion == MODO_CONFIGURACION) &&
            ((HAL_GetTick() - last_testlora_tx) >= TESTLORA_PERIOD_MS)) {
            last_testlora_tx = HAL_GetTick();
            Lora_EnviarTestLora();
        }

        /* Drena TODAS las lineas pendientes, no solo una — el modulo puede
         * mandar varias juntas en un solo bloque de DMA (ver comentario de
         * Lora_StoreBytes() en Lora.h). */
        char linea[LORA_LINE_MAX_LEN];
        while (Lora_PopLine(&s_lora_task, linea, sizeof(linea))) {
            Lora_ProcesarLinea(linea);
        }

        if (s_lora_enviar_telemetria) {
            s_lora_enviar_telemetria = false;
            Lora_EnviarTelemetria();
        }

        osDelay(50U);
    }
}

#if TASK_SENSORS_ENABLE
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
    bool luz_ok = (light_begin_st == HAL_OK);

    bool imu_ok = (LSM6DSO32TR_Init(&s_imu_task) == LSM_OK);

    /* Una sola linea de resumen en vez de 3 separadas — mas facil de leer
     * de un vistazo en el logger. */
    if (luz_ok && imu_ok) {
        Log_Print("SENSORS", "Sensores configurados y listos (luz=OK imu=OK), arrancando ciclo de lectura.");
    } else {
        Log_Printf("SENSORS", "Sensores configurados con errores (luz=%s imu=%s), arrancando ciclo de lectura.",
                   luz_ok ? "OK" : "FALLO", imu_ok ? "OK" : "FALLO");
    }

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
            g_exercise_data.bateria_ch = bat_data.soc_pct;   /* bateria de ESTA tarjeta */
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

        /* Detalle completo (bateria/lux/mag/gyro) comentado a proposito
         * mientras se depura BLE+LoRa — ensuciaba mucho el logger. Solo se
         * deja esta linea corta para confirmar que el ciclo si corrio.
         * Descomentar Inicializacion_PrintSensorsData() cuando haga falta
         * ver el detalle de sensores otra vez. */
        // Inicializacion_PrintSensorsData();
        Log_Print("SENSORS", "TASKSensores");

        osDelay(SENSORS_PERIOD_MS);
    }
}
#endif /* TASK_SENSORS_ENABLE */

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
                        Leds_ParpadeoCalibracionOk();
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
 * @brief  Blinkea rojo/verde/azul (o combinaciones: cian=verde+azul,
 *         magenta=rojo+azul) en los 3 pares fisicos N veces, con periodo
 *         on/off period_ms. Helper compartido de BluetoothTask. Usa
 *         BT_SleepAndLog() en vez de osDelay() para no perder de vista
 *         nada que llegue por Bluetooth mientras parpadea.
 */
static void BT_Blink(bool red, bool green, bool blue, uint32_t period_ms, uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++) {
        if (red)   Leds_SetRojo(true);
        if (green) Leds_SetVerde(true);
        if (blue)  Leds_SetAzul(true);
        BT_SleepAndLog(period_ms);
        if (red)   Leds_SetRojo(false);
        if (green) Leds_SetVerde(false);
        if (blue)  Leds_SetAzul(false);
        BT_SleepAndLog(period_ms);
    }
}

/**
 * @brief  Maneja "$DSCON" — llega en cualquier momento, sin importar que
 *         estabamos esperando. Responde $ACKDSCON y parpadea cian
 *         (verde+azul) LEDS_BT_DSCON_COUNT veces (LEDS_BT_DSCON_MS) —
 *         homologado con la secuencia equivalente de la Mira.
 * @note   Bug real encontrado con hardware 2026-09-10: el modulo BL654
 *         (SensoresM.sb) ya reintenta la conexion BLE SOLO tras un corte
 *         breve de RF (su propio TimerStart(2, RETRY_DELAY_MS, 0) en el
 *         manejador de desconexion) — confirmado viendo un "$ACKCON"
 *         nuevo llegar segundos despues de un "$DSCON", sin que nosotros
 *         hicieramos nada. Antes esta funcion se quedaba inerte para
 *         siempre apenas veia un DSCON, ignorando ese reintento automatico
 *         del modulo — por eso "nunca hubo una desconexion real" del otro
 *         lado (la Mira/kit) pero nosotros si nos dabamos por vencidos.
 *         Ahora, tras el parpadeo, se espera BT_DSCON_RECONNECT_WINDOW_MS
 *         a ver si llega un "$ACKCON" nuevo.
 * @retval true  si se reconecto solo dentro de la ventana (llego un
 *               "$ACKCON" nuevo) — el caller debe retomar el flujo desde
 *               "esperando READY" (ver BluetoothTask, retomar_ready).
 * @retval false si no se reconecto — en ese caso esta funcion NUNCA
 *               regresa (se queda inerte para siempre, parpadeo terminado,
 *               sin mas reintentos; los callers NO deben poner codigo
 *               despues de llamarla asumiendo que regresa false, revisar
 *               siempre el valor de retorno con un if).
 */
static bool BT_HandleDSCON(void)
{
    static const uint8_t ackdscon[] = "$ACKDSCON\r";
    Bt_Transmit(&s_bt_task, ackdscon, sizeof(ackdscon) - 1U);
    Log_Print("BT", "DSCON recibido — desconectados.");

    s_bt_conectado = false;
    /* s_ack_pendiente NO se toca aqui a proposito — es del caller
     * (BT_HandleRun()/BT_HandleEndS() tienen su propio while que depende
     * de el; tocarlo aqui rompia esos loops si esta funcion llegaba a
     * regresar). */

    BT_Blink(false, true, true, LEDS_BT_DSCON_MS, LEDS_BT_DSCON_COUNT);   /* cian = verde+azul */

    Log_Print("BT", "Esperando reconexion automatica del modulo...");
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < BT_DSCON_RECONNECT_WINDOW_MS) {
        if (s_bt_task.rx_ready) {
            if (strcmp((char *)s_bt_task.rx_buffer, "ACKCON") == 0) {
                Bt_ResetRx(&s_bt_task);
                Log_Print("BT", "Reconectado solo (ACKCON nuevo tras DSCON) — retomando enlace.");
                Leds_SetAzul(true);
                s_bt_conectado = true;
                return true;
            }
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    Log_Print("BT", "No se reconecto a tiempo — BluetoothTask se queda inerte.");
    for (;;) {
        osDelay(1000U);
    }
}

/**
 * @brief  Maneja el arranque de MODO_EJERCICIO: manda "$RUN\r" a la Mira,
 *         espera "ACKRUN" (sin timeout, mismo criterio que ACKCON — $RUN ya
 *         lo confirmo el gateway por LoRa, no tiene sentido abandonar la
 *         espera) y, al confirmarse, hace la señal visual de arranque:
 *         parpadeo rapido magenta (LEDS_RUN_CUENTA_COUNT x LEDS_RUN_CUENTA_MS)
 *         seguido de un parpadeo corto de confirmacion (LEDS_RUN_INICIO_COUNT
 *         x LEDS_RUN_INICIO_MS). Recien ahi entra a
 *         MODO_EJERCICIO de verdad (Modo_SetOperacion), que es lo que
 *         arranca el envio periodico de telemetria en LoraTask.
 * @note   Llamada desde el loop de escucha de BluetoothTask cuando LoraTask
 *         marca s_lora_run_recibido — ver ahi.
 */
static void BT_HandleRun(void)
{
    static const uint8_t run_msg[] = "$RUN\r";
    Bt_Transmit(&s_bt_task, run_msg, sizeof(run_msg) - 1U);
    s_ack_pendiente = ACK_RUN;
    Log_Print("BT", "RUN enviado a la Mira, esperando ACKRUN...");

    while (s_ack_pendiente == ACK_RUN) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                /* Si BT_HandleDSCON() regresa true, el modulo se
                 * reconecto solo — pero aqui a media espera de ACKRUN no
                 * hay forma de "retomar desde READY" como en
                 * BluetoothTask (haria falta reenviar $CONF completo,
                 * fuera del alcance de esta funcion). TODO: reintento
                 * completo del handshake pendiente de definir para este
                 * caso — de momento solo se sigue esperando ACKRUN. */
                (void)BT_HandleDSCON();
            }
            if (strcmp((char *)s_bt_task.rx_buffer, "ACKRUN") == 0) {
                Bt_ResetRx(&s_bt_task);
                s_ack_pendiente = ACK_NINGUNO;
                break;
            }
            Log_Printf("BT", "Esperaba ACKRUN, llego: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    Log_Print("BT", "ACKRUN recibido — parpadeo visual de arranque (magenta).");
    Leds_ParpadeoMagenta(LEDS_RUN_CUENTA_COUNT, LEDS_RUN_CUENTA_MS);
    Leds_ParpadeoMagenta(LEDS_RUN_INICIO_COUNT, LEDS_RUN_INICIO_MS);

    Modo_SetOperacion(MODO_EJERCICIO);
    Log_Print("BT", "MODO_EJERCICIO activo.");
}

/**
 * @brief  Maneja el fin de ejercicio por orden del administrador ($END_S
 *         por LoRa, el encargado detiene la partida a media partida): manda
 *         "$END_S\r" a la Mira, espera "ACKEND_S" (sin timeout, mismo
 *         criterio que ACKRUN) y, al confirmarse, parpadea rojo
 *         LEDS_END_S_COUNT veces (LEDS_END_S_MS) — homologado con la
 *         secuencia equivalente de la Mira (prompt del usuario, 2026-09-09).
 *         Termina el ejercicio: regresa a MODO_CONFIGURACION, lo que ya
 *         detiene el envio periodico de telemetria en LoraTask (gateado por
 *         g_modo_operacion == MODO_EJERCICIO, ver ahi).
 * @note   Llamada desde el loop de escucha de BluetoothTask cuando LoraTask
 *         marca s_lora_end_s_recibido — ver ahi.
 */
static void BT_HandleEndS(void)
{
    static const uint8_t end_s_msg[] = "$END_S\r";
    Bt_Transmit(&s_bt_task, end_s_msg, sizeof(end_s_msg) - 1U);
    s_ack_pendiente = ACK_END_S;
    Log_Print("BT", "END_S enviado a la Mira, esperando ACKEND_S...");

    while (s_ack_pendiente == ACK_END_S) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                /* Mismo caso que en BT_HandleRun() — TODO: reintento
                 * completo del handshake pendiente para este caso. */
                (void)BT_HandleDSCON();
            }
            if (strcmp((char *)s_bt_task.rx_buffer, "ACKEND_S") == 0) {
                Bt_ResetRx(&s_bt_task);
                s_ack_pendiente = ACK_NINGUNO;
                break;
            }
            Log_Printf("BT", "Esperaba ACKEND_S, llego: %s", (char *)s_bt_task.rx_buffer);
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    Log_Print("BT", "ACKEND_S recibido — ejercicio terminado, parpadeo rojo.");
    Leds_ParpadeoRojo(LEDS_END_S_COUNT, LEDS_END_S_MS);

    Modo_SetOperacion(MODO_CONFIGURACION);
    Log_Print("BT", "MODO_CONFIGURACION activo — telemetria periodica detenida.");
}

/**
 * @brief  Maneja "$END_A" — la Mira avisa que termino el ejercicio de su
 *         lado (por Bluetooth, directo, no pasa por LoRa). Responde
 *         "$ACKEND_A\r", hace la secuencia visual colorida de fin de juego
 *         (Leds_ParpadeoFinJuego()) y regresa a MODO_CONFIGURACION.
 */
static void BT_HandleEndA(void)
{
    static const uint8_t ackend_a[] = "$ACKEND_A\r";
    Bt_Transmit(&s_bt_task, ackend_a, sizeof(ackend_a) - 1U);
    Log_Print("BT", "END_A recibido — ACKEND_A enviado, parpadeo de fin de juego.");

    Leds_ParpadeoFinJuego();

    Modo_SetOperacion(MODO_CONFIGURACION);
    Log_Print("BT", "MODO_CONFIGURACION activo.");
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
 *         (ver BT_HandleDSCON()) — parpadea cian y espera
 *         BT_DSCON_RECONNECT_WINDOW_MS a ver si el modulo se reconecta
 *         solo (SensoresM.sb ya reintenta la conexion BLE por su cuenta
 *         tras un corte breve de RF). Si se reconecta (llega un "$ACKCON"
 *         nuevo), retoma el flujo desde "esperando READY" (goto
 *         retomar_ready); si no, ahi si se queda inerte para siempre.
 * @note   Espera a s_lora_conf_listo (LoraTask ya recibio y guardo un
 *         $CONF real) antes de mandar $CON — ya no hay delay simulado.
 * @note   TODO: que hacer tras un timeout de ACKCONF (reintentar, reportar
 *         error, etc.) sigue pendiente de definir — de momento solo se
 *         loguea y se sigue al loop de escucha.
 */
static void BluetoothTask(void *argument)
{
    (void)argument;

    Bt_Init(&s_bt_task);
    Log_Print("BT", "BluetoothTask arrancada.");

    s_bt_conectado  = false;
    s_ack_pendiente = ACK_NINGUNO;

    Log_Print("BT", "Esperando $CONF por LoRa...");
    while (!s_lora_conf_listo) {
        osDelay(50U);
    }
    Log_Printf("BT", "Datos de LoRa listos: orden=%u lora=%u equipo=%s alias=%s vidas=%u balas=%u tiempo=%lu mac=%s",
               g_exercise_data.orden, g_exercise_data.lora, g_exercise_data.team_name,
               g_exercise_data.player_name, g_exercise_data.lives, g_exercise_data.ammo,
               (unsigned long)g_exercise_data.tiempo, g_exercise_data.mac);

    Bt_SendAdvertise(&s_bt_task, g_exercise_data.mac);   /* $CON<mac>\r */
    s_ack_pendiente = ACK_CON;
    Log_Print("BT", "CON enviado, esperando ACKCON (parpadeo azul, sin limite de tiempo)...");

    bool     led_on     = false;
    uint32_t last_blink = HAL_GetTick();
    for (;;) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                if (BT_HandleDSCON()) {
                    /* Se reconecto solo, el ACKCON nuevo ya se consumio
                     * dentro de BT_HandleDSCON() — retomar desde ahi. */
                    goto retomar_ready;
                }
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
            BT_Blink(true, false, false, LEDS_BT_ERROR_MS, LEDS_BT_ERROR_COUNT);
        }

        if ((HAL_GetTick() - last_blink) >= BT_BLINK_MS) {
            last_blink = HAL_GetTick();
            led_on     = !led_on;
            Leds_SetAzul(led_on);
        }

        osDelay(20U);
    }

    /* Enlazados — LEDs azules fijos mientras Mira y nosotros intercambiamos
     * servicios/configuracion. Se apagan hasta que llegue ACKCONF.
     * retomar_ready: tambien es a donde se salta si BT_HandleDSCON()
     * detecta que el modulo se reconecto solo (ver su doc comment) —
     * un $ACKCON nuevo deja al enlace exactamente en este mismo punto. */
retomar_ready:
    Leds_SetAzul(true);
    s_bt_conectado = true;
    Log_Print("BT", "ACKCON recibido — enlazados, esperando READY...");

    for (;;) {
        if (s_bt_task.rx_ready) {
            if (strncmp((char *)s_bt_task.rx_buffer, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                if (BT_HandleDSCON()) {
                    goto retomar_ready;
                }
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
                if (BT_HandleDSCON()) {
                    goto retomar_ready;
                }
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
        /* Senal para LoraTask: ya se confirmo el CONF con Mira, toca
         * mandar la telemetria de vuelta al gateway con ack=1. */
        g_exercise_data.ack   = 1U;
        s_lora_enviar_telemetria = true;
    } else {
        s_ack_pendiente = ACK_NINGUNO;
        Log_Print("BT", "ERROR: no llego ACKCONF a tiempo.");
        g_exercise_data.ack   = 0U;
        s_lora_enviar_telemetria = true;
        /* TODO: pendiente de definir que mas hacemos aqui (reintentar, etc.). */
    }

    /* Ya se confirmo el intercambio (o se agoto el tiempo) — no hace falta
     * seguir indicando "enlazados" con los LEDs. */
    Leds_Apagar();

    /* Loop de escucha para siempre — DSCON se atiende aqui tambien, en
     * cualquier momento de la partida. */
    Log_Print("BT", "BluetoothTask entra al loop de escucha.");
    for (;;) {
        if (s_lora_run_recibido) {
            s_lora_run_recibido = false;
            BT_HandleRun();
        }

        if (s_lora_end_s_recibido) {
            s_lora_end_s_recibido = false;
            BT_HandleEndS();
        }

        if (s_bt_task.rx_ready) {
            char *line = (char *)s_bt_task.rx_buffer;

            if (strncmp(line, "DSCON", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                if (BT_HandleDSCON()) {
                    goto retomar_ready;
                }
            } else if (strncmp(line, "END_A", 5U) == 0) {
                Bt_ResetRx(&s_bt_task);
                BT_HandleEndA();
            } else if (strncmp(line, "A_AP", 4U) == 0) {
                /* $A_AP<balas>,<pct_bateria>\r — Mira lo manda cada 200ms
                 * (solo si cambio algo). Actualiza municion y bateria_ap;
                 * bateria_ap NO se agrega todavia a la telemetria LoRa, ver
                 * comentario en Inicializacion.h. La Mira necesita el ACK
                 * de cada uno para saber que si llego (ver pregunta del
                 * usuario, 2026-09-08) — se manda siempre, tanto si el
                 * parseo salio bien como si vino mal formado. */
                const char *contenido = line + 4;
                char *coma = strchr(contenido, ',');
                if (coma != NULL) {
                    int balas   = atoi(contenido);
                    int bateria = atoi(coma + 1);
                    g_exercise_data.ammo       = (uint16_t)balas;
                    g_exercise_data.bateria_ap = (uint8_t)bateria;
                    Log_Printf("BT", "A_AP recibido: balas=%d bateria_ap=%d%%", balas, bateria);
                } else {
                    Log_Printf("BT", "ERROR: A_AP mal formado: %s", line);
                }
                static const uint8_t ack_a_ap[] = "$ACKA_AP\r";
                Bt_Transmit(&s_bt_task, ack_a_ap, sizeof(ack_a_ap) - 1U);
                Bt_ResetRx(&s_bt_task);
            } else {
                Log_Printf("BT", "RX no reconocido: %s", line);
                Bt_ResetRx(&s_bt_task);
            }
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
#if TASK_GPS_ENABLE
    gpsTaskHandle     = osThreadNew(GpsTask, NULL, &gpsTask_attributes);
#endif
    loraTaskHandle    = osThreadNew(LoraTask, NULL, &loraTask_attributes);
#if TASK_SENSORS_ENABLE
    sensorsTaskHandle = osThreadNew(SensorsTask, NULL, &sensorsTask_attributes);
#endif
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
#if TASK_GPS_ENABLE
    if (huart->Instance == GPS_UART->Instance) {
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
        return;
    }
#endif

    if (huart->Instance == LORA_UART->Instance) {
        if (Size >= s_lora_dma_last_pos) {
            Lora_StoreBytes(&s_lora_task, &s_lora_dma_buf[s_lora_dma_last_pos],
                            (uint16_t)(Size - s_lora_dma_last_pos));
        } else {
            Lora_StoreBytes(&s_lora_task, &s_lora_dma_buf[s_lora_dma_last_pos],
                            (uint16_t)(LORA_DMA_BUF_SIZE - s_lora_dma_last_pos));
            Lora_StoreBytes(&s_lora_task, &s_lora_dma_buf[0], Size);
        }
        s_lora_dma_last_pos = Size;
    }
}
