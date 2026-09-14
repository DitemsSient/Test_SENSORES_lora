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
#include <math.h>

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
/* Bajado de 25000 a 100 el 2026-09-15: ya NO es solo diagnostico — el
 * conteo de pasos (Steps_Update(), ver abajo) necesita muestrear el
 * acelerometro varias veces por segundo para no perderse los picos de cada
 * paso (cadencia tipica al caminar ~1.5-2.5 Hz, con 100ms/10Hz hay margen
 * de sobra). Si esto sube de nuevo, el detector de pasos deja de servir. */
#define SENSORS_PERIOD_MS  100U
#define CALIB_PERIOD_MS      300U

/* ==================  DETECCION DE PASOS (acelerometro)  ==================== */

/* Umbral de histeresis sobre la magnitud del vector de aceleracion (g).
 * En reposo |v|~1.0 (solo gravedad); cada paso mete un pico dinamico
 * encima de eso. Cruzar STEP_THRESHOLD_HIGH_G hacia arriba (armado) cuenta
 * un paso; hay que volver a cruzar STEP_THRESHOLD_LOW_G hacia abajo para
 * "rearmar" el detector antes de que cuente el siguiente — evita contar
 * varias veces el mismo pico ruidoso. STEP_MIN_INTERVAL_MS es un segundo
 * filtro (refractario) por si el rearme es demasiado rapido.
 * VALORES: ajustados 2026-09-15 tras primera prueba real — con
 * 1.15/0.95 solo se contaron 4 de ~12-15 pasos reales dados por el
 * usuario, porque varios picos de |v| se quedaban en 1.10-1.14, justo
 * debajo del umbral. Bajado a 1.08/0.97 — sigue muy por arriba del ruido
 * de reposo medido (|v| fluctuaba +-0.01/0.02g quieta), pero ya agarra
 * picos mas suaves. Sigue siendo un punto de partida, falta otra prueba
 * con conteo real para confirmar que no se paso para el otro lado
 * (contar de mas). */
#define STEP_THRESHOLD_HIGH_G   1.08f
#define STEP_THRESHOLD_LOW_G    0.97f
#define STEP_MIN_INTERVAL_MS    300U
#define CALIB_RAW_MIN         16U    /**< raw_count > esto para considerar el buffer valido (ver Test_IR: tramas reales de 2 bytes dan 17-19 deltas) */
#define CALIB_VALID_WORD  0xAA55U

#define LORA_DMA_BUF_SIZE    256U
#define LORA_EXERCISE_PERIOD_MS 10000U /**< Cada cuanto se manda telemetria completa en MODO_EJERCICIO */

/* Cola de atacantes pendientes de reportar (ver s_atacante_buffer mas
 * abajo) — mientras haya alguno pendiente, la telemetria sale cada
 * ATACANTE_REPORT_PERIOD_MS (2s) en vez de LORA_EXERCISE_PERIOD_MS (10s),
 * para vaciar la cola mas rapido sin tener que mandar un frame aparte. */
#define ATACANTE_BUFFER_MAX       50U
#define ATACANTE_REPORT_PERIOD_MS 2000U
/* LoRaWAN Clase A (AT+QCLASS=A): solo se puede recibir downlink justo
 * despues de mandar un uplink — el gateway no puede "empujar" nada, tiene
 * que esperar a que nosotros transmitamos. Por eso TESTLORA funciona como
 * "abrir ventana de recepcion" durante MODO_CONFIGURACION: entre mas
 * seguido se manda, mas rapido llega el $CONF/$RUN real. 500ms es
 * demasiado agresivo (airtime, limites del network server, bateria) —
 * 2s es buen balance, mucho mas responsivo que los 30s de antes sin
 * saturar nada. En MODO_EJERCICIO no hace falta nada de esto: la
 * telemetria de cada LORA_EXERCISE_PERIOD_MS ya abre su propia ventana. */
#define TESTLORA_PERIOD_MS   15000U    /**< Cada cuanto se manda "TESTLORA" mientras estamos en MODO_CONFIGURACION */

/* En TEST_SN_LORA (2026-09-15) es al reves de la tarjeta de pruebas
 * aislada de LoRa: aqui SI hay GPS y sensores reales conectados, lo unico
 * que falta es el modulo LoRa fisico — por eso GpsTask/SensorsTask vuelven
 * a 1U en esta rama. TESTLORA se queda igual (dentro del loop de LoraTask,
 * sin tarea aparte) porque ese envio no depende de si hay sensores o no. */
#define TASK_GPS_ENABLE          1U
#define TASK_SENSORS_ENABLE      1U

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
 * una, sin necesidad de mutex... EXCEPTO s_lora_conf_listo: BluetoothTask
 * tambien la pone en false si se agota BT_ACKCON_TIMEOUT_MS sin conectar
 * (ver "retomar_inicio"), para volver a esperar un $CONF fresco en vez
 * del mismo de antes. Ventana de carrera teorica si un $CONF nuevo llega
 * justo en ese instante, pero es un bool simple (escritura atomica en
 * Cortex-M) y el caso es raro/no critico — en el peor caso se espera al
 * siguiente $CONF. */
static volatile bool s_lora_conf_listo       = false;
static volatile bool s_lora_enviar_telemetria = false;

/* Pausa el TESTLORA periodico entre que llega el $CONF real y que se manda
 * la telemetria de vuelta con ack=1/0 (tras ACKCONF/timeout) — evita que
 * el TESTLORA "dummy" se pegue con ese envio real en el modulo (bug real
 * visto con hardware: con TESTLORA cada pocos segundos, algun envio se
 * perdia/mezclaba si caia justo junto al de la Mira). Se marca true al
 * parsear el CONF (Lora_ProcesarLinea) y false otra vez cuando ya se mando
 * la telemetria real (LoraTask, junto con s_lora_enviar_telemetria). */
static volatile bool s_lora_testlora_pausado  = false;

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

/* CalibrateTask marca esta bandera cuando un impacto real valido deja las
 * vidas en 0 — LoraTask la revisa para mandar $END_M al gateway y correr
 * Lora_ManejarFinPorVidas() (ver ahi). Mismo patron volatile de siempre. */
static volatile bool s_vidas_agotadas = false;

/* LoraTask (dentro de Lora_ManejarFinPorVidas()) la marca en false antes
 * de mandar $END_M y espera a que Lora_ProcesarLinea() la ponga en true al
 * ver "ACKEND_M" — mismo patron que las demas banderas de ack por LoRa. */
static volatile bool s_lora_ackend_m_recibido = false;

/* Cola circular de atacantes pendientes de reportar — CalibrateTask
 * escribe (productor, un disparo valido a la vez), LoraTask lee/consume
 * (consumidor, uno por cada telemetria que manda). Mismo patron que la
 * cola de lineas de Lora_StoreBytes()/Lora_PopLine(): un solo escritor, un
 * solo lector, sin mutex. Si se llena (50 pendientes sin mandar), los
 * disparos nuevos se descartan — mejor perder el reporte de uno viejo que
 * trabarse esperando espacio. */
static uint8_t s_atacante_buffer[ATACANTE_BUFFER_MAX];
static uint8_t s_atacante_head  = 0U;   /**< Proximo indice a escribir (CalibrateTask) */
static uint8_t s_atacante_tail  = 0U;   /**< Proximo indice a leer (LoraTask)           */
static uint8_t s_atacante_count = 0U;   /**< Cuantos atacantes hay pendientes de mandar */

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
    /* Anuncio de arranque QUITADO (2026-09-15) — ya sale en el bloque
     * consolidado de Tareas_CrearTareas(), antes de osKernelStart(). */

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
    if (strncmp(line, "+QEVT:", 6U) != 0) {
        /* eco de comando, "OK", "NO_NETWORK_JOINED", "+QEVT:SEND_CONFIRMED",
         * etc. — nada que hacer, y en modo normal ni se loguea (ver
         * LORA_VERBOSE_LOG en Lora.h) porque no trae datos del gateway. */
#if LORA_VERBOSE_LOG
        Log_Printf("LORA", "[DEBUG] Linea RX: %s", line);
#endif
        return;
    }

    Log_Printf("LORA", "Downlink recibido: %s", line);

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

    char *conf_pos     = strstr(decoded_str, "CONF");
    char *run_pos      = strstr(decoded_str, "RUN");
    char *end_s_pos    = strstr(decoded_str, "END_S");
    char *ackend_m_pos = strstr(decoded_str, "ACKEND_M");   /* respuesta a NUESTRO $END_M, ver Lora_ManejarFinPorVidas() */

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
            /* Pausa el TESTLORA periodico hasta que se mande la telemetria
             * real (ver s_lora_testlora_pausado) — evita que se pegue con
             * el handshake de Bluetooth/el envio real del ack. */
            s_lora_testlora_pausado = true;
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
    } else if (ackend_m_pos != NULL) {
        /* Respuesta a nuestro propio $END_M (ver Lora_ManejarFinPorVidas(),
         * que es quien esta esperando esto en su propio loop bloqueante) —
         * solo se marca la bandera, el manejo real vive alla. */
        Log_Print("LORA", "ACKEND_M recibido.");
        s_lora_ackend_m_recibido = true;
    } else {
        Log_Printf("LORA", "Payload de +QEVT sin CONF/RUN/END_S/ACKEND_M reconocido: %s", decoded_str);
    }
}

/* [PRUEBA] Simulacion de movimiento para ver cambios en Unity mientras
 * GpsTask esta deshabilitada (TASK_GPS_ENABLE=0) — punto de partida las
 * coordenadas reales que dio el usuario (su companero de pruebas). Paso
 * chico a proposito (~5m por envio) para que se note un desplazamiento
 * pequeno en el mapa, no un salto brusco. SOLO se aplica a latitud/
 * longitud cuando TASK_GPS_ENABLE==0 (tarjeta de pruebas sin GPS real) —
 * en TEST_SN_LORA (2026-09-15), con GpsTask habilitada y GPS real
 * conectado, esto se deshabilita para no pisar el fix real con datos
 * falsos; timestamp/orientacion/pasos se siguen llenando aqui de todos
 * modos porque orientacion/pasos todavia no se calculan de verdad (ver
 * TODO en Inicializacion.h). */
#define SIM_GPS_LAT_INICIAL     19.436408
#define SIM_GPS_LON_INICIAL    -99.176603
#define SIM_GPS_STEP_DEG          0.00005   /* ~5m por envio a esta latitud */

#if TASK_GPS_ENABLE
/**
 * @brief  Convierte year/month/day/hour/minute/second del GPS a timestamp
 *         Unix (segundos UTC desde 1970-01-01), usando el algoritmo de
 *         calendario de Howard Hinnant (dominio publico, solo enteros, sin
 *         floats). GPS.c guarda esos campos ya en HORA LOCAL (les aplica
 *         Gps_GetTimezoneOffset() antes de entregarlos, ver Gps_Process())
 *         — aqui se le resta ese mismo offset de vuelta para recuperar
 *         UTC real, que es lo que necesita un timestamp Unix de verdad
 *         (2026-09-15, decidido con el usuario: UTC, no hora local).
 * @param  h  Handle de GPS (normalmente &s_gps_task) — se asume que ya se
 *            confirmo h->data.datetime_valid antes de llamar esto.
 * @retval Timestamp Unix en UTC.
 */
static uint32_t Gps_ComputeEpochUTC(const Gps_Handle_t *h)
{
    int32_t y = (int32_t)h->data.year;
    int32_t m = (int32_t)h->data.month;
    int32_t d = (int32_t)h->data.day;

    y -= (m <= 2) ? 1 : 0;
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    uint32_t yoe = (uint32_t)(y - era * 400);
    uint32_t doy = (uint32_t)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    int32_t  dias_desde_epoch = era * 146097 + (int32_t)doe - 719468;

    uint32_t epoch_local = (uint32_t)dias_desde_epoch * 86400U
                          + (uint32_t)h->data.hour   * 3600U
                          + (uint32_t)h->data.minute * 60U
                          + (uint32_t)h->data.second;

    int8_t  offset_h  = Gps_GetTimezoneOffset(h->data.longitude);
    int32_t epoch_utc = (int32_t)epoch_local - ((int32_t)offset_h * 3600);

    return (uint32_t)epoch_utc;
}
#endif

/**
 * @brief  Llena g_exercise_data.timestamp/orientacion/pasos (y
 *         latitud/longitud si TASK_GPS_ENABLE==0, ver nota arriba) antes
 *         de cada telemetria.
 * @note   timestamp: con GPS real y fix valido (h->data.datetime_valid),
 *         usa el Unix real en UTC (Gps_ComputeEpochUTC()). Sin fix
 *         todavia (o en la tarjeta de pruebas sin GPS), sigue un contador
 *         de 1 en 1 — decidido con el usuario 2026-09-15: la app espera
 *         que este campo SIEMPRE cambie entre envios para refrescar la
 *         UI, aunque no sea un timestamp real todavia.
 */
static void Lora_SimularMovimiento(void)
{
    static bool     inicializado = false;
    static uint32_t ts_sim = 0U;

    if (!inicializado) {
#if !TASK_GPS_ENABLE
        g_exercise_data.latitud  = SIM_GPS_LAT_INICIAL;
        g_exercise_data.longitud = SIM_GPS_LON_INICIAL;
#endif
        ts_sim = HAL_GetTick() / 1000U;
        inicializado = true;
    } else {
#if !TASK_GPS_ENABLE
        g_exercise_data.latitud  += SIM_GPS_STEP_DEG;   /* un poco hacia arriba (norte) */
        g_exercise_data.longitud += SIM_GPS_STEP_DEG;   /* un poco hacia la derecha (este) */
#endif
        ts_sim++;
    }

#if TASK_GPS_ENABLE
    if (s_gps_task.data.datetime_valid) {
        g_exercise_data.timestamp = Gps_ComputeEpochUTC(&s_gps_task);
    } else {
        g_exercise_data.timestamp = ts_sim;
    }
#else
    g_exercise_data.timestamp = ts_sim;
#endif

    /* orientacion y pasos: YA NO se simulan — Orientation_Update() y
     * Steps_Update() en SensorsTask (magnetometro/acelerometro reales)
     * llenan g_exercise_data.orientacion/pasos. orientacion sigue sin
     * calibrar (ver Orientacion_e en Inicializacion.h), pendiente a
     * proposito. */
}

/**
 * @brief  Manda la telemetria de 14 campos de vuelta al gateway
 *         (ID,numOrden,vidas,municion,bateria,latitud,longitud,altitud,
 *         orientacion,pasos,ack,timestamp,bateria_ap,atacante_numero_lora
 *         — latitud ANTES que longitud, bateria_ap y atacante_numero_lora
 *         agregados al final el 2026-09-15, ver g_exercise_data),
 *         codificada a hex sobre AT+QSEND=1:1:<hex>\r\n (igual que
 *         mandarPorLora() del codigo de referencia).
 * @note   atacante_numero_lora: 0 significa "nadie nos ataco en este
 *         envio" — si hay atacantes pendientes en s_atacante_buffer, aqui
 *         viene el numero/canal LoRa de UNO de ellos (uno por envio, ver
 *         LoraTask). Con la cola llena la telemetria se manda cada
 *         ATACANTE_REPORT_PERIOD_MS en vez de LORA_EXERCISE_PERIOD_MS.
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
    /* Orden lat/lon: LATITUD antes que LONGITUD — asi lo manda de verdad
     * generarCadena() del codigo de referencia del companero (CODIGO_LORA/
     * lora_kg200z.c), aunque su propio comentario diga "longitud,latitud"
     * (comentario desactualizado, el codigo real pasa latStr primero).
     * bateria_ap y atacante_numero_lora se agregan al final (campos 13 y
     * 14, 2026-09-15) — el resto de los 12 campos originales no cambia de
     * lugar. */
    int  n = snprintf(csv, sizeof(csv), "%u,%u,%u,%u,%u,%.5f,%.5f,%.1f,%u,%u,%u,%lu,%u,%u",
                       g_exercise_data.lora, g_exercise_data.orden,
                       g_exercise_data.lives, g_exercise_data.ammo,
                       g_exercise_data.bateria_ch,
                       (double)g_exercise_data.latitud, (double)g_exercise_data.longitud,
                       (double)g_exercise_data.altitud,
                       g_exercise_data.orientacion, g_exercise_data.pasos,
                       g_exercise_data.ack, (unsigned long)g_exercise_data.timestamp,
                       g_exercise_data.bateria_ap, g_exercise_data.atacante_numero_lora);

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
 * @note   Se pausa (ver s_lora_testlora_pausado) entre que llega el $CONF
 *         real y que se manda la telemetria de vuelta con el ack — bug
 *         real visto con hardware: si el TESTLORA caia muy pegado al envio
 *         real del ack, el modulo a veces perdia/mezclaba uno de los dos.
 */
static void Lora_EnviarTestLora(void)
{
    static const char testlora_msg[] = "TESTLORA";
    /* Log solo la PRIMERA vez (2026-09-15, pedido del usuario) — se manda
     * cada TESTLORA_PERIOD_MS mientras dure MODO_CONFIGURACION, imprimirlo
     * cada vez ensuciaba el logger sin aportar nada nuevo. Si hace falta
     * ver cada envio a detalle, usar Lora_DebugDump() (ver Lora.h). */
    static bool primera_vez = true;

    char hex_payload[(sizeof(testlora_msg) * 2U) + 1U];
    Lora_EncodeToHex((const uint8_t *)testlora_msg, sizeof(testlora_msg) - 1U, hex_payload);

    char cmd[sizeof(hex_payload) + 16U];
    int  cmd_len = snprintf(cmd, sizeof(cmd), "AT+QSEND=1:1:%s\r\n", hex_payload);
    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        Log_Print("LORA", "ERROR: comando AT+QSEND de TESTLORA demasiado grande.");
        return;
    }

    Lora_Transmit(&s_lora_task, (const uint8_t *)cmd, (uint16_t)cmd_len);
    if (primera_vez) {
        Log_Print("LORA", "Iniciando envio periodico de TESTLORA (para leer cola de ChirpStack)...");
        primera_vez = false;
    }
}

/**
 * @brief  Manda "END_M" (texto plano, codificado a hex sobre AT+QSEND, sin
 *         framing $/*** — igual que TESTLORA, ya no hace falta desde el
 *         rewrite a lineas +QEVT). Avisa al gateway que este jugador se
 *         quedo sin vidas.
 */
static void Lora_EnviarEndM(void)
{
    static const char end_m_msg[] = "END_M";

    char hex_payload[(sizeof(end_m_msg) * 2U) + 1U];
    Lora_EncodeToHex((const uint8_t *)end_m_msg, sizeof(end_m_msg) - 1U, hex_payload);

    char cmd[sizeof(hex_payload) + 16U];
    int  cmd_len = snprintf(cmd, sizeof(cmd), "AT+QSEND=1:1:%s\r\n", hex_payload);
    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        Log_Print("LORA", "ERROR: comando AT+QSEND de END_M demasiado grande.");
        return;
    }

    Lora_Transmit(&s_lora_task, (const uint8_t *)cmd, (uint16_t)cmd_len);
    Log_Print("LORA", "END_M enviado (nos quedamos sin vidas).");
}

/**
 * @brief  Fin de ejercicio POR NOSOTROS (vidas en 0, a diferencia de
 *         $END_S que lo ordena el administrador): manda "END_M" al
 *         gateway y espera "ACKEND_M" (sin timeout — mismo criterio que
 *         ACKRUN/ACKEND_S, el gateway ya sabe que nos quedamos sin vidas,
 *         no tiene caso abandonar la espera). Mientras espera, sigue
 *         drenando y procesando cualquier otra linea que llegue (igual que
 *         el loop principal de LoraTask). Al confirmarse:
 *         manda la telemetria final (datos ya actualizados: vidas=0,
 *         ultima posicion, etc.), regresa a MODO_CONFIGURACION, y hace el
 *         mismo parpadeo colorido de fin de juego que BT_HandleEndA()
 *         (Leds_ParpadeoFinJuego()).
 * @note   Llamada desde el loop principal de LoraTask cuando CalibrateTask
 *         marca s_vidas_agotadas — ver ahi.
 */
static void Lora_ManejarFinPorVidas(void)
{
    s_lora_ackend_m_recibido = false;
    Lora_EnviarEndM();

    char linea[LORA_LINE_MAX_LEN];
    while (!s_lora_ackend_m_recibido) {
        while (Lora_PopLine(&s_lora_task, linea, sizeof(linea))) {
            Lora_ProcesarLinea(linea);
        }
        osDelay(20U);
    }

    Log_Print("LORA", "ACKEND_M confirmado — mandando telemetria final.");
    Lora_EnviarTelemetria();

    Modo_SetOperacion(MODO_CONFIGURACION);
    Log_Print("LORA", "MODO_CONFIGURACION activo — nos quedamos sin vidas, ejercicio terminado.");

    Leds_ParpadeoFinJuego();
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
 *         avisa a BluetoothTask (s_lora_end_s_recibido, fin de ejercicio
 *         por orden del administrador); "ACKEND_M" -> marca
 *         s_lora_ackend_m_recibido (respuesta a nuestro propio "END_M", ver
 *         Lora_ManejarFinPorVidas() — fin de ejercicio porque a NOSOTROS se
 *         nos acabaron las vidas, disparado por CalibrateTask via
 *         s_vidas_agotadas, mas abajo en este mismo loop).
 * @note   La telemetria de 12 campos (Lora_EnviarTelemetria()) se manda en
 *         dos casos: una vez cuando BluetoothTask marca
 *         s_lora_enviar_telemetria (tras ACKCONF), y periodica cada
 *         LORA_EXERCISE_PERIOD_MS mientras g_modo_operacion==MODO_EJERCICIO.
 */
static void LoraTask(void *argument)
{
    (void)argument;

    /* Reporte de heap/handles y anuncio de arranque QUITADOS de aqui
     * (2026-09-15) — ahora salen en el bloque consolidado de
     * Tareas_CrearTareas(), impreso ANTES de osKernelStart() (deterministico,
     * sin pelearse con el orden real en que el scheduler arranca cada
     * tarea). */

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
     * de RUN) y no este pausado (ver s_lora_testlora_pausado), se manda
     * "TESTLORA" — ver Lora_EnviarTestLora(). En cuanto entra MODO_EJERCICIO
     * esto se detiene solo (la condicion de abajo deja de cumplirse) y la
     * telemetria real de arriba toma el relevo. */
    uint32_t last_testlora_tx = 0U;

    for (;;) {
        /* Mientras haya atacantes pendientes de reportar, la telemetria
         * sale cada ATACANTE_REPORT_PERIOD_MS (2s) en vez de
         * LORA_EXERCISE_PERIOD_MS (10s), para vaciar la cola mas rapido —
         * un atacante por envio. Sin pendientes, cadencia normal. */
        uint32_t periodo_actual = (s_atacante_count > 0U) ? ATACANTE_REPORT_PERIOD_MS : LORA_EXERCISE_PERIOD_MS;

        if ((g_modo_operacion == MODO_EJERCICIO) &&
            ((HAL_GetTick() - last_exercise_tx) >= periodo_actual)) {
            last_exercise_tx = HAL_GetTick();

            if (s_atacante_count > 0U) {
                g_exercise_data.atacante_numero_lora = s_atacante_buffer[s_atacante_tail];
                s_atacante_tail = (uint8_t)((s_atacante_tail + 1U) % ATACANTE_BUFFER_MAX);
                s_atacante_count--;
            } else {
                g_exercise_data.atacante_numero_lora = 0U;   /* 0 = nadie nos ataco en este envio */
            }

            Lora_EnviarTelemetria();
        }

        if ((g_modo_operacion == MODO_CONFIGURACION) && !s_lora_testlora_pausado &&
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

            /* Ya se mando la telemetria real (ack=1/0 tras ACKCONF/timeout)
             * — se retoma el TESTLORA periodico, pero con el cronometro
             * reiniciado para que el siguiente salga hasta dentro de un
             * TESTLORA_PERIOD_MS completo, no pegado a este envio. */
            s_lora_testlora_pausado = false;
            last_testlora_tx = HAL_GetTick();
        }

        if (s_vidas_agotadas) {
            s_vidas_agotadas = false;
            Lora_ManejarFinPorVidas();
        }

        osDelay(50U);
    }
}

/**
 * @brief  Calcula el heading (angulo respecto al norte magnetico) a partir
 *         de la ultima lectura del magnetometro, y lo reduce a uno de los
 *         8 sectores cardinales de Orientacion_e (ver Inicializacion.h).
 *         Llenado en g_exercise_data.orientacion.
 * @param  mag  Ultima lectura del MMC5983MA (ver MMC_Data_t).
 * @note   PENDIENTE de afinar (2026-09-15, decidido con el usuario): no hay
 *         calibracion de hard/soft-iron ni compensacion de tilt (la
 *         calibracion que existe es con la tarjeta plana; el montaje real
 *         va a ser vertical, pegado al pecho) ni correccion de declinacion
 *         magnetica (heading magnetico, no geografico). Formula estandar
 *         de brujula 2D: atan2(mag_y, mag_x) — asume la tarjeta plana
 *         respecto al suelo, que es justo lo que va a dejar de ser cierto
 *         en el montaje final. Se manda de todos modos para ver el
 *         comportamiento en pruebas reales; el numero puede salir raro.
 */
static void Orientation_Update(const MMC_Data_t *mag)
{
    float heading_deg = atan2f(mag->y_uT, mag->x_uT) * (180.0f / 3.14159265f);
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }

    uint16_t sector = (uint16_t)(((heading_deg + 22.5f) / 45.0f)) % 8U;
    g_exercise_data.orientacion = sector;
}

/**
 * @brief  Detector de pasos por histeresis sobre la magnitud del vector de
 *         aceleracion. Llamar una vez por cada lectura nueva de IMU (cada
 *         SENSORS_PERIOD_MS, ver SensorsTask) — NO es una funcion pura,
 *         mantiene el estado del "armado" entre llamadas via static.
 * @param  accel_mag_g  Magnitud del vector accel de la ultima lectura, en g.
 * @note   Primer paso: solo el acelerometro (ver STEP_THRESHOLD_HIGH_G/LOW_G
 *         arriba) — el usuario decidio dejar orientacion (magnetometro)
 *         pendiente para despues, 2026-09-15. Umbrales sin validar todavia
 *         con caminata real, ajustar una vez que se pruebe en persona.
 */
static void Steps_Update(float accel_mag_g)
{
    static bool     armado    = true;   /* true = listo para contar el siguiente pico */
    static uint32_t last_tick = 0U;

    uint32_t now = HAL_GetTick();

    if (armado && accel_mag_g >= STEP_THRESHOLD_HIGH_G
        && (now - last_tick) >= STEP_MIN_INTERVAL_MS) {
        g_exercise_data.pasos++;
        armado    = false;
        last_tick = now;
        Log_Printf("STEPS", "************** PASO detectado (|v|=%.3fg) — total=%u **************",
                   (double)accel_mag_g, g_exercise_data.pasos);
    } else if (!armado && accel_mag_g <= STEP_THRESHOLD_LOW_G) {
        armado = true;
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

    /* Anuncio de arranque QUITADO (2026-09-15, pedido del usuario) — ya
     * sale en el bloque consolidado de Tareas_CrearTareas(), es repetitivo
     * anunciarlo otra vez aqui. */

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

    /* Prescaler de ciclos (2026-09-15) — NO se crea una tarea aparte para
     * luz/bateria/magnetometro (el heap de FreeRTOS ya anda muy justo, ver
     * "Heap libre tras crear tareas" en el log de arranque) ni un osTimer
     * (tambien consume su propia tarea/stack de servicio). En vez de eso,
     * un contador simple dentro del mismo ciclo de SensorsTask: el
     * acelerometro (pasos) se lee SIEMPRE, cada SENSORS_PERIOD_MS, porque
     * eso si necesita la cadencia rapida; luz/bateria/mag cambian lento y
     * de momento no tienen consumidor urgente (mag es para orientacion,
     * pendiente a proposito) asi que se leen cada N ciclos nada mas. */
    static uint32_t s_sensors_cycle = 0U;
    const uint32_t LUX_EVERY_N_CYCLES  = 50U;   /* ~5s a 100ms/ciclo   */
    const uint32_t BAT_EVERY_N_CYCLES  = 50U;   /* ~5s a 100ms/ciclo   */
    const uint32_t MAG_EVERY_N_CYCLES  = 10U;   /* ~1s a 100ms/ciclo   */

    for (;;) {
        s_sensors_cycle++;

        if ((s_sensors_cycle % LUX_EVERY_N_CYCLES) == 0U) {
            float lux = 0.0f;
            TSL2571_RawData_t light_raw;
            I2C1Bus_Lock();
            HAL_StatusTypeDef lux_st = TSL2571_ReadLux(&s_light_task, 1U, 200U, &lux, &light_raw);
            I2C1Bus_Unlock();
            if (lux_st == HAL_OK) {
                g_exercise_data.lux = lux;
            }
        }

        if ((s_sensors_cycle % BAT_EVERY_N_CYCLES) == 0U) {
            BatGauge_Data_t bat_data;
            I2C1Bus_Lock();
            BatGauge_Update(&bat_data);
            I2C1Bus_Unlock();
            if (bat_data.is_ready) {
                g_exercise_data.bateria_ch = bat_data.soc_pct;   /* bateria de ESTA tarjeta */
            }
        }

        LSM_Data_t imu_data;
        bool imu_read_ok = (LSM6DSO32TR_ReadAll(&s_imu_task, &imu_data) == LSM_OK);
        if (imu_read_ok) {
            g_exercise_data.gyro_x_dps = imu_data.gx_dps;
            g_exercise_data.gyro_y_dps = imu_data.gy_dps;
            g_exercise_data.gyro_z_dps = imu_data.gz_dps;
        }

        bool mag_read_ok = false;
        MMC_Data_t mag_data = {0};
        if ((s_sensors_cycle % MAG_EVERY_N_CYCLES) == 0U) {
            mag_read_ok = (MMC5983MA_ReadAll(&mag_data) == MMC_OK);
            if (mag_read_ok) {
                g_exercise_data.mag_x_uT = mag_data.x_uT;
                g_exercise_data.mag_y_uT = mag_data.y_uT;
                g_exercise_data.mag_z_uT = mag_data.z_uT;
            }
        }

        /* Logs de detalle (accel/gyro/mag crudos) QUITADOS a proposito
         * (2026-09-15) — ya sirvieron para validar que los sensores dan
         * lecturas coherentes, ahora solo ensuciaban el logger. El unico
         * log que queda de este ciclo es el de Steps_Update() cuando
         * detecta un paso (tag [STEPS], con los asteriscos). */
        if (imu_read_ok) {
            float accel_mag_g = sqrtf(imu_data.ax_g * imu_data.ax_g +
                                       imu_data.ay_g * imu_data.ay_g +
                                       imu_data.az_g * imu_data.az_g);
            Steps_Update(accel_mag_g);
        }

        if (mag_read_ok) {
            Orientation_Update(&mag_data);
        }

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
 *           - MODO_EJERCICIO: disparo real — la trama trae 2 palabras de
 *             16 bits, "dato" (frame_buf[0..1]) y "ash" (frame_buf[2..3]),
 *             la misma informacion mandada por duplicado como verificacion
 *             (sin formula de hash real, es solo redundancia). Valido
 *             solo si dato == ash; en ese caso se descuenta 1 VIDA
 *             (g_exercise_data.lives) y se reporta por los 2 canales:
 *             Bluetooth ("$A_SN<vidas>\r" a la Mira, vidas YA actualizada)
 *             y LoRa (se encola "dato" — el numero/canal de quien nos
 *             disparo — en s_atacante_buffer, para que LoraTask lo mande
 *             en el campo atacante_numero_lora de la siguiente telemetria,
 *             ver ATACANTE_REPORT_PERIOD_MS). Si dato != ash, se descarta
 *             (log de todos modos, para depurar). Si tras esto las vidas
 *             ya estan en 0, se marca s_vidas_agotadas para que LoraTask
 *             mande $END_M al gateway (ver Lora_ManejarFinPorVidas()).
 */
static void CalibrateTask(void *argument)
{
    (void)argument;

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
                    /* MODO_EJERCICIO — disparo real, ver nota arriba. */
                    uint16_t dato = 0U;
                    uint16_t ash  = 0U;
                    if (ir_handle.frame_len >= 4U) {
                        dato = ((uint16_t)ir_handle.frame_buf[0] << 8U) | ir_handle.frame_buf[1];
                        ash  = ((uint16_t)ir_handle.frame_buf[2] << 8U) | ir_handle.frame_buf[3];
                    }

                    Log_Printf("CALIB", "Disparo: dato=0x%04X ash=0x%04X", dato, ash);

                    if (ir_handle.frame_len >= 4U && dato == ash) {
                        if (g_exercise_data.lives > 0U) {
                            g_exercise_data.lives--;
                        }
                        Log_Printf("CALIB", "Disparo valido — vidas restantes=%u",
                                   g_exercise_data.lives);

                        char    asn_msg[24];
                        int32_t asn_len = snprintf(asn_msg, sizeof(asn_msg), "$A_SN%u\r", g_exercise_data.lives);
                        if (asn_len > 0 && (size_t)asn_len < sizeof(asn_msg)) {
                            Bt_Transmit(&s_bt_task, (uint8_t *)asn_msg, (uint16_t)asn_len);
                        }

                        /* Encola al atacante (el "dato" recibido ES su
                         * numero/canal LoRa) para que LoraTask lo reporte
                         * en el campo atacante_numero_lora de la proxima
                         * telemetria — ver ATACANTE_BUFFER_MAX arriba. */
                        if (s_atacante_count < ATACANTE_BUFFER_MAX) {
                            s_atacante_buffer[s_atacante_head] = (uint8_t)dato;
                            s_atacante_head = (uint8_t)((s_atacante_head + 1U) % ATACANTE_BUFFER_MAX);
                            s_atacante_count++;
                        } else {
                            Log_Print("CALIB", "ERROR: buffer de atacantes lleno (50) — se descarta este.");
                        }

                        if (g_exercise_data.lives == 0U) {
                            s_vidas_agotadas = true;
                        }
                    } else {
                        Log_Print("CALIB", "Disparo invalido (dato != ash o trama incompleta) — descartado.");
                    }
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
 *         estabamos esperando. Responde $ACKDSCON y espera
 *         BT_DSCON_RECONNECT_WINDOW_MS (5s) a ver si el modulo se
 *         reconecta solo — SOLO si esa ventana se agota SIN reconectar se
 *         parpadea cian (verde+azul) LEDS_BT_DSCON_COUNT veces
 *         (LEDS_BT_DSCON_MS) y ahi si se da por vencido (decidido con el
 *         usuario 2026-09-15: nada de parpadear antes de confirmar que de
 *         verdad se perdio la conexion — homologado con la secuencia
 *         equivalente de la Mira, solo que ahora condicionado).
 * @note   Bug real encontrado con hardware 2026-09-10: el modulo BL654
 *         (SensoresM.sb) ya reintenta la conexion BLE SOLO tras un corte
 *         breve de RF (su propio TimerStart(2, RETRY_DELAY_MS, 0) en el
 *         manejador de desconexion) — confirmado viendo un "$ACKCON"
 *         nuevo llegar segundos despues de un "$DSCON", sin que nosotros
 *         hicieramos nada. Antes esta funcion se quedaba inerte para
 *         siempre apenas veia un DSCON, ignorando ese reintento automatico
 *         del modulo — por eso "nunca hubo una desconexion real" del otro
 *         lado (la Mira/kit) pero nosotros si nos dabamos por vencidos.
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
    Log_Print("BT", "DSCON recibido — esperando reconexion automatica del modulo (sin parpadear todavia)...");

    s_bt_conectado = false;
    /* s_ack_pendiente NO se toca aqui a proposito — es del caller
     * (BT_HandleRun()/BT_HandleEndS() tienen su propio while que depende
     * de el; tocarlo aqui rompia esos loops si esta funcion llegaba a
     * regresar). */

    /* Nada de parpadear "por si acaso" — solo si se confirma que de
     * verdad no hubo reconexion (ver abajo). Decidido con el usuario
     * 2026-09-15. */
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < BT_DSCON_RECONNECT_WINDOW_MS) {
        if (s_bt_task.rx_ready) {
            if (strcmp((char *)s_bt_task.rx_buffer, "ACKCON") == 0) {
                Bt_ResetRx(&s_bt_task);
                Log_Print("BT", "Reconectado solo (ACKCON nuevo tras DSCON) — retomando enlace, sin parpadeo.");
                Leds_SetAzul(true);
                s_bt_conectado = true;
                return true;
            }
            Bt_ResetRx(&s_bt_task);
        }
        osDelay(20U);
    }

    Log_Print("BT", "No se reconecto a tiempo — desconexion real, parpadeo cian y BluetoothTask se queda inerte.");
    BT_Blink(false, true, true, LEDS_BT_DSCON_MS, LEDS_BT_DSCON_COUNT);   /* cian = verde+azul */
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
 * @note   Secuencia (actualizada 2026-09-15):
 *         retomar_inicio: espera s_lora_conf_listo (LoraTask ya recibio y
 *         guardo un $CONF real) -> $CON<mac> -> espera "ACKCON" hasta
 *         BT_ACKCON_TIMEOUT_MS (60s; si llega otra cosa, parpadeo rojo
 *         3x400ms y se sigue esperando). Si se agota sin ACKCON: nunca
 *         conecto por BLE — se manda "$CANCELCON\r" al modulo (deja de
 *         insistir en conectarse), se avisa al gateway con
 *         ack=BT_GW_ACK_SIN_CONEXION, y se regresa a retomar_inicio a
 *         esperar un $CONF fresco (sin mensaje de "reintenta", se trata
 *         igual que la primera conexion).
 *         Si SI llega ACKCON: azul fijo (enlazados) -> espera "READY" ->
 *         $CONF<datos> (solo mac1, mac2 no se manda) -> espera "ACKCONF"
 *         hasta BT_ACKCONF_TIMEOUT_MS, un solo intento (sin reintentar el
 *         envio, decision del usuario 2026-09-15: caso raro) -> loop de
 *         escucha para siempre. Si nunca llega ACKCONF: se avisa al
 *         gateway con ack=BT_GW_ACK_SIN_ACKCONF, sin ninguna accion extra
 *         a proposito (ver su doc comment en Bluetooth.h).
 *         "$DSCON" se atiende SIEMPRE, en cualquier punto de la secuencia
 *         (ver BT_HandleDSCON()) — espera BT_DSCON_RECONNECT_WINDOW_MS
 *         (5s) a ver si el modulo se reconecta solo (SensoresM.sb ya
 *         reintenta la conexion BLE por su cuenta tras un corte breve de
 *         RF), SIN parpadear todavia. Si reconecta (llega un "$ACKCON"
 *         nuevo), retoma el flujo desde "esperando READY" (goto
 *         retomar_ready) sin haber parpadeado nada; si no reconecta en
 *         esa ventana, ahi si parpadea cian y se queda inerte para
 *         siempre (desconexion real, sin manejo de reconexion por
 *         lejania — pendiente a proposito).
 */
static void BluetoothTask(void *argument)
{
    (void)argument;

    Bt_Init(&s_bt_task);

    s_bt_conectado  = false;
    s_ack_pendiente = ACK_NINGUNO;

    /* retomar_inicio: aqui regresa el flujo si se agota BT_ACKCON_TIMEOUT_MS
     * sin ACKCON (ver abajo) — se trata exactamente como si fuera la
     * primera conexion: espera un $CONF fresco por LoRa, sin ningun
     * mensaje especial de "reintenta ahora" (decidido con el usuario
     * 2026-09-15). */
retomar_inicio:
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
    Log_Printf("BT", "CON enviado, esperando ACKCON (hasta %lus)...", (unsigned long)(BT_ACKCON_TIMEOUT_MS / 1000U));

    bool     led_on      = false;
    uint32_t last_blink  = HAL_GetTick();
    uint32_t ackcon_start = HAL_GetTick();
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

        if ((HAL_GetTick() - ackcon_start) >= BT_ACKCON_TIMEOUT_MS) {
            /* Nunca conecto por BLE (MAC mal registrada, dispositivo
             * elegido mal, etc.) — le decimos al modulo que deje de
             * insistir, avisamos al gateway con ack=2, y regresamos al
             * principio a esperar un $CONF fresco. */
            static const uint8_t cancelcon[] = "$CANCELCON\r";
            Bt_Transmit(&s_bt_task, cancelcon, sizeof(cancelcon) - 1U);
            Log_Print("BT", "ERROR: nunca llego ACKCON — $CANCELCON enviado, avisando al gateway (ack=2).");

            s_ack_pendiente = ACK_NINGUNO;
            Leds_Apagar();
            g_exercise_data.ack     = (uint8_t)BT_GW_ACK_SIN_CONEXION;
            s_lora_enviar_telemetria = true;
            s_lora_conf_listo        = false;   /* esperar un $CONF nuevo, no el mismo de antes */
            goto retomar_inicio;
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
     * primera MAC (mac2 se guarda pero no se manda). Un solo intento — el
     * reintento (reenviar $CONF si no llega ACKCONF) se quito a proposito
     * (2026-09-15, decision del usuario): caso raro, no vale la pena la
     * complejidad extra por ahora. */
    char    payload[160];
    int32_t n = snprintf(payload, sizeof(payload), "$CONF%u,%u,%s,%s,%u,%u,%lu,%s\r",
                          g_exercise_data.orden, g_exercise_data.lora,
                          g_exercise_data.team_name, g_exercise_data.player_name,
                          g_exercise_data.lives, g_exercise_data.ammo,
                          (unsigned long)g_exercise_data.tiempo, g_exercise_data.mac);

    if (n > 0 && (size_t)n < sizeof(payload)) {
        Bt_Transmit(&s_bt_task, (uint8_t *)payload, (uint16_t)n);
        Log_Print("BT", "CONF enviado, esperando ACKCONF...");
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
         * mandar la telemetria de vuelta al gateway con este estado. */
        g_exercise_data.ack   = (uint8_t)BT_GW_ACK_CONFIRMADO;
        s_lora_enviar_telemetria = true;
    } else {
        s_ack_pendiente = ACK_NINGUNO;
        Log_Print("BT", "ERROR: no llego ACKCONF a tiempo — se avisa al gateway.");
        g_exercise_data.ack   = (uint8_t)BT_GW_ACK_SIN_ACKCONF;
        s_lora_enviar_telemetria = true;
        /* TODO: pendiente de definir el comando de desconexion BLE +
         * reintento completo del handshake desde $CON (2026-09-14) — por
         * ahora, tras avisar al gateway, se sigue igual al loop de
         * escucha sin forzar ninguna desconexion. */
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

    /* Bloque de arranque consolidado (2026-09-15, pedido del usuario) — se
     * imprime AQUI a proposito, ANTES de osKernelStart(), no desde cada
     * tarea por separado: en este punto solo corre un hilo (main, el
     * scheduler todavia no arranca), asi que Log_Print()/Log_Printf() se
     * encolan en orden garantizado, uno detras de otro, sin pelearse con
     * el orden real (no determinista) en que el scheduler haria correr
     * cada tarea. Log_InitQueue() ya corrio (Tareas_InicializarMutex(),
     * antes que esta funcion) asi que encolar aqui es seguro — Log_Task
     * drena la cola en cuanto el scheduler arranque, en este mismo orden. */
    Log_Print("RTOS", "*********************************************");
    Log_Print("RTOS", "Tareas iniciadas:");
#if TASK_GPS_ENABLE
    Log_Printf("RTOS", "  GpsTask       : %s - GPS L86-M33 (DMA+IDLE)", (gpsTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
#else
    Log_Print("RTOS", "  GpsTask       : deshabilitada (TASK_GPS_ENABLE=0)");
#endif
    Log_Printf("RTOS", "  LoraTask      : %s - protocolo LoRa con el gateway", (loraTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
#if TASK_SENSORS_ENABLE
    Log_Printf("RTOS", "  SensorsTask   : %s - IMU/magnetometro/luz/bateria, pasos+orientacion", (sensorsTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
#else
    Log_Print("RTOS", "  SensorsTask   : deshabilitada (TASK_SENSORS_ENABLE=0)");
#endif
    Log_Printf("RTOS", "  CalibrateTask : %s - deteccion de disparos IR", (calibrateTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
    Log_Printf("RTOS", "  LoggerTask    : %s - consumidor de log por USB", (loggerTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
    Log_Printf("RTOS", "  BluetoothTask : %s - protocolo BLE con la Mira", (bluetoothTaskHandle != NULL) ? "arrancada" : "ERROR (NULL)");
    Log_Printf("RTOS", "Heap libre: %u de %u bytes", (unsigned)xPortGetFreeHeapSize(), (unsigned)configTOTAL_HEAP_SIZE);
    Log_Print("RTOS", "*********************************************");
    Log_Blank();   /* separa el bloque de arranque del resto del log */
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
