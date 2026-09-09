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
#include <stdbool.h>

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

/* ============================  STRUCTURES  ================================ */

/** @brief Resultado (OK/FALLO) de cada paso de Inicializacion_Run(), uno por
 *         modulo. Se va llenando conforme avanza el init; Diagnostico_Print()
 *         lo imprime completo por el Logger cuando haga falta. */
typedef struct {
    bool i2c_completo;    /**< Todas las direcciones I2C esperadas respondieron */
    bool motovibrador;
    bool buzzer;
    bool flash;
    bool magnetometro;
    bool imu;
    bool batterymonitor;
    bool sensorluz;
    bool bluetooth;
    bool lora;
    bool rx_ir;           /**< Solo init — no hay forma de autoverificar sin transmisor externo */
    bool rgb;
    bool gps;
} Diagnostico_t;

extern Diagnostico_t Diagnostico;

/** @brief Modo de operacion global de la tarjeta. Nace SIEMPRE en
 *         MODO_CONFIGURACION (ver Inicializacion_Run()) — Sensores entra a
 *         MODO_EJERCICIO solo al mandar/procesar $RUN (protocolo Bluetooth
 *         con Mira, ver memoria de protocolo), y regresa a
 *         MODO_CONFIGURACION con $END o $DSCON. Determina, entre otras
 *         cosas, cual interrupcion (LoRa vs recepcion IR de disparos) tiene
 *         mayor prioridad en el NVIC — ver Modo_SetOperacion(). */
typedef enum {
    MODO_CONFIGURACION = 0,   /**< Arranque, emparejamiento BLE, $*<datos>  */
    MODO_EJERCICIO,            /**< Cuenta regresiva -> disparo habilitado  */
} ModoOperacion_e;

extern ModoOperacion_e g_modo_operacion;

/** @brief Datos del ejercicio/jugador — el "pizarron" compartido de todo el
 *         sistema, todo se escribe aqui:
 *         - Payload de config que llega por LoRa ($CONF<datos>, 9 campos:
 *           numOrden,ID,equipo,alias,vidas,municion,tiempo,mac1,mac2 — ver
 *           CODIGO_LORA de referencia) y que se retransmite a Mira por
 *           Bluetooth (mismo protocolo $CONF<datos>, 8 campos: solo mac1,
 *           mac2 no se manda).
 *         - Telemetria de vuelta al gateway por LoRa (12 campos:
 *           ID,numOrden,vidas,municion,bateria,latitud,longitud,altitud,
 *           orientacion,pasos,ack,timestamp — latitud ANTES que longitud,
 *           asi lo manda de verdad generarCadena() del codigo de
 *           referencia del companero (CODIGO_LORA/lora_kg200z.c), aunque
 *           su propio comentario diga lo contrario. Reusa los mismos
 *           orden/lora/lives/ammo/bateria_ch de arriba, ver campos nuevos
 *           abajo). El "ack" de esta telemetria es lo que confirma al
 *           gateway que el $CONF se recibio bien (1) o no (0) — no es un
 *           mensaje aparte, es este mismo frame con ack=1.
 *         - $A_AP<balas>,<pct_bateria>\r que manda Mira por Bluetooth cada
 *           200ms (solo si cambio algo) — actualiza ammo y bateria_ap.
 *         - Lecturas de sensores que llena SensorsTask cada ciclo.
 *         Nace en 0 salvo los placeholders de arranque (team_name,
 *         player_name, bateria_ch — ver Inicializacion.c) hasta que $CONF
 *         los sobreescriba. */
typedef struct {
    uint8_t  orden;             /**< Orden/turno asignado por el servidor remoto */
    uint8_t  lora;               /**< Identificador/canal LoRa del jugador       */
    char     team_name[32];      /**< "equipo" en el CSV                         */
    char     player_name[32];    /**< "alias" en el CSV                          */
    uint8_t  lives;              /**< "vidas" — solo nosotros la modificamos (impactos), Mira no la manda */
    uint16_t ammo;                /**< "municion"/"balas" — CONF inicial, y luego actualizada por $A_AP de Mira */
    uint32_t tiempo;             /**< Duracion del ejercicio, segundos           */
    char     mac[18];            /**< "mac1" del CSV — la que se usa para Bluetooth */
    char     mac2[18];           /**< "mac2" del CSV — reservada, sin uso por ahora */

    /* Dos baterias distintas — no son lo mismo. bateria_ch (chaleco = esta
     * tarjeta, Sensores) es la unica que se manda en la telemetria por
     * ahora; bateria_ap (apuntador = Mira, viene de $A_AP) se guarda pero
     * NO se agrega todavia al envio por LoRa — pendiente hasta que se
     * actualice la base de datos del otro lado para tener este campo. */
    uint8_t  bateria_ch;         /**< Bateria de ESTA tarjeta (chaleco) — la llena SensorsTask, es la que se manda por LoRa */
    uint8_t  bateria_ap;         /**< Bateria de la Mira (apuntador) — la llena BluetoothTask via $A_AP, TODO: agregar al envio de telemetria */

    /* Telemetria de vuelta al gateway por LoRa (los que faltaban de la
     * lista de arriba) — en 0 hasta que GPS/IMU los vayan llenando.
     * "orientacion" y "pasos" salen del IMU/magnetometro de esta tarjeta
     * (todavia no se calculan, solo el campo esta listo); "longitud"/
     * "latitud"/"altitud" salen de GpsTask cuando haya fix. */
    float    longitud;
    float    latitud;
    float    altitud;
    uint16_t orientacion;
    uint16_t pasos;
    uint8_t  ack;                /**< 1 si Mira confirmo el $CONF con $ACKCONF, 0 si no */
    uint32_t timestamp;

    /* Sensores — llenados por SensorsTask */
    float    lux;
    float    mag_x_uT, mag_y_uT, mag_z_uT;
    float    gyro_x_dps, gyro_y_dps, gyro_z_dps;
} ExerciseGameData_t;

extern ExerciseGameData_t g_exercise_data;

/* ================================  API  =================================== */

/**
 * @brief  Imprime el Diagnostico_t completo por el Logger (OK/FALLO por
 *         modulo). Se llama una vez al final de Inicializacion_Run(), pero
 *         es publica para poder invocarla de nuevo donde haga falta.
 */
void Diagnostico_Print(void);

/**
 * @brief  Imprime los 8 campos de g_exercise_data que vienen del protocolo
 *         ($*<datos>: orden, lora, equipo, alias, vidas, balas, tiempo,
 *         mac). Se llama al procesar $*<datos> y donde se necesite
 *         confirmar el estado actual del ejercicio.
 */
void Inicializacion_PrintExerciseData(void);

/**
 * @brief  Imprime las ultimas lecturas de sensores de g_exercise_data (luz,
 *         bateria, magnetometro, giroscopio) — debug generico, llamado por
 *         SensorsTask al final de cada ciclo (ver Tareas.c).
 */
void Inicializacion_PrintSensorsData(void);

/**
 * @brief  Aplica el modo de operacion: actualiza g_modo_operacion y
 *         reacomoda las prioridades del NVIC entre la interrupcion de LoRa
 *         y la de recepcion IR de disparos (ver comentario en el .c —
 *         swap valido en runtime, siempre dentro del rango seguro para
 *         FreeRTOS: configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY..15).
 * @param  modo  Nuevo modo de operacion.
 */
void Modo_SetOperacion(ModoOperacion_e modo);

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
 * @retval true si TODAS las direcciones esperadas respondieron, false si
 *         alguna falto.
 */
bool Search_Disp_I2C(void);

#ifdef __cplusplus
}
#endif

#endif /* INICIALIZACION_H */
