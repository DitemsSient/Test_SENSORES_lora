/**
 * @file    Lora.h
 * @brief   Driver for LoRa UART module on STM32L4xx.
 *
 * @details Provides basic UART transmit/receive interface for a LoRa module.
 *          CubeMX configuration:
 *          - UART peripheral in Asynchronous mode at the desired baud rate
 *            (typically 9600 or 115200).
 *          - No hardware flow control required.
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "Inicializacion.h"   /* ExerciseGameData_t — destino de Lora_ParseDatos()/Lora_ParseHexDatos() */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================  CONFIGURATION  ================================= */

/* UART handle — update to match CubeMX .ioc */

#define LORA_UART               (&huart2)

/* Timeouts */

#define LORA_TX_TIMEOUT_MS      500U    /**< Transmit timeout in ms          */
#define LORA_RX_TIMEOUT_MS      500U    /**< Receive timeout in ms           */
#define LORA_WAKE_DELAY_MS      100U    /**< Wait after wakeup byte (ms)     */

/* Buffer sizes */

#define LORA_TX_BUFFER_SIZE     256U    /**< Max transmit payload bytes      */
#define LORA_RX_BUFFER_SIZE     256U    /**< Max receive payload bytes       */

/* Configuracion inicial con el gateway (modulo KG200Z, comandos AT tipo
 * Quectel — ver CODIGO_LORA/ para el codigo de referencia original). Todo
 * esto corre en Inicializacion_Run(), bloqueante, antes del RTOS. */
#define LORA_CMD_TIMEOUT_MS      200U   /**< Timeout de respuesta a un AT+QXXX comun */
#define LORA_JOIN_TIMEOUT_MS   15000U   /**< Timeout esperando "JOINED" tras AT+QJOIN=1 */
#define LORA_CSV_FIELD_COUNT      9U    /**< numOrden,ID,equipo,alias,vidas,municion,tiempo,mac1,mac2 */

/* El primer comando AT que se manda tras encender/despertar el modulo a
 * veces no "pega" (el modulo todavia esta arrancando internamente) —
 * confirmado con hardware real 2026-09-09: el mismo "ATQ" fallaba una vez
 * y funcionaba a la tercera. El codigo de referencia del companero
 * (CODIGO_LORA/lora_kg200z.c, setupLoRa()) tambien manda un "ATQ" de
 * cortesia antes del que si se checa, por la misma razon. En vez de un
 * solo intento "de cortesia", aqui se reintenta hasta LORA_ATQ_RETRIES
 * veces cualquier comando que sea el primero tras una pausa larga (el ATQ
 * de Lora_Setup() y el de Lora_Connect() antes del QJOIN). */
#define LORA_ATQ_RETRIES           3U
#define LORA_RESET_SETTLE_MS     500U   /**< Espera tras AT+QRFS (reset de fabrica) antes de reconfigurar */

/* ========================  ENUMERATIONS  ================================== */

/* Driver status / error codes */

typedef enum {
    LORA_OK             = 0,    /**< Operation successful                    */
    LORA_ERR_PARAM,             /**< Invalid or NULL parameter               */
    LORA_ERR_UART,              /**< UART communication error                */
    LORA_ERR_TIMEOUT,           /**< Operation timed out                     */
    LORA_ERR_BUSY,              /**< Module busy                             */
    LORA_ERR_OVERFLOW           /**< Buffer overflow                         */
} LoraStatus_e;

/* ============================  STRUCTURES  ================================ */

/* LoRa driver control handle */

typedef struct {
    UART_HandleTypeDef *huart;          /**< CubeMX-generated UART handle    */
    uint8_t             tx_buffer[LORA_TX_BUFFER_SIZE]; /**< Transmit buffer */
    uint8_t             rx_buffer[LORA_RX_BUFFER_SIZE]; /**< Receive buffer  */
    uint16_t            rx_count;       /**< Bytes accumulated in rx_buffer  */
    uint8_t             rx_byte;        /**< Last byte from UART interrupt   */
    bool                rx_ready;       /**< true when data is available     */

    /* Contadores del cierre "***"/"2A2A2A" — ver Lora_StoreBytes(). Se
     * reinician en '$' y cada vez que aparece algo que rompe la racha. */
    uint8_t             star_raw_streak;   /**< '*' crudos consecutivos vistos       */
    uint8_t             star_hex_streak;   /**< pares "2A"/"2a" alineados consecutivos */
} Lora_Handle_t;

/* ================================  API  =================================== */

/**
 * @brief  Initializes the LoRa handle and binds it to the configured UART.
 * @param  h  Pointer to the LoRa handle.
 */
LoraStatus_e Lora_Init(Lora_Handle_t *h);

/**
 * @brief  Transmits a data buffer through the LoRa module.
 * @param  h     Pointer to the LoRa handle.
 * @param  data  Pointer to the data to send.
 * @param  len   Number of bytes to send.
 */
LoraStatus_e Lora_Transmit(Lora_Handle_t *h, const uint8_t *data, uint16_t len);

/**
 * @brief  Receives data from the LoRa module (blocking).
 * @param  h     Pointer to the LoRa handle.
 * @param  data  Destination buffer.
 * @param  len   Number of bytes to receive.
 */
LoraStatus_e Lora_Receive(Lora_Handle_t *h, uint8_t *data, uint16_t len);

/**
 * @brief  Accumulates one received byte into the internal buffer.
 * @param  h  Pointer to the LoRa handle.
 * @note   Call from the UART receive ISR (HAL_UART_RxCpltCallback).
 */
void Lora_StoreByte(Lora_Handle_t *h);

/**
 * @brief  Resets the receive buffer and byte counter.
 * @param  h  Pointer to the LoRa handle.
 */
void Lora_ResetRx(Lora_Handle_t *h);

/**
 * @brief  Acumula un bloque completo de bytes (modo DMA) reconociendo el
 *         framing $...*** de LoRa (cierre triple, no un solo '*' — ver
 *         nota de ruido abajo). Dos formas de recibirlo, ambas soportadas:
 *         (1) $ y * como bytes crudos (pruebas armadas a mano) — reinicia
 *         el buffer en '$', marca rx_ready al ver 3 '*' crudos seguidos;
 *         (2) TODO codificado en hex, incluido el $ y el * (asi manda
 *         ChirpStack de verdad) — en ese caso $ y * nunca llegan como
 *         bytes crudos, llegan como texto "24"/"2A". Por eso tambien se revisa,
 *         cada vez que se completa un PAR alineado de caracteres hex, si
 *         es "2A"/"2a" — al ver 3 pares seguidos se marca el cierre igual,
 *         y esos 6 caracteres se recortan del buffer para que no
 *         contaminen el ultimo campo real al decodificar.
 *         No hace falta detectar el "24" de apertura: LoraTask decodifica
 *         todo y busca "CONF" con strstr(), que lo encuentra este donde
 *         este en el texto decodificado.
 *         Por que triple y no uno solo: esto viaja por RF real (LoRa), un
 *         solo bit corrupto en transito podria formar un "2A"/'*' falso a
 *         mitad del payload y cerrar el frame antes de tiempo — exigir 3
 *         seguidos hace esa coincidencia practicamente imposible por ruido.
 *         A diferencia de Bt_StoreByte()/Gps_StoreBytes() (que cierran en
 *         '\r'), aqui NO se puede depender de un '\r' incidental: en
 *         produccion nadie "da Enter", el emisor tiene que poner '***' (o
 *         su forma hex "2A2A2A") a proposito al final del payload. Llamar
 *         desde HAL_UARTEx_RxEventCallback.
 * @param  h     Pointer to the LoRa handle.
 * @param  data  Buffer con los bytes recibidos.
 * @param  len   Cuantos bytes hay en data.
 */
void Lora_StoreBytes(Lora_Handle_t *h, const uint8_t *data, uint16_t len);

/**
 * @brief  Codifica bytes crudos a texto hexadecimal ASCII ("41421A...").
 *         Equivalente a codificarJsonToHex() del codigo de referencia.
 * @param  data      Bytes a codificar.
 * @param  data_len  Cuantos bytes hay en data.
 * @param  out       Buffer de salida, debe medir al menos (data_len*2 + 1).
 */
void Lora_EncodeToHex(const uint8_t *data, size_t data_len, char *out);

/**
 * @brief  Sends the sleep AT command to the RM1262 module.
 * @param  h  Pointer to the LoRa handle.
 * @note   Primary command: AT+SLEEP. Wake up by sending any byte on UART.
 *         If the module does not respond, try the alternative commands
 *         listed in the source file.
 */
LoraStatus_e Lora_Sleep(Lora_Handle_t *h);

/**
 * @brief  Wakes the RM1262 from sleep by sending a dummy byte on UART.
 * @param  h  Pointer to the LoRa handle.
 * @note   Waits LORA_WAKE_DELAY_MS for the module to become ready before
 *         returning. Call before any transmit or receive operation.
 */
LoraStatus_e Lora_WakeUp(Lora_Handle_t *h);

/* ==================  CONFIGURACION CON EL GATEWAY (KG200Z)  =============== */

/**
 * @brief  Configura el modulo (region US915, subbanda 2, clase A, ADR,
 *         low-power) — equivalente a setupLoRa() del codigo de referencia
 *         (ver CODIGO_LORA/lora_kg200z.c). Bloqueante, solo se llama una
 *         vez en Inicializacion_Run(), antes del RTOS.
 * @param  h  Pointer to the LoRa handle (ya inicializado con Lora_Init()).
 * @retval LORA_OK si el modulo responde y queda configurado, LORA_ERR_UART
 *         si no responde a AT basico.
 */
LoraStatus_e Lora_Setup(Lora_Handle_t *h);

/**
 * @brief  Hace join a la red LoRaWAN (equivalente a connectLoRa()). Manda
 *         AT+QJOIN=1, espera el "OK" inmediato (este firmware del KG200Z
 *         NO manda "MAC txDone" como el codigo de referencia asumia —
 *         confirmado con hardware real 2026-09-09) y luego espera "JOINED"
 *         asincrono. Si no confirma, manda un reset de fabrica (AT+QRFS),
 *         reconfigura con Lora_Setup() y reintenta el join una vez mas
 *         antes de rendirse — mismo patron que CODIGO_LORA/lora_kg200z.c
 *         (connectLoRa()) al fallar el join.
 * @param  h  Pointer to the LoRa handle.
 * @retval LORA_OK si el join se confirmo ("JOINED"), LORA_ERR_TIMEOUT si no
 *         (incluso despues del reset de fabrica).
 */
LoraStatus_e Lora_Connect(Lora_Handle_t *h);

/**
 * @brief  Decodifica una cadena hexadecimal ASCII ("41421A...") a bytes
 *         crudos. Equivalente a decodeHexToBytes() del codigo de
 *         referencia.
 * @param  hex_str  Cadena hex, NUL-terminada (case-insensitive).
 * @param  out      Buffer de salida.
 * @param  max_len  Tamaño maximo de out.
 * @retval Bytes decodificados en out.
 */
size_t Lora_DecodeHexToBytes(const char *hex_str, uint8_t *out, size_t max_len);

/**
 * @brief  Parsea un CSV de 9 campos ya decodificado (ASCII, no hex) —
 *         "numOrden,ID,equipo,alias,vidas,municion,tiempo,mac1,mac2",
 *         opcionalmente envuelto en `{...}` — y llena los campos
 *         correspondientes de ExerciseGameData_t (mac1 -> out->mac,
 *         mac2 -> out->mac2). Equivalente a parsearDatosLora().
 * @param  csv_ascii  Texto CSV ya decodificado.
 * @param  out        Estructura a llenar (normalmente &g_exercise_data).
 * @retval true si se reconocieron los 9 campos, false si el formato no
 *         coincide (out queda sin tocar en ese caso).
 */
bool Lora_ParseDatos(const char *csv_ascii, ExerciseGameData_t *out);

/**
 * @brief  Combina Lora_DecodeHexToBytes() + Lora_ParseDatos() en un solo
 *         paso — util para inyectar tramas de prueba en hexadecimal
 *         directo (ej. escribiendo a mano por terminal al UART de LoRa,
 *         mientras no exista LoraTask real).
 * @param  hex_str  Cadena hex ASCII recibida (ej. de s_lora.rx_buffer).
 * @param  out      Estructura a llenar (normalmente &g_exercise_data).
 * @retval true si se decodifico y parseo correctamente.
 */
bool Lora_ParseHexDatos(const char *hex_str, ExerciseGameData_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LORA_H */
