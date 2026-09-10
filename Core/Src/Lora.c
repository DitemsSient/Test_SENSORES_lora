/**
 * @file    Lora.c
 * @brief   LoRa UART module driver implementation for STM32F4xx.
 *
 * @details Uses HAL_UART_Transmit / HAL_UART_Receive for blocking transfers.
 *          Interrupt-driven reception is supported via Lora_StoreByte().
 *
 * @date    April 8, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Lora.h"
#include "Logger.h"
#include <string.h>
#include <stdlib.h>

/* ========================  EXTERNAL HAL HANDLES  ========================== */

extern UART_HandleTypeDef huart2;

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Manda un comando y espera "expect" en la respuesta, dentro de
 *         timeout_ms. Resetea el rx antes de mandar. Bloqueante (HAL_Delay
 *         en pedacitos de 20ms) — solo se usa desde Inicializacion_Run(),
 *         antes del RTOS.
 */
static bool Lora_SendAndWait(Lora_Handle_t *h, const char *cmd, const char *expect, uint32_t timeout_ms)
{
    Lora_ResetRx(h);
    Lora_Transmit(h, (const uint8_t *)cmd, (uint16_t)strlen(cmd));

    uint32_t start = HAL_GetTick();
    bool     found = false;
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (h->rx_count > 0U && strstr((char *)h->rx_buffer, expect) != NULL) {
            found = true;
            break;
        }
        HAL_Delay(20U);
    }

    /* Imprime SIEMPRE lo que en verdad contesto el modulo, encontrara o no
     * "expect" — util para depurar el init a ojo. No hace falta DMA para
     * esto: la recepcion IT (Lora_StoreByte(), armada por Lora_Init())
     * ya va llenando h->rx_buffer byte a byte, solo faltaba loguearlo. */
    Log_Printf("LORA", "[INIT] %s -> %s", cmd,
               (h->rx_count > 0U) ? (char *)h->rx_buffer : "(sin respuesta)");

    return found;
}

/**
 * @brief  Manda "ATQ\r\n" sin esperar ni revisar la respuesta — "cortesia"
 *         para despertar/sincronizar el modulo antes del primer comando
 *         que si se checa, igual que hace el codigo de referencia del
 *         companero (CODIGO_LORA/lora_kg200z.c, setupLoRa(): un ATQ de
 *         cortesia + HAL_Delay(200) antes del ATQ real). Llamarla un par
 *         de veces seguidas cuando el modulo recien se encendio/desperto.
 */
static void Lora_SendATQDefault(Lora_Handle_t *h)
{
    Lora_ResetRx(h);
    Lora_Transmit(h, (const uint8_t *)"ATQ\r\n", 5U);
    HAL_Delay(200U);
}

/**
 * @brief  Igual que Lora_SendAndWait(), pero reintenta hasta "retries"
 *         veces si no llega "expect" — el primer comando tras una pausa
 *         larga (modulo recien encendido/despertado) a veces no "pega" la
 *         primera vez, confirmado con hardware real (ver comentario de
 *         LORA_ATQ_RETRIES en Lora.h). Cada intento fallido espera
 *         timeout_ms antes del siguiente, igual que el codigo de
 *         referencia del companero (que manda un "ATQ" de cortesia antes
 *         del que si cuenta).
 */
static bool Lora_SendAndWaitRetry(Lora_Handle_t *h, const char *cmd, const char *expect,
                                   uint32_t timeout_ms, uint8_t retries)
{
    for (uint8_t i = 0U; i < retries; i++) {
        if (Lora_SendAndWait(h, cmd, expect, timeout_ms)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief  Espera "expect" en lo que ya se fue acumulando en rx_buffer, SIN
 *         mandar nada nuevo ni resetear — para respuestas asincronas que
 *         llegan despues de un comando ya mandado (ej. "JOINED" tras
 *         AT+QJOIN=1, que primero contesta "OK" de inmediato y luego
 *         "JOINED" cuando el gateway responde).
 */
static bool Lora_WaitFor(Lora_Handle_t *h, const char *expect, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (h->rx_count > 0U && strstr((char *)h->rx_buffer, expect) != NULL) {
            return true;
        }
        HAL_Delay(20U);
    }
    return false;
}

static uint8_t Lora_HexCharToVal(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0U;
}

/* ========================  PUBLIC FUNCTIONS  =============================== */

LoraStatus_e Lora_Init(Lora_Handle_t *h)
{
    if (h == NULL) {
        Log_Print("LORA", "Error: handle NULL en Init");
        return LORA_ERR_PARAM;
    }

    Log_Print("LORA", "Iniciando modulo LoRa...");

    h->huart    = LORA_UART;
    h->rx_count = 0U;
    h->rx_byte  = 0U;
    h->rx_ready = false;
    memset(h->tx_buffer, 0, LORA_TX_BUFFER_SIZE);
    memset(h->rx_buffer, 0, LORA_RX_BUFFER_SIZE);

    /* Arm interrupt reception for the first byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);

    Log_Print("LORA", "LoRa inicializado correctamente");
    return LORA_OK;
}

LoraStatus_e Lora_Transmit(Lora_Handle_t *h, const uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return LORA_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, data, len, LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

LoraStatus_e Lora_Receive(Lora_Handle_t *h, uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0U) {
        return LORA_ERR_PARAM;
    }

    HAL_StatusTypeDef hal = HAL_UART_Receive(h->huart, data, len, LORA_RX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

void Lora_StoreByte(Lora_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    if (h->rx_count < LORA_RX_BUFFER_SIZE) {
        h->rx_buffer[h->rx_count] = h->rx_byte;
        h->rx_count++;
    }

    /* Re-arm interrupt for next byte */
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);
}

void Lora_ResetRx(Lora_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    h->rx_count        = 0U;
    h->rx_ready        = false;
    h->star_raw_streak = 0U;
    h->star_hex_streak = 0U;
    memset(h->rx_buffer, 0, LORA_RX_BUFFER_SIZE);
}

void Lora_StoreBytes(Lora_Handle_t *h, const uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL) return;

    for (uint16_t i = 0U; i < len; i++) {
        char c = (char)data[i];

        if (c == '$') {
            /* $ crudo — caso en que alguien arma el frame a mano, sin pasar
             * todo por hex (ej. pruebas directas por terminal). */
            h->rx_count        = 0U;
            h->rx_ready        = false;
            h->rx_buffer[0]    = '\0';
            h->star_raw_streak = 0U;
            h->star_hex_streak = 0U;
            continue;
        }
        if (c == '*') {
            /* * crudo — cuenta la racha, cierra al tercero seguido (ver
             * comentario de "por que triple" en Lora.h). */
            h->star_raw_streak++;
            if ((h->star_raw_streak >= 3U) && (h->rx_count > 0U)) {
                h->rx_ready = true;
            }
            continue;
        }
        h->star_raw_streak = 0U;   /* cualquier otro byte rompe la racha de '*' crudos */

        if (h->rx_count < (LORA_RX_BUFFER_SIZE - 1U)) {
            h->rx_buffer[h->rx_count] = (uint8_t)c;
            h->rx_count++;
            h->rx_buffer[h->rx_count] = '\0';
        }

        /* ChirpStack manda TODO codificado en hex, incluyendo el $ y el *
         * propios del mensaje — nunca llegan como bytes crudos en ese caso,
         * llegan como los caracteres de texto "24" y "2A". Por eso ademas
         * revisamos: cada vez que se completa un PAR alineado de caracteres
         * hex (posiciones 0-1, 2-3, 4-5..., nunca a la mitad de un par —
         * si no, un "2A" que cae a caballo entre dos bytes distintos daria
         * un cierre falso), si ese par es "2A"/"2a" cuenta para la racha;
         * al tercer par seguido se cierra el frame Y se recortan esos 6
         * caracteres del buffer, para que el "***" codificado no quede
         * pegado al ultimo campo real al decodificar. No hace falta buscar
         * el "24" de apertura — el contenido decodificado se busca con
         * strstr("CONF", ...) en LoraTask, que encuentra la palabra este
         * donde este. */
        if ((h->rx_count >= 2U) && ((h->rx_count % 2U) == 0U)) {
            char par_hi = (char)h->rx_buffer[h->rx_count - 2U];
            char par_lo = (char)h->rx_buffer[h->rx_count - 1U];
            if (par_hi == '2' && (par_lo == 'A' || par_lo == 'a')) {
                h->star_hex_streak++;
                if (h->star_hex_streak >= 3U) {
                    h->rx_count      -= 6U;   /* quita los 3 pares "2A" del cierre */
                    h->rx_buffer[h->rx_count] = '\0';
                    h->rx_ready = true;
                }
            } else {
                h->star_hex_streak = 0U;
            }
        }
    }
}

void Lora_EncodeToHex(const uint8_t *data, size_t data_len, char *out)
{
    static const char hex_chars[] = "0123456789ABCDEF";

    if (data == NULL || out == NULL) return;

    for (size_t i = 0U; i < data_len; i++) {
        out[i * 2U]      = hex_chars[(data[i] >> 4U) & 0x0FU];
        out[i * 2U + 1U] = hex_chars[data[i] & 0x0FU];
    }
    out[data_len * 2U] = '\0';
}

/* ========================  LOW POWER  ====================================== */

LoraStatus_e Lora_Sleep(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* Primary command for RM1262 — verify with hardware if module does not
     * respond. Alternative commands to try (uncomment one at a time):
     *
     *   "AT+SLPM\r\n"       — common Semtech / RAK variant
     *   "AT+SLEEP=1\r\n"    — some modules require a parameter
     *   "AT+LOWPOWER\r\n"   — used by certain LoRaWAN stacks
     *   "AT+PSLP\r\n"       — Murata / STM32WL variant
     */
    const uint8_t cmd[] = "AT+SLEEP\r\n";
    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, cmd,
                                              sizeof(cmd) - 1U,
                                              LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    return LORA_OK;
}

LoraStatus_e Lora_WakeUp(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* Any UART byte wakes the RM1262 from sleep */
    const uint8_t wake = 0xFFU;
    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, &wake, 1U,
                                              LORA_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? LORA_ERR_TIMEOUT : LORA_ERR_UART;
    }

    HAL_Delay(LORA_WAKE_DELAY_MS);
    return LORA_OK;
}

/* ==================  CONFIGURACION CON EL GATEWAY (KG200Z)  =============== */

LoraStatus_e Lora_Setup(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* 2 ATQ de cortesia antes del real — ver Lora_SendATQDefault(). */
    Lora_SendATQDefault(h);
    Lora_SendATQDefault(h);

    if (!Lora_SendAndWaitRetry(h, "ATQ\r\n", "OK", LORA_CMD_TIMEOUT_MS, LORA_ATQ_RETRIES)) {
        Log_Print("LORA", "ERROR: modulo no responde a ATQ.");
        return LORA_ERR_UART;
    }

    Lora_SendAndWait(h, "AT+QVL=0\r\n", "OK", LORA_CMD_TIMEOUT_MS);   /* log level, no critico */

    /* Region US915 / subbanda 2 (canales 8-15) / clase A / ADR / low-power
     * — mismos valores que CODIGO_LORA/lora_kg200z.c. Solo se cambia lo que
     * no esta ya en el valor esperado (AT+QBAND=8 reinicia el modulo solo). */
    bool cambios = false;

    if (!Lora_SendAndWait(h, "AT+QBAND=?\r\n", "QBAND:8", LORA_CMD_TIMEOUT_MS)) {
        cambios = Lora_SendAndWait(h, "AT+QBAND=8\r\n", "OK", LORA_CMD_TIMEOUT_MS) || cambios;
        HAL_Delay(100U);
    }
    if (!Lora_SendAndWait(h, "AT+QCHAN=?\r\n", "FF00:0000:0000:0000:0000:0000", LORA_CMD_TIMEOUT_MS)) {
        cambios = Lora_SendAndWait(h, "AT+QCHAN=FF00:0000:0000:0000:0000:0000\r\n", "OK", LORA_CMD_TIMEOUT_MS) || cambios;
    }
    if (!Lora_SendAndWait(h, "AT+QCLASS=?\r\n", "QCLASS: A", LORA_CMD_TIMEOUT_MS)) {
        cambios = Lora_SendAndWait(h, "AT+QCLASS=A\r\n", "OK", LORA_CMD_TIMEOUT_MS) || cambios;
    }
    if (!Lora_SendAndWait(h, "AT+QADR=?\r\n", "QADR:1", LORA_CMD_TIMEOUT_MS)) {
        cambios = Lora_SendAndWait(h, "AT+QADR=1\r\n", "OK", LORA_CMD_TIMEOUT_MS) || cambios;
    }
    if (!Lora_SendAndWait(h, "AT+QLPMOD=?\r\n", "Lowpower mode is enabled!", LORA_CMD_TIMEOUT_MS)) {
        cambios = Lora_SendAndWait(h, "AT+QLPMOD=1\r\n", "OK", LORA_CMD_TIMEOUT_MS) || cambios;
    }

    if (cambios) {
        Lora_SendAndWait(h, "AT+QCS\r\n", "OK", 500U);      /* guarda config */
        Lora_Transmit(h, (const uint8_t *)"ATZQ\r\n", 6U);  /* reset — no hay respuesta */
        HAL_Delay(250U);
        Lora_SendAndWait(h, "AT+QVL=0\r\n", "OK", LORA_CMD_TIMEOUT_MS);
        Log_Print("LORA", "Configuracion aplicada y modulo reseteado.");
    } else {
        Log_Print("LORA", "Ya estaba configurado, sin cambios.");
    }

    return LORA_OK;
}

/**
 * @brief  Un solo intento de AT+QJOIN=1 + espera de "JOINED" — separado de
 *         Lora_Connect() para poder reintentarlo despues de un reset de
 *         fabrica sin duplicar el codigo (ver Lora_Connect()).
 * @note   El codigo de referencia del companero espera "MAC txDone" como
 *         ACK inmediato de AT+QJOIN=1, pero con hardware real (2026-09-09)
 *         este firmware del KG200Z NUNCA manda esa frase — solo contesta
 *         "OK" liso (confirmado viendo el log crudo con
 *         Lora_SendAndWait()). Con "MAC txDone" el codigo se rendia a los
 *         2s sin llegar nunca a esperar "JOINED" de verdad. Ahora se
 *         acepta el "OK" real como ACK inmediato.
 */
static LoraStatus_e Lora_IntentarJoin(Lora_Handle_t *h)
{
    if (!Lora_SendAndWait(h, "AT+QJOIN=1\r\n", "OK", 2000U)) {
        return LORA_ERR_TIMEOUT;
    }

    /* "JOINED" llega asincrono, cuando el gateway responde — no se manda
     * nada nuevo aqui, solo se sigue revisando lo acumulado. */
    if (!Lora_WaitFor(h, "JOINED", LORA_JOIN_TIMEOUT_MS)) {
        return LORA_ERR_TIMEOUT;
    }

    return LORA_OK;
}

LoraStatus_e Lora_Connect(Lora_Handle_t *h)
{
    if (h == NULL) return LORA_ERR_PARAM;

    /* "ATQ" de cortesia antes del QJOIN — mismo motivo que en Lora_Setup():
     * el primer comando tras una pausa larga a veces no pega. No es fatal
     * si falla (el modulo puede seguir respondiendo al QJOIN igual), solo
     * se reintenta unas veces sin abortar el connect por esto. */
    Lora_SendATQDefault(h);
    Lora_SendATQDefault(h);
    Lora_SendAndWaitRetry(h, "ATQ\r\n", "OK", LORA_CMD_TIMEOUT_MS, LORA_ATQ_RETRIES);

    if (Lora_IntentarJoin(h) == LORA_OK) {
        Log_Print("LORA", "Join confirmado (JOINED).");
        return LORA_OK;
    }

    /* Primer intento de join fallido — mismo caso que el codigo de
     * referencia del companero (CODIGO_LORA/lora_kg200z.c, connectLoRa()):
     * si no llega el "OK" inmediato (o nunca llega "JOINED"), se asume que
     * el modulo quedo en un estado raro y se le manda un reset de fabrica
     * (AT+QRFS) antes de reintentar — a diferencia de la referencia (que
     * solo reconfigura y se queda ahi, sin volver a pedir join), aqui SI
     * se reintenta el join una vez mas despues del reset+reconfiguracion,
     * para darle una oportunidad real de recuperarse sola. */
    Log_Print("LORA", "Join fallo — reset de fabrica (AT+QRFS) y reintentando...");
    Lora_Transmit(h, (const uint8_t *)"AT+QRFS\r\n", 9U);   /* no responde nada */
    HAL_Delay(LORA_RESET_SETTLE_MS);

    if (Lora_Setup(h) != LORA_OK) {
        Log_Print("LORA", "ERROR: no se pudo reconfigurar tras el reset de fabrica.");
        return LORA_ERR_TIMEOUT;
    }

    if (Lora_IntentarJoin(h) == LORA_OK) {
        Log_Print("LORA", "Join confirmado (JOINED) tras reset de fabrica.");
        return LORA_OK;
    }

    Log_Print("LORA", "ERROR: fallo el join incluso despues del reset de fabrica.");
    return LORA_ERR_TIMEOUT;
}

/* ==========================  PARSEO DE DATOS  ============================== */

size_t Lora_DecodeHexToBytes(const char *hex_str, uint8_t *out, size_t max_len)
{
    if (hex_str == NULL || out == NULL) return 0U;

    size_t len     = strlen(hex_str);
    size_t out_len = 0U;

    for (size_t i = 0U; (i + 1U) < len && out_len < max_len; i += 2U) {
        uint8_t hi = Lora_HexCharToVal(hex_str[i]);
        uint8_t lo = Lora_HexCharToVal(hex_str[i + 1U]);
        out[out_len++] = (uint8_t)((hi << 4U) | lo);
    }
    return out_len;
}

bool Lora_ParseDatos(const char *csv_ascii, ExerciseGameData_t *out)
{
    if (csv_ascii == NULL || out == NULL) return false;

    char datos[300];
    strncpy(datos, csv_ascii, sizeof(datos) - 1U);
    datos[sizeof(datos) - 1U] = '\0';

    size_t len   = strlen(datos);
    size_t start = 0U;
    if (len > 0U && datos[0] == '{') { start = 1U; }
    if (len > 0U && datos[len - 1U] == '}') { datos[len - 1U] = '\0'; len--; }

    char *contenido = datos + start;
    len = strlen(contenido);

    char   *campos[LORA_CSV_FIELD_COUNT];
    uint8_t idx        = 0U;
    size_t  campo_start = 0U;

    for (size_t i = 0U; i <= len && idx < LORA_CSV_FIELD_COUNT; i++) {
        if (i == len || contenido[i] == ',') {
            contenido[i] = '\0';
            campos[idx++] = &contenido[campo_start];
            campo_start = i + 1U;
        }
    }

    if (idx < LORA_CSV_FIELD_COUNT) {
        Log_Print("LORA", "ERROR: se esperaban 9 campos separados por comas.");
        return false;
    }

    out->orden = (uint8_t)atoi(campos[0]);
    out->lora  = (uint8_t)atoi(campos[1]);
    strncpy(out->team_name,   campos[2], sizeof(out->team_name) - 1U);
    out->team_name[sizeof(out->team_name) - 1U] = '\0';
    strncpy(out->player_name, campos[3], sizeof(out->player_name) - 1U);
    out->player_name[sizeof(out->player_name) - 1U] = '\0';
    out->lives  = (uint8_t)atoi(campos[4]);
    out->ammo   = (uint16_t)atoi(campos[5]);
    out->tiempo = (uint32_t)atoi(campos[6]);
    strncpy(out->mac, campos[7], sizeof(out->mac) - 1U);
    out->mac[sizeof(out->mac) - 1U] = '\0';
    strncpy(out->mac2, campos[8], sizeof(out->mac2) - 1U);
    out->mac2[sizeof(out->mac2) - 1U] = '\0';

    return true;
}

bool Lora_ParseHexDatos(const char *hex_str, ExerciseGameData_t *out)
{
    uint8_t decoded[300];
    size_t  n = Lora_DecodeHexToBytes(hex_str, decoded, sizeof(decoded) - 1U);
    decoded[n] = '\0';
    return Lora_ParseDatos((const char *)decoded, out);
}
