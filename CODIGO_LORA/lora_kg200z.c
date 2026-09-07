#include "main.h"
#include "lora_kg200z.h"
#include "helpers.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t conectionLora = 0;

/* ==========================================================================
 *  Buffer circular de RX del UART de LoRa (llenado por interrupción)
 * ========================================================================== */
#define LORA_RXBUF_SZ   512U   /* Tamaño del buffer circular de bytes crudos */
#define LORA_LINE_MAX   256U   /* Tamaño máximo de una línea armada          */

/* Buffer circular: el HAL escribe directamente en él, continuando
 * siempre desde donde se quedó (loraRxHead) */
uint8_t loraDmaBuf[LORA_RXBUF_SZ];
static volatile uint16_t loraRxHead = 0;   /* Escribe la ISR   */
static volatile uint16_t loraRxTail = 0;   /* Lee el main loop */

static char     loraLineBuf[LORA_LINE_MAX];
static uint16_t loraLineLen = 0;

/* Buffer para almacenar cadena a enviar por Lora */
#define JSON_BUF_SZ 250
static char jsonBuf[JSON_BUF_SZ] = { 0 };



// **************************************************************************************************
// ***************************************** LORA IRQ ***********************************************
// **************************************************************************************************

/* Espacio libre antes de alcanzar loraRxTail (para no pisar datos que el
 * main loop todavía no ha leído). Se deja siempre 1 byte de margen para
 * poder distinguir buffer lleno de buffer vacío (head == tail). */
static inline uint16_t Lora_RxBuf_FreeSpace(void)
{
    return (uint16_t)((loraRxTail - loraRxHead - 1 + LORA_RXBUF_SZ) % LORA_RXBUF_SZ);
}

/* Arma la recepción por idle-line, escribe a partir de loraRxHead y sin pasarse ni del
 * final del arreglo ni del espacio aún no leído por el main loop (loraRxTail). */
void Lora_ArmReceive(void)
{
    uint16_t spaceToEnd = (uint16_t)(LORA_RXBUF_SZ - loraRxHead);
    uint16_t freeSpace   = Lora_RxBuf_FreeSpace();
    uint16_t chunk        = (spaceToEnd < freeSpace) ? spaceToEnd : freeSpace;

    if (chunk == 0) return;  /* Buffer lleno: el main loop no ha leído nada todavía */

    HAL_UARTEx_ReceiveToIdle_IT(UART_LORA, &loraDmaBuf[loraRxHead], chunk);
}


/* Contadores de diagnóstico */
static volatile uint32_t loraRxByteCount = 0;
static volatile uint32_t loraErrCount    = 0;
static volatile uint32_t loraLastErrCode = 0;

/* Callback: dispara cuando se detecta "idle" en la línea (el módulo
 * dejó de transmitir)  o cuando el bloque pedido se llenó por completo. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        loraRxHead = (uint16_t)((loraRxHead + Size) % LORA_RXBUF_SZ);
        loraRxByteCount += Size;

        Lora_ArmReceive();  /* Rearma continuando desde la nueva posición de loraRxHead */
    }
}

/* Callback: se dispara ante error de trama/paridad/ruido/overrun. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        loraLastErrCode = huart2.ErrorCode;   /* HAL_UART_ERROR_ORE / FE / NE / PE */
        loraErrCount++;
        __HAL_UART_CLEAR_PEFLAG(UART_LORA);     /* Limpia flags de error */
        Lora_ArmReceive();                      /* Rearma */
    }
}



// **************************************************************************************************
// ***************************************** LORA UART **********************************************
// **************************************************************************************************
static inline uint8_t Lora_RxBuf_Available(void)
{
    return (loraRxHead != loraRxTail) ? 1U : 0U;
}

static inline uint8_t Lora_RxBuf_Pop(void)
{
    uint8_t b = loraDmaBuf[loraRxTail];
    loraRxTail = (uint16_t)((loraRxTail + 1) % LORA_RXBUF_SZ);
    return b;
}

static void Lora_ClearRx(void)
{
    while (Lora_RxBuf_Available()) { Lora_RxBuf_Pop(); }
    loraLineLen = 0;
}

/* Intenta armar una línea terminada en '\n', recorta '\r'
 * Devuelve 1 si completó una línea. */
static uint8_t Lora_TryGetLine(char *out, size_t maxLen)
{
    while (Lora_RxBuf_Available()) {
        uint8_t b = Lora_RxBuf_Pop();

        if (b == '\n') {
            uint16_t len = loraLineLen;
            if (len > 0 && loraLineBuf[len - 1] == '\r') len--;
            if (len >= maxLen) len = (uint16_t)(maxLen - 1);

            memcpy(out, loraLineBuf, len);
            out[len] = '\0';
            loraLineLen = 0;
            return 1;
        } else {
            if (loraLineLen < (LORA_LINE_MAX - 1)) {
                loraLineBuf[loraLineLen++] = (char)b;
            }
            /* Si una línea individual excede LORA_LINE_MAX se recorta, no se pierde el flujo */
        }
    }
    return 0;
}




// **************************************************************************************************
// ***** Parseo del payload recibido: {numOrden,ID,equipo,alias,vidas,municion,tiempo,mac1,mac2} ****
// **************************************************************************************************
#define NUM_CAMPOS 9
uint8_t parsearDatosLora(const char *texto)
{
    char datos[300];
    strncpy(datos, texto, sizeof(datos) - 1);
    datos[sizeof(datos) - 1] = '\0';

    size_t len = strlen(datos);
    size_t start = 0;

    if (len > 0 && datos[0] == '{') { start = 1; }
    if (len > 0 && datos[len - 1] == '}') { datos[len - 1] = '\0'; len--; }

    char *contenido = datos + start;
    len = strlen(contenido);

    char *campos[NUM_CAMPOS];
    int idx = 0;
    size_t campoStart = 0;

    for (size_t i = 0; i <= len && idx < NUM_CAMPOS; i++) {
        if (i == len || contenido[i] == ',') {
            contenido[i] = '\0';
            campos[idx++] = &contenido[campoStart];
            campoStart = i + 1;
        }
    }

    if (idx < NUM_CAMPOS) {
        dbg_println("Error: se esperaban 9 campos separados por comas.");
        return 0;
    }

    numOrden = (uint8_t)atoi(campos[0]);
    ID       = (uint8_t)atoi(campos[1]);
    strncpy(equipo, campos[2], sizeof(equipo) - 1); equipo[sizeof(equipo) - 1] = '\0';
    strncpy(alias,  campos[3], sizeof(alias)  - 1); alias[sizeof(alias) - 1]   = '\0';
    vidas       = (uint8_t)atoi(campos[4]);
    municiones  = (uint16_t)atoi(campos[5]);
    tiempo      = (uint16_t)atoi(campos[6]);
    strncpy(mac1, campos[7], sizeof(mac1) - 1); mac1[sizeof(mac1) - 1] = '\0';
    strncpy(mac2, campos[8], sizeof(mac2) - 1); mac2[sizeof(mac2) - 1] = '\0';

    return 1;
}


// **************************************************************************************************
// **************** Espera del evento de dato: +QEVT:<puerto>:<lenHex>:<payloadHex> *****************
// **************************************************************************************************
uint8_t esperarDatoLora(char *hexPayloadOut, size_t maxLen, uint32_t timeoutMs)
{
    uint32_t startTime = HAL_GetTick();
    char line[LORA_LINE_MAX];

    while ((HAL_GetTick() - startTime) < timeoutMs) {
        while (Lora_TryGetLine(line, sizeof(line))) {
            if (strlen(line) == 0) continue;

            if (strncmp(line, "+QEVT:", 6) == 0) {
                char *p1 = strchr(line + 6, ':');
                char *p2 = (p1 != NULL) ? strchr(p1 + 1, ':') : NULL;

                if (p1 != NULL && p2 != NULL) {
                    const char *payload = p2 + 1;
                    if (strlen(payload) > 0 && strchr(payload, ',') == NULL) {
                        strncpy(hexPayloadOut, payload, maxLen - 1);
                        hexPayloadOut[maxLen - 1] = '\0';
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}


// **************************************************************************************************
// **************************************** Funciones LORA ******************************************
// **************************************************************************************************

uint8_t loraResponse(const char *textoBuscado, uint32_t timeoutMs, uint8_t exactMatch)
{
    uint32_t startTime = HAL_GetTick();
    uint8_t errorDetected = 0;
    char line[LORA_LINE_MAX];
    line[0] = '\0';

    while ((HAL_GetTick() - startTime) < timeoutMs) {
        while (Lora_TryGetLine(line, sizeof(line))) {
            if (strlen(line) == 0) continue;

//            dbg_print("LoRa-> ");
//            dbg_println(line);

            if (exactMatch) {
                if (strcmp(line, textoBuscado) == 0) return 1;
            } else {
                if (strstr(line, textoBuscado) != NULL) return 1;
            }

            if (strstr(line, "ERROR") != NULL) { errorDetected = 1; break; }
            if (strstr(line, "FAIL")  != NULL) { errorDetected = 1; break; }
            if (strstr(line, "TEST_PARAM_OVERFLOW") != NULL) { errorDetected = 1; break; }
            if (strstr(line, "NO_NETWORK_JOINED")    != NULL) { errorDetected = 1; break; }
            if (strstr(line, "BUSY_ERROR") != NULL) {
                HAL_Delay(500);
                timeoutMs += 3000;   /* Espera un poco más */
                continue;
            }
        }

        if (errorDetected) {
            dbg_print("LORA err: ");
            dbg_println(line);
            return 0;
        }
    }

    return 0;
}

uint8_t sendAndCheckLora(const char *command, const char *response, uint32_t timeoutMs, uint8_t exactMatch)
{
    Lora_ClearRx();

//    dbg_print("LoRa  <-");
//    dbg_println(command);

	HAL_UART_Transmit(UART_LORA, (uint8_t *)command, (uint16_t)strlen(command), HAL_MAX_DELAY);
	HAL_UART_Transmit(UART_LORA, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    return loraResponse(response, timeoutMs, exactMatch);
}

static uint8_t resetLora(void)
{
    HAL_UART_Transmit(UART_LORA, (uint8_t *)"ATZQ\r\n", 6, HAL_MAX_DELAY); /* No hay respuesta */
    HAL_Delay(250);

    sendAndCheckLora("AT+QVL=0", "OK", 200, 1);  /* Log level */
    return 1;
}

static uint8_t newLineLora(void)
{
    return sendAndCheckLora("ATQ", "OK", 250, 1);
}

static uint8_t checkConectionLora(uint8_t reconnectLora)
{
    if (conectionLora) return 1;

    if (!newLineLora()) {
        dbg_println("Error: No respuesta Lora");
        return 0;
    }
    Lora_ClearRx();

    if (sendAndCheckLora("AT+QSEND=1:1:FF", "SEND_CONFIRMED", 5000, 0)) {
        conectionLora = 1;
        dbg_println("LoRa enlazado.");
        return 1;
    }

    conectionLora = 0;

    if (reconnectLora) return connectLoRa();

    dbg_println("Error: LoRa no enlazado");
    return 0;
}

uint8_t connectLoRa(void)
{
    if (!newLineLora()) {
        dbg_println("Error: No respuesta Lora");
        return 0;
    }

    resetLora();
    sendAndCheckLora("AT+QVL=2", "OK", 200, 1);  /* momentáneamente */

    if (!sendAndCheckLora("AT+QJOIN=1", "MAC txDone", 2000, 0)) {
        dbg_println("LORA RESET DE FÁBRICA");
        HAL_UART_Transmit(UART_LORA, (uint8_t *)"AT+QRFS\r\n", 9, HAL_MAX_DELAY);
        HAL_Delay(500);
        setupLoRa();
        return conectionLora;
    }

    if (loraResponse("JOINED", 15000, 0)) {
        HAL_Delay(200);
        conectionLora = 1;
        sendAndCheckLora("AT+QVL=0", "OK", 200, 1);
        dbg_println("LoRa enlazado.");
        return 1;
    }

    sendAndCheckLora("AT+QVL=0", "OK", 200, 1);
    conectionLora = 0;
    dbg_println("Error: LoRa no enlazado");
    return 0;
}

static uint8_t SaveResetLora(void)
{
    if (!sendAndCheckLora("AT+QCS", "OK", 500, 1)) return 0;
    return resetLora();
}

uint8_t setupLoRa(void)
{
    newLineLora();
    Lora_ClearRx();
    HAL_Delay(200);

    if (!newLineLora()) {
        dbg_println("Error: No respuesta Lora");
        return 0;
    }

    sendAndCheckLora("AT+QVL=0", "OK", 200, 1);  /* Log level */

    uint8_t cambiosLora = 0;
    uint8_t err = 0;

    /* Región: US915 */
    if (!sendAndCheckLora("AT+QBAND=?", "QBAND:8", 200, 0)) {
        if (sendAndCheckLora("AT+QBAND=8", "OK", 200, 1)) cambiosLora = 1;
        else err++;
        HAL_Delay(100); /* AT+QBAND=8 reinicia el módulo automáticamente */
    }

    /* Sub-banda 2 (canales 8-15) */
    if (!sendAndCheckLora("AT+QCHAN=?", "FF00:0000:0000:0000:0000:0000", 200, 0)) {
        if (sendAndCheckLora("AT+QCHAN=FF00:0000:0000:0000:0000:0000", "OK", 200, 1)) cambiosLora = 1;
        else err++;
    }

    /* Clase LoRaWAN: Class A */
    if (!sendAndCheckLora("AT+QCLASS=?", "QCLASS: A", 200, 0)) {
        if (sendAndCheckLora("AT+QCLASS=A", "OK", 200, 1)) cambiosLora = 1;
        else err++;
    }

    /* Adaptive Data Rate */
    if (!sendAndCheckLora("AT+QADR=?", "QADR:1", 200, 0)) {
        if (sendAndCheckLora("AT+QADR=1", "OK", 200, 1)) cambiosLora = 1;
        else err++;
    }

    /* Low Power Mode */
    if (!sendAndCheckLora("AT+QLPMOD=?", "Lowpower mode is enabled!", 200, 0)) {
        if (sendAndCheckLora("AT+QLPMOD=1", "OK", 200, 1)) cambiosLora = 1;
        else err++;
    }

    if (cambiosLora) {
        SaveResetLora();
        dbg_println("LoRa configurado y reseteado.");
    }

    if (err) {
        dbg_print("Errores al configurar Lora: ");
        dbg_println_u32(err);
        return 0;
    }

    return 1;
}

uint8_t mandarPorLora(void)
{
    char hexData[(JSON_BUF_SZ * 2) + 1] = { 0 };
    codificarJsonToHex((const uint8_t *)jsonBuf, strlen(jsonBuf), hexData);

    if (!newLineLora()) {
        dbg_println("Error: No respuesta Lora");
        return 0;
    }
    Lora_ClearRx();

    HAL_UART_Transmit(UART_LORA, (uint8_t *)"AT+QSEND=1:1:", 13, HAL_MAX_DELAY);
    HAL_UART_Transmit(UART_LORA, (uint8_t *)hexData, (uint16_t)strlen(hexData), HAL_MAX_DELAY);
    HAL_UART_Transmit(UART_LORA, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    return loraResponse("SEND_CONFIRMED", 10000, 0);
}


// **************************************************************************************************
// Generar cadena: ID,numOrden,vidas,municion,batería,longitud,latitud,altitud,orientación,pasos,ack,timestamp
// **************************************************************************************************
uint8_t generarCadena(void)
{
    char latStr[16];
    char lonStr[16];
    char altStr[16];

    floatToStr(latitud,  5, latStr, sizeof(latStr));
    floatToStr(longitud, 5, lonStr, sizeof(lonStr));
    floatToStr(altitud,  2, altStr, sizeof(altStr));

    int written = snprintf(
        jsonBuf, sizeof(jsonBuf),
        "%d,%d,%d,%d,%d,%s,%s,%s,%d,%d,%d,%lu",
        ID, numOrden, vidas, municiones, bateria,
        latStr, lonStr, altStr,
        orientacion, pasos, ack, (unsigned long)timestamp
    );

    if (written < 0 || written >= (int)sizeof(jsonBuf)) {
        dbg_println("Error: cadena demasiado grande.");
        return 0;
    }

    dbg_print("Cadena a enviar: ");
    dbg_println(jsonBuf);
    return 1;
}

