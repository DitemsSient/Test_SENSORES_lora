
#include "main.h"
#include "helpers.h"

#include <stdio.h>
#include <string.h>
//#include <stm32l4xx_hal_uart.h>

/* Convierte un float a texto con N decimales, sin usar printf con %f
 * (evita tener que activar "Use float with printf" en el linker).
 * Redondea al decimal más cercano. */
void floatToStr(float val, uint8_t decimals, char *out, size_t outSize)
{
    if (val < 0.0f) {
        *out++ = '-';
        outSize--;
        val = -val;
    }

    long mult = 1;
    for (uint8_t i = 0; i < decimals; i++) mult *= 10;

    long scaled  = (long)(val * (float)mult + 0.5f);
    long intPart = scaled / mult;
    long fracPart = scaled % mult;

    snprintf(out, outSize, "%ld.%0*ld", intPart, (int)decimals, fracPart);
}



// **************************************************************************************************
// ***************************************** HEX handles ********************************************
// **************************************************************************************************
static uint8_t hexCharToVal(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

size_t decodeHexToBytes(const char *hexStr, uint8_t *output, size_t maxLen)
{
    size_t len = strlen(hexStr);
    size_t outLen = 0;

    for (size_t i = 0; (i + 1) < len && outLen < maxLen; i += 2) {
        uint8_t hiVal = hexCharToVal(hexStr[i]);
        uint8_t loVal = hexCharToVal(hexStr[i + 1]);
        output[outLen++] = (uint8_t)((hiVal << 4) | loVal);
    }
    return outLen;
}

void codificarJsonToHex(const uint8_t *data, size_t datalen, char *output)
{
    static const char hex_chars[] = "0123456789ABCDEF";
    for (size_t i = 0; i < datalen; ++i) {
        output[i * 2]     = hex_chars[(data[i] >> 4) & 0x0F];
        output[i * 2 + 1] = hex_chars[data[i] & 0x0F];
    }
    output[datalen * 2] = '\0';
}



// **************************************************************************************************
// ************************************** Debug por USART3 ******************************************
// **************************************************************************************************

void dbg_print(const char *s)
{
    HAL_UART_Transmit(UART_DEBUG, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void dbg_println(const char *s)
{
    dbg_print(s);
    dbg_print("\r\n");
}

void dbg_println_u32(uint32_t v)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
    dbg_println(buf);
}
