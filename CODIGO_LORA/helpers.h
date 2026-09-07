#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include <stdint.h>
#include <stddef.h>

void floatToStr(float val, uint8_t decimals, char *out, size_t outSize);


size_t decodeHexToBytes(const char *hexStr, uint8_t *output, size_t maxLen);
void codificarJsonToHex(const uint8_t *data, size_t datalen, char *output);


void dbg_print(const char *s);
void dbg_println(const char *s);
void dbg_println_u32(uint32_t v);

#endif /* INC_HELPERS_H_ */
