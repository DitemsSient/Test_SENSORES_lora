// =========================================================
// MODO DE OPERACIÓN — comentar los que no se usen
// =========================================================
//#define MODE_TESTER
//#define MODE_DECODER
#define MODE_CALIBRATE

// =========================================================
// CONFIGURACIÓN
// =========================================================
#define RX_PIN              A6
#define FRAME_BUF_SIZE      16u

// Umbrales de clasificación (lógica "menor que" en cascada)
// STM32 programa: BIT0=400, BIT1=800, INTER=1200, SYNC=1600
#define NOISE_THRESH        200u    // < 200  → ignorar (ruido / glitch)
#define BIT0_THRESH         600u    // < 600  → BIT 0
#define BIT1_THRESH         1050u   // < 1050 → BIT 1
#define INTER_THRESH        1600u   // < 1400 → INTER
                                    // ≥ 1400 → SYNC

// Código de calibración esperado
#define CAL_CODE_B0         0xABu
#define CAL_CODE_B1         0xCDu
#define CAL_CODE_B2         0xEFu

// =========================================================
// INCLUDES
// =========================================================
#include <Wire.h>
#include <stdint.h>
#include "LP55231_Direct.h"

// =========================================================
// VARIABLES GLOBALES
// =========================================================
static bool           estado_high = false;
static unsigned long  inicio_high = 0;

// --- Decoder ---
static uint8_t  dec_counter   = 0;     // bits + INTERs acumulados (reset en SYNC)
static uint8_t  dec_bit_count = 0;     // bits del byte en curso
static uint8_t  dec_byte_buf  = 0;     // byte en construcción
static uint8_t  dec_bytes[3];          // 3 bytes decodificados
static uint8_t  dec_byte_idx  = 0;

// =========================================================
// UTILIDADES
// =========================================================

static uint8_t ComputeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) chk ^= data[i];
    return chk;
}

// =========================================================
// LP55231 — Blink calibración
// =========================================================

static void LP_BlinkCalibration(void) {
    // Verde 1 segundo: canales R(0,3,6)=0, G(1,4,7)=180, B(2,5,8)=0
    for (uint8_t i = 0; i < 9; i++) {
        SetChannelPWM1(i, (i % 3 == 1) ? 180 : 0);
    }
    delay(1000);
    for (uint8_t i = 0; i < 9; i++) SetChannelPWM1(i, 0);
}

// =========================================================
// MODO TESTER
// =========================================================

static void Tester_ProcessTime(unsigned long duracion) {
    if (duracion < NOISE_THRESH) return;

    Serial.print("HIGH=");
    Serial.print(duracion);
    Serial.print(" us -> ");

    if (duracion < BIT0_THRESH) {
        Serial.println("BIT 0");
    } else if (duracion < BIT1_THRESH) {
        Serial.println("BIT 1");
    } else if (duracion < INTER_THRESH) {
        Serial.println("INTER");
    } else {
        Serial.println("SYNC");
    }
}

// =========================================================
// MODO CALIBRATE — imprime como TESTER + acumula bits + chequea al final
// =========================================================
// BIT0/BIT1: print + acumula bit + cont++
// INTER:     print + cont++
// SYNC:      print + cont=0 + reset buffer
// Al final de cada símbolo: si cont >= 27, imprime los 3 bytes acumulados

static uint8_t  cal_buf[3];
static uint8_t  cal_byte_actual   = 0;
static uint8_t  cal_contador_bits = 0;
static uint8_t  cal_idx           = 0;
static uint8_t  cal_counter       = 0;

static void Calibrate_AddBit(uint8_t bit) {
    cal_byte_actual = (cal_byte_actual << 1) | bit;
    cal_contador_bits++;
    if (cal_contador_bits == 8) {
        if (cal_idx < 3) cal_buf[cal_idx++] = cal_byte_actual;
        cal_byte_actual   = 0;
        cal_contador_bits = 0;
    }
}

static void Calibrate_ProcessTime(unsigned long duracion) {
    if (duracion < NOISE_THRESH) return;

    Serial.print("HIGH="); Serial.print(duracion); Serial.print(" us -> ");

    if (duracion < BIT0_THRESH) {
        Serial.println("BIT 0");
        Calibrate_AddBit(0);
        cal_counter++;
    }
    else if (duracion < BIT1_THRESH) {
        Serial.println("BIT 1");
        Calibrate_AddBit(1);
        cal_counter++;
    }
    else if (duracion < INTER_THRESH) {
        Serial.println("INTER");
        cal_counter++;
    }
    else {
        Serial.println("SYNC");
        cal_counter       = 0;
        cal_byte_actual   = 0;
        cal_contador_bits = 0;
        cal_idx           = 0;
        memset(cal_buf, 0, sizeof(cal_buf));
    }

    if (cal_counter >= 27) {
        Serial.print("  >> ["); Serial.print(cal_counter); Serial.print("] 0x");
        if (cal_buf[0] < 0x10) Serial.print("0");
        Serial.print(cal_buf[0], HEX);
        if (cal_buf[1] < 0x10) Serial.print("0");
        Serial.print(cal_buf[1], HEX);
        if (cal_buf[2] < 0x10) Serial.print("0");
        Serial.println(cal_buf[2], HEX);
        LP_BlinkCalibration();
    }
}

// =========================================================
// MODO DECODER  (híbrido: imprime como TESTER + decodifica)
// =========================================================
// Cuenta símbolos recibidos: BIT0 +1, BIT1 +1, INTER +1  → total esperado 27
// SYNC: si counter==27 imprime el hex y compara; luego resetea todo.

static void Decoder_Reset(void) {
    dec_counter   = 0;
    dec_bit_count = 0;
    dec_byte_buf  = 0;
    dec_byte_idx  = 0;
    memset(dec_bytes, 0, sizeof(dec_bytes));
}

static void Decoder_AddBit(uint8_t bit) {
    dec_byte_buf = (dec_byte_buf << 1) | bit;
    dec_bit_count++;
    if (dec_bit_count == 8) {
        if (dec_byte_idx < 3) dec_bytes[dec_byte_idx++] = dec_byte_buf;
        dec_byte_buf  = 0;
        dec_bit_count = 0;
    }
}

static void Decoder_TryDecode(void) {
    Serial.print("  >> Decoded ["); Serial.print(dec_counter); Serial.print("/27]: 0x");
    if (dec_bytes[0] < 0x10) Serial.print("0");
    Serial.print(dec_bytes[0], HEX);
    if (dec_bytes[1] < 0x10) Serial.print("0");
    Serial.print(dec_bytes[1], HEX);
    if (dec_bytes[2] < 0x10) Serial.print("0");
    Serial.print(dec_bytes[2], HEX);

    if (dec_counter == 27
        && dec_bytes[0] == CAL_CODE_B0
        && dec_bytes[1] == CAL_CODE_B1
        && dec_bytes[2] == CAL_CODE_B2) {
        Serial.println(" | MATCH — blink!");
        LP_BlinkCalibration();
    } else if (dec_counter != 27) {
        Serial.println(" | trama incompleta");
    } else {
        Serial.println(" | NO MATCH");
    }
}

static void Decoder_ProcessTime(unsigned long duracion) {
    if (duracion < NOISE_THRESH) return;

    Serial.print("HIGH="); Serial.print(duracion); Serial.print(" us -> ");

    if (duracion < BIT0_THRESH) {
        Serial.println("BIT 0");
        Decoder_AddBit(0);
        dec_counter++;
    }
    else if (duracion < BIT1_THRESH) {
        Serial.println("BIT 1");
        Decoder_AddBit(1);
        dec_counter++;
    }
    else if (duracion < INTER_THRESH) {
        Serial.println("INTER");
        dec_counter++;
    }
    else {
        Serial.println("SYNC");
        if (dec_counter > 0) {
            Decoder_TryDecode();
        }
        Decoder_Reset();
    }
}

// =========================================================
// SETUP / LOOP
// =========================================================
void setup() {
    Serial.begin(115200);
    pinMode(RX_PIN, INPUT);
    while (!Serial) {}

    Begin1();
    Enable1();
    for (uint8_t i = 0; i < 9; i++) SetDriveCurrent1(i, 0x4F);

#ifdef MODE_TESTER
    Serial.println("Modo: TESTER");
#elif defined(MODE_DECODER)
    Serial.println("Modo: DECODER");
#else
    Serial.println("Modo: CALIBRATE");
#endif
}

void loop() {
    int lectura = digitalRead(RX_PIN);

    if (lectura == HIGH && !estado_high) {
        estado_high = true;
        inicio_high = micros();
    }

    if (lectura == LOW && estado_high) {
        estado_high = false;
        unsigned long duracion = micros() - inicio_high;

#ifdef MODE_TESTER
        Tester_ProcessTime(duracion);
#elif defined(MODE_DECODER)
        Decoder_ProcessTime(duracion);
#else
        Calibrate_ProcessTime(duracion);
#endif
    }
}
