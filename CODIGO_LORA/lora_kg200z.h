/*
 * lora_app.h
 *
 * Target: STM32L433
 *
 *  - UART2 (huart2) -> Módulo LoRa KG200Z (interrupción Rx NVIC)
 *  - UART3 (huart3) -> Debug (consola)
 *
 */

#ifndef LORA_APP_H
#define LORA_APP_H

#include <stdint.h>
#include <stddef.h>
#include "stm32l4xx_hal.h"

/* Funciones internas que ahora se usan también desde main.c */

void Lora_ArmReceive(void);

uint8_t setupLoRa(void);
uint8_t connectLoRa(void);
uint8_t sendAndCheckLora(const char *command, const char *response, uint32_t timeoutMs, uint8_t exactMatch);
uint8_t esperarDatoLora(char *hexPayloadOut, size_t maxLen, uint32_t timeoutMs);
uint8_t generarCadena(void);
uint8_t parsearDatosLora(const char *texto);
uint8_t mandarPorLora(void);
uint8_t loraResponse(const char *textoBuscado, uint32_t timeoutMs, uint8_t exactMatch);


/* Variables globales que ahora se usan también desde main.c */

extern uint8_t  numOrden;
extern uint8_t  ID;
extern char     equipo[15];
extern char     alias[15];
extern uint8_t  vidas;
extern uint16_t municiones;
extern uint16_t tiempo;
extern char     mac1[15];
extern char     mac2[15];

extern uint8_t  bateria;
extern float    latitud;
extern float    longitud;
extern float    altitud;
extern uint16_t orientacion;
extern uint16_t pasos;
extern uint8_t  ack;
extern uint32_t timestamp;


#endif /* LORA_APP_H */
