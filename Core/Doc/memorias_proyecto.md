# Memorias del Proyecto

## Contexto General

Sistema táctico que simula gotcha mediante impactos por transmisión láser (IR). El sistema está compuesto por dos PCBs que trabajan en conjunto, montadas sobre el usuario. Ambas PCBs utilizan microcontroladores STM32F439ZI y se comunican entre sí por RS485.

---

## PCB 1 — Mira (Frontal)

Función principal: **transmitir un mensaje a través de un LED IR** con el ID del atacante, simulando el disparo/bala.

Componentes y funciones clave:
- **LED IR**: Emisor del "disparo". Transmite un paquete con el ID del jugador atacante.
- **Pantalla OLED**: Proyecta diferentes menús antes y durante el ejercicio (configuración, estado del juego, HUD).
- **Bluetooth**: Comunicación con la PCB 2 (sensores/recepción).
- **Sensores y actuadores complementarios**: LEDs RGB, vibrador, sensor de luz, IMU, entre otros. No críticos para la funcionalidad principal.

---

## PCB 2 — Sensores / Recepción (Trasera)

Función principal: **recibir impactos IR, geolocalizar al usuario y transmitir datos en tiempo real**.

Componentes y funciones clave:
- **Sensor IR**: Recibe el impacto láser y decodifica el ID del atacante.
- **Módulo GPS (L86-M33)**: Ubicación exacta del usuario en tiempo real.
- **Módulo LoRa**: Transmite información del juego a base de datos remota.
- **Módulo Bluetooth**: Comunicación con la PCB 1.
- **RS485**: Comunicación con PCB de respaldo trasera (impactos por espalda, micro independiente).

> **Nota:** La sección "Sistema de Menús OLED (PCB 1 — Mira)" que existía aquí (menú, pantallas,
> Test HW por botones) se retiró de este repo por pertenecer exclusivamente a la PCB 1 (Mira).
> Esta tarjeta (Sensores) no tiene OLED ni botones físicos.
