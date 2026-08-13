# Tareas de Pruebas — Tarjeta Sensores

Registro de las tareas de prueba corridas en `Core/Src/main.c` sobre esta tarjeta
(STM32L452CEUx). Cada tarea se prueba una a la vez: se activa, se compila, se
saca el `.elf`, se revisa en hardware, y solo se pasa a la siguiente cuando se
confirma. Las tareas anteriores quedan comentadas en el código (no borradas)
para referencia.

---

## Resueltas / corridas

| # | Tarea | Objetivo | Periférico | Notas |
|---|-------|----------|------------|-------|
| 1 | UART de RS485 (Logger) | Confirmar `Log_Print`/`Log_Printf` por UART4 (PA0/PA1), 115200 8N1 | `huart4` | Base para ver el resultado de las demás tareas |
| 2 | Escaneo de bus I2C1 | Barrer 0x01–0x7E y distinguir bus vacío de bus atascado | `hi2c1` | **En pausa** — ver sección de pendientes |
| 3 | Buzzer (melodía) | Tocar `ode_to_joy` completa una vez al boot | `htim3` CH2 / PA7 | Corrida, código queda comentado |
| 4 | Motovibrador | Encender el motor ERM 5 s al boot (modo GPIO) | PH0 | Corrida, código queda comentado |
| 5 | Flash SPI (MX25L6445E) | Escribir `"MENSAJE EN LA FLASH DE PRUEBA"` una vez y releerlo cada 3 s | `hspi2`, CS PB5 | Corrida, código queda comentado |
| 6 | LoRa (RM1262) — AT por poleo | Mandar `"AT\r\n"` cada 3 s y buscar `"OK"` en la respuesta | `huart2` | Por poleo, sin `Lora_Init()` (evita `HAL_UART_Receive_IT`). Corrida, código queda comentado |
| 7 | GPS (L86-M33) — lectura cruda por poleo | Forzar encendido (`Gps_ForceOn()`) y leer bytes crudos del UART cada ciclo, buscando texto NMEA (`$...`) | `huart1` | Por poleo, sin `Gps_Init()`. El GPS no usa comandos AT — transmite NMEA por su cuenta. **Activa actualmente** |

---

## Pendientes

Del listado completo de drivers de esta tarjeta, falta correr:

- **Bluetooth (BL654) — AT por poleo.** Mismo patrón que LoRa: enlazar el handle a mano (sin `Bt_Init()`, que arma `HAL_UART_Receive_IT`), mandar `"AT\r\n"` cada 3 s por `huart3`, buscar `"OK"`.
- **RS485.** `RS485_Init()` + `RS485_Send()` de una trama mínima. **Bloqueado**: el Logger comparte hoy el mismo UART (`huart4`) con RS485 — hay que quitarle el UART al Logger (o dejarlo mudo) antes de esta prueba para no interferir. Programado para el jueves según lo platicado.
- **Receptor IR (2 sensores, PH3/PB0).** **Bloqueado de fondo**: el `.ioc` no tiene ningún timer configurado en modo Input Capture — el driver (`Receptor_Infrarrojo_TSOP`) lo necesita para funcionar. Hay que habilitar uno en CubeMX antes de poder siquiera correr esta tarea.
- **PowerManager (Suspend/Wake).** Prueba final, una vez que los drivers individuales que coordina (Flash, GPS, IMU, luz, gauge, LoRa) ya estén validados por separado.

## En pausa (no correr todavía)

- **Todo lo relacionado a I2C1**: escaneo (Tarea 2), BatteryMonitor (BQ27441), LSM6DSO32TR, MMC5983MA, SensorLuz_TSL2571, Driver_RGB (LP55231). El escaneo I2C en la tarjeta Mira dio problema de bus — no se retoma nada de I2C en esta tarjeta hasta que eso se valide.
- **ModoProgramacion.** `ModoProgramacion.h` sigue apuntando a `GPIOF/PIN_12`, que no existe en este MCU (UFQFPN48 no tiene GPIOF). Pendiente de que se defina el pin real (candidatos vistos en el `.ioc`: PB1 `SEL_PROG_LORA_O_BLE` / PB2 `SEL_PROG_MCU_O_ModulosFTDI`) antes de poder probarlo.
