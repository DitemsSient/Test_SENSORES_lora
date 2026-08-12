# Referencia de Bajo Consumo — ProyectoPruebas_STM32

**Plataforma:** STM32F439ZI  
**Fecha:** Mayo 28, 2026  
**Rama de desarrollo:** feature/BajoConsumo

---

## Módulos con soporte de bajo consumo

| Módulo | IC / Componente | Modo bajo consumo | Consumo aprox. | API implementada |
|--------|----------------|-------------------|----------------|-----------------|
| Flash | MX25L6445E | Deep Power-Down | ~1 µA | `Flash_PowerDown()` / `Flash_WakeUp()` |
| GPS | L86-M33 | Standby (PMTK161) | ~20 µA | `Gps_SendMTK(GPS_CMD_STANDBY)` / `GPS_CMD_HOT_RESTART` |
| IMU | ICM-20948 | Sleep bit + AK09916 power-down | ~8 µA | `ICM20948_Sleep()` / `ICM20948_WakeUp()` |
| Sensor de luz | TSL2571 | ALS disable (PON+AEN=0) | <1 µA | `TSL2571_Disable()` / `TSL2571_Enable()` |
| Display OLED | SSD1306 | Display OFF (0xAE) | ~10 µA | `ssd1306_sleep()` / `ssd1306_wakeup()` |
| Gauge batería | BQ27441 | Hibernate | ~1 µA | `BatGauge_Hibernate()` / `BatGauge_WakeUp()` |
| LED RGB | LP55231 | Disable (EN pin LOW) | ~1 µA | `LP55231_Disable()` / `LP55231_Enable()` |
| LoRa | RM1262 | Sleep (AT command) | <10 µA | `Lora_Sleep()` / `Lora_WakeUp()` |
| Bluetooth | BL654 (Nordic nRF52) | Auto-sleep (Standby Doze) | ~3.1 µA | Sin comando — automático |


---

## Módulos SIN soporte de bajo consumo

| Módulo | Razón |
|--------|-------|
| **LED RGB simple** (`LedRGB`) | GPIO discreto — se apaga con `LedRGB_Off()`, no tiene IC con modo sleep |
| **Buzzer** | PWM sobre GPIO — se detiene con `Buzzer_Stop()`, sin IC dedicado |
| **Motor vibrador** (`Motovibrador`) | GPIO discreto — se apaga con `Vibrator_Off()`, sin IC dedicado |
| **Sensor Hall** (`SensorHall`) | Sensor pasivo analógico/digital, sin interfaz de control de consumo |
| **Multiplexor CD4051B** | IC combinacional sin modo sleep — siempre activo mientras tenga VCC |
| **RS-485** (`Comunicacion_RS485`) | Transceptor sin pin de shutdown configurado en esta aplicación |
| **Transmisor IR** (`Transmsion_Laser_IR`) | GPIO discreto — se apaga deteniendo la señal PWM |
| **Receptor IR TSOP** (`Receptor_Infrarrojo_TSOP`) | Sin interfaz de control — siempre activo |
| **Demo_Luz_Multiplexor_Potencia** | Módulo de aplicación (capa lógica), no driver de IC |

> **Nota:** los actuadores discretos (LED, Buzzer, Vibrador) se apagan en `PowerManager_SuspendAll()` antes de cualquier IC.

---

## Coordinador: PowerManager

Archivo: `Core/Inc/PowerManager.h` / `Core/Src/PowerManager.c`

```c
// Inicialización — una vez, después de todos los Init()
PowerManager_Init(&hgps, &icm, &tsl, &rgb, &hlora);

// Suspender todo
PM_Result_t result;
PowerManager_SuspendAll(&result);   // retorna PM_OK o PM_ERR_PARTIAL

// Despertar todo
PowerManager_WakeAll(&result);

// Bajo consumo del MCU — NO IMPLEMENTADO AÚN
PowerManager_MCUSleep();            // stub vacío
```

### Orden de suspend

1. LedRGB_Off / Buzzer_Stop / Vibrator_Off
2. Flash → Deep Power-Down
3. GPS → Standby + FORCE_ON LOW
4. ICM-20948 → Sleep
5. TSL2571 → Disable
6. BQ27441 → Hibernate
7. LP55231 → Disable
8. LoRa → AT+SLEEP
9. SSD1306 → Display OFF
10. BL654 → automático (sin comando)

### Orden de wake

1. Flash → Wake
2. BQ27441 → Wake
3. ICM-20948 → Wake
4. TSL2571 → Enable
5. LP55231 → Enable
6. LoRa → byte dummy (0xFF)
7. GPS → FORCE_ON + HOT_RESTART
8. SSD1306 → Display ON
9. BL654 → automático

---


