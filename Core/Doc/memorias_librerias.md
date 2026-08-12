# Memorias de Librerías

Cada sección corresponde a un driver. El título (`## nombre`) es el hint que se pasa
entre paréntesis en `/new-rama <rama> (nombre)`.

---

## buzzer

Controla un buzzer pasivo para generar tonos y melodías musicales mediante PWM. Permite reproducir notas individuales, secuencias completas con tempo configurable y silencios. Es la capa de retroalimentación sonora del sistema (alertas, confirmaciones, boot).

**Archivo:** `Buzzer.h` / `Buzzer.c` · **Versión:** 1.0.0  
**Componente:** Buzzer pasivo via PWM  
**Timer:** `htim4`, canal `TIM_CHANNEL_1`  
**Clock efectivo:** `BUZZER_TIMER_CLK = 1 000 000 Hz` (APB1 / (prescaler+1))

Estructura de nota: `BuzzerNote_t { int16_t note [Hz], int16_t duration }`.  
Duration negativo = nota con puntillo (×1.5).  
Macro de longitud: `MELODY_LEN(arr)`.

**API:**
- `Buzzer_Init()` — arranca PWM, llamar tras `MX_TIM4_Init()`
- `Buzzer_PlayTone(freq)` — tono continuo (0 = silencio)
- `Buzzer_Stop()` — silencia inmediatamente
- `Buzzer_PlayMelody(melody, length, bpm)` — bloqueante, usa `HAL_Delay`
- `Buzzer_Test()` — tono A4 continuo 5 s (para uso del sistema TestHW)

**Melodías predefinidas** (en `Buzzer_Melodias.h`): `pink_panther`, `ode_to_joy`,
`alert`, `boot_ok`. Uso: `Buzzer_PlayMelody(boot_ok, MELODY_LEN(boot_ok), 140)`.

Notas disponibles: C4–B5 + C6/D6/E6 + `SILENCE`. Duraciones: `WHOLE`, `HALF`,
`QUARTER`, `EIGHTH`, `SIXTEENTH`.

---

## motovibrador

Controla un motor de vibración ERM/moneda para retroalimentación háptica. Soporta encendido/apagado simple (GPIO) e intensidad variable (PWM). Incluye patrones predefinidos como tick, notificación, alerta, llamada y SOS.

**Archivo:** `Motovibrador.h` / `Motovibrador.c` · **Versión:** 1.0.0  
**Componente:** Motor ERM/moneda  
**Modo activo:** `VIBRATOR_MODE_GPIO` (PC10). PWM disponible pero comentado (htim3, CH1, ARR=999).

Intensidad 0–100 %. En modo GPIO: 0=OFF, >0=ON.  
Tipo: `VibStep_t { uint8_t intensity, uint16_t duration_ms }`.  
Macro: `VIB_PATTERN_LEN(arr)`. Gap entre pasos: `VIBRATOR_STEP_GAP_MS = 10 ms`.

**Patrones predefinidos:** `VIB_PATTERN_TICK`, `_NOTIFICATION`, `_ALERT`, `_CALL`,
`_RAMP`, `_HEARTBEAT`, `_SOS` (con sus `_LEN`).

**API:**
- `Vibrator_Init()` — init GPIO o PWM según define activo
- `Vibrator_Set(intensity)` — 0–100 %
- `Vibrator_On()` / `Vibrator_Off()`
- `Vibrator_PlayPattern(pattern, length)` — bloqueante
- `Vibrator_Test()` — motor ON 5 s (para uso del sistema TestHW)

---

## SensorLuz

Driver para el sensor de luz ambiente TSL2571 via I2C. Mide la iluminación en lux a partir de dos canales ADC internos (broadband e IR). Permite ajustar ganancia y tiempo de integración, y promediar N lecturas para mayor estabilidad.

**Archivo:** `SensorLuz_TSL2571.h` / `SensorLuz_TSL2571.c` · **Versión:** 1.0.0  
**Componente:** TSL2571 sensor de luz ambiente  
**Bus:** I2C, dirección 7-bit `0x39`

Handle: `TSL2571_t { hi2c, addr, timeout_ms, atime, gain }`.  
Raw: `TSL2571_RawData_t { ch0 (broadband), ch1 (IR), saturated }`.  
Ganancias: `TSL2571_GAIN_1X/8X/16X/120X`.

**API:**
- `TSL2571_Attach(dev, hi2c, addr, timeout_ms)` — binding
- `TSL2571_Begin(dev, atime, gain)` — power on + ALS enable
- `TSL2571_ReadLux(dev, nSamples, gapMs, &lux, &raw)` — promedio + conversión a lux
- `TSL2571_ReadRawChannels(dev, &raw)` — lectura directa CH0/CH1
- `TSL2571_SetGain / SetATime / Enable / Disable / WriteReg / ReadReg`
- `TSL2571_Test()` → `uint8_t` — I2C ACK + ch0 > 0 (TestHW; usa `extern TSL2571_t tsl`)

Fórmula integración: `Tint = 2.7296 × (256 − ATIME) ms`.

---

## RS485

Implementa comunicación half-duplex RS485 con trama propia (SOF + payload + checksum + EOF). Serializa y deserializa una estructura de datos con cmd, dos variables, un flag y texto. La recepción es byte a byte desde ISR. Se usa para comunicación entre PCBs del sistema.

**Archivo:** `Comunicacion_RS485.h` / `Comunicacion_RS485.c` · **Versión:** 1.0.0  
**Componente:** Comunicación RS485 half-duplex  
**UART:** `huart1` (deshabilitado en test — compartido con GPS)  
**Pin DE/RE:** PA11

Trama: `[ SOF1('-') | SOF2('*') | payload | CHK | EOF1('-') | EOF2('*') ]`  
Payload: `RS485_Data_t { cmd, var1, var2, flag, *text, len_text }` (`__packed`).  
Recepción byte a byte: `RS485_frame_t { cont_frame, frame_ready, rx_byte, *buff_frame, len_frame }`.

**API:**
- `RS485_Init(h, huart)` / `RS485_SetTx()` / `RS485_SetRx()`
- `RS485_Send(h, data)` — serializa y transmite
- `RS485_Parse(frame, data)` — llama desde main loop cuando `frame_ready == true`
- `RS485_StoreByte(frame)` — llama desde ISR UART
- `RS485_PackPayload / UnpackPayload / Checksum8 / size_frame / size_payload`
- `RS485_Test()` → `uint8_t` — envía trama mínima, verifica HAL_OK (TX only; TestHW)

---

## IR_Receptor

Recibe y decodifica tramas IR del protocolo propio del sistema usando Timer Input Capture. Detecta el impacto del "disparo" y extrae el ID del atacante. Cuenta con modo tester para calibración de tiempos y tolerancias.

**Archivo:** `Receptor_Infrarrojo_TSOP.h` / `Receptor_Infrarrojo_TSOP.c` · **Versión:** 1.0.0  
**Componente:** Receptor IR TSOP via Timer Input Capture  
**Timer IC:** `htim2`, canal `TIM_CHANNEL_1` (BOTHEDGE)  
**Pin TSOP:** PA1

Mismo protocolo que transmisor (MARK/SPACE0/SPACE1/INTER/SYNC).  
Tolerancia: ±200 µs en todos los rangos.  
Handle: `ir_receiver_t` (no manipular directamente).  
Config: `ir_config_t { *htim, tim_channel }`.

**API:**
- `IR_Init(receiver, config)` / `IR_DeInit(receiver)`
- `IR_CaptureCallback(receiver, htim)` — llamar desde `HAL_TIM_IC_CaptureCallback`
- `IR_Process(receiver)` — llamar en main loop
- `IR_FrameReady_Callback(data, len)` — implementar en `main.c` (callback usuario)
- `IR_GetLastError / ResetParser / IR_ClassifySpace / IR_ComputeChecksum`
- Modo tester: `IR_Tester_Init / _CaptureCallback / _Analyze / _Reset`

---

## Bluetooth

Interfaz UART para módulos Bluetooth serie (HC-05/HC-06/HM-10). Permite transmitir y recibir datos de forma bloqueante, con acumulación byte a byte desde ISR. Se usa para la comunicación entre PCB1 (mira) y PCB2 (sensores).

**Archivo:** `Bluetooth.h` / `Bluetooth.c` · **Versión:** 1.0.0  
**Componente:** Módulo Bluetooth UART (HC-05/HC-06/HM-10)  
**UART:** `huart1` (deshabilitado en test — compartido con GPS)

Buffers TX/RX: 256 bytes. Handle: `Bt_Handle_t { huart, tx_buf, rx_buf, rx_count, rx_byte, rx_ready }`.

**API:**
- `Bt_Init(h)` — binding UART
- `Bt_Transmit(h, data, len)` — bloqueante
- `Bt_Receive(h, data, len)` — bloqueante
- `Bt_StoreByte(h)` — llamar desde ISR UART
- `Bt_ResetRx(h)` — limpia buffer de recepción
- `Bt_Test()` → `uint8_t` — envía `AT\r\n`, busca "OK" en respuesta con timeout (TestHW)

---

## Lora

Interfaz UART para módulo LoRa de largo alcance. Transmite datos del juego (vidas, impactos, posición GPS) a una base de datos remota durante el ejercicio. Estructura idéntica al driver Bluetooth pero independiente para poder habilitarlos por separado.

**Archivo:** `Lora.h` / `Lora.c` · **Versión:** 1.0.0  
**Componente:** Módulo LoRa UART  
**UART:** `huart1` (deshabilitado en test — compartido con GPS)

Buffers TX/RX: 256 bytes. Handle: `Lora_Handle_t { huart, tx_buf, rx_buf, rx_count, rx_byte, rx_ready }`.

**API:**
- `Lora_Init(h)` / `Lora_Transmit(h, data, len)` / `Lora_Receive(h, data, len)`
- `Lora_StoreByte(h)` — desde ISR · `Lora_ResetRx(h)`

---

## Flash

Driver para memoria NOR Flash SPI MX25L6445E (8 MB). Ofrece tres niveles de API: comandos directos, operaciones seguras con manejo automático de páginas/Write Enable, y utilidades de diagnóstico. Se usa para almacenamiento persistente de configuración o logs.

**Archivo:** `Flash.h` / `Flash.c` · **Versión:** 1.0.0  
**Componente:** NOR Flash MX25L6445EZNI (Macronix 64 Mbit = 8 MB) via SPI  
**SPI:** `hspi1` · **CS:** PA8  
**JEDEC ID:** Manufacturer `0xC2`, Device `0x2017`

Geometría: página 256 B, sector 4 KB, bloque 32/64 KB. Rango: `0x000000–0x7FFFFF`.  
Regla crítica: Write solo vuelca 1→0; para volver 0→1 hay que borrar sector primero.

**Issue conocido:** velocidad SPI alta causa corrupción en lecturas. Usar `/16` o `/32
como prescaler para testing.

**API — Nivel 3 (utilidades):**  
`Flash_Init / ReadID / IsBusy / WaitBusy / UnprotectAll / PowerDown / WakeUp`

**API — Nivel 1 (comandos directos):**  
`Flash_ReadRaw / PageProgram / EraseSector / EraseBlock32K / EraseBlock64K / EraseChip`

**API — Nivel 2 (operaciones seguras):**  
`Flash_Read / Flash_Write` (maneja fragmentación de páginas)  
`Flash_ModifySector` — read-patch-erase-rewrite dentro de un sector (requiere 4 KB RAM)

**Self-test:**  
`Flash_Test()` → `uint8_t` — erase sector `0x7FF000` → write 16 × `0xA5` → read → compara → erase; retorna 1 si coincide

---

## GPS

Parser NMEA completo para el módulo L86-M33. Extrae posición, altitud, velocidad, rumbo, satélites, calidad de fix y fecha/hora local desde sentencias GPRMC y GPGGA. Incluye interfaz de comandos MTK (fix rate, salida, restart, standby) y sistema de tracking de distancia acumulada por Haversine.

**Archivo:** `GPS.h` / `GPS.c` · **Versión:** 3.0.0  
**Componente:** Módulo GPS L86-M33  
**UART:** `huart1` · **FORCE_ON:** PA12 (HIGH = activo)  
**Offset UTC:** `GPS_UTC_OFFSET_H = -6` (CST México)

Modelo de recepción: ISR acumula bytes → `Gps_StoreByte()` → `sentence_ready` → `Gps_Process()` en main loop.  
Sentencias: GPRMC (posición, velocidad, fecha) + GPGGA (altitud, satélites, HDOP, fix).

Handle: `Gps_Handle_t`:
- `data` (`GpsData_t`) — lat, lon, alt, speed_kmh, course, satellites, fix_quality, hdop, fecha/hora local, `position_str`
- `tracking_active`, `total_distance` (m), `last_lat`, `last_lon` — sistema de tracking

**Fix rates:** `GPS_FIX_0_5HZ / 1HZ / 5HZ / 10HZ`  
**Output presets:** `GPS_OUTPUT_RMC_GGA / RMC_ONLY / ALL / DEFAULT`  
**Comandos MTK:** restart (hot/warm/cold/full), standby, baud rate, AIC, constelaciones (GPS+GLONASS)

**API:**
- `Gps_Init(h)` — FORCE_ON + arm ISR
- `Gps_Process(h)` — parsea sentencia pendiente, retorna `GPS_OK` o `GPS_ERR_NO_FIX`
- `Gps_StoreByte(h)` — desde `HAL_UART_RxCpltCallback`
- `Gps_SendMTK(h, cmd)` — calcula checksum y envía comando PMTK
- `Gps_CheckWiring / Gps_Reset / Gps_FormatPosition`
- `Gps_ForceOn / Gps_ForceOff`
- `Gps_StartTracking(h)` — activa tracking, reinicia `total_distance`, guarda coord inicial
- `Gps_UpdateTracking(h)` — Haversine entre puntos, acumula si delta ∈ [3 m, 80 m]; llamar cada 3 s
- `Gps_StopTracking(h)` — desactiva sin borrar `total_distance`

Debug: `#define GPS_DEBUG` compila `GpsDebug_t` con contadores y último sentence.

---

## LSM6DSO32TR

Driver para el IMU de 6 ejes ST LSM6DSO32TR (acelerómetro + giroscopio) via I2C. Entrega los seis ejes en dos transacciones burst de 6 bytes cada una, aplica calibración de bias del giroscopio y cuenta con recuperación automática por SW reset ante errores I2C. Soporta modo power-down (ODR=0) para bajo consumo.

**Archivo:** `LSM6DSO32TR.h` / `LSM6DSO32TR.c` · **Versión:** 1.0.0  
**Componente:** IMU 6 ejes ST LSM6DSO32TR (accel + gyro)  
**Bus:** I2C1 (`hi2c1`) · **Dirección:** `0x6A` (SA0=GND)

Configuración fija: accel ±16 g / gyro ±500 dps, ODR 104 Hz HP, BDU habilitado.  
WHO_AM_I esperado: `0x6C`.

Handle: `LSM6DSO32TR_t { cal, initialized, consecutive_errors, total_recoveries }`.  
Datos: `LSM_Data_t { ax/ay/az_raw, gx/gy/gz_raw, ax/ay/az_g, gx/gy/gz_dps, temp_c }`.  
Calibración: `LSM_Cal_t { gx/gy/gz_bias_dps, calibrated }`. Bias aplicado automáticamente en `ReadAll`.

**Recuperación automática I2C:** a los 3 errores consecutivos ejecuta SW reset + reconfiguración, preserva calibración.  
Bajo consumo: `PowerDown` escribe ODR=0 en CTRL1_XL y CTRL2_G; `PowerOn` restaura la configuración original.

**API:**
- `LSM6DSO32TR_Init(dev)` — SW reset + WHO_AM_I + configura accel/gyro/BDU
- `LSM6DSO32TR_WhoAmI(dev, &id)` — retorna 0x6C
- `LSM6DSO32TR_CalibrateGyroBias(dev)` — 200 muestras × 10 ms ≈ 2 s en reposo
- `LSM6DSO32TR_ReadAll(dev, &out)` — burst gyro + burst accel + temperatura, bias corregido
- `LSM6DSO32TR_Recover(dev)` — SW reset + reconfig, preserva cal
- `LSM6DSO32TR_PowerDown(dev)` — ODR=0 en ambos sensores (~5 µA)
- `LSM6DSO32TR_PowerOn(dev)` — restaura ODR 104 HP
- `LSM6DSO32TR_Test(dev)` — WHO_AM_I + una lectura completa

---

## MMC5983MA

Driver para el magnetómetro de 3 ejes MEMSIC MMC5983MA via I2C. Opera en modo continuo a 10 Hz con auto SET/RESET activado por hardware en cada medición. Entrega 18 bits de resolución por eje, detecta saturación automáticamente y emite un SET pulse de recuperación cuando es necesario.

**Archivo:** `MMC5983MA.h` / `MMC5983MA.c` · **Versión:** 1.0.0  
**Componente:** Magnetómetro 3 ejes MEMSIC MMC5983MA  
**Bus:** I2C1 (`hi2c1`) · **Dirección:** `0x30` (SA0=GND)

ODR activo: 10 Hz (definido en `MMC_ODR_ACTIVE`). Auto SET/RESET habilitado (CTRL0 bit 5).  
Resolución: 18 bits por eje (0–262143); cero de campo en `131072` (2^17). Sensibilidad: 163.84 counts/µT.  
Product ID esperado: `0x30`.

Datos: `MMC_Data_t { x/y/z_uT, x/y/z_raw }`.

**SET pulse:** aplicado en `Init` y automáticamente en cada muestra (modo auto). Si se detecta saturación en cualquier eje durante `ReadAll`, se emite un SET adicional y se relee.

**API:**
- `MMC5983MA_Init()` — SET pulse + verifica ID + activa modo continuo con auto SR
- `MMC5983MA_ReadAll(&out)` — burst de 7 bytes, ensambla 18 bits, convierte a µT; SET automático si satura
- `MMC5983MA_Set()` — SET pulse manual (desgausado), bloquea 1 ms
- `MMC5983MA_WhoAmI(&id)` — retorna 0x30
- `MMC5983MA_Test()` — WHO_AM_I + lectura + validación de rango ±800 µT

---

## BatteryMonitor

Monitorea el estado de la batería Li-Ion mediante el gauge BQ27441-G1 (I2C), que reporta voltaje, corriente, SOC, capacidad y SOH. Incluye también una lectura analógica secundaria via ADC conectada a un divisor resistivo.

**Archivo:** `BatteryMonitor.h` / `BatteryMonitor.c` · **Versión:** 1.0.0  
**Componente:** Gauge BQ27441-G1 (I2C) + lectura analógica ADC  
**I2C:** `hi2c1`, dirección 7-bit `0x55` · **ADC pot:** `hadc1`, `ADC_CHANNEL_0`  
**Batería diseño:** 400 mAh / 1480 mWh / terminate 3000 mV

`BatGauge_Data_t { voltage_mV, avg_current_mA, soc_pct, remaining_mAh, full_cap_mAh, soh_pct, temp_c10, flags }`.

**API:**
- `BatGauge_Init()` — verifica device type `0x0421`
- `BatGauge_ReadAll(&data)` — todos los registros clave
- `BatGauge_ReadReg(reg, &val)` — registro individual
- `BatGauge_Control(subcmd, &val)` — sub-comandos de control
- `BatGauge_SoftReset()` — reset suave (exit CFGUPDATE si queda atascado)
- `BatGauge_Configure()` — escribe parámetros de batería en NVM (solo una vez)
- `BatAdc_ReadVoltage_mV()` / `BatAdc_ReadRaw()` — lectura ADC del divisor

---

## Driver_RGB

Controla el chip LP55231, un controlador I2C de 9 canales LED con PWM individual por canal, faders maestros y curva logarítmica. Se usa para iluminación RGB de alta resolución (efectos, estados del juego) en la PCB principal.

**Archivo:** `Driver_RGB.h` / `Driver_RGB.c` · **Versión:** 1.0.0  
**Componente:** Controlador LP55231 (9 canales LED) via I2C  
**Bus:** I2C1 · **Dirección 7-bit:** `0x32`

Handle: `LP55231_t { *hi2c, addr, timeout_ms }`.  
9 canales identificados con `LP55231_Led_e { LP_CH1..LP_CH9 }` (enum renombrado para evitar
conflicto con notas musicales `D4/D5/D6` de Buzzer.h), 3 master faders.

**API:**
- `LP55231_Attach(dev, hi2c, addr, timeout_ms)` — binding
- `LP55231_Begin(dev)` — reset + prepare
- `LP55231_Enable / Disable / Reset`
- `LP55231_SetChannelPWM(dev, channel, value)` — PWM 0–255
- `LP55231_SetDriveCurrent(dev, channel, value)` — corriente por canal
- `LP55231_SetMasterFader(dev, fader, value)` — fader maestro (0–2)
- `LP55231_SetLogBrightness(dev, channel, enable)` — curva logarítmica
- `LP55231_AssignChannelToMasterFader(dev, channel, fader)`
- `DriverRGB_Test()` — cicla R→G→B × 2 (1.5 s/color = 9 s) en todos los LEDs (TestHW)

Mapeo de canales asumido: R=LP_CH1/LP_CH4/LP_CH7, G=LP_CH2/LP_CH5/LP_CH8, B=LP_CH3/LP_CH6/LP_CH9.
Ajustar arrays `ch_red/ch_green/ch_blue` en `Driver_RGB.c` si el layout de la PCB difiere.

Uso típico en main: `LP55231_SetDriveCurrent` a `0xFF` para todos los canales, luego `SetChannelPWM`.

---

## PowerManager

Coordinador de bajo consumo del sistema. Pone en modo sleep o despierta todos los periféricos que tienen un modo de bajo consumo, en el orden correcto. Actúa como capa de orquestación por encima de los drivers individuales.

**Archivo:** `PowerManager.h` / `PowerManager.c` · **Versión:** 1.0.0  
**Documentación:** `Core/Doc/BajoConsumo_Referencia.md`  
**Rama de desarrollo:** `feature/BajoConsumo`

ICs gestionados: Flash MX25L6445E, GPS L86-M33, ICM-20948, TSL2571, SSD1306, BQ27441, LP55231, RM1262 LoRa.  
BL654 Bluetooth: auto-sleep, sin comando.  
Actuadores (LED, Buzzer, Vibrador): se apagan incondicionalmente en Suspend.

Handles registrados vía `PowerManager_Init()`: GPS, ICM20948, TSL2571, LP55231, LoRa.  
Flash, SSD1306, BQ27441, LedRGB, Buzzer, Vibrador usan handles internos de sus módulos.

**API:**
- `PowerManager_Init(hgps, himu, hlight, hrgb, hlora)` — registra handles; llamar una vez tras todos los Init()
- `PowerManager_SuspendAll(&result)` — suspende todo; retorna `PM_OK` o `PM_ERR_PARTIAL`
- `PowerManager_WakeAll(&result)` — despierta todo en orden inverso
- `PowerManager_MCUSleep()` — **STUB — NO IMPLEMENTADO**

`PM_Result_t` tiene un bool por cada IC: `flash_ok, gps_ok, imu_ok, light_ok, display_ok, gauge_ok, rgb_ok, lora_ok`.

**Pendientes:**
1. Implementar `PowerManager_MCUSleep()`: Stop mode + wakeup EXTI en USART1 RX (PA10). Requiere cambio en `.ioc` primero.
2. Validar `AT+SLEEP` en hardware RM1262. Alternativas comentadas en `Lora_Sleep()`.

---

> Nota: las secciones `LedRGB` (GPIO discreto), `SensorHall`, `Multiplexor` (CD4051B),
> `IR_Laser` (transmisor), `Menu` y `Display` (OLED SSD1306) se retiraron de este documento
> por no tener driver presente en este repo — son componentes exclusivos de la PCB 1 (Mira).
