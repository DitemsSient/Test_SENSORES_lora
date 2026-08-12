# IMU + Magnetómetro — Documentación del Sistema Inercial

**Versión:** 1.0.0  
**Componentes:** LSM6DSO32TR (acelerómetro + giroscopio) + MMC5983MA (magnetómetro)  
**Archivos:** `Core/Inc/LSM6DSO32TR.h` / `Core/Src/LSM6DSO32TR.c`  
`Core/Inc/MMC5983MA.h` / `Core/Src/MMC5983MA.c`

---


## LSM6DSO32TR — Acelerómetro + Giroscopio

### ¿Qué mide?

| Sensor | Qué detecta | Unidad | Rango configurado |
|---|---|---|---|
| Acelerómetro | Fuerzas de aceleración (movimiento, gravedad) | g | ±16 g |
| Giroscopio | Velocidad de rotación | dps (grados por segundo) | ±500 dps |
| Temperatura | Temperatura interna del die | °C | — |

El acelerómetro con ±16 g es útil para detectar golpes y orientación gruesa. El giroscopio a ±500 dps cubre movimientos rápidos del arma sin saturarse.

### Configuración

- **Dirección I2C:** `0x6A` (SA0 a GND)
- **ODR:** 104 Hz (modo alto rendimiento)
- **WHO_AM_I esperado:** `0x6C`
- **BDU habilitado:** garantiza que los registros alto y bajo de cada eje se lean del mismo instante

### Calibración del giroscopio

El giroscopio tiene un pequeño error sistemático en reposo llamado **bias** — incluso sin moverse, reporta un valor distinto de cero. La función `CalibrateGyroBias()` lo corrige:

1. Promedia 200 lecturas con el dispositivo completamente quieto (~2 segundos)
2. Guarda el promedio como bias en el handle (`dev->cal`)
3. A partir de ese momento, `ReadAll()` resta el bias automáticamente en cada lectura

> Mantener la tarjeta quieta durante los ~2 s que dura la calibración al arrancar.

### Recuperación automática

Si `ReadAll()` falla 3 veces consecutivas por error I2C, el driver ejecuta un **SW reset** y reconfigura el sensor automáticamente, preservando la calibración guardada en el handle. Los contadores `consecutive_errors` y `total_recoveries` permiten diagnosticar la estabilidad del bus.

### API principal

| Función | Descripción |
|---|---|
| `LSM6DSO32TR_Init(dev)` | SW reset, verifica WHO_AM_I, configura accel+gyro |
| `LSM6DSO32TR_CalibrateGyroBias(dev)` | Calibración de bias (~2 s en reposo) |
| `LSM6DSO32TR_ReadAll(dev, &out)` | Lee accel + gyro + temperatura, aplica bias |
| `LSM6DSO32TR_PowerDown(dev)` | Pone ambos sensores en modo power-down (~5 µA) |
| `LSM6DSO32TR_PowerOn(dev)` | Restaura ODR 104 Hz, espera 15 ms |
| `LSM6DSO32TR_Recover(dev)` | SW reset + reconfiguración, preserva calibración |
| `LSM6DSO32TR_Test(dev)` | WHO_AM_I + una lectura completa (TestHW) |

---

## MMC5983MA — Magnetómetro

### ¿Qué mide?

Mide el **campo magnético** en tres ejes (X, Y, Z) en microteslas (µT). Funciona como una brújula de alta precisión: con los tres ejes se puede calcular la orientación absoluta del dispositivo respecto al campo magnético terrestre.

- **Resolución:** 18 bits por eje (0 a 262143 cuentas)
- **Cero de campo:** 131072 (2^17, punto medio del rango)
- **Sensibilidad:** 163.84 cuentas por µT

### Configuración

- **Dirección I2C:** `0x30` (SA0 a GND)
- **ODR:** 10 Hz — suficiente para rastrear la orientación del arma, bajo consumo
- **Product ID esperado:** `0x30`

### SET pulse — por qué existe y cuándo ocurre

El sensor usa un material ferromagnético interno para medir el campo. Ese material puede quedar **magnetizado** por campos externos fuertes (un imán, un motor cerca), lo que corrompe las lecturas.

El **SET pulse** es un pulso de corriente breve que el chip se aplica a sí mismo para resetear ese material a un estado conocido — como "recalibrar el cero" del sensor.

En este driver el SET ocurre en tres momentos:

1. **Al inicializar** — siempre, para partir de un estado limpio
2. **Automáticamente en cada medición** — el chip lo hace solo con el modo `AUTO_SR` habilitado (un bit de configuración, sin costo de tiempo)
3. **Si se detecta saturación** — si algún eje llega al límite del rango (cerca de 0 o de 262143), `ReadAll()` emite un SET extra y relee los datos antes de retornar

### API principal

| Función | Descripción |
|---|---|
| `MMC5983MA_Init()` | SET pulse + verifica ID + activa modo continuo 10 Hz con auto SR |
| `MMC5983MA_ReadAll(&out)` | Burst de 7 bytes, ensambla 18 bits, convierte a µT |
| `MMC5983MA_Set()` | SET pulse manual (si se expuso a un imán fuerte) |
| `MMC5983MA_WhoAmI(&id)` | Lee Product ID, espera 0x30 |
| `MMC5983MA_Test()` | WHO_AM_I + lectura + validación de rango (TestHW) |

---

## Bajo consumo

Ambos drivers están integrados en el **PowerManager** del sistema:

| Chip | Modo normal | Modo bajo consumo | Función |
|---|---|---|---|
| LSM6DSO32TR | ~0.55 mA (104 Hz HP) | ~5 µA (ODR=0) | `PowerDown()` / `PowerOn()` |
| MMC5983MA | ~0.5 mA (10 Hz continuo) | ~1 µA (idle) | El modo continuo se detiene al cortar alimentación del sistema |

`PowerManager_SuspendAll()` llama a `LSM6DSO32TR_PowerDown()` automáticamente.  
`PowerManager_WakeAll()` llama a `LSM6DSO32TR_PowerOn()` y espera 15 ms antes de continuar.

---

