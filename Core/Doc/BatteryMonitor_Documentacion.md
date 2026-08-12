# BatteryMonitor — Documentación del Driver BQ27441-G1

**Versión:** 2.0.0  
**Archivos:** `Core/Inc/BatteryMonitor.h` / `Core/Src/BatteryMonitor.c`

---

## ¿Qué es el BQ27441?

El BQ27441-G1 es un **monitor de combustible** (*fuel gauge*) de Texas Instruments para baterías de litio de una celda. No es simplemente un voltímetro: mide voltaje, corriente y temperatura de forma continua, y con esos datos **estima** cuánta carga le queda a la batería, qué tan sana está, y cuánto tiempo le queda. Se comunica con el microcontrolador por I2C (dirección 0x55).

---

## Parámetros que reporta

| Campo | Unidad | Descripción |
|---|---|---|
| `voltage_mV` | mV | Voltaje instantáneo de la batería. Una celda LiPo cargada mide ~4100–4200 mV; vacía, ~3000–3200 mV. |
| `avg_current_mA` | mA (con signo) | Corriente promedio. **Positivo** = la batería se está cargando. **Negativo** = se está descargando. |
| `soc_pct` | % (0–100) | *State of Charge* — porcentaje de carga restante. El equivalente a la "barrita de batería". |
| `remaining_mAh` | mAh | Capacidad restante en miliamperios-hora. Si el diseño es de 400 mAh y `soc_pct` es 50%, aquí verás ~200 mAh. |
| `full_cap_mAh` | mAh | Capacidad máxima real que el gauge aprendió del ciclo de uso. En gauge nuevo es igual al diseño (400 mAh). Con el tiempo puede bajar si la batería envejece. |
| `soh_pct` | % (0–100) | *State of Health* — qué tan sana está la batería respecto a su capacidad original. 100% = batería nueva. Requiere un ciclo de carga/descarga completo para calcularse; en gauge recién configurado vale 0. |
| `charge_state` | enum | `QUIET` = corriente < 60 mA en ambas direcciones. `CHARGING` = cargando activamente. `DISCHARGING` = descargando activamente. |
| `is_full` | 0 / 1 | 1 cuando `soc >= 99%` **o** cuando voltaje ≥ 4180 mV y la corriente de carga bajó a ≤ 20 mA (criterio de carga completa). |
| `is_ready` | 0 / 1 | 1 si el último ciclo de lectura fue exitoso. **Cuando vale 0, todos los demás campos están en 0.** Indica que la batería fue desconectada o hay un fallo de comunicación. |

---

## ITPOR — Lo más importante del BQ27441

**ITPOR** (*Impedance Track Power-On Reset*) es la bandera más crítica del chip. Se activa cada vez que el BQ27441 pierde alimentación por completo, lo que ocurre cuando:

- La batería se desconecta
- El sistema se apaga y la batería queda sin carga
- Se detecta un restablecimiento interno del chip

### ¿Por qué importa tanto?

Cuando ITPOR está activo significa que el chip **perdió sus parámetros operativos en RAM** (aunque los valores de diseño siguen en NVM). El gauge no puede calcular SOC ni corriente de forma confiable hasta que se reconfigure y se le indique que hay una batería conectada.

Si se ignora ITPOR y se leen los datos de todas formas, los valores serán incorrectos o sin sentido.

### ¿Qué hace el driver cuando lo detecta?

`BatGauge_Update()` revisa el registro FLAGS en cada llamada. Si encuentra ITPOR activo, llama automáticamente a `BatGauge_Configure()`, que ejecuta la siguiente secuencia de recuperación:

```
1. Unseal el chip (llaves 0x8000 / 0x8000)
2. Entrar a modo CFGUPDATE
3. Escribir los parámetros de la batería en NVM:
      Design Capacity  = 400 mAh
      Design Energy    = 1480 mWh
      Terminate Voltage= 2800 mV
      Taper Rate       = 100
4. Calcular y escribir el checksum del bloque (255 - suma de 32 bytes)
5. Salir de CFGUPDATE con Soft Reset
6. Sellar el chip
7. Enviar: CLEAR_HIBERNATE → IT_ENABLE → BAT_INSERT
```

Después de esta secuencia el chip se inicializa correctamente y `is_ready` vuelve a 1.

> **Nota:** El checksum calculado por *read-back* (leer los 32 bytes ya escritos y calcular ahí el checksum) es un requisito del fabricante. Si se calcula antes de escribir los datos, el chip rechaza la configuración y ITPOR nunca se limpia.

---

## Cómo usar el driver

### 1. Inicialización (una sola vez)

```c
HAL_StatusTypeDef st = BatGauge_Init();
// st == HAL_OK  → chip detectado y configurado
// st == HAL_ERROR → no hay respuesta en I2C (revisar cableado)
```

`BatGauge_Init()` verifica que el chip sea un BQ27441 (Device Type 0x0421) y siempre corre la configuración completa, independientemente de si el NVM ya tiene los valores correctos.

### 2. Actualización periódica (cada ~3 segundos)

```c
BatGauge_Data_t bat_data;

// En el loop principal:
static uint32_t last = 0;
if (HAL_GetTick() - last >= 3000U) {
    last = HAL_GetTick();
    BatGauge_Update(&bat_data);
}
```

`BatGauge_Update()` maneja internamente todo lo siguiente sin intervención del usuario:
- Verificar si el chip responde en I2C
- Detectar y manejar ITPOR
- Recuperar el bus I2C si se detecta un fallo de comunicación
- Leer todos los registros y clasificar el estado de carga

### 3. Leer los datos

```c
if (bat_data.is_ready) {
    // Datos válidos
    uint16_t soc   = bat_data.soc_pct;        // ej. 75 (%)
    uint16_t volts = bat_data.voltage_mV;      // ej. 3850 (mV)
    int16_t  curr  = bat_data.avg_current_mA;  // ej. -120 (mA, descargando)
} else {
    // Batería desconectada o fallo — todos los campos son 0
}
```

---

## Diagrama de estados internos

```
        BatGauge_Init()
               │
               ▼
        ┌─────────────┐
        │  Verificar  │── HAL_ERROR ──► sin chip en I2C
        │  Device ID  │
        └──────┬──────┘
               │ OK
               ▼
        BatGauge_Configure()
               │
               ▼
         gaugeReady = 1
               │
    ┌──────────▼───────────────────────────────────────┐
    │              BatGauge_Update() (cada 3 s)        │
    │                                                  │
    │  ¿waitingRecovery y < 800 ms?  ──► salir         │
    │                                                  │
    │  ¿responde en I2C?  ──No──► is_ready=0, datos=0  │
    │          │                                       │
    │         Sí                                       │
    │          │                                       │
    │  ¿FLAGS == 0xFFFF?  ──Sí──► recuperar bus I2C    │
    │          │                                       │
    │  ¿ITPOR activo?  ──Sí──► BatGauge_Configure()    │
    │          │                                       │
    │    Leer registros                                │
    │          │                                       │
    │  ¿soc o volt == 0xFFFF?  ──Sí──► datos=0        │
    │          │                                       │
    │    Clasificar charge_state, is_full              │
    │    is_ready = 1                                  │
    └──────────────────────────────────────────────────┘
```

---

## Parámetros de la batería (macros en el .h)

Si se cambia la celda de batería, actualizar estas macros en `BatteryMonitor.h`:

```c
#define BAT_DESIGN_CAP_MAH      400U    // Capacidad nominal de la celda en mAh
#define BAT_DESIGN_ENERGY_MWH   1480U   // Energía nominal en mWh (= cap * voltaje_nom)
#define BAT_TERMINATE_MV        2800U   // Voltaje mínimo antes de considerar la batería vacía
#define BAT_TAPER_CURRENT_MA    40U     // Corriente de corte al final de la carga
```

Después de cambiar cualquier macro, llamar a `BatGauge_Configure()` para que los nuevos valores se escriban en el NVM del chip.
