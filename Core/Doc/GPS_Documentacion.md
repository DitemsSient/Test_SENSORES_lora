# Documentacion de la Libreria GPS (L86-M33) — v3.0.0

Libreria para comunicacion con el modulo GPS Quectel L86-M33 sobre STM32F4xx.
Parsea sentencias NMEA 0183 (GPRMC y GPGGA) y envia comandos MTK para configurar el modulo.

---

## Indice

1. [Configuracion en GPS.h](#configuracion-en-gpsh)
2. [Funciones publicas](#funciones-publicas)
3. [Comandos MTK](#comandos-mtk)
4. [Modo Debug](#modo-debug)
5. [Estructuras de datos](#estructuras-de-datos)
6. [Funciones internas](#funciones-internas)
7. [Flujo de uso tipico](#flujo-de-uso-tipico)
8. [Notas importantes](#notas-importantes)

---

## Configuracion en GPS.h

Antes de compilar, revisa y ajusta estos `#define` segun tu hardware:

| Macro | Default | Que configurar |
|-------|---------|----------------|
| `GPS_UART` | `(&huart1)` | Cambialo si tu GPS esta en otro USART (ej. `&huart2`). |
| `GPS_FORCE_ON_PORT` | `GPIOA` | Puerto del pin FORCE_ON del L86. |
| `GPS_FORCE_ON_PIN` | `GPIO_PIN_12` | Pin FORCE_ON. Debe estar configurado como GPIO Output en CubeMX. |
| `GPS_UTC_OFFSET_H` | `(-6)` | Offset de zona horaria en horas. CST Mexico = -6, CET Europa = +1, JST Japon = +9. La hora en `GpsData_t` se guarda ya convertida a hora local. |
| `GPS_TX_TIMEOUT_MS` | `500` | Timeout de transmision UART para comandos MTK. |
| `GPS_SENTENCE_MAX_LEN` | `120` | Tamano maximo de una sentencia NMEA. No deberia necesitar cambio. |
| `GPS_DEBUG` | Comentado | Descomenta `#define GPS_DEBUG` para habilitar contadores y buffers de diagnostico. Ver seccion [Modo Debug](#modo-debug). |

---

## Funciones publicas

### `Gps_Init(Gps_Handle_t *h)`

Inicializa el handle, activa el pin FORCE_ON (mantiene al modulo encendido), limpia buffers y arma la primera interrupcion UART. Llamar una sola vez al inicio del programa.

### `Gps_Process(Gps_Handle_t *h)`

Funcion principal del driver. Revisa si hay una sentencia NMEA completa acumulada por la ISR. Si la hay, valida el checksum, identifica si es RMC o GGA, la parsea y actualiza `h->data`. Si la posicion es valida, tambien formatea `position_str`.

Llamar en cada iteracion del `while(1)`. No es bloqueante: si no hay sentencia lista retorna `GPS_ERR_NO_FIX` inmediatamente.

### `Gps_StoreByte(Gps_Handle_t *h)`

Acumula un byte recibido en el buffer de sentencia y rearma la interrupcion UART. **Solo llamar desde `HAL_UART_RxCpltCallback()`**, nunca desde el loop principal.

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GPS_UART->Instance) {
        Gps_StoreByte(&hgps);
    }
}
```

### `Gps_SendMTK(Gps_Handle_t *h, const char *cmd)`

Envia un comando MTK (PMTK) al modulo con enmarcado y checksum automaticos. Esta es la funcion central para toda la configuracion del GPS.

**Como funciona:**
1. Recibe el cuerpo del comando como string (ej. `"PMTK314,0,1,0,1,0,..."`).
2. Calcula el checksum XOR de todos los caracteres.
3. Arma la trama completa: `$cuerpo*XX\r\n`.
4. Transmite por UART de forma bloqueante.

**Ejemplos de uso con los defines de la libreria:**

```c
// Configurar que sentencias envia el GPS (solo RMC + GGA)
Gps_SendMTK(&hgps, GPS_OUTPUT_RMC_GGA);

// Cambiar la tasa de fix a cada 2 segundos
Gps_SendMTK(&hgps, GPS_FIX_0_5HZ);

// Reinicio en frio (borra almanaque y efemerides, tarda ~35s en conseguir fix)
Gps_SendMTK(&hgps, GPS_CMD_COLD_RESTART);

// Habilitar GPS + GLONASS
Gps_SendMTK(&hgps, GPS_CMD_GPS_GLONASS);

// Cambiar baud rate a 115200 (recuerda cambiar tambien el UART en CubeMX)
Gps_SendMTK(&hgps, GPS_CMD_BAUD_115200);

// Enviar cualquier comando PMTK personalizado
Gps_SendMTK(&hgps, "PMTK869,1,1");  // Habilitar EASY (prediccion de efemerides)
```

**Retorna:** `GPS_OK`, `GPS_ERR_TIMEOUT`, `GPS_ERR_UART` o `GPS_ERR_OVERFLOW`.

### `Gps_ForceOn(void)` / `Gps_ForceOff(void)`

Controlan el pin FORCE_ON del L86-M33. Con FORCE_ON en HIGH el modulo permanece encendido. Con LOW permite que entre en standby. `Gps_Init()` llama a `Gps_ForceOn()` automaticamente.

### `Gps_CheckWiring(Gps_Handle_t *h)`

Verifica si el modulo esta fisicamente conectado revisando si se han recibido bytes por UART. Solo funciona con `GPS_DEBUG` habilitado (necesita `chars_processed`). Util durante el arranque para confirmar conexion.

### `Gps_Reset(Gps_Handle_t *h)`

Limpia el buffer de sentencia y todos los datos parseados a cero. No reinicializa el UART ni detiene la recepcion.

### `Gps_FormatPosition(Gps_Handle_t *h)`

Formatea latitud y longitud en `h->data.position_str` como `"lat,lon"` con 6 decimales. Se llama automaticamente desde `Gps_Process()` cuando hay fix valido.

---

## Comandos MTK

Todos los comandos se envian con `Gps_SendMTK()`. La libreria incluye defines para los mas comunes:

### Seleccion de sentencias (PMTK314)

Controla que sentencias NMEA emite el GPS. Los campos del PMTK314 son (en orden): GLL, RMC, VTG, GGA, GSA, GSV, + 10 reservados, MCHN. Valor: 0=off, 1=cada fix, 2=cada 2 fixes, etc.

| Define | Efecto |
|--------|--------|
| `GPS_OUTPUT_RMC_GGA` | Solo RMC + GGA. **Configuracion recomendada.** |
| `GPS_OUTPUT_RMC_ONLY` | Solo RMC (posicion, velocidad, fecha). Minimo necesario. |
| `GPS_OUTPUT_ALL` | Todas las sentencias estandar a 1 Hz. |
| `GPS_OUTPUT_DEFAULT` | Restaura configuracion de fabrica. |

### Tasa de fix (PMTK220)

| Define | Efecto |
|--------|--------|
| `GPS_FIX_0_5HZ` | 1 fix cada 2 segundos. Ahorra energia. |
| `GPS_FIX_1HZ` | 1 fix por segundo (default del modulo). |
| `GPS_FIX_5HZ` | 5 fixes por segundo. |
| `GPS_FIX_10HZ` | 10 fixes por segundo. |

### Reinicio

| Define | Efecto |
|--------|--------|
| `GPS_CMD_HOT_RESTART` | Reinicio caliente. Conserva almanaque y efemerides. Fix rapido (~1s). |
| `GPS_CMD_WARM_RESTART` | Reinicio tibio. Conserva almanaque. Fix en ~30s. |
| `GPS_CMD_COLD_RESTART` | Reinicio frio. Borra todo. Fix en ~35s. |
| `GPS_CMD_FULL_COLD` | Reinicio frio completo + reset a fabrica. |

### Standby

| Define | Efecto |
|--------|--------|
| `GPS_CMD_STANDBY` | Entra en modo standby. Se despierta con cualquier byte por UART. |

### Baud rate

| Define | Efecto |
|--------|--------|
| `GPS_CMD_BAUD_9600` | 9600 baud (default del L86). |
| `GPS_CMD_BAUD_38400` | 38400 baud. |
| `GPS_CMD_BAUD_57600` | 57600 baud. |
| `GPS_CMD_BAUD_115200` | 115200 baud. |

> **Importante:** Despues de cambiar el baud rate, el modulo responde a la nueva velocidad inmediatamente. Debes reconfigurar el UART en CubeMX y reinicializar el periferico para que coincida.

### Otros

| Define | Efecto |
|--------|--------|
| `GPS_CMD_AIC_ON` / `GPS_CMD_AIC_OFF` | Habilita/deshabilita Active Interference Cancellation. |
| `GPS_CMD_GPS_ONLY` | Solo constelacion GPS. |
| `GPS_CMD_GPS_GLONASS` | GPS + GLONASS (mas satelites, mejor precision). |
| `GPS_CMD_FW_VERSION` | Consulta la version de firmware del modulo. |

---

## Modo Debug

Por defecto, `GPS_DEBUG` esta comentado en `GPS.h` y el driver compila sin diagnosticos extra.

Para habilitarlo, descomenta la linea:

```c
#define GPS_DEBUG
```

Esto agrega al handle (`Gps_Handle_t`) una estructura `GpsDebug_t` con:

| Campo | Descripcion |
|-------|-------------|
| `chars_processed` | Total de bytes recibidos por la ISR desde el inicio. |
| `sentences_ok` | Sentencias parseadas con checksum correcto. |
| `sentences_fail` | Sentencias con checksum incorrecto. |
| `last_sentence[120]` | Ultima sentencia parseada exitosamente. |
| `raw[256]` | Buffer circular con los ultimos 256 bytes crudos recibidos. |
| `raw_idx` | Indice de escritura en el buffer circular. |

Para monitorear en el debugger (Live Expressions de STM32CubeIDE):

```
hgps.debug.chars_processed
hgps.debug.sentences_ok
hgps.debug.sentences_fail
hgps.debug.last_sentence
```

> `Gps_CheckWiring()` depende de `chars_processed`, por lo que solo funciona con `GPS_DEBUG` habilitado.

---

## Estructuras de datos

### `GpsData_t` — Datos parseados

| Campo | Tipo | Descripcion |
|-------|------|-------------|
| `latitude` | `double` | Grados decimales. Positivo = Norte, negativo = Sur. |
| `longitude` | `double` | Grados decimales. Positivo = Este, negativo = Oeste. |
| `altitude` | `double` | Metros sobre el nivel del mar (de GGA). |
| `position_valid` | `bool` | `true` si el fix actual es confiable. |
| `speed_kmh` | `double` | Velocidad sobre el suelo en km/h (de RMC). |
| `course` | `double` | Rumbo en grados 0-360 (de RMC). Ruidoso a baja velocidad. |
| `satellites` | `uint8_t` | Satelites en uso (de GGA). |
| `fix_quality` | `GpsFix_e` | `GPS_FIX_INVALID` (0), `GPS_FIX_GPS` (1) o `GPS_FIX_DGPS` (2). |
| `hdop` | `double` | Dilucion horizontal de precision. Menor = mejor. |
| `year, month, day` | `uint16_t / uint8_t` | Fecha en hora **local** (UTC + offset). |
| `hour, minute, second` | `uint8_t` | Hora **local** (UTC + GPS_UTC_OFFSET_H). |
| `datetime_valid` | `bool` | `true` cuando se recibio fecha/hora del GPS. |
| `position_str` | `char[32]` | String `"lat,lon"` con 6 decimales, listo para enviar o mostrar. |

### `GpsFix_e` — Calidad del fix

| Valor | Significado |
|-------|-------------|
| `GPS_FIX_INVALID` (0) | Sin fix. |
| `GPS_FIX_GPS` (1) | Fix GPS estandar. |
| `GPS_FIX_DGPS` (2) | Fix diferencial (mas preciso). |

### `GpsStatus_e` — Codigos de retorno

| Valor | Significado |
|-------|-------------|
| `GPS_OK` | Exito. |
| `GPS_ERR_PARAM` | Parametro NULL o invalido. |
| `GPS_ERR_UART` | Error UART de HAL. |
| `GPS_ERR_TIMEOUT` | Timeout en transmision. |
| `GPS_ERR_NO_FIX` | No hay sentencia pendiente. |
| `GPS_ERR_CHECKSUM` | Checksum NMEA no coincide. |
| `GPS_ERR_PARSE` | Sentencia con formato incorrecto. |
| `GPS_ERR_OVERFLOW` | Buffer overflow al armar comando. |

---

## Funciones internas

Funciones `static` dentro de `GPS.c`. No se pueden llamar desde afuera.

- **`nmea_checksum()`** — XOR de todos los bytes entre `$` y `*`.
- **`hex_to_val()`** — Convierte un caracter hex a su valor numerico (0-15).
- **`next_field()`** — Avanza el puntero al siguiente campo NMEA (despues de la coma).
- **`parse_double()` / `parse_int()`** — Parsean un numero desde un campo NMEA. Campo vacio retorna 0.
- **`nmea_to_degrees()`** — Convierte formato NMEA (`dddmm.mmmm`) a grados decimales. Ej: `2503.1234` = 25 + (3.1234/60) = 25.052057 grados.
- **`apply_timezone()`** — Suma `GPS_UTC_OFFSET_H` a la hora UTC y maneja rollover de dia/mes/ano.
- **`parse_rmc()`** — Parsea sentencia GPRMC: hora, status, posicion, velocidad (nudos a km/h), rumbo, fecha. Llama a `apply_timezone()`.
- **`parse_gga()`** — Parsea sentencia GPGGA: posicion, calidad de fix, satelites, HDOP, altitud.
- **`parse_sentence()`** — Valida checksum y despacha a `parse_rmc()` o `parse_gga()` segun el tipo.

---

## Flujo de uso tipico

```
Flujo de datos:

  Modulo GPS --UART--> rx_byte --ISR--> sentence[] --Process--> GpsData_t
                         |                  |                       |
                   Gps_StoreByte()    byte a byte            latitude
                   (desde callback)   hasta '\n'             longitude
                                                              altitude
                                                              speed_kmh
                                                              satellites
                                                              position_str
```

### Codigo minimo en main.c

```c
/* Declaracion global */
Gps_Handle_t hgps;

/* USER CODE BEGIN 2 (antes del while) */
Gps_Init(&hgps);
Gps_SendMTK(&hgps, GPS_OUTPUT_RMC_GGA);   // Solo RMC + GGA
Gps_SendMTK(&hgps, GPS_FIX_0_5HZ);        // Fix cada 2 segundos

/* USER CODE BEGIN 3 (dentro del while) */
if (Gps_Process(&hgps) == GPS_OK) {
    if (hgps.data.position_valid) {
        // Usar hgps.data.latitude, longitude, altitude, etc.
        // hgps.data.position_str tiene "lat,lon" listo
    }
}

/* En HAL_UART_RxCpltCallback */
if (huart->Instance == GPS_UART->Instance) {
    Gps_StoreByte(&hgps);
}
```

### Secuencia de arranque

1. `Gps_Init()` — Activa FORCE_ON, limpia buffers, arma interrupcion UART.
2. `Gps_SendMTK()` — Configura sentencias y tasa de fix.
3. Esperar. El modulo tarda entre 1-3 segundos (hot start) hasta 35 segundos (cold start) en conseguir fix.
4. En el loop, llamar `Gps_Process()` continuamente. Los primeros datos tendran `position_valid == false` hasta que se adquieran suficientes satelites.

---

## Notas importantes

- **El GPS transmite continuamente** una vez encendido. No necesitas pedirle datos.
- **`Gps_Process()` no es bloqueante.** Si no hay sentencia lista, retorna inmediatamente.
- **RMC y GGA se complementan:** RMC trae velocidad, rumbo y fecha. GGA trae altitud, satelites y HDOP. La configuracion recomendada es `GPS_OUTPUT_RMC_GGA`.
- **La hora se guarda en hora local**, no UTC. Ajusta `GPS_UTC_OFFSET_H` en el `.h`.
- **Velocidad y rumbo son ruidosos** cuando el GPS esta quieto con pocos satelites. Es comportamiento normal.
- **V_BCKP mantiene el almanaque** cuando se apaga VCC. Esto permite hot/warm start mas rapidos.
- **ANTSTATUS=OPEN** es un mensaje que el L86 envia siempre (deteccion de antena). No es un error.
- **Si cambias el baud rate** con `GPS_CMD_BAUD_*`, debes reconfigurar el UART en CubeMX para que coincida.
- **Checksum obligatorio:** sentencias corruptas se descartan automaticamente. Monitorea `debug.sentences_fail` (con `GPS_DEBUG`) para ver cuantas se perdieron.
