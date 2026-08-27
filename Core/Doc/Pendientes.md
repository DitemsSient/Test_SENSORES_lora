# Pendientes

Lista de tareas abiertas para el firmware de la PCB 2 (Sensores). Se actualiza
conforme avanza el trabajo — no es un historial, es el estado actual.

## 1. Cambiar el MCU y re-verificar el bootloader USB

**Estado:** abierto (2026-08-27)

Esta tarjeta dejó de poder entrar al bootloader USB de fábrica (DFU) después
de conectar un jumper al pin `WAKEUP` del GPS (dominio RTC/backup del
módulo, debía quedar N/C según su datasheet). Se descartaron por software:

- Reset del dominio backup (`__HAL_RCC_BACKUPRESET_FORCE/RELEASE`) — sin
  efecto.
- Apagar el LSE antes de saltar al bootloader (`Bootloader.c`,
  `Bootloader_JumpToSystemMemory()`) — sin efecto.
- Cristal LSE revisado con osciloscopio: amplitud comparable a una tarjeta
  sana — descarta el cristal como causa.

El SWD (ST-Link) y el modo Logger (USB CDC de la propia app) siguen
funcionando bien en esta tarjeta — solo falla la enumeración del bootloader
ROM de fábrica. Conclusión de trabajo: posible daño puntual en el MCU
causado por esa maniobra del jumper.

**Siguiente paso:** cambiar el MCU (STM32L433CCUx) de esta tarjeta y
verificar que el bootloader USB vuelva a funcionar. Al resoldar:
- NO conectar el jumper del `WAKEUP` del GPS — dejarlo N/C.
- Orden de verificación sugerido: SWD normal → modo Logger (USB CDC) →
  modo bootloader (USB DFU, PB1+PB2 en bajo).
- Los fixes de software que se dejaron en `Bootloader.c` (deinit de USB,
  reset de dominio backup, apagar LSE antes de saltar) son inofensivos y se
  quedan pase lo que pase con el MCU nuevo.

## 2. Retomar la prueba de GPS (L76-L) una vez resuelto el punto 1

**Estado:** en pausa, bloqueado por el punto 1 (no hay bootloader USB para
reflashear rápido, pero SWD funciona — se puede retomar por ahí si urge).

Última prueba: el módulo (L76-L, confirmado compatible con el set de
comandos PMTK/NMEA del driver `GPS.c`, pensado originalmente para L86)
respondió bien a `GPS_OUTPUT_RMC_GGA` + `GPS_FIX_1HZ` — confirmado con TTL
directo en su TX. Con el NVIC de USART1 ya habilitado (faltaba, causaba
que `Gps_StoreByte()` nunca corriera) el `[GPS-RAW]` del Logger debería
mostrar las tramas — pendiente confirmar en pantalla una vez que se pueda
reflashear.

Pendiente aparte: `Gps_ForceOn()`/`Gps_ForceOff()` en `GPS.c` mueven el pin
`GPS_FORCE_ON_PIN` (PA8) como si fuera un `FORCE_ON` activo-alto genérico
(válido para el L86). En el L76-L ese mismo pin físico resultó ser
`WAKEUP` (dominio RTC/backup, debe quedar N/C o en bajo). Falta decidir:
dejar de manejar ese pin por software para este módulo, o confirmar si el
L76-L tiene un verdadero `FORCE_ON` en otro pin.

## 3. Limpieza del receptor infrarrojo — en curso

**Estado:** en curso (2026-08-27)

Se valida la captura por EXTI + `DWT->CYCCNT` (sin Timer Input Capture,
este MCU/encapsulado no tiene un timer libre con canal IC disponible) como
la forma definitiva de leer el TSOP. Se está reemplazando la librería de
prueba (`Recepcion_TEST.h/.c`) por un driver de producción
(`Receptor_Infrarrojo_EXTI.h/.c`), y dejando `Receptor_Infrarrojo_TSOP.h/.c`
(la versión basada en Timer Input Capture) como referencia/dato histórico,
comentada y fuera de compilación — ver el encabezado de esos archivos para
el detalle de por qué no se usa en este hardware.
