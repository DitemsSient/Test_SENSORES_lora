/**
 * @file    Bootloader.c
 * @brief   Driver implementation for the software jump to the USB DFU bootloader.
 *
 * @date    August 23, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Bootloader.h"
#include "Driver_RGB.h"
#include "usbd_conf.h"

/* Handle del LP55231, ya atado/Begin/Enable por el caller antes de llamar
 * a Bootloader_CheckAndEnter() — ver main.c USER CODE 2. */
extern LP55231_t rgb;

/* Handle del USB device (CDC), ya inicializado por MX_USB_DEVICE_Init() en
 * main() ANTES de que se revisen los pines del bootloader. Hay que
 * des-inicializarlo (baja el pull-up USB) antes de saltar al bootloader de
 * sistema — si no, el host todavia ve el CDC enumerado cuando el MCU salta,
 * y el ROM bootloader trata de re-enumerar sobre un bus que Windows cree
 * ocupado (se ve como "se conecta, se desconecta, error amarillo"). */
extern PCD_HandleTypeDef hpcd_USB_FS;

/* Par1 del mapeo fisico confirmado de esta tarjeta: D1=verde, D2=rojo. */
#define BOOTLOADER_LED_CH_GREEN     0U   /* D1 */
#define BOOTLOADER_LED_CH_RED       1U   /* D2 */

/* ======================  STATIC FUNCTIONS  ================================ */

/**
 * @brief  Resets clocks/peripherals to a clean state and jumps to the
 *         System Memory reset vector. Never returns.
 * @note   Mirrors what the hardware boot selector does when BOOT0 is HIGH
 *         at reset, just triggered from running application code instead.
 */
static void Bootloader_JumpToSystemMemory(void) {
    void (*SysMemBootJump)(void);

    /* Apaga el USB (baja el pull-up) ANTES de tocar clocks/NVIC, para que
     * el host detecte la desconexion del CDC limpiamente antes de que el
     * ROM bootloader intente re-enumerar. Un pequeno delay le da tiempo al
     * host de procesarlo. */
    HAL_PCD_DeInit(&hpcd_USB_FS);

    /* HAL_PCD_DeInit() solo apaga el reloj del periferico USB, no limpia
     * sus registros (direccion, endpoints, etc. quedan con lo que dejo el
     * CDC de la app). Forzamos un reset real via RCC para que el ROM
     * bootloader arranque desde el mismo estado que tendria justo despues
     * de un power-on, sin arrastrar nada. */
    __HAL_RCC_USB_FORCE_RESET();
    __HAL_RCC_USB_RELEASE_RESET();

    /* DIAGNOSTICO: esta tarjeta SI tiene el LSE (cristal 32.768kHz en
     * PC14/PC15) configurado y encendido por SystemClock_Config(). El
     * dominio backup/RTC (donde vive el LSE) NO se resetea con
     * HAL_RCC_DeInit() — sigue corriendo cuando el bootloader ROM de fabrica
     * toma control. Si el ROM intenta sincronizar su reloj USB contra un
     * LSE marginal (senal debil vista en el osciloscopio), la precision de
     * reloj que exige USB Full-Speed se pierde y la enumeracion falla justo
     * en el primer descriptor — sin que la app se entere, porque ella usa
     * el PLL para su propio USB, no el LSE. Lo apagamos explicitamente
     * antes de saltar para que el ROM arranque limpio, sin este cristal de
     * por medio.
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);

    HAL_Delay(100U);

    /* Bring clocks/peripherals back to their reset state so the bootloader
     * starts from the same conditions it would after a real reset. */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* Stop SysTick and mask every interrupt before leaving. */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    __disable_irq();
    for (uint8_t i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    __enable_irq();

    /* Remap System Memory to 0x00000000 and point VTOR at its vector table,
     * so the bootloader sees its own vectors, not the application's. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    SCB->VTOR = BOOTLOADER_SYSMEM_ADDR;

    /* Read the bootloader's initial stack pointer and reset handler from
     * its vector table, then jump. */
    uint32_t jump_address = *(__IO uint32_t *)(BOOTLOADER_SYSMEM_ADDR + 4U);
    SysMemBootJump = (void (*)(void))jump_address;

    __set_MSP(*(__IO uint32_t *)BOOTLOADER_SYSMEM_ADDR);
    SysMemBootJump();

    /* Never reached. */
    while (1) { }
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

Bootloader_Status_e Bootloader_CheckAndEnter(void) {
    bool pin1_low = (HAL_GPIO_ReadPin(BOOTLOADER_BTN_PORT, BOOTLOADER_BTN_PIN) == GPIO_PIN_RESET);
    bool pin2_low = (HAL_GPIO_ReadPin(BOOTLOADER_BTN2_PORT, BOOTLOADER_BTN2_PIN) == GPIO_PIN_RESET);
    bool pressed  = pin1_low && pin2_low;

    if (pressed) {
        /* DIAGNOSTICO TEMPORAL: colores invertidos (verde aqui, antes rojo)
         * para confirmar a simple vista si el firmware que se esta flasheando
         * de verdad es el que se esta corriendo. Revertir a
         * BOOTLOADER_LED_CH_RED cuando se confirme. */
        for (uint8_t i = 0U; i < BOOTLOADER_LED_BLINK_COUNT; i++) {
            LP55231_SetChannelPWM(&rgb, BOOTLOADER_LED_CH_GREEN, 0xFFU);
            HAL_Delay(BOOTLOADER_LED_BLINK_MS);
            LP55231_SetChannelPWM(&rgb, BOOTLOADER_LED_CH_GREEN, 0x00U);
            HAL_Delay(BOOTLOADER_LED_BLINK_MS);
        }
        Bootloader_JumpToSystemMemory();
        /* Unreachable. */
    }

    /* No entrado: no se toca el LED aqui — main.c decide verde/azul segun
     * PB2 (modo Logger o modo FTDI), reutilizando la misma lectura que ya
     * usa para condicionar Log_Init(). Ver USER CODE 2. */
    return BOOTLOADER_NOT_ENTERED;
}
