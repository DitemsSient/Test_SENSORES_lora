/**
 * @file    Flash.h
 * @brief   Driver for MX25L6445EZNI SPI NOR flash memory on STM32L4xx.
 *
 * @details Three-level API for Macronix 64 Mbit (8 MB) flash over SPI:
 *
 *          Level 1 — Direct commands (low level):
 *              Individual SPI command wrappers. Useful for targeted operations
 *              and debugging.
 *
 *          Level 2 — Safe operations (mid level):
 *              Handles page boundaries, Write Enable, and WIP polling
 *              automatically. Use these 90 % of the time.
 *
 *          Level 3 — Utilities:
 *              Device presence check, ID read, sector unprotect, power modes.
 *
 *          Memory organization (MX25L6445E):
 *          ───────────────────────────────────
 *          Total capacity : 8 MB (64 Mbit)
 *          Pages          : 32 768 × 256 B  (write unit)
 *          Sectors        : 2 048 × 4 KB    (minimum erase unit)
 *          32 KB blocks   : 256 × 32 KB
 *          64 KB blocks   : 128 × 64 KB
 *          Address range  : 0x000000 – 0x7FFFFF
 *
 *          Ground rules:
 *          1. Read: any time, any address, any size.
 *          2. Write: max 256 bytes per operation, must not cross a page boundary.
 *          3. Erase: minimum one 4 KB sector. Leaves all bytes at 0xFF.
 *          4. Write can only flip bits 1 → 0, never 0 → 1.
 *          5. Write Enable (0x06) is mandatory before every write or erase.
 *
 *          CubeMX configuration:
 *          - SPI (e.g., SPI1) in Full-Duplex Master mode.
 *          - Data Size: 8 bits, First Bit: MSB, CPOL: Low, CPHA: 1 Edge (Mode 0).
 *          - Baud Rate: start with /16 or /32 for testing.
 *          - NSS: Software (CS managed manually via FLASH_CS_PORT/PIN).
 *          - CS pin as GPIO_Output, Push-Pull, No Pull, High Speed, initial state High.
 *
 * @date    March 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef FLASH_H
#define FLASH_H

#include "stm32l4xx_hal.h"

/* ========================  CONFIGURATION  ================================= */

/* SPI handle and chip-select GPIO — update to match CubeMX .ioc */

#define FLASH_SPI               (&hspi2)

#define FLASH_CS_PORT           GPIOB
#define FLASH_CS_PIN            GPIO_PIN_5

/**
 * @brief SPI operation timeout in milliseconds.
 */
#define FLASH_SPI_TIMEOUT       100U

/* ====================  DEVICE CONSTANTS  ================================== */

/* Expected JEDEC ID for MX25L6445EZNI (Macronix 64 Mbit) */

#define FLASH_MANUFACTURER_ID   0xC2U   /**< Macronix manufacturer code      */
#define FLASH_DEVICE_ID         0x2017U /**< Memory type + capacity           */

/* Memory geometry */

#define NORFLASH_PAGE_SIZE      256U            /**< Bytes per page. Nombrado NORFLASH_ (no FLASH_)
                                                       porque stm32l4xx_hal_flash.h ya define su
                                                       propio FLASH_PAGE_SIZE para la flash interna
                                                       del MCU — usar FLASH_PAGE_SIZE aquí causaba
                                                       "redefined". */
#define FLASH_SECTOR_SIZE       4096U           /**< Bytes per sector (4 KB)   */
#define FLASH_BLOCK_32K_SIZE    32768U          /**< Bytes per 32 KB block     */
#define FLASH_BLOCK_64K_SIZE    65536U          /**< Bytes per 64 KB block     */
#define FLASH_TOTAL_SIZE        0x800000U       /**< Total capacity (8 MB)     */

#define FLASH_PAGE_COUNT        32768U          /**< Total pages               */
#define FLASH_SECTOR_COUNT      2048U           /**< Total sectors             */
#define FLASH_BLOCK_64K_COUNT   128U            /**< Total 64 KB blocks        */

#define FLASH_PAGE_MASK         0xFFFFFF00U     /**< Page-alignment mask       */
#define FLASH_SECTOR_MASK       0xFFFFF000U     /**< Sector-alignment mask     */

/* SPI command opcodes (MX25L6445E datasheet) */

#define FLASH_CMD_JEDEC_ID          0x9FU   /**< Read JEDEC ID (3 bytes)          */
#define FLASH_CMD_READ_DATA         0x03U   /**< Read Data                        */
#define FLASH_CMD_FAST_READ         0x0BU   /**< Fast Read (with dummy byte)      */
#define FLASH_CMD_WRITE_ENABLE      0x06U   /**< Write Enable                     */
#define FLASH_CMD_WRITE_DISABLE     0x04U   /**< Write Disable                    */
#define FLASH_CMD_PAGE_PROGRAM      0x02U   /**< Page Program (max 256 bytes)     */
#define FLASH_CMD_SECTOR_ERASE      0x20U   /**< Sector Erase 4 KB                */
#define FLASH_CMD_BLOCK_ERASE_32K   0x52U   /**< Block Erase 32 KB                */
#define FLASH_CMD_BLOCK_ERASE_64K   0xD8U   /**< Block Erase 64 KB                */
#define FLASH_CMD_CHIP_ERASE        0xC7U   /**< Chip Erase                       */
#define FLASH_CMD_READ_STATUS       0x05U   /**< Read Status Register             */
#define FLASH_CMD_WRITE_STATUS      0x01U   /**< Write Status Register            */
#define FLASH_CMD_POWER_DOWN        0xB9U   /**< Enter Deep Power-Down            */
#define FLASH_CMD_RELEASE_PD        0xABU   /**< Release from Deep Power-Down     */

/* Status Register bit masks */

#define FLASH_STATUS_WIP        0x01U   /**< Write In Progress (1 = busy)    */
#define FLASH_STATUS_WEL        0x02U   /**< Write Enable Latch               */
#define FLASH_STATUS_BP0        0x04U   /**< Block Protect bit 0              */
#define FLASH_STATUS_BP1        0x08U   /**< Block Protect bit 1              */
#define FLASH_STATUS_BP2        0x10U   /**< Block Protect bit 2              */
#define FLASH_STATUS_BP3        0x20U   /**< Block Protect bit 3              */

/* ========================  ENUMERATIONS  ================================== */

/* Driver return codes */

typedef enum {
    FLASH_OK            = 0,    /**< Operation successful                    */
    FLASH_ERR_SPI       = 1,    /**< SPI communication error                 */
    FLASH_ERR_BUSY      = 2,    /**< Flash busy — WIP timeout                */
    FLASH_ERR_PARAM     = 3,    /**< Invalid parameter (address or size)     */
    FLASH_ERR_ID        = 4,    /**< JEDEC ID does not match expected value  */
    FLASH_ERR_PROTECTED = 5     /**< Sector is write-protected               */
} FlashStatus_e;

/* ============================  STRUCTURES  ================================ */

/* Flash device identification */

typedef struct {
    uint8_t  manufacturer;  /**< Manufacturer ID (0xC2 = Macronix)      */
    uint16_t device_id;     /**< Device ID (0x2017 = 64 Mbit)           */
} FlashID_t;

/* ================================  API  =================================== */

/* Level 3 — Utilities */

/**
 * @brief  Asserts CS high, waits 10 ms for power-up, then reads and verifies
 *         the JEDEC ID. Returns FLASH_ERR_ID if the ID does not match.
 */
FlashStatus_e Flash_Init(void);

/**
 * @brief  Reads the 3-byte JEDEC ID into the FlashID_t structure.
 * @param  id  Pointer to the FlashID_t structure to fill.
 */
FlashStatus_e Flash_ReadID(FlashID_t *id);

/**
 * @brief  Returns 1 if the WIP bit in the Status Register is set, 0 otherwise.
 */
uint8_t Flash_IsBusy(void);

/**
 * @brief  Polls Flash_IsBusy() until the flash is idle or timeout_ms expires.
 * @param  timeout_ms  Maximum wait time in milliseconds.
 * @note   Returns FLASH_ERR_BUSY if the timeout is reached.
 */
FlashStatus_e Flash_WaitBusy(uint32_t timeout_ms);

/**
 * @brief  Clears all BP bits in the Status Register via Write Enable + Write Status.
 * @note   Call if Flash_EraseSector() or Flash_Write() returns FLASH_ERR_PROTECTED.
 */
FlashStatus_e Flash_UnprotectAll(void);

/**
 * @brief  Sends the Deep Power-Down command (~1 µA standby current).
 * @note   Requires Flash_WakeUp() before any further commands.
 */
FlashStatus_e Flash_PowerDown(void);

/**
 * @brief  Releases the flash from Deep Power-Down and waits ~30 µs.
 */
FlashStatus_e Flash_WakeUp(void);

/* Level 1 — Direct commands */

/**
 * @brief  Reads length bytes starting at address using the Read Data command.
 * @param  address  Start address (0x000000 – 0x7FFFFF).
 * @param  buffer   Destination buffer.
 * @param  length   Number of bytes to read (no practical limit).
 * @note   Internally fragments reads exceeding 65535 bytes due to HAL's
 *         uint16_t size parameter.
 */
FlashStatus_e Flash_ReadRaw(uint32_t address, uint8_t *buffer, uint32_t length);

/**
 * @brief  Programs up to 256 bytes within a single page.
 * @param  address  Start address (must stay within the same page).
 * @param  data     Data to write.
 * @param  length   Byte count (max 256, must not cross a page boundary).
 * @note   Use Flash_Write() for writes that may span multiple pages.
 */
FlashStatus_e Flash_PageProgram(uint32_t address, const uint8_t *data, uint16_t length);

/**
 * @brief  Erases a 4 KB sector (~60 ms). All bytes in the sector become 0xFF.
 * @param  address  Any address within the target sector.
 */
FlashStatus_e Flash_EraseSector(uint32_t address);

/**
 * @brief  Erases a 32 KB block (~200 ms).
 * @param  address  Any address within the target block.
 */
FlashStatus_e Flash_EraseBlock32K(uint32_t address);

/**
 * @brief  Erases a 64 KB block (~400 ms).
 * @param  address  Any address within the target block.
 */
FlashStatus_e Flash_EraseBlock64K(uint32_t address);

/**
 * @brief  Erases the entire chip (~30 s). Use only when strictly necessary.
 */
FlashStatus_e Flash_EraseChip(void);

/* Level 2 — Safe operations */

/**
 * @brief  Alias for Flash_ReadRaw(). Provided for API consistency.
 * @param  address  Start address.
 * @param  buffer   Destination buffer.
 * @param  length   Number of bytes to read.
 */
FlashStatus_e Flash_Read(uint32_t address, uint8_t *buffer, uint32_t length);

/**
 * @brief  Writes data of any size to any address, handling page boundaries.
 * @param  address  Start address.
 * @param  data     Data to write.
 * @param  length   Number of bytes (may span multiple pages).
 * @note   Does NOT erase before writing — the target area must already be 0xFF.
 *         Use Flash_EraseSector() first, or Flash_ModifySector() to preserve
 *         existing data.
 */
FlashStatus_e Flash_Write(uint32_t address, const uint8_t *data, uint32_t length);

/**
 * @brief  Modifies data inside a sector without losing surrounding content.
 * @param  address  Start address of the bytes to modify.
 * @param  data     New data.
 * @param  length   Number of bytes to modify (must fit within one sector).
 * @note   Internally: read sector → patch buffer → erase sector → rewrite.
 *         Requires 4 KB of static RAM. If interrupted between erase and
 *         rewrite, the sector contents are lost. Data must not span two sectors.
 */
FlashStatus_e Flash_ModifySector(uint32_t address, const uint8_t *data, uint32_t length);

/* ========================  SELF-TEST  ==================================== */

/**
 * @brief  Writes a pattern to the last sector, reads it back, and compares.
 * @return 1 if the read-back matches, 0 on any error or mismatch.
 * @note   Leaves the test sector erased after the test.
 */
uint8_t Flash_Test(void);

#endif /* FLASH_H */
