/**
 * @file    Flash.c
 * @brief   Driver implementation for MX25L6445EZNI SPI NOR flash memory.
 *
 * @date    March 28, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#include "Flash.h"
#include <string.h>

/* ======================  EXTERNAL HAL HANDLES  ============================ */

extern SPI_HandleTypeDef hspi2;

/* ======================  STATIC VARIABLES  ================================ */

/**
 * @brief 4 KB sector buffer for Flash_ModifySector() read-modify-write cycles.
 * @note  Permanently occupies 4 KB of RAM. Use a dynamic buffer if RAM is tight.
 */
static uint8_t sector_buffer[FLASH_SECTOR_SIZE];

/* ======================  STATIC FUNCTIONS  ================================ */

/* CS control helpers */

static void Flash_CS_Low(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
}

static void Flash_CS_High(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Transmits a byte buffer over SPI.
 * @param  data    Buffer to transmit.
 * @param  length  Number of bytes.
 * @note   Returns FLASH_ERR_SPI if HAL_SPI_Transmit fails.
 */
static FlashStatus_e Flash_SPI_Transmit(uint8_t *data, uint16_t length) {
    if (HAL_SPI_Transmit(FLASH_SPI, data, length, FLASH_SPI_TIMEOUT) != HAL_OK) {
        return FLASH_ERR_SPI;
    }
    return FLASH_OK;
}

/**
 * @brief  Receives a byte buffer over SPI.
 * @param  data    Destination buffer.
 * @param  length  Number of bytes to receive.
 * @note   Returns FLASH_ERR_SPI if HAL_SPI_Receive fails.
 */
static FlashStatus_e Flash_SPI_Receive(uint8_t *data, uint16_t length) {
    if (HAL_SPI_Receive(FLASH_SPI, data, length, FLASH_SPI_TIMEOUT) != HAL_OK) {
        return FLASH_ERR_SPI;
    }
    return FLASH_OK;
}

/**
 * @brief  Asserts CS, sends a 1-byte command, deasserts CS.
 * @param  cmd  Command opcode.
 */
static FlashStatus_e Flash_SendCommand(uint8_t cmd) {
    Flash_CS_Low();
    FlashStatus_e st = Flash_SPI_Transmit(&cmd, 1U);
    Flash_CS_High();
    return st;
}

/**
 * @brief  Sends the Write Enable opcode (required before every write or erase).
 */
static FlashStatus_e Flash_WriteEnable(void) {
    return Flash_SendCommand(FLASH_CMD_WRITE_ENABLE);
}

/**
 * @brief  Reads the Status Register and returns its raw byte value.
 */
static uint8_t Flash_ReadStatus(void) {
    uint8_t cmd    = FLASH_CMD_READ_STATUS;
    uint8_t status = 0U;

    Flash_CS_Low();
    Flash_SPI_Transmit(&cmd, 1U);
    Flash_SPI_Receive(&status, 1U);
    Flash_CS_High();

    return status;
}

/**
 * @brief  Builds a 4-byte command header: [opcode][addr[23:16]][addr[15:8]][addr[7:0]].
 * @param  header   4-byte destination buffer.
 * @param  cmd      SPI command opcode.
 * @param  address  24-bit target address.
 */
static void Flash_BuildHeader(uint8_t *header, uint8_t cmd, uint32_t address) {
    header[0] = cmd;
    header[1] = (uint8_t)((address >> 16) & 0xFFU);
    header[2] = (uint8_t)((address >>  8) & 0xFFU);
    header[3] = (uint8_t)( address        & 0xFFU);
}

/**
 * @brief  Generic erase helper: sends Write Enable, then the erase command with
 *         address header, and polls until the operation completes.
 * @param  address     Any address within the erase target.
 * @param  cmd         Erase opcode (sector, 32K block, 64K block).
 * @param  timeout_ms  Maximum wait time for WIP polling.
 */
static FlashStatus_e Flash_EraseGeneric(uint32_t address, uint8_t cmd,
                                        uint32_t timeout_ms) {
    if (address >= FLASH_TOTAL_SIZE) return FLASH_ERR_PARAM;

    FlashStatus_e st = Flash_WriteEnable();
    if (st != FLASH_OK) return st;

    uint8_t header[4];
    Flash_BuildHeader(header, cmd, address);

    Flash_CS_Low();
    st = Flash_SPI_Transmit(header, 4U);
    Flash_CS_High();

    if (st != FLASH_OK) return st;

    return Flash_WaitBusy(timeout_ms);
}

/* ================================  API  =================================== */

/* Public functions declared in the .h */

/**
 * @brief  Sets CS high, waits 10 ms for power-up, reads the JEDEC ID,
 *         and compares it against the expected manufacturer and device IDs.
 */
FlashStatus_e Flash_Init(void) {
    Flash_CS_High();
    HAL_Delay(10U);

    FlashID_t id;
    FlashStatus_e st = Flash_ReadID(&id);
    if (st != FLASH_OK) return st;

    if (id.manufacturer != FLASH_MANUFACTURER_ID || id.device_id != FLASH_DEVICE_ID) {
        return FLASH_ERR_ID;
    }

    return FLASH_OK;
}

/**
 * @brief  Sends the JEDEC ID command and reads 3 bytes: manufacturer, type, capacity.
 */
FlashStatus_e Flash_ReadID(FlashID_t *id) {
    uint8_t cmd         = FLASH_CMD_JEDEC_ID;
    uint8_t response[3] = {0U};

    Flash_CS_Low();
    FlashStatus_e st = Flash_SPI_Transmit(&cmd, 1U);
    if (st == FLASH_OK) {
        st = Flash_SPI_Receive(response, 3U);
    }
    Flash_CS_High();

    if (st == FLASH_OK) {
        id->manufacturer = response[0];
        id->device_id    = ((uint16_t)response[1] << 8) | response[2];
    }

    return st;
}

/**
 * @brief  Reads the Status Register and tests the WIP bit.
 */
uint8_t Flash_IsBusy(void) {
    return (Flash_ReadStatus() & FLASH_STATUS_WIP) ? 1U : 0U;
}

/**
 * @brief  Loops on Flash_IsBusy() until it returns 0 or timeout_ms elapses.
 */
FlashStatus_e Flash_WaitBusy(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();

    while (Flash_IsBusy()) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return FLASH_ERR_BUSY;
        }
    }

    return FLASH_OK;
}

/**
 * @brief  Sends Write Enable then writes 0x00 to the Status Register to clear
 *         all Block Protect bits, then polls until the operation finishes.
 */
FlashStatus_e Flash_UnprotectAll(void) {
    FlashStatus_e st = Flash_WriteEnable();
    if (st != FLASH_OK) return st;

    uint8_t cmd[2] = { FLASH_CMD_WRITE_STATUS, 0x00U };

    Flash_CS_Low();
    st = Flash_SPI_Transmit(cmd, 2U);
    Flash_CS_High();

    if (st != FLASH_OK) return st;

    return Flash_WaitBusy(100U);
}

/**
 * @brief  Sends the Deep Power-Down opcode.
 */
FlashStatus_e Flash_PowerDown(void) {
    return Flash_SendCommand(FLASH_CMD_POWER_DOWN);
}

/**
 * @brief  Sends the Release from Power-Down opcode and waits 1 ms for tRES1.
 */
FlashStatus_e Flash_WakeUp(void) {
    FlashStatus_e st = Flash_SendCommand(FLASH_CMD_RELEASE_PD);
    HAL_Delay(1U);
    return st;
}

/**
 * @brief  Sends the Read Data command with a 4-byte header, then receives the
 *         data in 65535-byte chunks to work around HAL's uint16_t size limit.
 */
FlashStatus_e Flash_ReadRaw(uint32_t address, uint8_t *buffer, uint32_t length) {
    if ((address + length) > FLASH_TOTAL_SIZE) return FLASH_ERR_PARAM;

    uint8_t header[4];
    Flash_BuildHeader(header, FLASH_CMD_READ_DATA, address);

    Flash_CS_Low();
    FlashStatus_e st = Flash_SPI_Transmit(header, 4U);

    if (st == FLASH_OK) {
        uint32_t remaining = length;
        uint8_t *ptr       = buffer;

        while (remaining > 0U && st == FLASH_OK) {
            uint16_t chunk = (remaining > 0xFFFFU) ? 0xFFFFU : (uint16_t)remaining;
            st        = Flash_SPI_Receive(ptr, chunk);
            ptr       += chunk;
            remaining -= chunk;
        }
    }
    Flash_CS_High();

    return st;
}

/**
 * @brief  Validates that length ≤ 256 and that address + length stays within
 *         the same page, sends Write Enable, then transmits the 4-byte header
 *         followed by the data, and polls for completion.
 */
FlashStatus_e Flash_PageProgram(uint32_t address, const uint8_t *data, uint16_t length) {
    if (length == 0U || length > NORFLASH_PAGE_SIZE) return FLASH_ERR_PARAM;
    if ((address + length) > FLASH_TOTAL_SIZE)    return FLASH_ERR_PARAM;

    uint32_t page_end = (address & FLASH_PAGE_MASK) + NORFLASH_PAGE_SIZE;
    if ((address + length) > page_end)            return FLASH_ERR_PARAM;

    FlashStatus_e st = Flash_WriteEnable();
    if (st != FLASH_OK) return st;

    uint8_t header[4];
    Flash_BuildHeader(header, FLASH_CMD_PAGE_PROGRAM, address);

    Flash_CS_Low();
    st = Flash_SPI_Transmit(header, 4U);
    if (st == FLASH_OK) {
        st = Flash_SPI_Transmit((uint8_t *)data, length);
    }
    Flash_CS_High();

    if (st != FLASH_OK) return st;

    return Flash_WaitBusy(50U);
}

/**
 * @brief  Delegates to Flash_EraseGeneric() with the Sector Erase opcode (~60 ms).
 */
FlashStatus_e Flash_EraseSector(uint32_t address) {
    return Flash_EraseGeneric(address, FLASH_CMD_SECTOR_ERASE, 500U);
}

/**
 * @brief  Delegates to Flash_EraseGeneric() with the 32 KB Block Erase opcode (~200 ms).
 */
FlashStatus_e Flash_EraseBlock32K(uint32_t address) {
    return Flash_EraseGeneric(address, FLASH_CMD_BLOCK_ERASE_32K, 1000U);
}

/**
 * @brief  Delegates to Flash_EraseGeneric() with the 64 KB Block Erase opcode (~400 ms).
 */
FlashStatus_e Flash_EraseBlock64K(uint32_t address) {
    return Flash_EraseGeneric(address, FLASH_CMD_BLOCK_ERASE_64K, 2000U);
}

/**
 * @brief  Sends Write Enable, sends the Chip Erase command, then polls for
 *         completion with a 60-second timeout.
 */
FlashStatus_e Flash_EraseChip(void) {
    FlashStatus_e st = Flash_WriteEnable();
    if (st != FLASH_OK) return st;

    st = Flash_SendCommand(FLASH_CMD_CHIP_ERASE);
    if (st != FLASH_OK) return st;

    return Flash_WaitBusy(60000U);
}

/**
 * @brief  Direct alias for Flash_ReadRaw().
 */
FlashStatus_e Flash_Read(uint32_t address, uint8_t *buffer, uint32_t length) {
    return Flash_ReadRaw(address, buffer, length);
}

/**
 * @brief  Fragments the write into page-aligned chunks, calling Flash_PageProgram()
 *         for each, handling page boundary alignment automatically.
 */
FlashStatus_e Flash_Write(uint32_t address, const uint8_t *data, uint32_t length) {
    if (length == 0U)                          return FLASH_ERR_PARAM;
    if ((address + length) > FLASH_TOTAL_SIZE) return FLASH_ERR_PARAM;

    uint32_t      offset    = 0U;
    uint32_t      remaining = length;
    FlashStatus_e st        = FLASH_OK;

    while (remaining > 0U && st == FLASH_OK) {
        uint32_t current_addr  = address + offset;
        uint16_t page_offset   = (uint16_t)(current_addr % NORFLASH_PAGE_SIZE);
        uint16_t bytes_in_page = NORFLASH_PAGE_SIZE - page_offset;
        uint16_t chunk         = (remaining < bytes_in_page) ? (uint16_t)remaining : bytes_in_page;

        st         = Flash_PageProgram(current_addr, &data[offset], chunk);
        offset    += chunk;
        remaining -= chunk;
    }

    return st;
}

/**
 * @brief  Reads the full sector into sector_buffer, patches the target bytes,
 *         erases the sector, then rewrites the entire sector from the buffer.
 */
FlashStatus_e Flash_ModifySector(uint32_t address, const uint8_t *data, uint32_t length) {
    if (length == 0U) return FLASH_ERR_PARAM;

    uint32_t sector_start  = address & FLASH_SECTOR_MASK;
    uint32_t offset_in_sec = address - sector_start;

    if ((offset_in_sec + length) > FLASH_SECTOR_SIZE) return FLASH_ERR_PARAM;

    FlashStatus_e st = Flash_ReadRaw(sector_start, sector_buffer, FLASH_SECTOR_SIZE);
    if (st != FLASH_OK) return st;

    memcpy(&sector_buffer[offset_in_sec], data, length);

    st = Flash_EraseSector(sector_start);
    if (st != FLASH_OK) return st;

    return Flash_Write(sector_start, sector_buffer, FLASH_SECTOR_SIZE);
}

/* ========================  SELF-TEST  ==================================== */

#define FLASH_TEST_ADDR     0x7FF000U
#define FLASH_TEST_LEN      16U
#define FLASH_TEST_PATTERN  0xA5U

uint8_t Flash_Test(void)
{
    static uint8_t write_buf[FLASH_TEST_LEN];
    static uint8_t read_buf[FLASH_TEST_LEN];

    for (uint8_t i = 0U; i < FLASH_TEST_LEN; i++) {
        write_buf[i] = FLASH_TEST_PATTERN;
    }

    if (Flash_EraseSector(FLASH_TEST_ADDR) != FLASH_OK)                     { return 0U; }
    if (Flash_Write(FLASH_TEST_ADDR, write_buf, FLASH_TEST_LEN) != FLASH_OK) { return 0U; }
    if (Flash_Read(FLASH_TEST_ADDR, read_buf,  FLASH_TEST_LEN)  != FLASH_OK) { return 0U; }

    for (uint8_t i = 0U; i < FLASH_TEST_LEN; i++) {
        if (read_buf[i] != FLASH_TEST_PATTERN) { return 0U; }
    }

    Flash_EraseSector(FLASH_TEST_ADDR);
    return 1U;
}
