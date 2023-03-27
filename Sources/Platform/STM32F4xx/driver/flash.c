/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "main.h"
#include "driver/flash.h"

/*************************************************************************//**
 * Implementation of flash layer to store non-volatile data that survives resets.
 *
 * Since the STM32F401RE has rather larger flash sectors, a few tricks have
 * been implemented to enable the NVM feature of the AFBR-S50 API.
 *
 * First, two sectors of size 16kbyte are allocated for the storage of data.
 * These two sections are used alternating to save the data. The reason is that
 * each sector must be erased fully before any data can be written. This means,
 * the data must be buffered somewhere. Unfortunately it's not easily possible
 * to buffer such a large amount of data in RAM. Thus, the data is copied from
 * the currently active flash sector to the currently unused sector and the
 * data that needs to be written to the device is inserted while copying.
 *
 * A few bytes at the beginning of the sector are reserved to represent the
 * current status of the section. If the first byte of a section is 0, the
 * sector holds the data. If it is 0xFF, it is currently unused. After erasing
 * the data, all bytes are 0xFF in the erased sector.
 *
 * The algorithm is as follows:
 * 1. Unlock the flash driver
 * 2. Erase the unused sector in oder to be ready for writing.
 * 3. Copy the current data from the current sector and add the user data at
 *    the specified position.
 * 4. Erase the old sector where the data has been written from to invalidate
 *    the sector.
 * 5. Finish by locking the flash driver.
 * ***************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! The size of a single flash sector. */
#define FLASH_SECTOR_SIZE 0x4000U

/*! Address offset for header bytes. */
#define FLASH_ADDRESS_OFFSET 16U

/*! Unlock the flash memory for writing. */
#define FLASH_UNLOCK() do { if (HAL_FLASH_Unlock() != HAL_OK) return ERROR_FAIL; } while(0)

/*! Lock the flash memory after writing. */
#define FLASH_LOCK() do { if (HAL_FLASH_Lock() != HAL_OK) return ERROR_FAIL; } while(0)

/*! Erase the flash memory \p sector before writing. */
#define FLASH_ERASE_SECTOR(sector) do { \
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR ); \
        FLASH_Erase_Sector(sector, VOLTAGE_RANGE_3); \
    } while(0)

/*! Address of flash sector 1. */
#define FLASH_SECTOR_1_ADDR ((uint32_t)(userConfig))

/*! Address of flash sector 2. */
#define FLASH_SECTOR_2_ADDR ((uint32_t)(userConfig) + FLASH_SECTOR_SIZE)

/*! Obtains the current sector. */
#define FLASH_CURRENT_SECTOR() ((userConfig[0] == 0x00U) ? FLASH_SECTOR_1 : FLASH_SECTOR_2)

/*! Obtains the not current sector. */
#define FLASH_NOT_CURRENT_SECTOR() ((userConfig[0] == 0x00U) ? FLASH_SECTOR_2 : FLASH_SECTOR_1)

/*! Macro to determine the address vector of a specified flash sector by its index. */
#define FLASH_SECTOR_ADDRESS(startaddr, index, offset) \
    ((uint32_t)(((startaddr) + FLASH_ADDRESS_OFFSET) + ((index) * (FLASH_BLOCK_SIZE)) + (offset)))

/*! Obtains the current sector address. */
#define FLASH_CURRENT_SECTOR_ADDR() ((userConfig[0] == 0x00U) ? FLASH_SECTOR_1_ADDR : FLASH_SECTOR_2_ADDR)

/*! Obtains the not current sector address. */
#define FLASH_NOT_CURRENT_SECTOR_ADDR() ((userConfig[0] == 0x00U) ? FLASH_SECTOR_2_ADDR : FLASH_SECTOR_1_ADDR)

/*! Array that points to the reserved flash memory for user data. */
__attribute__((__section__(".user_data"))) const volatile uint8_t userConfig[2*FLASH_SECTOR_SIZE]; // sector 1+2

/*!*****************************************************************************
 * @brief   Initializes global flash properties structure members.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Init(void)
{
    static_assert(FLASH_SECTOR_SIZE > FLASH_TOTAL_SIZE + FLASH_ADDRESS_OFFSET,
                  "Flash sector size exceeded!");
    static_assert(((FLASH_TOTAL_SIZE + FLASH_ADDRESS_OFFSET) & 0xF) == 0,
                  "Flash sector size must be multiple of 16 bytes!");
    return STATUS_OK;
}


status_t Flash_Read(uint32_t index, uint32_t offset,
                    uint8_t * data, uint32_t size)
{
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    if (size == 0) return ERROR_INVALID_ARGUMENT;
    if (FLASH_SECTOR_ADDRESS(0, index, offset) + size > FLASH_TOTAL_SIZE + FLASH_ADDRESS_OFFSET)
        return ERROR_OUT_OF_RANGE;

    memcpy(data, (uint8_t*)FLASH_SECTOR_ADDRESS(FLASH_CURRENT_SECTOR_ADDR(), index, offset), size);
    return STATUS_OK;
}

status_t Flash_Write(uint32_t index, uint32_t offset,
                     uint8_t const *data, uint32_t size)
{
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    if (size == 0) return ERROR_INVALID_ARGUMENT;
    if (FLASH_SECTOR_ADDRESS(0, index, offset) + size > FLASH_TOTAL_SIZE + FLASH_ADDRESS_OFFSET)
        return ERROR_OUT_OF_RANGE;

    uint32_t const rd_sector = FLASH_CURRENT_SECTOR();
    uint32_t const wr_sector = FLASH_NOT_CURRENT_SECTOR();
    uint32_t const rd_sector_addr = FLASH_CURRENT_SECTOR_ADDR();
    uint32_t const wr_sector_addr = FLASH_NOT_CURRENT_SECTOR_ADDR();

    uint32_t const start = FLASH_SECTOR_ADDRESS(wr_sector_addr, index, offset);
    uint32_t const stop = start + size;

    /* rd: 0000... wr: ????.... */

    FLASH_UNLOCK();
    FLASH_ERASE_SECTOR(wr_sector);

    /* rd: 0000... wr: FFFF.... */

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, wr_sector_addr, 0) != HAL_OK)
    {
        FLASH_LOCK();
        return ERROR_FAIL;
    }

    /* Copy data to other sector and fill in new data.
     * Note: the size is limited to FLASH_TOTAL_SIZE to increase access speed. */

    uint32_t rd = rd_sector_addr + FLASH_ADDRESS_OFFSET;
    uint32_t wr = wr_sector_addr + FLASH_ADDRESS_OFFSET;
    for (uint32_t i = 0; i < FLASH_TOTAL_SIZE >> 2; ++i)
    {
        uint32_t val = 0;
        for (uint32_t j = 0; j < sizeof(uint32_t); ++j)
        {
            if (wr == start)
            {
                rd = (uint32_t) data;
            }
            else if (wr == stop)
            {
                rd = rd_sector_addr + (wr - wr_sector_addr);
            }

            val |= (*((uint8_t*) rd)) << (j << 3);

            ++wr;
            ++rd;
        }

        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, wr - sizeof(uint32_t), val) != HAL_OK)
        {
            FLASH_LOCK();
            return ERROR_FAIL;
        }
    }

    /* cur: 0000... oth: 00dd.... */

    FLASH_ERASE_SECTOR(rd_sector);

    /* cur: ffff... oth: FFFF.... */

    FLASH_LOCK();
    return STATUS_OK;
}

status_t Flash_Clear(uint32_t index, uint32_t offset, uint32_t size)
{
    if (size == 0) return ERROR_INVALID_ARGUMENT;
    if (FLASH_SECTOR_ADDRESS(0, index, offset) + size > FLASH_TOTAL_SIZE + FLASH_ADDRESS_OFFSET)
        return ERROR_OUT_OF_RANGE;

    uint32_t const rd_sector = FLASH_CURRENT_SECTOR();
    uint32_t const wr_sector = FLASH_NOT_CURRENT_SECTOR();
    uint32_t const rd_sector_addr = FLASH_CURRENT_SECTOR_ADDR();
    uint32_t const wr_sector_addr = FLASH_NOT_CURRENT_SECTOR_ADDR();

    uint32_t const start = FLASH_SECTOR_ADDRESS(wr_sector_addr, index, offset);
    uint32_t const stop = start + size;

    /* rd: 0000... wr: ????.... */

    FLASH_UNLOCK();
    FLASH_ERASE_SECTOR(wr_sector);

    /* rd: 0000... wr: FFFF.... */

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, wr_sector_addr, 0) != HAL_OK)
    {
        FLASH_LOCK();
        return ERROR_FAIL;
    }

    /* Copy data to other sector and fill in new data.
     * Note: the size is limited to FLASH_TOTAL_SIZE to increase access speed. */
    uint32_t rd = rd_sector_addr + FLASH_ADDRESS_OFFSET;
    uint32_t wr = wr_sector_addr + FLASH_ADDRESS_OFFSET;
    for (uint32_t i = 0; i < FLASH_TOTAL_SIZE >> 2; ++i)
    {
        uint32_t val = 0;
        for (uint32_t j = 0; j < sizeof(uint32_t); ++j)
        {
            uint8_t b = (wr >= start && wr < stop) ? 0 : *((uint8_t*)rd);
            val |= b << (j << 3);
            ++wr;
            ++rd;
        }

        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, wr - sizeof(uint32_t), val) != HAL_OK)
        {
            FLASH_LOCK();
            return ERROR_FAIL;
        }
    }

    /* cur: 0000... oth: 00dd.... */

    FLASH_ERASE_SECTOR(rd_sector);

    /* cur: ffff... oth: FFFF.... */

    FLASH_LOCK();
    return STATUS_OK;
}

status_t Flash_ClearAll(void)
{
    return Flash_Clear(0, 0, FLASH_API_BLOCK_SIZE);
}

