/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file contains implementations of the non-volatile memory module.
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

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "nvm.h"
#include "flash.h"
#include "argus.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! A flash block index dedicated to the non-volatile memory module of the AFBR-S50 API. */
#define FLASH_API_BLOCK_INDEX 2

/*! The number of flash blocks dedicated to the non-volatile memory module of the AFBR-S50 API. */
#define FLASH_API_BLOCK_COUNT 3

/*! The size in bytes of the non-volatile memory module if the AFBR-S50 API. */
#define FLASH_API_BLOCK_SIZE   (FLASH_API_BLOCK_COUNT * FLASH_BLOCK_SIZE)

/*! The size of the flash header. */
#define NVM_HEADER_SIZE 16

/*! The index position of the software version number in the NVM. */
#define NVM_VERSION_IDX 0

/*! The index position of the chip ID in the NVM. */
#define NVM_CHIP_ID_IDX 4

/*! The index position of the slot count value in the NVM. */
#define NVM_SLOT_COUNT_IDX 8

/*! Gets a 32-bit value from the NVM memory buffer \p buf at index position \p idx. */
#define NVM_GET32(buf, idx) \
    (uint32_t)(((buf)[idx] << 24) | ((buf)[(idx)+1] << 16) | ((buf)[(idx)+2] <<  8) | ((buf)[(idx)+3]))

/*! Sets a 32-bit value (\p val) at index position \p idx to the NVM memory buffer \p buf. */
#define NVM_SET32(buf, idx, val) do { \
    for (uint32_t i = 0; i < 4; ++i) { \
        uint8_t v = val >> (24 - 8 * i); \
        if (buf[idx + i] != v) { buf[idx + i] = v; } \
    } } while (0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t NVM_WriteBlock(uint32_t device_id, uint32_t block_size, uint8_t const * buf)
{
    assert(block_size == ARGUS_NVM_BLOCK_SIZE);
    assert(FLASH_BLOCK_SIZE >= NVM_HEADER_SIZE + block_size);

    /* Find empty slot or the one which is already used with the device.
     *
     *  - Each slot has a counter, version and device id written as identifier.
     *  - If version is not the current, the slot is considered to be empty (or invalid due to out-dated version).
     *  - If all slots are occupied, the oldest slot is used, determined by a counter.
     *  - Each write to a slot increments a counter value to determine the oldest slot
     *    (i.e. the one with the lowest counter value.)
     *  */

    int32_t slot_idx = -1;              // the index of the newest slot
    int32_t oldest_idx = -1;            // the index of the oldest slot
    uint32_t slot_count = 0;            // current slot count; must be increase if < max_count
    uint32_t max_count = 0;             // max count of all slots; determines the newest slot
    uint32_t min_count = UINT32_MAX;    // min count of all slots; determines the oldest slot;
    uint8_t data[FLASH_BLOCK_SIZE] = { 0 };

    for (uint32_t offset = 0; offset < FLASH_API_BLOCK_SIZE; offset += (block_size + NVM_HEADER_SIZE))
    {
        /* Read slot header from NVM memory. */
        status_t status = Flash_Read(FLASH_API_BLOCK_INDEX, offset, data, NVM_HEADER_SIZE);
        if (status < STATUS_OK) break;

        /* Slot empty? -> Save index as potential slot. */
        if (NVM_GET32(data, NVM_VERSION_IDX) != Argus_GetAPIVersion())
        {
            if (slot_idx < 0)
            {
                slot_idx = offset;
            }
        }
        else
        {
            /* Find current max. count. */
            if (NVM_GET32(data, NVM_SLOT_COUNT_IDX) > max_count)
            {
                max_count = NVM_GET32(data, NVM_SLOT_COUNT_IDX);
            }

            /* Find min. count of all non-empty slots. */
            if (NVM_GET32(data, NVM_SLOT_COUNT_IDX) <= min_count)
            {
                oldest_idx = offset;
                min_count = NVM_GET32(data, NVM_SLOT_COUNT_IDX);
            }

            /* Slot already used for current device? -> Save index. */
            if (NVM_GET32(data, NVM_CHIP_ID_IDX) == device_id)
            {
                slot_idx = offset;
            }
        }
    }

    /* If no slot has been found -> use oldest */
    if (slot_idx < 0)
    {
        slot_idx = oldest_idx;
    }

    /* Increase the slot count . */
    slot_count = max_count + 1;

    /* Write Header */
    NVM_SET32(data, NVM_SLOT_COUNT_IDX, slot_count);
    NVM_SET32(data, NVM_CHIP_ID_IDX, device_id);
    NVM_SET32(data, NVM_VERSION_IDX, Argus_GetAPIVersion());
    memcpy(&(data[NVM_HEADER_SIZE]), buf, block_size);

    /* Write Data. */
    return Flash_Write(FLASH_API_BLOCK_INDEX, slot_idx, data, FLASH_BLOCK_SIZE);
}

status_t NVM_ReadBlock(uint32_t device_id, uint32_t block_size, uint8_t * buf)
{
    assert(block_size == ARGUS_NVM_BLOCK_SIZE);
    status_t status = STATUS_OK;

    /* Read Non-Volatile memory from the platform. */
    for (uint32_t offset = 0; offset < FLASH_API_BLOCK_SIZE; offset += (block_size + NVM_HEADER_SIZE))
    {
        /* Read slot header from NVM memory. */
        uint8_t header[NVM_HEADER_SIZE] = { 0 };
        status = Flash_Read(FLASH_API_BLOCK_INDEX, offset, header, NVM_HEADER_SIZE);
        if (status < STATUS_OK) break;

        /* API Version and Device ID */
        if (NVM_GET32(header, NVM_VERSION_IDX) == Argus_GetAPIVersion() &&
            NVM_GET32(header, NVM_CHIP_ID_IDX) == device_id)
        {
            status = Flash_Read(FLASH_API_BLOCK_INDEX, offset + NVM_HEADER_SIZE, buf, block_size);
            if (status < STATUS_OK) break;

            return STATUS_OK;
        }

        status = ERROR_NVM_EMPTY;
    }

    /* Reset buffer in case of any error. */
    memset(buf, 0, block_size);
    return status;
}
