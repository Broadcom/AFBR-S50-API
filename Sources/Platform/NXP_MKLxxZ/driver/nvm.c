/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file contains implementations of the non-volatile memory module.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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
#include "driver/flash.h"
#include "argus.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t NVM_Write(uint32_t offset, uint32_t size, uint8_t const * buf);
static status_t NVM_Read(uint32_t offset, uint32_t size, uint8_t * buf);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t NVM_WriteBlock(uint32_t device_id, uint32_t block_size, uint8_t const * buf)
{
    assert(buf != 0);
    if (device_id == 0) return ERROR_INVALID_ARGUMENT;
    if (block_size == 0) return ERROR_INVALID_ARGUMENT;

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
    uint8_t header[NVM_HEADER_SIZE] = { 0 };

    const uint32_t slot_size = block_size + NVM_HEADER_SIZE;
    for (uint32_t offset = 0; offset <= FLASH_API_BLOCK_SIZE - slot_size; offset += slot_size)
    {
        /* Read slot header from NVM memory. */
        status_t status = NVM_Read(offset, NVM_HEADER_SIZE, header);
        if (status < STATUS_OK) return status;

        /* Slot empty? -> Save index as potential slot. */
        if (NVM_GET32(header, NVM_VERSION_IDX) != Argus_GetAPIVersion())
        {
            if (slot_idx < 0)
            {
                slot_idx = offset;
            }
        }
        else
        {
            /* Find current max. count. */
            if (NVM_GET32(header, NVM_SLOT_COUNT_IDX) > max_count)
            {
                max_count = NVM_GET32(header, NVM_SLOT_COUNT_IDX);
            }

            /* Find min. count of all non-empty slots. */
            if (NVM_GET32(header, NVM_SLOT_COUNT_IDX) <= min_count)
            {
                oldest_idx = offset;
                min_count = NVM_GET32(header, NVM_SLOT_COUNT_IDX);
            }

            /* Slot already used for current device? -> Save index. */
            if (NVM_GET32(header, NVM_CHIP_ID_IDX) == device_id)
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
    NVM_SET32(header, NVM_SLOT_COUNT_IDX, slot_count);
    NVM_SET32(header, NVM_CHIP_ID_IDX, device_id);
    NVM_SET32(header, NVM_VERSION_IDX, Argus_GetAPIVersion());
    status_t status = NVM_Write(slot_idx, NVM_HEADER_SIZE, header);
    if (status < STATUS_OK) return status;

    /* Write Data. */
    return NVM_Write(slot_idx + NVM_HEADER_SIZE, block_size, buf);
}

static status_t NVM_Write(uint32_t offset, uint32_t size, uint8_t const * buf)
{
    if (offset + size > FLASH_API_BLOCK_SIZE)
        return ERROR_NVM_OUT_OF_RANGE;

    status_t status = STATUS_OK;
    uint32_t idx = FLASH_API_BLOCK_INDEX;
    while (offset >= FLASH_BLOCK_SIZE)
    {
        idx++;
        offset -= FLASH_BLOCK_SIZE;
    }

    while (size)
    {
        uint32_t size2 = size;
        if (offset + size > FLASH_BLOCK_SIZE)
            size2 = FLASH_BLOCK_SIZE - offset;

        status = Flash_Write(idx, offset, buf, size2);
        if (status < STATUS_OK) return status;

        size -= size2;
        buf += size2;
        offset = 0;
        idx++;
    }

    return status;
}

status_t NVM_ReadBlock(uint32_t device_id, uint32_t block_size, uint8_t * buf)
{
    assert(buf != 0);
    if (device_id == 0) return ERROR_INVALID_ARGUMENT;
    if (block_size == 0) return ERROR_INVALID_ARGUMENT;

    status_t status = STATUS_OK;

    /* Read Non-Volatile memory from the platform. */
    const uint32_t slot_size = block_size + NVM_HEADER_SIZE;
    for (uint32_t offset = 0; offset <= FLASH_API_BLOCK_SIZE - slot_size; offset += slot_size)
    {
        /* Read slot header from NVM memory. */
        uint8_t header[NVM_HEADER_SIZE] = { 0 };
        status = NVM_Read(offset, NVM_HEADER_SIZE, header);
        if (status < STATUS_OK) break;

        /* API Version and Device ID */
        if (NVM_GET32(header, NVM_VERSION_IDX) == Argus_GetAPIVersion() &&
            NVM_GET32(header, NVM_CHIP_ID_IDX) == device_id)
        {
            status = NVM_Read(offset + NVM_HEADER_SIZE, block_size, buf);
            if (status < STATUS_OK) break;

            return STATUS_OK;
        }

        status = ERROR_NVM_EMPTY;
    }

    /* Reset buffer in case of any error. */
    memset(buf, 0, block_size);
    return status;
}

static status_t NVM_Read(uint32_t offset, uint32_t size, uint8_t * buf)
{
    if (offset + size > FLASH_API_BLOCK_SIZE)
        return ERROR_NVM_OUT_OF_RANGE;

    status_t status = STATUS_OK;
    uint32_t idx = FLASH_API_BLOCK_INDEX;
    while (offset >= FLASH_BLOCK_SIZE)
    {
        idx++;
        offset -= FLASH_BLOCK_SIZE;
    }

    while (size)
    {
        uint32_t size2 = size;
        if (offset + size > FLASH_BLOCK_SIZE)
            size2 = FLASH_BLOCK_SIZE - offset;

        status = Flash_Read(idx, offset, buf, size2);
        if (status < STATUS_OK) return status;

        size -= size2;
        buf += size2;
        offset = 0;
        idx++;
    }

    return status;
}

status_t NVM_Clear(void)
{
    for (uint32_t idx = FLASH_API_BLOCK_INDEX; idx < FLASH_API_BLOCK_INDEX + FLASH_API_BLOCK_COUNT; idx++)
    {
        status_t status = Flash_Clear(idx, 0, FLASH_BLOCK_SIZE);
        if (status < STATUS_OK) return status;
    }
    return STATUS_OK;
}
