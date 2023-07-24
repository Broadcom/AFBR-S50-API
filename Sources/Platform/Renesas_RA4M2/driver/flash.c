/*************************************************************************//**
 * @file
 * @brief       This file is part of Argus API
 * @details     This file provides an interface to the flash module.
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
 *
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "flash.h"
#include "bsp_api.h"
#include "hal_data.h"
#include "driver/irq.h"
#include "debug.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The Flash driver offers a StatusGet function, but it doesn't offer
 * the constants to interpret the returned status. Thus, a hardcoded constant
 * is created here to mirror the internal driver Status constant. This
 * constant needs to be equal to RM_VEE_FLASH_PRV_STATES_READY */
#define FLASH_STATE_READY   1U

/*! Specific value used for the driver to indicate a successful memory clear operation. */
#define FLASH_DEFAULT_ID    65535U

/*! Generic timeout for Flash operations. */
#define FLASH_OP_TIMEOUT_MS 2000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

#ifdef DEBUG
static volatile bool isInitialized = false;
#endif

/*! Flag for avoiding overlapping write/clear operations. */
static volatile bool isWriting = false;

/*! Buffer used for read-modifywrite operations (a block needs to be written as a whole). */
static uint8_t writeBuffer[FLASH_BLOCK_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t Flash_Init(void)
{
    assert(!isInitialized);

    fsp_err_t ret = RM_VEE_FLASH_Open(&g_vee0_ctrl, &g_vee0_cfg);
    if (ret != FSP_SUCCESS)
    {
        return ERROR_FAIL;
    }
    else
    {
#ifdef DEBUG
        isInitialized = true;
#endif
        return STATUS_OK;
    }
}

status_t Flash_Read(uint32_t index, uint32_t offset,
                    uint8_t * data,
                    uint32_t size)
{
    assert(isInitialized);

    if ((data == 0) || (size == 0))
        return ERROR_INVALID_ARGUMENT;

    if (index > g_vee0_cfg.record_max_id)
        return ERROR_INVALID_ARGUMENT;

    if (offset + size > FLASH_BLOCK_SIZE)
        return ERROR_OUT_OF_RANGE;

    uint8_t * pData;
    uint32_t len;
    fsp_err_t ret = RM_VEE_FLASH_RecordPtrGet(&g_vee0_ctrl, index, (uint8_t**)&pData, &len);

    status_t retVal;
    if (ret == FSP_SUCCESS)
    {
        if (offset + size > len)
        {
            // Trying to read past the end of the block
            retVal = ERROR_OUT_OF_RANGE;
        }
        else
        {
            // Valid read request, copy the data to the supplied buffer
            memcpy(data, pData + offset, size);
            retVal = STATUS_OK;
        }

    }
    else if (ret == FSP_ERR_NOT_FOUND)
    {
        retVal = ERROR_NVM_EMPTY;
    }
    else
    {
        retVal = ERROR_FAIL;
    }

    return retVal;
}

status_t Flash_Write(uint32_t index, uint32_t offset,
                     uint8_t const * data,
                     uint32_t size)
{
    assert(isInitialized);

    if (data == 0) return ERROR_INVALID_ARGUMENT;
    if (size == 0) return ERROR_INVALID_ARGUMENT;

    if (index > g_vee0_cfg.record_max_id)
        return ERROR_INVALID_ARGUMENT;

    if (offset + size > FLASH_BLOCK_SIZE)
        return ERROR_OUT_OF_RANGE;

    ltc_t start;
    Time_GetNow(&start);

    // Check and block the mutex for writing the flash
    IRQ_LOCK();
    while (isWriting)
    {
        IRQ_UNLOCK();

        // give the chance to the Flash callback to clear the writing flag

        if (Time_CheckTimeoutMSec(&start, FLASH_OP_TIMEOUT_MS))
            return ERROR_TIMEOUT;

        IRQ_LOCK();
    }

    isWriting = true;
    IRQ_UNLOCK();

    uint8_t const * pWriteSource = data;

    if (size < FLASH_BLOCK_SIZE)
    {
        // Read the old values for this block. This is necessary because the write operation
        // just creates a new block with only the new data - it doesn't automatically carry over
        // the unchanged data. So we need to manually copy the previous data to an assembly buffer
        // and override the requested part, then write it back to Flash.

        uint8_t * pPrevData;
        uint32_t prevDataLen;
        fsp_err_t retRead = RM_VEE_FLASH_RecordPtrGet(&g_vee0_ctrl, index, (uint8_t**)&pPrevData, &prevDataLen);

        if (retRead == FSP_SUCCESS)
        {
            assert(prevDataLen == FLASH_BLOCK_SIZE);
            memcpy(writeBuffer, pPrevData, prevDataLen);
        }
        else if (retRead == FSP_ERR_NOT_FOUND)
        {
            memset(writeBuffer, 0, sizeof(writeBuffer));
        }
        else
        {
            return ERROR_FAIL;
        }

        // Replace the bytes as requested
        memcpy(writeBuffer + offset, data, size);

        // Set the write to Flash from the Assembly buffer
        pWriteSource = writeBuffer;
    }

    // Always write the data in complete blocks
    fsp_err_t ret = RM_VEE_FLASH_RecordWrite(&g_vee0_ctrl, index, pWriteSource, FLASH_BLOCK_SIZE);

    Time_GetNow(&start);

    // Wait for the write op to finish
    while (isWriting)
    {
        if (Time_CheckTimeoutMSec(&start, FLASH_OP_TIMEOUT_MS))
            return ERROR_TIMEOUT;
    }

    if (ret != FSP_SUCCESS)
        return ERROR_FAIL;
    else
        return STATUS_OK;
}

status_t Flash_Clear(uint32_t index, uint32_t offset, uint32_t size)
{
    (void)index;
    (void)offset;
    (void)size;
    return ERROR_NOT_IMPLEMENTED;
}

status_t Flash_ClearAll(void)
{
    assert(isInitialized);

    ltc_t start;
    Time_GetNow(&start);

    // Acquire the mutex for writing the flash (with timeout)
    IRQ_LOCK();
    while (isWriting)
    {
        IRQ_UNLOCK();
        // give the chance to the Flash callback to clear the writing flag

        if (Time_CheckTimeoutMSec(&start, FLASH_OP_TIMEOUT_MS))
            return ERROR_TIMEOUT;

        IRQ_LOCK();
    }

    isWriting = true;
    IRQ_UNLOCK();

    // Start the clear operation (blocking)
    fsp_err_t ret = RM_VEE_FLASH_Format(&g_vee0_ctrl, NULL);

    // As clear operation is blocking, no callback will be called,
    // so the flag must be cleared here.
    isWriting = true;

    if (ret != FSP_SUCCESS)
        return ERROR_FAIL;

    rm_vee_status_t status;
    ret = RM_VEE_FLASH_StatusGet(&g_vee0_ctrl, &status);
    if (ret != FSP_SUCCESS)
        return ERROR_FAIL;

    if (status.last_id != FLASH_DEFAULT_ID)
        return ERROR_FAIL;
    else
        return STATUS_OK;
}

void Flash_callback(rm_vee_callback_args_t * p_args)
{
    if ((NULL != p_args) && (RM_VEE_STATE_READY == p_args->state))
    {
        isWriting = false;
    }
}
