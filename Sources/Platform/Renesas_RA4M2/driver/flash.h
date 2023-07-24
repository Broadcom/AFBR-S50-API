/*************************************************************************//**
 * @file
 * @brief       this file is part of Argus API
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


#ifndef FLASH_H
#define FLASH_H

/*!***************************************************************************
 * @defgroup    flash : Flash Module.
 * @ingroup     platform
 * @brief       Flash Module Interface.
 * @details     This module provides an interface to the device flash memory.
 *              It control the read and write of data from/to the non-volatile
 *              flash memory.
 * @addtogroup  flash
 * @{
 *****************************************************************************/

#include "utility/status.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! The size of a single (virtual) flash sector. */
#define FLASH_BLOCK_SIZE 0x400

/*! The number of flash sectors. */
#define FLASH_BLOCK_COUNT 5

/*! The total size in bytes of all flash sectors. */
#define FLASH_TOTAL_SIZE (FLASH_BLOCK_COUNT * FLASH_BLOCK_SIZE)

/*! The flash block index dedicated for the configuration data. */
#define FLASH_EXPL_CFG_INDEX 0

/*! The flash block index dedicated for the calibration data. */
#define FLASH_EXPL_CAL_INDEX 1

/*! A flash block index dedicated to the non-volatile memory module of the AFBR-S50 API. */
#define FLASH_API_BLOCK_INDEX 2

/*! The number of flash blocks dedicated to the non-volatile memory module of the AFBR-S50 API. */
#define FLASH_API_BLOCK_COUNT 3

/*! The size in bytes of the non-volatile memory module if the AFBR-S50 API. */
#define FLASH_API_BLOCK_SIZE (FLASH_API_BLOCK_COUNT * FLASH_BLOCK_SIZE)

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
        uint8_t v = (uint8_t)(val >> (24 - 8 * i)); \
        if (buf[idx + i] != v) { buf[idx + i] = v; } \
    } } while (0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!*****************************************************************************
 * @brief   Initializes global flash properties structure members.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Init(void);

/*!*****************************************************************************
 * @brief   Reads data from a specified flash sector.
 * @param   index The flash sector index from the end of memory.
 * @param   offset The start address relative to the sector start.
 * @param   data Pointer to the destination data buffer.
 * @param   size The size of data to be read.
 *                Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Read(uint32_t index, uint32_t offset,
                    uint8_t * data, uint32_t size);

/*!*****************************************************************************
 * @brief   Programs flash with data to a specified flash sector.
 * @param   index The flash sector index from the end of memory.
 * @param   offset The start address relative to the sector start.
 * @param   data Pointer to the source data buffer.
 * @param   size The size of data to be written.
 *                Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Write(uint32_t index, uint32_t offset,
                     uint8_t const * data, uint32_t size);

/*!*****************************************************************************
 * @brief   Clears flash with zeros from a specified flash sector.
 * @param   index The flash sector index from the end of memory.
 * @param   offset The start address relative to the sector start.
 * @param   size The size of data to be cleared.
 *                Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Clear(uint32_t index, uint32_t offset, uint32_t size);

/*!*****************************************************************************
 * @brief   Clears complete user flash with zeros.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_ClearAll(void);

/*! @} */
#endif /* FLASH_H */
