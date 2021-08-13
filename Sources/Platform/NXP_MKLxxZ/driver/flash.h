/*************************************************************************//**
 * @file
 * @brief    	this file is part of Argus API
 * @details		This file provides an interface to the flash module.
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
 *
 *****************************************************************************/


#ifndef FLASH_H
#define FLASH_H

/*!***************************************************************************
 * @defgroup	flash : Flash Module.
 * @ingroup		platform
 * @brief		Flash Module Interface.
 * @details		This module provides an interface to the device flash memory.
 *				It control the read and write of data from/to the non-volatile
 *				flash memory.
 * @addtogroup 	flash
 * @{
 *****************************************************************************/

#include "nvm.h"
#include "utility/platform_status.h"
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! The size of a single flash sector. */
#define FLASH_BLOCK_SIZE 0x400

/*! The number of flash sectors. */
#define FLASH_BLOCK_COUNT 5

/*! The total size in bytes of all flash sectors. */
#define FLASH_TOTAL_SIZE (FLASH_BLOCK_COUNT * FLASH_BLOCK_SIZE)


/*! The index of the configuration flash sector. */
#define FLASH_CFG_INDEX (0)

/*! The offset in bytes of the configuration flash sector. */
#define FLASH_CFG_OFFSET (FLASH_SECTOR_CFG_INDEX * FLASH_BLOCK_SIZE)

/*! The size in bytes of the configuration flash sector. */
#define FLASH_CFG_SIZE   (FLASH_BLOCK_SIZE)


/*! The index of the calibration flash sector. */
#define FLASH_CAL_INDEX (1)

/*! The offset in bytes of the calibration flash sector. */
#define FLASH_CAL_OFFSET (FLASH_SECTOR_CAL_INDEX * FLASH_BLOCK_SIZE)

/*! The size in bytes of the calibrationo flash sector. */
#define FLASH_CAL_SIZE   (FLASH_BLOCK_SIZE)


/*! The index of the API data flash sector. */
#define FLASH_API_INDEX (2)

/*! The offset in bytes of the API data flash sector. */
#define FLASH_API_OFFSET (FLASH_API_INDEX * FLASH_BLOCK_SIZE)

/*! The size in bytes of the API data flash sector. */
#define FLASH_API_SIZE   (3 * FLASH_BLOCK_SIZE)





/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!*****************************************************************************
 * @brief	Initializes global flash properties structure members.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Init(void);

/*!*****************************************************************************
 * @brief	Reads data from a specified flash sector.
 * @param	index The flash sector index from the end of memory.
 * @param	offset The start address relative to the sector start.
 * @param	data Pointer to the destination data buffer.
 * @param	size The size of data to be read.
 * 				  Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Read(uint32_t index, uint32_t offset,
					uint8_t * data, uint32_t size);

/*!*****************************************************************************
 * @brief	Programs flash with data to a specified flash sector.
 * @param	index The flash sector index from the end of memory.
 * @param	offset The start address relative to the sector start.
 * @param	data Pointer to the source data buffer.
 * @param	size The size of data to be written.
 * 				  Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Write(uint32_t index, uint32_t offset,
					 uint8_t const * data, uint32_t size);

/*!*****************************************************************************
 * @brief	Clears flash with zeros from a specified flash sector.
 * @param	index The flash sector index from the end of memory.
 * @param	offset The start address relative to the sector start.
 * @param	size The size of data to be cleared.
 * 				  Maximum is #FLASH_SECTOR_SIZE - offset.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_Clear(uint32_t index, uint32_t offset, uint32_t size);

/*!*****************************************************************************
 * @brief	Clears complete user flash with zeros.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Flash_ClearAll(void);

/*! @} */
#endif /* FLASH_H */
