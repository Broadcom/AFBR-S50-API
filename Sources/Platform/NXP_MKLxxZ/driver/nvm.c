/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file contains implementations of the non-volatile memory module.
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

#include "driver/flash.h"

#if !(defined(AFBR_FMT_BUILD) && AFBR_FMT_BUILD)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! The flash block index dedicated for the non-volatile memory module. */
#define NVM_FLASH_INDEX_1 2

/*! The flash block index dedicated for the non-volatile memory module. */
#define NVM_FLASH_INDEX_2 3

/*! The flash block index dedicated for the non-volatile memory module. */
#define NVM_FLASH_INDEX_3 4

/*! The number of flash block reserved for writing. */
#define NVM_FLASH_SECTOR_COUNT 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t NVM_Init(uint32_t size)
{
	if (size > NVM_FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)
		return ERROR_NVM_OUT_OF_RANGE;

	return STATUS_OK;
}

status_t NVM_Clear(void)
{
	status_t
	status = Flash_Clear(NVM_FLASH_INDEX_1, 0, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK) return status;
	status = Flash_Clear(NVM_FLASH_INDEX_2, 0, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK) return status;
	status = Flash_Clear(NVM_FLASH_INDEX_3, 0, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK) return status;
	return status;
}

status_t NVM_Write(uint32_t offset, uint32_t size, uint8_t const * buf)
{
	if (offset + size > NVM_FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)
		return ERROR_NVM_OUT_OF_RANGE;

	status_t status = STATUS_OK;
	uint32_t idx = NVM_FLASH_INDEX_1;
	while (offset >= FLASH_SECTOR_SIZE)
	{
		idx++;
		offset -= FLASH_SECTOR_SIZE;
	}

	while (size)
	{
		uint32_t size2 = size;
		if (offset + size > FLASH_SECTOR_SIZE)
			size2 = FLASH_SECTOR_SIZE - offset;

		status = Flash_Write(idx, offset, buf, size2);
		if (status < STATUS_OK) return status;

		size -= size2;
		buf += size2;
		offset = 0;
		idx++;
	}

	return status;
}

status_t NVM_Read(uint32_t offset, uint32_t size, uint8_t * buf)
{
	if (offset + size > NVM_FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)
		return ERROR_NVM_OUT_OF_RANGE;

	status_t status = STATUS_OK;
	uint32_t idx = NVM_FLASH_INDEX_1;
	while (offset >= FLASH_SECTOR_SIZE)
	{
		idx++;
		offset -= FLASH_SECTOR_SIZE;
	}

	while (size)
	{
		uint32_t size2 = size;
		if (offset + size > FLASH_SECTOR_SIZE)
			size2 = FLASH_SECTOR_SIZE - offset;

		status = Flash_Read(idx, offset, buf, size2);
		if (status < STATUS_OK) return status;

		size -= size2;
		buf += size2;
		offset = 0;
		idx++;
	}

	return status;
}
#else

status_t NVM_Clear(void)
{
	return STATUS_OK;
}

#endif /* AFBR_FMT_BUILD */
