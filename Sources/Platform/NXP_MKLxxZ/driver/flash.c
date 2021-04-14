/*************************************************************************//**
 * @file
 * @brief    	This file is part of Argus API
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

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "flash.h"
#include "driver/fsl_flash.h"
#include "driver/irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! Macro to determine the address vector of a specified flash sector by its index. */
#define FLASH_SECTOR_ADDRESS(index) \
	((uint32_t)(((FLASH_BLOCK_TOTAL_SIZE) - ((index) + 1U) * (FLASH_SECTOR_SIZE))))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Flash driver Structure */
static flash_config_t s_flashDriver;


/*******************************************************************************
 * Code
 ******************************************************************************/

status_t Flash_Init(void)
{
	 /* Clean up Flash driver Structure*/
	memset(&s_flashDriver, 0, sizeof(flash_config_t));

	/* Setup flash driver structure for device and initialize variables. */
	status_t status = FLASH_Init(&s_flashDriver);
	if(status < STATUS_OK) return status;

	return STATUS_OK;
}

static status_t Flash_EraseSector(uint32_t index)
{
	/* Erase the flash sector.  */
	status_t status = FLASH_Erase(
			&s_flashDriver,
			FLASH_SECTOR_ADDRESS(index),
			FLASH_SECTOR_SIZE,
			kFLASH_ApiEraseKey);
	if(status < STATUS_OK) return status;

	/* Verify erased sector */
	status = FLASH_VerifyErase(
			&s_flashDriver,
			FLASH_SECTOR_ADDRESS(index),
			FLASH_SECTOR_SIZE,
			kFLASH_MarginValueUser);
	if(status < STATUS_OK) return status;

	return status;
}

status_t Flash_Read(uint32_t index, uint32_t offset,
					uint8_t * data, uint32_t size)
{
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	if (size == 0) return ERROR_INVALID_ARGUMENT;
	if (offset + size > FLASH_SECTOR_SIZE)
		return ERROR_OUT_OF_RANGE;

	memcpy(data, (uint8_t*)(FLASH_SECTOR_ADDRESS(index) + offset), size);
	return STATUS_OK;
}

status_t Flash_Write(uint32_t index, uint32_t offset,
					 uint8_t const * data, uint32_t size)
{
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	if (size == 0) return ERROR_INVALID_ARGUMENT;
	if (offset + size > FLASH_SECTOR_SIZE)
		return ERROR_OUT_OF_RANGE;

	IRQ_LOCK(); // prevent read-while-write violation

	/* Read the current data: */
	uint8_t sector [FLASH_SECTOR_SIZE];
	status_t status = Flash_Read(index, 0, sector, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Insert new data into sector: */
	memcpy(sector + offset, data, size);

	/* Delete current data */
	status = Flash_EraseSector(index);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Write back the data: */
	status = FLASH_Program(&s_flashDriver, FLASH_SECTOR_ADDRESS(index),
						   (uint32_t*)sector, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Verify the program process */
	uint32_t failAddr, failDat;
	status = FLASH_VerifyProgram(&s_flashDriver, FLASH_SECTOR_ADDRESS(index), FLASH_SECTOR_SIZE,
			(const uint32_t *)sector, kFLASH_MarginValueUser, &failAddr, &failDat);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	IRQ_UNLOCK();
	return status;
}

status_t Flash_Clear(uint32_t index, uint32_t offset, uint32_t size)
{
	if (size == 0) return ERROR_INVALID_ARGUMENT;
	if (offset + size > FLASH_SECTOR_SIZE)
		return ERROR_OUT_OF_RANGE;

	IRQ_LOCK(); // prevent read-while-write violation

	/* Read the current data: */
	uint8_t sector [FLASH_SECTOR_SIZE];
	status_t status = Flash_Read(index, 0, sector, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Clear data from sector: */
	memset(sector + offset, 0, size);

	/* Delete current data */
	status = Flash_EraseSector(index);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Write back the data: */
	status = FLASH_Program(&s_flashDriver, FLASH_SECTOR_ADDRESS(index),
						   (uint32_t*)sector, FLASH_SECTOR_SIZE);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	/* Verify the program process */
	uint32_t failAddr, failDat;
	status = FLASH_VerifyProgram(&s_flashDriver, FLASH_SECTOR_ADDRESS(index), FLASH_SECTOR_SIZE,
			(const uint32_t *)sector, kFLASH_MarginValueUser, &failAddr, &failDat);
	if (status < STATUS_OK)
	{
		IRQ_UNLOCK();
		return status;
	}

	IRQ_UNLOCK();
	return status;
}

