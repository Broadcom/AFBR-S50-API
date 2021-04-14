/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Demo Application.
 * @details		This file contains the main functionality of the Explorer Application.
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

#ifndef EXPLORER_FLASH_H
#define EXPLORER_FLASH_H

/*!***************************************************************************
 * @defgroup	explorerflash AFBR-S50 Explorer Flash Module
 * @ingroup		explorermain
 * @brief		AFBR-S50 Explorer Flash Module
 * @details		An application specific wrapper around the flash module to
 * 				save/load the application specific settings.
 *
 * @addtogroup 	explorerflash
 * @{
 *****************************************************************************/

#include "explorer_status.h"
#include "api/explorer_api.h"


/*!***************************************************************************
 * @brief	Loads the configuration from the flash memory.
 * @details
 * @param	argus The Argus device handle.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_LoadCfgFromFlash(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief	Loads the calibration from the flash memory.
 * @details
 * @param	argus The Argus device handle.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_LoadCalFromFlash(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief	Saves the configuration to the flash memory.
 * @details
 * @param	argus The Argus device handle.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SaveCfgToFlash(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief	Saves the calibration to the flash memory.
 * @details
 * @param	argus The Argus device handle.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SaveCalToFlash(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief	Deletes the configuration from the flash memory.
 * @details
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ClearCfgFromFlash(void);

/*!***************************************************************************
 * @brief	Deletes the calibration from the flash memory.
 * @details
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ClearCalFromFlash(void);

/*!***************************************************************************
 * @brief	Deletes the all application specific values from memory.
 * @details
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ClearFlash(void);

/*! @} */
#endif /* EXPLORER_FLASH_H */
