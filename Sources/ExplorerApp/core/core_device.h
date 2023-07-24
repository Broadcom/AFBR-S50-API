/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the hardware API for the Explorer Application.
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

#ifndef CORE_DEVICE_H
#define CORE_DEVICE_H

/*!***************************************************************************
 * @defgroup    core_device AFBR-S50 Explorer - Explorer Device functions
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer - Explorer Device functions
 * @details     Util functions for handling the Explorer device to be used
 *              by the rest of the API.
 * @addtogroup  core_device
 * @{
 *****************************************************************************/

#include "explorer_types.h"

/*!***************************************************************************
 * @brief   Gets the Argus device instance handle pointer.
 * @param   deviceID The Device ID of the selected Argus sensor.
 * @return  Returns the Argus device instance handle pointer.
 *****************************************************************************/
argus_hnd_t * ExplorerApp_GetArgusPtr(sci_device_t deviceID);

/*!***************************************************************************
 * @brief   Gets the Explorer instance handle pointer.
 * @param   deviceID The Device ID of the selected Argus sensor.
 * @return  Returns the Explorer instance handle pointer.
 *****************************************************************************/
explorer_t * ExplorerApp_GetExplorerPtr(sci_device_t deviceID);

/*!***************************************************************************
 * @brief   Gets the Explorer instance handle pointer given
 *          an Argus handle pointer.
 * @param   argus The Argus device pointer of the selected Argus sensor.
 * @return  Returns the Explorer instance handle pointer.
 *****************************************************************************/
explorer_t * ExplorerApp_GetExplorerPtrFromArgus(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Gets the device ID of the specified Explorer instance pointer.
 * @param   explorer The AFBR-Explorer control block.
 * @return  The Device ID of the selected Explorer instance.
 *****************************************************************************/
sci_device_t ExplorerApp_GetDeviceID(explorer_t * explorer);

/*!***************************************************************************
 * @brief   Gets the number of initialized Argus sensor devices.
 * @return  Returns the number of initialized Argus sensor devices.
 *****************************************************************************/
uint8_t ExplorerApp_GetInitializedExplorerCount();

/*!***************************************************************************
 * @brief   Gets the initialized Explorer instance handle pointer
 *          given its index in the list. This is used to obtain a consecutive
 *          list of instances, regardless of their DeviceID (the DeviceID
 *          could be non-consecutive).
 * @param   index Index of the initialized Explorer instance.
 * @return  Returns the selected Explorer instance handle pointer.
 *****************************************************************************/
explorer_t * ExplorerApp_GetInitializedExplorer(uint8_t index);


/*!***************************************************************************
 * @brief   Initializes the associated Argus device handle belonging to an
 *          Explorer instance handle pointer.
 * @param   explorer Explorer instance handle pointer to be initialized.
 * @param   mode Selected mode for the Argus sensor device.
 * @param   reinit Indicate whether the device should first be deinitialized.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_InitDevice(explorer_t * explorer, argus_mode_t mode, bool reinit);

/*!***************************************************************************
 * @brief   Reinitializes the device w/ default settings w/o MCU reset.
 * @param   explorer The AFBR-Explorer control block.
 * @param   mode The measurement mode to initialize with.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_DeviceReinit(explorer_t * explorer, argus_mode_t mode);

/*!***************************************************************************
 * @brief   Initializes the selected Explorer instance handle pointer
 *          with the selected device ID.
 * @param   deviceID The Device ID of the selected Argus sensor.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_InitExplorer(sci_device_t deviceID);

/*! @} */
#endif /* CORE_DEVICE_H */
