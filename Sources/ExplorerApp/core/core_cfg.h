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

#ifndef CORE_CFG_H
#define CORE_CFG_H

/*!***************************************************************************
 * @defgroup    core_cfg AFBR-S50 Explorer - Explorer configuration functions
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer - Explorer configuration functions
 * @details     Util functions for handling the configuration and modes of the
 *              Explorer devices.
 * @addtogroup  core_cfg
 * @{
 *****************************************************************************/

#include "explorer_types.h"


/*!***************************************************************************
 * @brief   Gets whether the debug mode is currently enabled.
 * @param   explorer The AFBR-Explorer control block.
 * @return  Returns true if the debug mode is currently enabled.
 *****************************************************************************/
bool ExplorerApp_GetDebugModeEnabled(explorer_t * explorer);

/*!***************************************************************************
 * @brief   Sets the debug mode to the device.
 * @param   explorer The AFBR-Explorer control block.
 * @param   debugMode The new debug mode enabled state.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SetDebugMode(explorer_t * explorer, bool debugMode);

/*!***************************************************************************
 * @brief   Getter for the default software configuration.
 * @param   cfg A pointer to the \link #explorer_cfg_t software
 *                configuration\endlink data structure that will be filled with
 *                the current configuration data.
 *****************************************************************************/
void ExplorerApp_GetDefaultConfiguration(explorer_cfg_t * cfg);

/*!***************************************************************************
 * @brief   Getter for the software configuration.
 * @param   explorer The AFBR-Explorer control block.
 * @param   cfg A pointer to the \link #explorer_cfg_t software
 *                configuration\endlink data structure that will be filled with
 *                the current configuration data.
 *****************************************************************************/
void ExplorerApp_GetConfiguration(explorer_t * explorer, explorer_cfg_t * cfg);

/*!***************************************************************************
 * @brief   Setter for the software configuration.
 * @param   explorer The AFBR-Explorer control block.
 * @param   cfg A pointer to the \link #explorer_cfg_t software
 *                configuration\endlink data structure that contains the new
 *                configuration data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SetConfiguration(explorer_t * explorer, explorer_cfg_t * cfg);

/*!***************************************************************************
 * @brief   Sets the device measurement mode and load default values.
 * @details The measurement mode is set to the specified mode and factory
 *          default configuration and calibration values are loaded.
 * @param   explorer The AFBR-Explorer control block.
 * @param   mode The new measurement mode to be set.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SetDeviceMeasurementMode(explorer_t * explorer, argus_mode_t mode);

/*!***************************************************************************
 * @brief   Resets the device measurement mode and load default values.
 * @details The measurement mode is reset to the factory default configuration
 *          and calibration values.
 * @param   explorer The AFBR-Explorer control block.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ResetDeviceMeasurementMode(explorer_t * explorer);

/*!***************************************************************************
 * @brief   Gets the current data output mode.
 * @param   explorer The AFBR-Explorer control block.
 * @return  Returns the current data output mode.
 *****************************************************************************/
data_output_mode_t ExplorerApp_GetDataOutputMode(explorer_t * explorer);

/*!***************************************************************************
 * @brief   Reset the Argus device to the default data streaming mode.
 * @param   explorer The Explorer handle.
 *****************************************************************************/
void ExplorerApp_ResetDefaultDataStreamingMode(explorer_t * explorer);

/*! @} */
#endif /* CORE_CFG_H */
