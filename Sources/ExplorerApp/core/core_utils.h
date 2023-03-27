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

#ifndef CORE_UTILS_H
#define CORE_UTILS_H

/*!***************************************************************************
 * @defgroup    core_utils AFBR-S50 Explorer Application - Util functions
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer Application - Util functions
 * @details     Util functions to be used by the rest of the API.
 * @addtogroup  core_utils
 * @{
 *****************************************************************************/

#include "explorer_types.h"

/*!***************************************************************************
 * @brief   Suspends the current active measurement on the device.
 * @details Suspends the currently ongoing measurements on the device by calling
 *          the #Argus_StopMeasurementTimer method. Further it checks if data
 *          evaluation of any raw measurement data is pending and performs a
 *          context switch to the corresponding data evaluation task in the task
 *          scheduler.
 *          After the function returns true, the measurements are suspended
 *          and the device is idle and ready to receive other API calls, e.g.
 *          configuration updates.
 *          If the function returns true, use the #ExplorerApp_StartTimerMeasurement
 *          to resume the measurement on the device again.
 * @param   argus The Argus device handler.
 * @return  Returns true if the measurements have been suspended and needs to
 *          be resumed afterwards.
 *****************************************************************************/
bool ExplorerApp_SuspendTimerMeasurement(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Starts the measurements.
 * @param   argus The Argus device handler.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_StartTimerMeasurement(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Stops the measurements and goes to idle state.
 * @param   argus The Argus device handler.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_StopTimerMeasurement(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Triggers a single measurement and goes to idle afterwards.
 * @param   argus The Argus device handler.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SingleMeasurement(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Aborts all device activity immediately.
 * @param   argus The Argus device handler.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_DeviceAbort(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Displays the unambiguous range in meter on the SLCD display.
 * @param   argus The Argus device handler.
 *****************************************************************************/
void ExplorerApp_DisplayUnambiguousRange(argus_hnd_t * argus);

/*! @} */
#endif /* CORE_UTILS_H */
