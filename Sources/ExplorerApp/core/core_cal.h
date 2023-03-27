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

#ifndef CORE_CALIB_H
#define CORE_CALIB_H

/*!***************************************************************************
 * @defgroup    core_calib AFBR-S50 Explorer - Explorer calibration functions
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer - Explorer calibration functions
 * @details     Util functions for handling the calibration of Explorer devices
 * @addtogroup  core_calib
 * @{
 *****************************************************************************/

#include "explorer_types.h"

/*!***************************************************************************
 * @brief   Triggers a crosstalk calibration measurement sequence.
 * @param   argus The Argus device handler.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ExecuteXtalkCalibrationSequence(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Triggers a crosstalk calibration measurement sequence.
 * @param   argus The Argus device handler.
 * @param   targetRange The calibration target distance in meter and Q9.22
 *                         format. Pass non-positive (0) value to execute
 *                         relative calibration sequence only.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ExecuteOffsetsCalibrationSequence(argus_hnd_t * argus, q9_22_t targetRange);


/*! @} */
#endif /* CORE_CALIB_H */
