/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the hardware API of the Explorer Application.
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

/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "core_cal.h"
#include "core_cfg.h"
#include "core_utils.h"

#include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

status_t ExplorerApp_ExecuteXtalkCalibrationSequence(argus_hnd_t * argus)
{
    assert(argus != NULL);

    bool resume = ExplorerApp_SuspendTimerMeasurement(argus);

    status_t status = Argus_ExecuteXtalkCalibrationSequence(argus);
    if (status < STATUS_OK) return status;

    do
    {
        status = Argus_GetStatus(argus);
    } while (status > STATUS_IDLE);

    if (resume) ExplorerApp_StartTimerMeasurement(argus);

    return status;
}

status_t ExplorerApp_ExecuteOffsetsCalibrationSequence(argus_hnd_t * argus, q9_22_t targetRange)
{
    assert(argus != NULL);

    bool resume = ExplorerApp_SuspendTimerMeasurement(argus);

    status_t status = STATUS_OK;
    if (targetRange <= 0)
    {
        status = Argus_ExecuteRelativeRangeOffsetCalibrationSequence(argus);
    }
    else
    {
        status = Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence(argus, targetRange);
    }
    if (status < STATUS_OK) return status;

    do
    {
        status = Argus_GetStatus(argus);
    } while (status > STATUS_IDLE);

    if (resume) ExplorerApp_StartTimerMeasurement(argus);

    return status;
}
