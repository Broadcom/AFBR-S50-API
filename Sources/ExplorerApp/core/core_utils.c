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
#include "core_utils.h"
#include <assert.h>
#include "driver/s2pi.h"
#include "debug.h"

#include "explorer_tasks.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! Callback function after measurement cycle was finished. */
extern status_t MeasurementReadyCallback(status_t status, argus_hnd_t * argus);


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Local functions
 ******************************************************************************/

bool ExplorerApp_SuspendTimerMeasurement(argus_hnd_t * argus)
{
    assert(argus != NULL);

    bool resume = Argus_IsTimerMeasurementActive(argus);
    if (resume) Argus_StopMeasurementTimer(argus);

    while (Argus_IsDataEvaluationPending(argus))
        ExplorerApp_SwitchContext(); // let evaluation task run...

    return resume;
}

status_t ExplorerApp_StartTimerMeasurement(argus_hnd_t * argus)
{
    assert(argus != NULL);

    status_t status = Argus_StartMeasurementTimer(argus, ExplorerApp_MeasurementReadyCallback);
    ExplorerApp_DisplayUnambiguousRange(argus);
    return status;
}

status_t ExplorerApp_StopTimerMeasurement(argus_hnd_t * argus)
{
    assert(argus != NULL);

    return Argus_StopMeasurementTimer(argus);
}

status_t ExplorerApp_SingleMeasurement(argus_hnd_t * argus)
{
    assert(argus != NULL);

    status_t status = STATUS_OK;
    do
    {
        status = Argus_TriggerMeasurement(argus, ExplorerApp_MeasurementReadyCallback);
    } while (status == STATUS_ARGUS_POWERLIMIT);
    return status;
}

status_t ExplorerApp_DeviceAbort(argus_hnd_t * argus)
{
    assert(argus != NULL);

    return Argus_Abort(argus);
}

void ExplorerApp_DisplayUnambiguousRange(argus_hnd_t * argus)
{
    assert(argus != NULL);

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

    /* Display clock frequency on LCD. */
    uint32_t measRange;
    if(Argus_GetConfigurationUnambiguousRange(argus, &measRange) == STATUS_OK)
    {
        uint8_t decPos = 1U;
        while (measRange > 9999U)
        {
            measRange /= 10U;
            decPos++;
        }
        SLCD_DisplayDecimalUnsigned(measRange);
        SLCD_SetDecimalPointPosition(decPos);
    }
#else
    (void) argus;
#endif
}
