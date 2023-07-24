/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the main functionality of the Explorer Application.
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
#include "core_device.h"
#include "core_flash.h"
#include "core_utils.h"
#include "core_cfg.h"
#include "explorer_config.h"

#include "driver/flash.h"
#include "debug.h"
#include <assert.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

status_t ExplorerApp_ClearFlash(void)
{
    static bool resumeArray[EXPLORER_DEVICE_COUNT];

    for (uint8_t idx = 0; idx < EXPLORER_DEVICE_COUNT; idx++)
    {
        explorer_t * pExplorer = ExplorerApp_GetInitializedExplorer(idx);
        if (pExplorer)
        {
            resumeArray[idx] = ExplorerApp_SuspendTimerMeasurement(pExplorer->Argus);
        }
        else
        {
            resumeArray[idx] = false;
        }
    }

    status_t status = Flash_ClearAll();

    for (uint8_t idx = 0; idx < EXPLORER_DEVICE_COUNT; idx++)
    {
        if (resumeArray[idx])
        {
            explorer_t * pExplorer = ExplorerApp_GetInitializedExplorer(idx);
            ExplorerApp_StartTimerMeasurement(pExplorer->Argus);
        }
    }

    if (status != STATUS_OK) return status;

    print("Successfully cleared complete flash memory!");
    return STATUS_OK;
}

status_t ExplorerApp_ClearUserCalFromFlash(explorer_t * explorer)
{
    assert(explorer != NULL);
    argus_hnd_t * argus = explorer->Argus;
    return Argus_ClearUserCalibration(argus);
}
