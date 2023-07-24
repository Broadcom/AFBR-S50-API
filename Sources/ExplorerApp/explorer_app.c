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
#include "core/core_device.h"
#include "core/core_flash.h"
#include "core/core_utils.h"
#include "api/explorer_api.h"
#include "api/explorer_api_cfg.h"
#include "api/explorer_api_cal.h"
#include "api/explorer_api_data.h"

#include "explorer_tasks.h"
#include "explorer_app.h"
#include "core/explorer_config.h"
#include "core/explorer_status.h"
#include "board/board.h"

#include "driver/s2pi.h"
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#endif
#include "debug.h"

#include <assert.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Returns the current system status.
 * @details Overwrites the weak definition on "sci_cmd.c".
 * @param   deviceID The device ID to query for status.
 * @return  Returns the current system status.
 *****************************************************************************/
status_t GetSystemStatus(sci_device_t deviceID);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t ExplorerApp_InitCommands()
{
    status_t status = SCI_Init();
    if (status < STATUS_OK) return status;

    status = ExplorerAPI_InitGeneral();
    if (status < STATUS_OK) return status;

    status = ExplorerAPI_InitData();
    if (status < STATUS_OK) return status;

    status = ExplorerAPI_InitCfg();
    if (status < STATUS_OK) return status;

    status = ExplorerAPI_InitCal();
    if (status < STATUS_OK) return status;

    return status;
}

status_t ExplorerApp_Init()
{
    Debug_ResetStackUsage();

    /* Initialize the board hardware /w watchdog timer disabled */
    status_t status = Board_Init();
    if (status < STATUS_OK)
    {
        error_log("Board initialization failed, error code: %d", status);
        assert(0);
        return status;
    }

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    SLCD_DisplayBar();
#endif

    /* Initialize the systems communication interface. */
    status = ExplorerApp_InitCommands();
    if (status < STATUS_OK)
    {
        assert(0);
        return status;
    }

    /* Initialize the AFBR-S50 Explorer task scheduler. */
    status = ExplorerApp_InitTasks();
    if (status < STATUS_OK)
    {
        assert(0);
        return status;
    }

    /* Initialize Devices */
    uint8_t devicesFound = 0;
    for (uint8_t deviceID = 1; deviceID <= EXPLORER_DEVICE_ID_MAX; deviceID++)
    {
        status = ExplorerApp_InitExplorer(deviceID);
        if (status == STATUS_OK)
        {
            devicesFound++;
            if (devicesFound == EXPLORER_DEVICE_COUNT) break;
        }
    }

    if (devicesFound == 0)
    {
        assert(0);
        return ERROR_ARGUS_NOT_CONNECTED;
    }

    return status;
}

status_t GetSystemStatus(sci_device_t deviceID)
{
    explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
    if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    status_t status = Argus_GetStatus(explorer->Argus);

    if (status == STATUS_IDLE)
    {
        status = Argus_Ping(explorer->Argus);
    }

    return status;
}
