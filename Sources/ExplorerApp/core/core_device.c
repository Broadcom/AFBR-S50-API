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

#include "core_device.h"
#include "core_cfg.h"
#include "core_flash.h"
#include "core_utils.h"
#include <assert.h>
#include "driver/s2pi.h"
#include "debug.h"
#include "explorer_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static explorer_t explorerArray[EXPLORER_DEVICE_COUNT] = { 0 };
static explorer_t * explorerIDMap[S2PI_SLAVE_COUNT + 1] = { 0 };

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

static status_t CheckConnectedDevice(sci_device_t slave)
{
    status_t status = STATUS_OK;

    uint8_t data[17U] = {0};
    for (uint8_t i = 1; i < 17U; ++i) data[i] = i;

    for (uint8_t n = 0; n < 2; n++)
    {
        data[0] = 0x04;
        status = S2PI_TransferFrame(slave, data, data, 17U, 0, 0);
        if (status < STATUS_OK)
        {
            return status;
        }

        ltc_t start;
        Time_GetNow(&start);
        do
        {
            status = S2PI_GetStatus(slave);
            if (Time_CheckTimeoutMSec(&start, 100))
            {
                status = ERROR_TIMEOUT;
            }
        }
        while (status == STATUS_BUSY);

        if (status < STATUS_OK)
        {
            S2PI_Abort(slave);
            return status;
        }
    }

    bool hasData = true;
    for (uint8_t i = 1; i < 17U; ++i)
    {
        uint8_t j = ~i; // devices w/ inverted MISO
        if ((data[i] != i) && (data[i] != j))
            hasData = false;
    }

    if (hasData) return STATUS_OK;

    return ERROR_ARGUS_NOT_CONNECTED;
}

/*******************************************************************************
 * Functions
 ******************************************************************************/

argus_hnd_t * ExplorerApp_GetArgusPtr(sci_device_t deviceID)
{
    assert(deviceID <= EXPLORER_DEVICE_ID_MAX);
    explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
//    assert(explorer != NULL && explorer->Argus != NULL);
    return explorer != NULL ? explorer->Argus : NULL;
}

explorer_t * ExplorerApp_GetExplorerPtr(sci_device_t deviceID)
{
    assert(deviceID <= EXPLORER_DEVICE_ID_MAX);
    return (deviceID <= EXPLORER_DEVICE_ID_MAX) ? explorerIDMap[deviceID] : NULL;
}

explorer_t * ExplorerApp_GetExplorerPtrFromArgus(argus_hnd_t * argus)
{
    assert(argus != NULL);

    for (uint8_t i = 0; i < EXPLORER_DEVICE_COUNT; i++)
    {
        if (explorerArray[i].Argus == argus)
            return &explorerArray[i];
    }

    assert(0);
    return NULL;
}

uint8_t ExplorerApp_GetInitializedExplorerCount()
{
    uint8_t count = 0;
    for (uint8_t idx = 0; idx < EXPLORER_DEVICE_COUNT; ++idx)
    {
        if (explorerArray[idx].Argus != NULL)
            count++;
    }
    return count;
}

explorer_t * ExplorerApp_GetInitializedExplorer(uint8_t index)
{
    assert(index < EXPLORER_DEVICE_COUNT);

    explorer_t * explorer = &explorerArray[index];
    return explorer->Argus != NULL ? explorer : NULL;
}

sci_device_t ExplorerApp_GetDeviceID(explorer_t * explorer)
{
    assert(explorer != NULL);
    return explorer->DeviceID;
}

status_t ExplorerApp_InitDevice(explorer_t * explorer, argus_mode_t mode, bool reinit)
{
    assert(explorer != NULL);

    if (explorer->Argus == NULL)
    {
        explorer->Argus = Argus_CreateHandle();
        if (explorer->Argus == NULL)
        {
            error_log("Failed to allocate the memory for the AFBR-S50 API handle.");
            assert(0);
            return ERROR_FAIL;
        }
    }

    sci_device_t slave = S2PI_SLAVE_NONE;
    if (reinit)
    {
        slave = (sci_device_t)Argus_GetSPISlave(explorer->Argus);
        status_t status = Argus_Deinit(explorer->Argus);
        if (status < STATUS_OK)
        {
            error_log("Failed to de-initialize the AFBR-S50 API handle, "
                      "error code: %d", status);
            return status;
        }
    }

    if (slave == S2PI_SLAVE_NONE)
    {
        slave = explorer->DeviceID;

        /* Check for device connection in terms of slave and baud rate. */
        status_t status = CheckConnectedDevice(slave);
        if (status < STATUS_OK)
        {
            Argus_DestroyHandle(explorer->Argus);
            explorer->Argus = NULL;
            /* Do not print error here as this happens upon initialization
             * while searching for devices. */
            // error_log("No suitable device connected, error code: %d", status);
            return status;
        }
    }

    assert(slave != S2PI_SLAVE_NONE);

    /* Device initialization */
    ltc_t start = Time_Now();
    status_t status = Argus_InitMode(explorer->Argus, slave, mode);
    uint32_t elapsed = Time_GetElapsedUSec(&start);
    print("Init Time: %d us", elapsed);
    if (status == ERROR_ARGUS_UNKNOWN_MODULE) status = STATUS_OK; // ignore unknown modules
    else if (status < STATUS_OK)
    {
        Argus_DestroyHandle(explorer->Argus);
        explorer->Argus = NULL;
        error_log("Failed to initialize AFBR-S50 API, error code: %d", status);
        return status;
    }

    ExplorerApp_ResetDefaultDataStreamingMode(explorer);
    ExplorerApp_DisplayUnambiguousRange(explorer->Argus);

    return STATUS_OK;
}

status_t ExplorerApp_DeviceReinit(explorer_t * explorer, argus_mode_t mode)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    status_t status = ExplorerApp_InitDevice(explorer, mode, true);
    ExplorerApp_ResetDefaultDataStreamingMode(explorer);
    ExplorerApp_DisplayUnambiguousRange(explorer->Argus);
    return status;
}

status_t ExplorerApp_InitExplorer(sci_device_t deviceID)
{
    assert(deviceID > 0u);
    status_t status;

    /* ensure the uninitialized device starts with a null mapping */
    assert(explorerIDMap[deviceID] == NULL);

    /* Slaves 1,5 and 2,6 are connected to the same pins on some boards  (5 and
    * 6 are legacy boards). Thus, we do not allow slaves 5 or 6 if slaves 1 or 2
    * respectively are already initialized. */
    if (deviceID > 4)
    {
        if (explorerIDMap[deviceID - 4] != NULL)
            return ERROR_NOT_SUPPORTED;
    }

    /* find an unused memory block and allocate it for that instance. */
    explorer_t * pExplorer = NULL;
    for (uint8_t idx = 0; idx < EXPLORER_DEVICE_COUNT; idx++)
    {
        if (explorerArray[idx].Argus == NULL)
        {
            pExplorer = &explorerArray[idx];
            break;
        }
    }

    /* Make sure there is an empty Explorer object available. */
    if (pExplorer == NULL)
    {
        error_log("Failed to allocate an empty explorer object for the AFBR-S50 API instance.");
        return ERROR_FAIL;
    }

    pExplorer->DeviceID = deviceID;
    ExplorerApp_GetDefaultConfiguration(&pExplorer->Configuration);

    /* Initialize connected devices. */
    status = ExplorerApp_InitDevice(pExplorer, 0, false);
    if (status < STATUS_OK) return status;

    status = ExplorerApp_SetConfiguration(pExplorer, &pExplorer->Configuration);
    if (status < STATUS_OK) return status;

    /* Only once all checks are completed map the Explorer device to its ID for usage
     * deviceID starts with 1, so a mapping is needed.
     * deviceID 0 is reserved for default device */
    explorerIDMap[deviceID] = pExplorer;

    if (explorerIDMap[0] == NULL)
    {
        explorerIDMap[0] = pExplorer;
    }

    return status;
}
