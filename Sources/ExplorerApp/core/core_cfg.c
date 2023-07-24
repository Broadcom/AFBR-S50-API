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
#include "core_cfg.h"
#include "core_device.h"
#include "core_utils.h"
#include <assert.h>
#include "driver/s2pi.h"
#include "debug.h"
#include "board/board_config.h"

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

bool ExplorerApp_GetDebugModeEnabled(explorer_t * explorer)
{
    return (bool)explorer->Configuration.DebugMode;
}

status_t ExplorerApp_SetDebugMode(explorer_t * explorer, bool debugMode)
{
    assert(explorer != NULL);
    explorer->Configuration.DebugMode = debugMode;

    return STATUS_OK;
}

void ExplorerApp_GetDefaultConfiguration(explorer_cfg_t * cfg)
{
    assert(cfg != 0);

    cfg->SPIBaudRate = SPI_BAUDRATE;

#if defined(DEBUG)
    cfg->DebugMode = true;
#else
    cfg->DebugMode = false;
#endif
    cfg->DataOutputMode = DATA_OUTPUT_STREAMING_FULL;
}

void ExplorerApp_GetConfiguration(explorer_t * explorer, explorer_cfg_t * cfg)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);
    assert(cfg != NULL);

    memcpy(cfg, &explorer->Configuration, sizeof(explorer_cfg_t));
}

status_t ExplorerApp_SetConfiguration(explorer_t * explorer, explorer_cfg_t * cfg)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    if (cfg->DataOutputMode != DATA_OUTPUT_STREAMING_1D &&
        cfg->DataOutputMode != DATA_OUTPUT_STREAMING_3D &&
        cfg->DataOutputMode != DATA_OUTPUT_STREAMING_FULL &&
        cfg->DataOutputMode != DATA_OUTPUT_STREAMING_1D_DEBUG &&
        cfg->DataOutputMode != DATA_OUTPUT_STREAMING_3D_DEBUG &&
        cfg->DataOutputMode != DATA_OUTPUT_STREAMING_FULL_DEBUG)
    {
        error_log("Explorer configuration failed: the data output mode (%d) is unknown.",
                  cfg->DataOutputMode);
        return ERROR_INVALID_ARGUMENT;
    }

    if (cfg->SPIBaudRate > SPI_MAX_BAUDRATE)
    {
        error_log("Explorer configuration failed: the SPI baud rate (%d) is too large.\n"
                  "It is reset to maximum value of %d bps.",
                  cfg->SPIBaudRate, SPI_MAX_BAUDRATE);
        cfg->SPIBaudRate = SPI_MAX_BAUDRATE;
    }

    if (explorer->Configuration.SPIBaudRate != cfg->SPIBaudRate)
    {
        const s2pi_slave_t slave = Argus_GetSPISlave(explorer->Argus);

        const status_t status = S2PI_SetBaudRate(slave, cfg->SPIBaudRate);
        if (status != STATUS_OK)
        {
            /* Check if the actual baud rate is within 10 % of the desired baud rate. */
            if (status == ERROR_S2PI_INVALID_BAUDRATE)
                error_log("S2PI: The requested baud rate (%d bps) is not supported! "
                          "The actual baud rate is %d bps.",
                          cfg->SPIBaudRate, S2PI_GetBaudRate(slave));
            else
                error_log("S2PI: Setting the new baud rate failed, "
                          "error code: %d", status);

            /* Reset baud rate to last setting. */
            S2PI_SetBaudRate(slave, explorer->Configuration.SPIBaudRate);
            return status;
        }
        cfg->SPIBaudRate = S2PI_GetBaudRate(slave);
        //print("S2PI: Baud Rate set to %d bps.", cfg->SPIBaudRate);
    }

    explorer_cfg_t backup_cfg;
    ExplorerApp_GetConfiguration(explorer, &backup_cfg);
    memcpy(&explorer->Configuration, cfg, sizeof(explorer_cfg_t));

    return ExplorerApp_SetDebugMode(explorer, cfg->DebugMode);
}

status_t ExplorerApp_SetDeviceMeasurementMode(explorer_t * explorer, argus_mode_t mode)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    explorer_cfg_t ecfg = { 0 };
    ExplorerApp_GetConfiguration(explorer, &ecfg);

    argus_hnd_t * argus = explorer->Argus;

    bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
    status_t status = Argus_SetMeasurementMode(argus, mode);
    ExplorerApp_ResetDefaultDataStreamingMode(explorer);
    ExplorerApp_DisplayUnambiguousRange(argus);
    if (resume) ExplorerApp_StartTimerMeasurement(argus);
    return status;
}

status_t ExplorerApp_ResetDeviceMeasurementMode(explorer_t * explorer)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    argus_hnd_t * argus = explorer->Argus;

    bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
    status_t status = Argus_ResetMeasurementMode(argus);
    ExplorerApp_ResetDefaultDataStreamingMode(explorer);
    ExplorerApp_DisplayUnambiguousRange(argus);
    if (resume) ExplorerApp_StartTimerMeasurement(argus);
    return status;
}

data_output_mode_t ExplorerApp_GetDataOutputMode(explorer_t * explorer)
{
    assert(explorer != NULL);
    return explorer->Configuration.DataOutputMode;
}

void ExplorerApp_ResetDefaultDataStreamingMode(explorer_t * explorer)
{
    assert(explorer != NULL);
    assert(explorer->Argus != NULL);

    argus_mode_t mode = ARGUS_MODE_SHORT_RANGE;
    if (Argus_GetMeasurementMode(explorer->Argus, &mode) == STATUS_OK)
    {
        explorer->Configuration.DataOutputMode = (mode & ARGUS_MODE_FLAG_HIGH_SPEED) ?
                DATA_OUTPUT_STREAMING_1D : DATA_OUTPUT_STREAMING_FULL;
    }
}
