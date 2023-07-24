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
#include "core/core_device.h"
#include "core/core_utils.h"
#include "core/core_cfg.h"
#include "core/core_cal.h"
#include "core/explorer_version.h"
#include "core/explorer_config.h"
#include "core/explorer_status.h"
#include "explorer_api.h"
#include "board/board.h"
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
 * Local functions
 ******************************************************************************/

/*******************************************************************************
 * Generic Commands
 ******************************************************************************/

/*******************************************************************************
 * Software Information Commands
 ******************************************************************************/

static status_t RxCmd_SoftwareInfo(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    return SCI_SendCommand(deviceID, CMD_SOFTWARE_INFO, 0, 0);
}
static status_t TxCmd_SoftwareInfo(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param,
                                   sci_data_t data)
{
    (void)param;
    (void)data;

    SCI_Frame_Queue32u(frame, EXPLORER_VERSION);
    SCI_Frame_Queue32u(frame, Argus_GetAPIVersion());

    if (deviceID == 0)
    {
        const uint8_t deviceCount = ExplorerApp_GetInitializedExplorerCount();
        SCI_Frame_Queue08u(frame, deviceCount);
        for (deviceID = EXPLORER_DEVICE_ID_MIN; deviceID <= EXPLORER_DEVICE_ID_MAX; deviceID++)
        {

            argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
            if (argus == NULL) continue;

            SCI_Frame_Queue08u(frame, deviceID);
            SCI_Frame_Queue08u(frame, Argus_GetModuleVersion(argus));
            SCI_Frame_Queue08u(frame, Argus_GetChipVersion(argus));
            SCI_Frame_Queue08u(frame, Argus_GetLaserType(argus));
            SCI_Frame_Queue24u(frame, Argus_GetChipID(argus));
        }
    }
    else
    {
        SCI_Frame_Queue08u(frame, 1);
        SCI_Frame_Queue08u(frame, deviceID);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus != NULL)
        {
            SCI_Frame_Queue08u(frame, Argus_GetModuleVersion(argus));
            SCI_Frame_Queue08u(frame, Argus_GetChipVersion(argus));
            SCI_Frame_Queue08u(frame, Argus_GetLaserType(argus));
            SCI_Frame_Queue24u(frame, Argus_GetChipID(argus));
        }
        else
        {
            SCI_Frame_Queue08u(frame, 0);
            SCI_Frame_Queue08u(frame, 0);
            SCI_Frame_Queue08u(frame, 0);
            SCI_Frame_Queue24u(frame, 0);
        }
    }

    char const * name = "AFBR-S50 Explorer App - ";
    for (char const * c = name; *c != '\0'; c++)
    {
        SCI_Frame_PutChar(*c, frame);
    }
    char const * build = Argus_GetBuildNumber();
    for (char const * c = build; *c != '\0'; c++)
    {
        SCI_Frame_PutChar(*c, frame);
    }

    return STATUS_OK;
}

static status_t RxCmd_SoftwareVersion(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    return SCI_SendCommand(deviceID, CMD_SOFTWARE_VERSION, 0, 0);
}
static status_t TxCmd_SoftwareVersion(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;
    (void)deviceID;
    SCI_Frame_Queue32u(frame, Argus_GetAPIVersion());

    char const * build = Argus_GetBuildNumber();
    for(char const * c = build; *c != '\0'; c++)
    {
        SCI_Frame_PutChar(*c, frame);
    }
    return STATUS_OK;
}

static status_t RxCmd_ModuleType(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    return SCI_SendCommand(deviceID, CMD_MODULE_TYPE, 0, 0);
}
static status_t TxCmd_ModuleType(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    SCI_Frame_Queue08u(frame, Argus_GetModuleVersion(argus));
    SCI_Frame_Queue08u(frame, Argus_GetChipVersion(argus));
    SCI_Frame_Queue08u(frame, Argus_GetLaserType(argus));
    return STATUS_OK;
}
static status_t RxCmd_ModuleUID(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    return SCI_SendCommand(deviceID, CMD_MODULE_UID, 0, 0);
}
static status_t TxCmd_ModuleUID(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    SCI_Frame_Queue24u(frame, Argus_GetChipID(argus));
    return STATUS_OK;
}

/*******************************************************************************
 * Device Control Commands
 ******************************************************************************/
static status_t RxCmd_MeasurementStop(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return ExplorerApp_StopTimerMeasurement(argus);
}
static status_t RxCmd_MeasurementSingle(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return ExplorerApp_SingleMeasurement(argus);
}
static status_t RxCmd_MeasurementAuto(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return ExplorerApp_StartTimerMeasurement(argus);
}
static status_t RxCmd_MeasurementCalibration(sci_device_t deviceID, sci_frame_t * frame)
{
    explorer_cal_sequence_t seq = (explorer_cal_sequence_t) SCI_Frame_Dequeue08u(frame);

    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

    switch (seq)
    {
        case CALIBRATION_SEQUENCE_XTALK:
            return ExplorerApp_ExecuteXtalkCalibrationSequence(argus);

        case CALIBRATION_SEQUENCE_OFFSETS:
        {
            q9_22_t target = 0;
            if (SCI_Frame_BytesToRead(frame) > 1)
                target = SCI_Frame_Dequeue32s(frame);
            return ExplorerApp_ExecuteOffsetsCalibrationSequence(argus, target);
        }
        default:
            return ERROR_SCI_INVALID_CMD_PARAMETER;
    }
}
static status_t RxCmd_DeviceReinit(sci_device_t deviceID, sci_frame_t * frame)
{
    explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
    if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

    argus_mode_t mode = 0; // 0 uses current mode
    if (SCI_Frame_BytesToRead(frame) > 1)
        mode = SCI_Frame_Dequeue08s(frame);

    return ExplorerApp_DeviceReinit(explorer, mode);
}
static status_t RxCmd_DeviceAbort(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame; // unused parameter
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return ExplorerApp_DeviceAbort(argus);
}


/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitGeneral()
{
    status_t status;

    status = SCI_SetRxTxCommand(CMD_SOFTWARE_INFO, RxCmd_SoftwareInfo, (sci_tx_cmd_fct_t)TxCmd_SoftwareInfo);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_SOFTWARE_VERSION, RxCmd_SoftwareVersion, (sci_tx_cmd_fct_t)TxCmd_SoftwareVersion);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_MODULE_TYPE, RxCmd_ModuleType, (sci_tx_cmd_fct_t)TxCmd_ModuleType);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_MODULE_UID, RxCmd_ModuleUID, (sci_tx_cmd_fct_t)TxCmd_ModuleUID);
    if(status < STATUS_OK) return status;

    status = SCI_SetRxCommand(CMD_MEASUREMENT_STOP, RxCmd_MeasurementStop);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxCommand(CMD_MEASUREMENT_SINGLE_SHOT, RxCmd_MeasurementSingle);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxCommand(CMD_MEASUREMENT_START, RxCmd_MeasurementAuto);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxCommand(CMD_MEASUREMENT_CALIBRATION, RxCmd_MeasurementCalibration);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxCommand(CMD_DEVICE_REINIT, RxCmd_DeviceReinit);
    if(status < STATUS_OK) return status;
    status = SCI_SetRxCommand(CMD_DEVICE_ABORT, RxCmd_DeviceAbort);
    if(status < STATUS_OK) return status;

    return status;
}



