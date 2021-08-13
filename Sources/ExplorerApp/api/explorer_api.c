/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Demo Application.
 * @details		This file contains the hardware API of the Explorer Application.
 * 
 * @copyright
 * 
 * Copyright (c) 2021, Broadcom Inc
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
#include "explorer_api.h"
#include "explorer_app.h"

#include "explorer_api_cfg.h"
#include "explorer_api_cal.h"
#include "explorer_api_data.h"

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
/*! AFBR-S50 Explorer Software configuration data. */
extern explorer_cfg_t ExplorerConfiguration;

/*! The Argus data structure for executing device specific commands. */
static argus_hnd_t * myArgusPtr;

/*******************************************************************************
 * Generic Commands
 ******************************************************************************/

/*******************************************************************************
 * Software Information Commands
 ******************************************************************************/
#include "explorer_version.h"

static status_t RxCmd_SoftwareInfo(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return SCI_SendCommand(CMD_SOFTWARE_INFO, 0, 0);
}
static status_t TxCmd_SoftwareInfo(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;

	SCI_Frame_Queue32u(frame, EXPLORER_VERSION);
	SCI_Frame_Queue32u(frame, Argus_GetAPIVersion());
	SCI_Frame_Queue08u(frame, Argus_GetModuleVersion(myArgusPtr));
	SCI_Frame_Queue08u(frame, Argus_GetChipVersion(myArgusPtr));
	SCI_Frame_Queue08u(frame, Argus_GetLaserType(myArgusPtr));
	SCI_Frame_Queue24u(frame, Argus_GetChipID(myArgusPtr));

	char const * name = "AFBR-S50 Explorer App - ";
	for (char const *c = name; *c != '\0'; c++)
	{
		SCI_Frame_PutChar(frame, (int)*c);
	}
	char const * build = Argus_GetBuildNumber();
	for (char const *c = build; *c != '\0'; c++)
	{
		SCI_Frame_PutChar(frame, (int)*c);
	}
	return STATUS_OK;
}
static status_t RxCmd_SoftwareVersion(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return SCI_SendCommand(CMD_SOFTWARE_VERSION, 0, 0);
}
static status_t TxCmd_SoftwareVersion(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	SCI_Frame_Queue32u(frame, Argus_GetAPIVersion());

	char const * build = Argus_GetBuildNumber();
	for(char const * c = build; *c != '\0'; c++)
	{
		SCI_Frame_PutChar(frame, (int)*c);
	}
	return STATUS_OK;
}

static status_t RxCmd_ModuleType(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return SCI_SendCommand(CMD_MODULE_TYPE, 0, 0);
}
static status_t TxCmd_ModuleType(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	SCI_Frame_Queue08u(frame, Argus_GetModuleVersion(myArgusPtr));
	SCI_Frame_Queue08u(frame, Argus_GetChipVersion(myArgusPtr));
	SCI_Frame_Queue08u(frame, Argus_GetLaserType(myArgusPtr));
	return STATUS_OK;
}
static status_t RxCmd_ModuleUID(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return SCI_SendCommand(CMD_MODULE_UID, 0, 0);
}
static status_t TxCmd_ModuleUID(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	SCI_Frame_Queue24u(frame, Argus_GetChipID(myArgusPtr));
	return STATUS_OK;
}

/*******************************************************************************
 * Device Control Commands
 ******************************************************************************/
static status_t RxCmd_MeasurementStop(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return ExplorerApp_StopTimerMeasurement();
}
static status_t RxCmd_MeasurementSingle(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return ExplorerApp_SingleMeasurement();
}
static status_t RxCmd_MeasurementAuto(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return ExplorerApp_StartTimerMeasurement();
}
static status_t RxCmd_MeasurementCalibration(sci_frame_t * frame)
{
	cal_sequence_t seq = (cal_sequence_t) SCI_Frame_Dequeue08u(frame);

	switch (seq)
	{
		case CALIBRATION_SEQUENCE_XTALK_MODE_A:
			return ExplorerApp_ExecuteXtalkCalibrationSequence(ARGUS_MODE_A);
			break;
		case CALIBRATION_SEQUENCE_XTALK_MODE_B:
			return ExplorerApp_ExecuteXtalkCalibrationSequence(ARGUS_MODE_B);
			break;
		case CALIBRATION_SEQUENCE_OFFSETS_MODE_A:
		{
			q9_22_t target = 0;
			if(SCI_Frame_BytesToRead(frame) > 1) target = SCI_Frame_Dequeue32s(frame);
			return ExplorerApp_ExecuteOffsetsCalibrationSequence(ARGUS_MODE_A, target);
		}	break;
		case CALIBRATION_SEQUENCE_OFFSETS_MODE_B:
		{
			q9_22_t target = 0;
			if(SCI_Frame_BytesToRead(frame) > 1) target = SCI_Frame_Dequeue32s(frame);
			return ExplorerApp_ExecuteOffsetsCalibrationSequence(ARGUS_MODE_B, target);
		}	break;
		default:
			return ERROR_SCI_INVALID_CMD_PARAMETER;
			break;
	}
}
static status_t RxCmd_DeviceReinit(sci_frame_t * frame)
{
	(void)frame; // unused parameter
	return ExplorerApp_DeviceReinit();
}


/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerApp_InitCommands(argus_hnd_t * argus)
{
	assert(argus != 0);

	myArgusPtr = argus;

	status_t status = SCI_Init();
	if(status < STATUS_OK) return status;

	status = ExplorerAPI_InitData();
	if(status < STATUS_OK) return status;

	status = ExplorerAPI_InitCfg(argus);
	if(status < STATUS_OK) return status;

	status = ExplorerAPI_InitCal(argus);
	if(status < STATUS_OK) return status;

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

	return status;
}



