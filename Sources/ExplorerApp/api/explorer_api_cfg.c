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
#include "explorer_api_cfg.h"
#include "explorer_app.h"
#include "explorer_flash.h"

#include "argus.h"

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

/*! The Argus data structure for executing device specific commands. */
static argus_hnd_t * myArgusPtr;

/*******************************************************************************
 * Parsing Functions
 ******************************************************************************/

static void Serialize_Cfg_DCA(sci_frame_t * frame, argus_cfg_dca_t const * dcacfg)
{
	assert(frame != 0);
	assert(dcacfg != 0);

	SCI_Frame_Queue08u(frame, (uint8_t)((dcacfg->Enabled > 0 ? 1U : 0U)
									  | (dcacfg->Enabled < 0 ? 2U : 0U)));
	SCI_Frame_Queue08u(frame, dcacfg->SatPxThLin);
	SCI_Frame_Queue08u(frame, dcacfg->SatPxThExp);
	SCI_Frame_Queue08u(frame, dcacfg->SatPxThRst);
	SCI_Frame_Queue16u(frame, dcacfg->Atarget);
	SCI_Frame_Queue16u(frame, dcacfg->AthLow);
	SCI_Frame_Queue16u(frame, dcacfg->AthHigh);
	SCI_Frame_Queue16u(frame, dcacfg->DepthNom);
	SCI_Frame_Queue16u(frame, dcacfg->DepthMin);
	SCI_Frame_Queue16u(frame, dcacfg->DepthMax);
	SCI_Frame_Queue16u(frame, dcacfg->PowerNom);
	SCI_Frame_Queue16u(frame, dcacfg->PowerMin);
	SCI_Frame_Queue08u(frame, dcacfg->GainNom);
	SCI_Frame_Queue08u(frame, dcacfg->GainMin);
	SCI_Frame_Queue08u(frame, dcacfg->GainMax);
	SCI_Frame_Queue08u(frame, dcacfg->PowerSavingRatio);
}
static void Deserialize_Cfg_DCA(sci_frame_t * frame, argus_cfg_dca_t * dcacfg)
{
	assert(frame != 0);
	assert(dcacfg != 0);

	/* Dynamic Configuration Adaption. */
	uint8_t tmp = SCI_Frame_Dequeue08u(frame);
	dcacfg->Enabled = (tmp & 1U) ? 1 : (tmp & 2U) ? -1 : 0;
	dcacfg->SatPxThLin = SCI_Frame_Dequeue08u(frame);
	dcacfg->SatPxThExp = SCI_Frame_Dequeue08u(frame);
	dcacfg->SatPxThRst = SCI_Frame_Dequeue08u(frame);
	dcacfg->Atarget = SCI_Frame_Dequeue16u(frame);
	dcacfg->AthLow = SCI_Frame_Dequeue16u(frame);
	dcacfg->AthHigh = SCI_Frame_Dequeue16u(frame);
	dcacfg->DepthNom = SCI_Frame_Dequeue16u(frame);
	dcacfg->DepthMin = SCI_Frame_Dequeue16u(frame);
	dcacfg->DepthMax = SCI_Frame_Dequeue16u(frame);
	dcacfg->PowerNom = SCI_Frame_Dequeue16u(frame);
	dcacfg->PowerMin = SCI_Frame_Dequeue16u(frame);
	dcacfg->GainNom = SCI_Frame_Dequeue08u(frame);
	dcacfg->GainMin = SCI_Frame_Dequeue08u(frame);
	dcacfg->GainMax = SCI_Frame_Dequeue08u(frame);
	dcacfg->PowerSavingRatio = SCI_Frame_Dequeue08u(frame);
}

static void Serialize_Cfg_PBA(sci_frame_t * frame, argus_cfg_pba_t const * pba)
{
	assert(frame != 0);
	assert(pba != 0);

	SCI_Frame_Queue08u(frame, pba->Enabled);
	SCI_Frame_Queue08u(frame, pba->Mode);
	SCI_Frame_Queue32u(frame, pba->PrefilterMask);
	SCI_Frame_Queue16u(frame, pba->AbsAmplThreshold);
	SCI_Frame_Queue08u(frame, pba->RelAmplThreshold);
	SCI_Frame_Queue16u(frame, pba->AbsMinDistanceScope);
	SCI_Frame_Queue08u(frame, pba->RelMinDistanceScope);
}
static void Deserialize_Cfg_PBA(sci_frame_t * frame, argus_cfg_pba_t * pba)
{
	assert(frame != 0);
	assert(pba != 0);

	pba->Enabled = SCI_Frame_Dequeue08u(frame);
	pba->Mode = SCI_Frame_Dequeue08u(frame);
	pba->PrefilterMask = SCI_Frame_Dequeue32u(frame);
	pba->AbsAmplThreshold = SCI_Frame_Dequeue16u(frame);
	pba->RelAmplThreshold = SCI_Frame_Dequeue08u(frame);
	pba->AbsMinDistanceScope = SCI_Frame_Dequeue16u(frame);
	pba->RelMinDistanceScope = SCI_Frame_Dequeue08u(frame);
}


/*******************************************************************************
 * Command Functions
 ******************************************************************************/

static status_t RxCmd_CfgDataOutputMode(sci_frame_t * frame)
{
	(void)frame;

	if(SCI_Frame_BytesToRead(frame) > 1)
	{	/* Master sending data... */
		explorer_cfg_t cfg;
		status_t status = ExplorerApp_GetConfiguration(&cfg);
		if(status < STATUS_OK) return status;
		cfg.DataOutputMode = SCI_Frame_Dequeue08u(frame);
		return ExplorerApp_SetConfiguration(&cfg);
	}
	else
	{	/* Master is requesting data... */
		return SCI_SendCommand(CMD_CONFIGURATION_DATA_OUTPUT_MODE, 0, 0);
	}
}
static status_t TxCmd_CfgDataOutputMode(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	explorer_cfg_t cfg;
	status_t status = ExplorerApp_GetConfiguration(&cfg);
	SCI_Frame_Queue08u(frame, cfg.DataOutputMode);
	return status;
}

static status_t RxCmd_CfgMeasurementMode(sci_frame_t * frame)
{
	if(SCI_Frame_BytesToRead(frame) > 1)
	{	/* Master sending data... */
		argus_mode_t mode = SCI_Frame_Dequeue08s(frame);
		status_t status = Argus_SetConfigurationMeasurementMode(myArgusPtr, mode);
		ExplorerApp_DisplayUnambiguousRange();
		return status;
	}
	else
	{	/* Master is requesting data... */
		return SCI_SendCommand(CMD_CONFIGURATION_MEASUREMENT_MODE, 0, 0);
	}
}
static status_t TxCmd_CfgMeasurementMode(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	argus_mode_t mode = 0;
	status_t status = Argus_GetConfigurationMeasurementMode(myArgusPtr, &mode);
	SCI_Frame_Queue08s(frame, mode);
	return status;
}

static status_t RxCmd_CfgFrameTime(sci_frame_t * frame)
{
	if(SCI_Frame_BytesToRead(frame) > 1)
	{	/* Master sending data... */
		uint32_t frameTime = SCI_Frame_Dequeue32u(frame);
		return Argus_SetConfigurationFrameTime(myArgusPtr, frameTime);
	}
	else
	{	/* Master is requesting data... */
		return SCI_SendCommand(CMD_CONFIGURATION_FRAME_TIME, 0, 0);
	}
}
static status_t TxCmd_CfgFrameTime(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	uint32_t frameTime = 0;
	status_t status = Argus_GetConfigurationFrameTime(myArgusPtr, &frameTime);
	SCI_Frame_Queue32u(frame, frameTime);
	return status;
}

static status_t RxCmd_CfgSmartPowerSaveEnabled(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			bool enabled = SCI_Frame_Dequeue08u(frame) != 0;
			return Argus_SetConfigurationSmartPowerSaveEnabled(myArgusPtr, mode, enabled);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CONFIGURATION_SMART_POWER_SAVE, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CfgSmartPowerSaveEnabled(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)data;

	assert(frame != 0);
	status_t status = STATUS_OK;

	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);

	bool enabled = 0;
	 status = Argus_GetConfigurationSmartPowerSaveEnabled(myArgusPtr, mode, &enabled);
	SCI_Frame_Queue08u(frame, enabled);

	return status;
}

static status_t RxCmd_CfgDualFrequencyMode(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			argus_dfm_mode_t dfm = SCI_Frame_Dequeue08u(frame);
			status_t status = Argus_SetConfigurationDFMMode(myArgusPtr, mode, dfm);
			ExplorerApp_DisplayUnambiguousRange();
			return status;
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CONFIGURATION_DUAL_FREQUENCY_MODE, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CfgDualFrequencyMode(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;

	assert(frame != 0);
	status_t status = STATUS_OK;

	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);

	argus_dfm_mode_t dfm;
	status = Argus_GetConfigurationDFMMode(myArgusPtr, mode, &dfm);
	SCI_Frame_Queue08u(frame, dfm);

	return status;
}

static status_t RxCmd_CfgShotNoiseMonitor(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			argus_snm_mode_t snm = SCI_Frame_Dequeue08u(frame);
			return Argus_SetConfigurationShotNoiseMonitorMode(myArgusPtr, mode, snm);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CONFIGURATION_SHOT_NOISE_MONITOR_MODE, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CfgShotNoiseMonitor(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;

	assert(frame != 0);
	status_t status = STATUS_OK;

	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);

	argus_snm_mode_t snm;
	status = Argus_GetConfigurationShotNoiseMonitorMode(myArgusPtr, mode, &snm);
	SCI_Frame_Queue08u(frame, snm);

	return status;
}

static status_t RxCmd_CfgDca(sci_frame_t * frame)
{
	if(SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if(SCI_Frame_BytesToRead(frame) > 1)
		{	/* Master sending data... */
			argus_cfg_dca_t dca = {0};
			Deserialize_Cfg_DCA(frame, &dca);
			return Argus_SetConfigurationDynamicAdaption(myArgusPtr, mode, &dca);
		}
		else
		{	/* Master is requesting data... */
			return SCI_SendCommand(CMD_CONFIGURATION_DCA, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CfgDca(sci_frame_t * frame,  sci_param_t param, sci_data_t data)
{
	assert(frame != 0);
	status_t status = STATUS_OK;

	if(param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t)param;
	SCI_Frame_Queue08u(frame, mode);

	if (data == 0)
	{
		argus_cfg_dca_t dca = {0};
		status = Argus_GetConfigurationDynamicAdaption(myArgusPtr, mode, &dca);
		Serialize_Cfg_DCA(frame, &dca);
	}
	else
	{
		Serialize_Cfg_DCA(frame, (argus_cfg_dca_t*)data);
	}

	return status;
}

static status_t RxCmd_CfgPba(sci_frame_t * frame)
{
	if(SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if(SCI_Frame_BytesToRead(frame) > 1)
		{	/* Master sending data... */
			argus_cfg_pba_t pba = {0};
			Deserialize_Cfg_PBA(frame, &pba);
			return Argus_SetConfigurationPixelBinning(myArgusPtr, mode, &pba);
		}
		else
		{	/* Master is requesting data... */
			return SCI_SendCommand(CMD_CONFIGURATION_PBA, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CfgPba(sci_frame_t * frame, sci_param_t param, void const * data)
{
	assert(frame != 0);
	status_t status = STATUS_OK;

	if(param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t)param;
	SCI_Frame_Queue08u(frame, mode);

	if (data == 0)
	{
		argus_cfg_pba_t pba = {0};
		status = Argus_GetConfigurationPixelBinning(myArgusPtr, mode, &pba);
		Serialize_Cfg_PBA(frame, &pba);
	}
	else
	{
		Serialize_Cfg_PBA(frame, (argus_cfg_pba_t*)data);
	}

	return status;
}

static status_t RxCmd_CfgSpi(sci_frame_t * frame)
{
	if(SCI_Frame_BytesToRead(frame) > 1)
	{	/* Master sending data... */
		explorer_cfg_t cfg;
		status_t status = ExplorerApp_GetConfiguration(&cfg);
		if(status < STATUS_OK) return status;
		cfg.SPISlave = SCI_Frame_Dequeue08s(frame);
		cfg.SPIBaudRate = SCI_Frame_Dequeue32u(frame);
		return ExplorerApp_SetConfiguration(&cfg);
	}
	else
	{	/* Master is requesting data... */
		return SCI_SendCommand(CMD_CONFIGURATION_SPI, 0, 0);
	}
}
static status_t TxCmd_CfgSpi(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void)param;
	(void)data;
	explorer_cfg_t cfg;
	status_t status = ExplorerApp_GetConfiguration(&cfg);
	SCI_Frame_Queue08s(frame, cfg.SPISlave);
	SCI_Frame_Queue32u(frame, cfg.SPIBaudRate);
	return status;
}

/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitCfg(argus_hnd_t * argus)
{
	assert(argus != 0);
	myArgusPtr = argus;

	status_t
	status = SCI_SetCommand(CMD_CONFIGURATION_DATA_OUTPUT_MODE, RxCmd_CfgDataOutputMode, TxCmd_CfgDataOutputMode);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_MEASUREMENT_MODE, RxCmd_CfgMeasurementMode, TxCmd_CfgMeasurementMode);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_FRAME_TIME, RxCmd_CfgFrameTime, TxCmd_CfgFrameTime);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_DUAL_FREQUENCY_MODE, RxCmd_CfgDualFrequencyMode, TxCmd_CfgDualFrequencyMode);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_SHOT_NOISE_MONITOR_MODE, RxCmd_CfgShotNoiseMonitor, TxCmd_CfgShotNoiseMonitor);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_SMART_POWER_SAVE, RxCmd_CfgSmartPowerSaveEnabled, TxCmd_CfgSmartPowerSaveEnabled);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_DCA, RxCmd_CfgDca, TxCmd_CfgDca);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_PBA, RxCmd_CfgPba, TxCmd_CfgPba);
	if (status < STATUS_OK) return status;
	status = SCI_SetCommand(CMD_CONFIGURATION_SPI, RxCmd_CfgSpi, TxCmd_CfgSpi);
	if (status < STATUS_OK) return status;

	return status;
}


