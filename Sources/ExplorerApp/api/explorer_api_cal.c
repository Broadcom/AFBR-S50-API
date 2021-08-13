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
#include "explorer_api_cal.h"
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

static void Serialize_Cal_P2PXtalk(sci_frame_t * frame, argus_cal_p2pxtalk_t const * cal)
{
	assert(frame != 0);
	assert(cal != 0);

	/* Pixel-To-Pixel Crosstalk */
	SCI_Frame_Queue16u(frame, cal->KcFactorS);
	SCI_Frame_Queue16u(frame, cal->KcFactorC);
	SCI_Frame_Queue16u(frame, cal->KcFactorSRefPx);
	SCI_Frame_Queue16u(frame, cal->KcFactorCRefPx);
	SCI_Frame_Queue08s(frame, cal->RelativeThreshold);
	SCI_Frame_Queue16u(frame, cal->AbsoluteTreshold);
}
static void Deserialize_Cal_P2PXtalk(sci_frame_t * frame, argus_cal_p2pxtalk_t * cal)
{
	assert(frame != 0);
	assert(cal != 0);

	/* Pixel-To-Pixel Crosstalk */
	cal->KcFactorS = SCI_Frame_Dequeue16s(frame);
	cal->KcFactorC = SCI_Frame_Dequeue16s(frame);
	cal->KcFactorSRefPx = SCI_Frame_Dequeue16s(frame);
	cal->KcFactorCRefPx = SCI_Frame_Dequeue16s(frame);
	cal->RelativeThreshold = SCI_Frame_Dequeue08u(frame);
	cal->AbsoluteTreshold = SCI_Frame_Dequeue16u(frame);
}


/*******************************************************************************
 * Command Functions
 ******************************************************************************/

static status_t RxCmd_CalGlobalRangeOffset(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			q0_15_t offset = SCI_Frame_Dequeue16s(frame);
			return Argus_SetCalibrationGlobalRangeOffset(myArgusPtr, mode, offset);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CALIBRATION_GLOBAL_RANGE_OFFSET, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CalGlobalRangeOffset(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	assert(frame != 0);
	status_t status = STATUS_OK;
	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);
	q0_15_t offset;
	status = Argus_GetCalibrationGlobalRangeOffset(myArgusPtr, mode, &offset);
	SCI_Frame_Queue16s(frame, offset);
	return status;
}

static status_t RxCmd_CalPixelRangeOffsets(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			q0_15_t offsets[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
			for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
			{
				for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
				{
					offsets[x][y] = SCI_Frame_Dequeue16s(frame);
				}
			}
			return Argus_SetCalibrationPixelRangeOffsets(myArgusPtr, mode, offsets);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CALIBRATION_PIXEL_RANGE_OFFSETS, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CalPixelRangeOffsets(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	assert(frame != 0);
	status_t status = STATUS_OK;
	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);
	q0_15_t offsets[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
	status = Argus_GetCalibrationPixelRangeOffsets(myArgusPtr, mode, offsets);
	for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
	{
		for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
		{
			SCI_Frame_Queue16s(frame, offsets[x][y]);
		}
	}
	return status;
}

static status_t RxCmd_CalResetPixelRangeOffsets(sci_frame_t * frame)
{
	argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
	return Argus_ResetCalibrationPixelRangeOffsets(myArgusPtr, mode);
}

static status_t RxCmd_CalRangeOffsetSeqSampleTime(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{ /* Master sending data... */
		uint16_t time = SCI_Frame_Dequeue16u(frame);
		return Argus_SetCalibrationRangeOffsetSequenceSampleTime(myArgusPtr, time);
	}
	else
	{ /* Master is requesting data... */
		return SCI_SendCommand(CMD_CALIBRATION_RANGE_OFFSET_SAMPLE_TIME, 0, 0);
	}
}
static status_t TxCmd_CalRangeOffsetSeqSampleTime(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	(void) param;
	assert(frame != 0);
	status_t status = STATUS_OK;
	uint16_t time;
	status = Argus_GetCalibrationRangeOffsetSequenceSampleTime(myArgusPtr, &time);
	SCI_Frame_Queue16u(frame, time);
	return status;
}

static status_t RxCmd_CalXtalkPixel2Pixel(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			argus_cal_p2pxtalk_t cal = { 0 };
			cal.Enabled = SCI_Frame_Dequeue08u(frame) != 0;
			Deserialize_Cal_P2PXtalk(frame, &cal);
			return Argus_SetCalibrationCrosstalkPixel2Pixel(myArgusPtr, mode, &cal);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CALIBRATION_XTALK_PIXEL_2_PIXEL, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CalXtalkPixel2Pixel(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	assert(frame != 0);
	status_t status = STATUS_OK;
	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);

	argus_cal_p2pxtalk_t cal = { 0 };
	status = Argus_GetCalibrationCrosstalkPixel2Pixel(myArgusPtr, mode, &cal);
	SCI_Frame_Queue08u(frame, cal.Enabled != 0);
	Serialize_Cal_P2PXtalk(frame, &cal);
	return status;
}

static status_t RxCmd_CalXtalkVectorTable(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{
		argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
		if (SCI_Frame_BytesToRead(frame) > 1)
		{ /* Master sending data... */
			xtalk_t xtalk[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
			for (uint_fast8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; ++f)
			{
				for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
				{
					for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
					{
						xtalk[f][x][y].dS = SCI_Frame_Dequeue16s(frame);
						xtalk[f][x][y].dC = SCI_Frame_Dequeue16s(frame);
					}
				}
			}
			return Argus_SetCalibrationCrosstalkVectorTable(myArgusPtr, mode, xtalk);
		}
		else
		{ /* Master is requesting data... */
			return SCI_SendCommand(CMD_CALIBRATION_XTALK_VECTOR_TABLE, mode, 0);
		}
	}
	else
	{
		return ERROR_SCI_FRAME_TOO_SHORT;
	}
}
static status_t TxCmd_CalXtalkVectorTable(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	assert(frame != 0);
	status_t status = STATUS_OK;
	if (param == 0) return ERROR_SCI_INVALID_CMD_PARAMETER;
	argus_mode_t mode = (argus_mode_t) param;
	SCI_Frame_Queue08u(frame, mode);
	xtalk_t xtalk[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
	status = Argus_GetCalibrationCrosstalkVectorTable(myArgusPtr, mode, xtalk);
	for (uint_fast8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; ++f)
	{
		for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
		{
			for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
			{
				SCI_Frame_Queue16s(frame, xtalk[f][x][y].dS);
				SCI_Frame_Queue16s(frame, xtalk[f][x][y].dC);
			}
		}
	}
	return status;
}

static status_t RxCmd_CalXtalkResetVectorTable(sci_frame_t * frame)
{
	argus_mode_t mode = SCI_Frame_Dequeue08u(frame);
	return Argus_ResetCalibrationCrosstalkVectorTable(myArgusPtr, mode);
}

static status_t RxCmd_CalXtalkSeqSampleTime(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{ /* Master sending data... */
		uint16_t time = SCI_Frame_Dequeue16u(frame);
		return Argus_SetCalibrationCrosstalkSequenceSampleTime(myArgusPtr, time);
	}
	else
	{ /* Master is requesting data... */
		return SCI_SendCommand(CMD_CALIBRATION_XTALK_SAMPLE_TIME, 0, 0);
	}
}
static status_t TxCmd_CalXtalkSeqSampleTime(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	(void) param;
	assert(frame != 0);
	status_t status = STATUS_OK;
	uint16_t time;
	status = Argus_GetCalibrationCrosstalkSequenceSampleTime(myArgusPtr, &time);
	SCI_Frame_Queue16u(frame, time);
	return status;
}

static status_t RxCmd_CalXtalkSeqMaxAmplitude(sci_frame_t * frame)
{
	if (SCI_Frame_BytesToRead(frame) > 1)
	{ /* Master sending data... */
		uq12_4_t ampl = SCI_Frame_Dequeue16u(frame);
		return Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(myArgusPtr, ampl);
	}
	else
	{ /* Master is requesting data... */
		return SCI_SendCommand(CMD_CALIBRATION_XTALK_MAX_AMPLITUDE, 0, 0);
	}
}
static status_t TxCmd_CalXtalkSeqMaxAmplitude(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) data;
	(void) param;
	assert(frame != 0);
	status_t status = STATUS_OK;
	uq12_4_t ampl;
	status = Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold(myArgusPtr, &ampl);
	SCI_Frame_Queue16u(frame, ampl);
	return status;
}

/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitCal(argus_hnd_t * argus)
{
	assert(argus != 0);
	myArgusPtr = argus;

	status_t
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_GLOBAL_RANGE_OFFSET, RxCmd_CalGlobalRangeOffset, TxCmd_CalGlobalRangeOffset);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_PIXEL_RANGE_OFFSETS, RxCmd_CalPixelRangeOffsets, TxCmd_CalPixelRangeOffsets);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxCommand(CMD_CALIBRATION_PIXEL_RANGE_OFFSETS_RESET, RxCmd_CalResetPixelRangeOffsets);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_RANGE_OFFSET_SAMPLE_TIME, RxCmd_CalRangeOffsetSeqSampleTime, TxCmd_CalRangeOffsetSeqSampleTime);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_XTALK_VECTOR_TABLE, RxCmd_CalXtalkVectorTable, TxCmd_CalXtalkVectorTable);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxCommand(CMD_CALIBRATION_XTALK_RESET_VECTOR_TABLE, RxCmd_CalXtalkResetVectorTable);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_XTALK_SAMPLE_TIME, RxCmd_CalXtalkSeqSampleTime, TxCmd_CalXtalkSeqSampleTime);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_XTALK_MAX_AMPLITUDE, RxCmd_CalXtalkSeqMaxAmplitude, TxCmd_CalXtalkSeqMaxAmplitude);
	if (status < STATUS_OK) return status;
	status = SCI_SetRxTxCommand(CMD_CALIBRATION_XTALK_PIXEL_2_PIXEL, RxCmd_CalXtalkPixel2Pixel, TxCmd_CalXtalkPixel2Pixel);
	if (status < STATUS_OK) return status;


	return status;
}


