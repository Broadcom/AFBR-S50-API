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
#include "explorer_api_data.h"

#include "utility/fp_rnd.h"
#include "utility/int_math.h"

#include "assert.h"


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
 * Parsing Functions
 ******************************************************************************/

static void Serialize_MeasurementData_FrameConfig(sci_frame_t * frame, argus_results_t const * res)
{
	SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.DigitalIntegrationDepth);
	SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.AnalogIntegrationDepth);
	SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.OutputPower);
	SCI_Frame_Queue08u(frame, (uint8_t) res->Frame.PixelGain);
}
static void Serialize_MeasurementData_RawData(sci_frame_t * frame, argus_results_t const * res)
{
	SCI_Frame_Queue08u(frame, ARGUS_PHASECOUNT);

	/* Raw Samples */
	uint32_t msk = 0;
	uint32_t const * data = res->Data;

	msk = res->Frame.PxEnMask;
	while (msk)
	{
		if (msk & 1)
		{
			for (uint_fast8_t p = 0; p < ARGUS_PHASECOUNT; ++p)
			{
				SCI_Frame_Queue24u(frame, *(data++));
			}
		}
		msk >>= 1U;
	}

	msk = res->Frame.ChEnMask;
	while (msk)
	{
		if (msk & 1)
		{
			for (uint_fast8_t p = 0; p < ARGUS_PHASECOUNT; ++p)
			{
				SCI_Frame_Queue24u(frame, *(data++));
			}
		}
		msk >>= 1U;
	}
}
static void Serialize_MeasurementData_3D(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
	for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
	{
		for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
		{
			if (!(res->Pixel[x][y].Status & PIXEL_OFF))
			{
				SCI_Frame_Queue08u(frame, res->Pixel[x][y].Status);
			}
		}
	}

	for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
	{
		for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
		{
			if (!(res->Pixel[x][y].Status & PIXEL_OFF))
			{
				if (res->Pixel[x][y].Range == ARGUS_RANGE_MAX)
				{
					SCI_Frame_Queue24s(frame, ARGUS_RANGE_MAX >> 8);
				}
				else
				{
					SCI_Frame_Queue24s(frame, fp_rnds(res->Pixel[x][y].Range, 8U));
				}
			}
		}
	}

	for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
	{
		for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
		{
			if (!(res->Pixel[x][y].Status & PIXEL_OFF))
			{
				SCI_Frame_Queue16u(frame, res->Pixel[x][y].Amplitude);
			}
		}
	}

	if (type == CMD_MEASUREMENT_DATA_DEBUG ||
		type == CMD_MEASUREMENT_DATA_3D_DEBUG)
	{
		for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
		{
			for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
			{
				if (!(res->Pixel[x][y].Status & PIXEL_OFF))
				{
					SCI_Frame_Queue16u(frame, res->Pixel[x][y].Phase);
				}
			}
		}
	}

	if (type == CMD_MEASUREMENT_DATA_DEBUG)
	{
		/* Ref. Pixel Data. */
		if (!(res->PixelRef.Status & PIXEL_OFF))
		{
			SCI_Frame_Queue08u(frame, res->PixelRef.Status);

			if (res->PixelRef.Range == ARGUS_RANGE_MAX)
			{
				SCI_Frame_Queue24s(frame, ARGUS_RANGE_MAX >> 8);
			}
			else
			{
				SCI_Frame_Queue24s(frame, fp_rnds(res->PixelRef.Range, 8U));
			}

			SCI_Frame_Queue16u(frame, res->PixelRef.Amplitude);
			SCI_Frame_Queue16u(frame, res->PixelRef.Phase);
		}
	}
}
static void Serialize_MeasurementData_1D(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
	if (type == CMD_MEASUREMENT_DATA_1D_DEBUG)
	{
		uint8_t px_count = 0;
		uint8_t sat_count = 0;
		for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
		{
			for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
			{
				if (!(res->Pixel[x][y].Status & (PIXEL_OFF | PIXEL_BIN_EXCL)))
				{
					px_count++;
				}
				if (res->Pixel[x][y].Status & PIXEL_SAT)
				{
					sat_count++;
				}
			}
		}
		SCI_Frame_Queue08u(frame, px_count);
		SCI_Frame_Queue08u(frame, sat_count);
	}

	if (res->Bin.Range == ARGUS_RANGE_MAX)
	{
		SCI_Frame_Queue24s(frame, ARGUS_RANGE_MAX >> 8);
	}
	else
	{
		SCI_Frame_Queue24s(frame, fp_rnds(res->Bin.Range, 8U));
	}

	SCI_Frame_Queue16u(frame, res->Bin.Amplitude);
}
static void Serialize_MeasurementData_Aux(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
	(void) type;
	SCI_Frame_Queue16u(frame, res->Auxiliary.VDD);
	SCI_Frame_Queue16u(frame, res->Auxiliary.VDDL);
	SCI_Frame_Queue16u(frame, res->Auxiliary.VSUB);
	SCI_Frame_Queue16u(frame, res->Auxiliary.IAPD);
	SCI_Frame_Queue16s(frame, res->Auxiliary.TEMP);
	SCI_Frame_Queue16u(frame, res->Auxiliary.BGL);
	SCI_Frame_Queue16u(frame, res->Auxiliary.SNA);
}
static void Serialize_MeasurementData(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
	SCI_Frame_Queue16u(frame, res->Status);
	SCI_Frame_Queue_Time(frame, &res->TimeStamp);

	SCI_Frame_Queue16u(frame, res->Frame.State);

	if (type == CMD_MEASUREMENT_DATA_RAW ||
	    type == CMD_MEASUREMENT_DATA_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_FULL ||
	    type == CMD_MEASUREMENT_DATA_3D_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_3D ||
	    type == CMD_MEASUREMENT_DATA_1D_DEBUG)
	{
		Serialize_MeasurementData_FrameConfig(frame, res);
	}

	if (type == CMD_MEASUREMENT_DATA_RAW ||
	    type == CMD_MEASUREMENT_DATA_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_FULL ||
	    type == CMD_MEASUREMENT_DATA_3D_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_3D)
	{
		SCI_Frame_Queue32u(frame, (uint32_t) res->Frame.PxEnMask);
	}

	if (type == CMD_MEASUREMENT_DATA_RAW ||
	    type == CMD_MEASUREMENT_DATA_DEBUG)
	{
		SCI_Frame_Queue32u(frame, (uint32_t) res->Frame.ChEnMask);
		Serialize_MeasurementData_RawData(frame, res);
	}

	if (type == CMD_MEASUREMENT_DATA_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_FULL ||
	    type == CMD_MEASUREMENT_DATA_3D_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_3D)
	{
		Serialize_MeasurementData_3D(frame, res, type);
	}

	if (type == CMD_MEASUREMENT_DATA_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_FULL ||
	    type == CMD_MEASUREMENT_DATA_1D_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_1D)
	{
		Serialize_MeasurementData_1D(frame, res, type);
	}

	if (type == CMD_MEASUREMENT_DATA_DEBUG ||
	    type == CMD_MEASUREMENT_DATA_FULL)
	{
		Serialize_MeasurementData_Aux(frame, res, type);
	}
}


/*******************************************************************************
 * Command Functions
 ******************************************************************************/

status_t SendMeasurementData(sci_cmd_t type, argus_results_t const * res)
{
	return SCI_SendCommand(type, 0, res);
}

static status_t TxCmd_MeasurementDataRaw(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_RAW);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementDataDebug(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_DEBUG);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementDataFull(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_FULL);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementData3DDebug(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_3D_DEBUG);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementData3D(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_3D);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementData1DDebug(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_1D_DEBUG);
	return STATUS_OK;
}

static status_t TxCmd_MeasurementData1D(sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
	(void) param;
	(void) data;
	assert(frame != 0);
	if (data == 0) return ERROR_INVALID_ARGUMENT;
	Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_1D);
	return STATUS_OK;
}

/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitData(void)
{
	status_t
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_RAW, TxCmd_MeasurementDataRaw);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_DEBUG, TxCmd_MeasurementDataDebug);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_FULL, TxCmd_MeasurementDataFull);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_1D_DEBUG, TxCmd_MeasurementData1DDebug);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_1D, TxCmd_MeasurementData1D);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_3D_DEBUG, TxCmd_MeasurementData3DDebug);
	if (status < STATUS_OK) return status;
	status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_3D, TxCmd_MeasurementData3D);
	if (status < STATUS_OK) return status;

	return status;
}

