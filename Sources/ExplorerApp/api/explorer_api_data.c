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
#include "explorer_api_data.h"

#include "api/argus_map.h"
#include "utility/fp_rnd.h"
#include "utility/int_math.h"

#include "assert.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Parses a 32-bit range value to a 24-bit value that can be sent over SCI. */
#define PARSE_RANGE(r) (((r) == ARGUS_RANGE_MAX) ? (ARGUS_RANGE_MAX >> 8) : fp_rnds((r), 8))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Parsing Functions
 ******************************************************************************/
static void Serialize_MeasurementData_Generic(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    (void) type; // unused

    assert(res->Status >= INT16_MIN && res->Status <= INT16_MAX);

    SCI_Frame_Queue16s(frame, (int16_t)res->Status);
    SCI_Frame_Queue_Time(frame, &res->TimeStamp);
}
static void Serialize_MeasurementData_FrameConfig(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    SCI_Frame_Queue32u(frame, res->Frame.State);

    if (type != CMD_MEASUREMENT_DATA_1D || res->Debug != 0)
    {
        SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.DigitalIntegrationDepth);
        SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.AnalogIntegrationDepth);
        SCI_Frame_Queue16u(frame, (uint16_t) res->Frame.OutputPower);
        SCI_Frame_Queue08u(frame, (uint8_t) res->Frame.PixelGain);
        SCI_Frame_Queue32u(frame, (uint32_t) res->Frame.PxEnMask);
    }

    if (type != CMD_MEASUREMENT_DATA_1D)
    {
        SCI_Frame_Queue32u(frame, (uint32_t) res->Frame.ChEnMask);
    }
    else if (res->Debug != 0) // 1D + Debug Mode
    {
        uint32_t bin_msk = 0xFFFFFFFFU;
        uint32_t sat_msk = 0x00000000U;
        uint8_t n = 0;
        for (const argus_pixel_t * px = res->Pixels; px < &res->PixelRef; ++px, ++n)
        {
            if (!(px->Status & (PIXEL_OFF | PIXEL_BIN_EXCL)))
            PIXELN_DISABLE(bin_msk, n);
            if (px->Status & PIXEL_SAT)
            PIXELN_ENABLE(sat_msk, n);
        }
        SCI_Frame_Queue32u(frame, bin_msk);
        SCI_Frame_Queue32u(frame, sat_msk);
    }
}
static void Serialize_MeasurementData_RawData(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    if (type != CMD_MEASUREMENT_DATA_FULL || res->Debug == 0) return;

    SCI_Frame_Queue08u(frame, ARGUS_PHASECOUNT);

    /* Raw Samples */
    uint32_t const * data = res->Debug->Data;

    uint32_t msk = res->Frame.PxEnMask;
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
    if (type == CMD_MEASUREMENT_DATA_1D) return;

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
    if (!(res->PixelRef.Status & PIXEL_OFF))
    {
        SCI_Frame_Queue08u(frame, res->PixelRef.Status);
    }

    for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
    {
        for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
        {
            if (!(res->Pixel[x][y].Status & PIXEL_OFF))
            {
                SCI_Frame_Queue24s(frame, PARSE_RANGE(res->Pixel[x][y].Range));
            }
        }
    }
    if (!(res->PixelRef.Status & PIXEL_OFF))
    {
        SCI_Frame_Queue24s(frame, PARSE_RANGE(res->PixelRef.Range));
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
    if (!(res->PixelRef.Status & PIXEL_OFF))
    {
        SCI_Frame_Queue16u(frame, res->PixelRef.Amplitude);
    }

    if (res->Debug != 0) // if debug mode is enabled
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
        if (!(res->PixelRef.Status & PIXEL_OFF))
        {
            SCI_Frame_Queue16u(frame, res->PixelRef.Phase);
        }
    }
}

static void Serialize_MeasurementData_1D(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    if (type == CMD_MEASUREMENT_DATA_3D) return;

    SCI_Frame_Queue24s(frame, PARSE_RANGE(res->Bin.Range));
    SCI_Frame_Queue16u(frame, res->Bin.Amplitude);
    SCI_Frame_Queue08u(frame, res->Bin.SignalQuality);
}
static void Serialize_MeasurementData_Aux(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    if (type != CMD_MEASUREMENT_DATA_FULL && res->Debug == 0) return;

    SCI_Frame_Queue16u(frame, res->Auxiliary.VDD);
    SCI_Frame_Queue16u(frame, res->Auxiliary.VDDL);
    SCI_Frame_Queue16u(frame, res->Auxiliary.VSUB);
    SCI_Frame_Queue16u(frame, res->Auxiliary.IAPD);
    SCI_Frame_Queue16s(frame, res->Auxiliary.TEMP);
    SCI_Frame_Queue16u(frame, res->Auxiliary.BGL);
    SCI_Frame_Queue16u(frame, res->Auxiliary.SNA);
}
static void Serialize_MeasurementData_Debug(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    (void) type; // unused
    if (res->Debug == 0) return;

    SCI_Frame_Queue32u(frame, res->Frame.IntegrationTime);
    SCI_Frame_Queue08u(frame, res->Frame.BiasCurrent);
    SCI_Frame_Queue08s(frame, res->Frame.PllOffset);
    SCI_Frame_Queue08u(frame, res->Frame.PllCtrlCur);

    SCI_Frame_Queue16u(frame, res->Debug->DCAAmplitude);

    /* Crosstalk Values */
    for (uint_fast8_t y = 0; y < (ARGUS_PIXELS_Y >> 1); ++y)
    {
        SCI_Frame_Queue16s(frame, res->Debug->XtalkPredictor[y].dS);
        SCI_Frame_Queue16s(frame, res->Debug->XtalkPredictor[y].dC);
    }
    for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
    {
        SCI_Frame_Queue16s(frame, res->Debug->XtalkMonitor[y].dS);
        SCI_Frame_Queue16s(frame, res->Debug->XtalkMonitor[y].dC);
    }
}
static void Serialize_MeasurementData(sci_frame_t * frame, argus_results_t const * res, sci_cmd_t type)
{
    assert((type == CMD_MEASUREMENT_DATA_FULL) ||
           (type == CMD_MEASUREMENT_DATA_FULL_DEBUG) ||
           (type == CMD_MEASUREMENT_DATA_3D) ||
           (type == CMD_MEASUREMENT_DATA_3D_DEBUG) ||
           (type == CMD_MEASUREMENT_DATA_1D) ||
           (type == CMD_MEASUREMENT_DATA_1D_DEBUG));

    /* For message types w/ DEBUG, the res.Debug structure must be available!
     * For message types w/o DEBUG, the res.Debug structure must be null!
     * DEBUG types are odd, i.e. check for (type & 0x01).
     * Not DEBUG types are even, i.e. check for !(type & 0x01). */
    assert(((type & 0x01) && (res->Debug != 0)) || (!(type & 0x01) && (res->Debug == 0)));

    /* remove _DEBUG, its determined by res.Debug */
    if (type & 0x01) type++;

    Serialize_MeasurementData_Generic(frame, res, type);
    Serialize_MeasurementData_FrameConfig(frame, res, type);
    Serialize_MeasurementData_RawData(frame, res, type);
    Serialize_MeasurementData_3D(frame, res, type);
    Serialize_MeasurementData_1D(frame, res, type);
    Serialize_MeasurementData_Aux(frame, res, type);
    Serialize_MeasurementData_Debug(frame, res, type);
}


/*******************************************************************************
 * Command Functions
 ******************************************************************************/

static status_t TxCmd_MeasurementDataFullDebug(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
    assert(frame != 0);
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_FULL_DEBUG);
    return STATUS_OK;
}

static status_t TxCmd_MeasurementDataFull(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
    assert(frame != 0);
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_FULL);
    return STATUS_OK;
}

static status_t TxCmd_MeasurementData3DDebug(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
    assert(frame != 0);
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_3D_DEBUG);
    return STATUS_OK;
}

static status_t TxCmd_MeasurementData3D(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
    assert(frame != 0);
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_3D);
    return STATUS_OK;
}

static status_t TxCmd_MeasurementData1DDebug(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
    assert(frame != 0);
    if (data == 0) return ERROR_INVALID_ARGUMENT;
    Serialize_MeasurementData(frame, (argus_results_t const *) data, CMD_MEASUREMENT_DATA_1D_DEBUG);
    return STATUS_OK;
}

static status_t TxCmd_MeasurementData1D(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)deviceID; /* deviceID is selected through the 'data' parameter, which corresponds to the desired device */
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
    status = SCI_SetTxCommand(CMD_MEASUREMENT_DATA_FULL_DEBUG, TxCmd_MeasurementDataFullDebug);
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

