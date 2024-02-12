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
#include "explorer_api_cal.h"
#include "core/core_device.h"
#include "core/core_utils.h"
#include "core/explorer_status.h"
#include "core/explorer_macro.h"

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
 * Parsing Functions
 ******************************************************************************/

static void Serialize_Cal_P2PXtalk(sci_frame_t * frame, argus_cal_p2pxtalk_t const * cal)
{
    /* Electrical Pixel-To-Pixel Crosstalk */
    const argus_cal_electrical_p2pxtalk_t * ecal = &cal->Electrical;
    SCI_Frame_Queue08u(frame, ecal->Enabled);
    SCI_Frame_Queue16s(frame, ecal->KcFactorS);
    SCI_Frame_Queue16s(frame, ecal->KcFactorC);
    SCI_Frame_Queue16s(frame, ecal->KcFactorSRefPx);
    SCI_Frame_Queue16s(frame, ecal->KcFactorCRefPx);
    SCI_Frame_Queue08u(frame, ecal->RelativeThreshold);
    SCI_Frame_Queue16u(frame, ecal->AbsoluteTreshold);

    /* Optical Pixel-To-Pixel Crosstalk */
    const argus_cal_optical_p2pxtalk_t * ocal = &cal->Optical;
    SCI_Frame_Queue08u(frame, ocal->Enabled);
    SCI_Frame_Queue16s(frame, ocal->CouplingCoeffS);
    SCI_Frame_Queue16s(frame, ocal->CouplingCoeffC);
}
static void Deserialize_Cal_P2PXtalk(sci_frame_t * frame, argus_cal_p2pxtalk_t * cal)
{
    argus_cal_electrical_p2pxtalk_t * ecal = &cal->Electrical;

    /* Electrical Pixel-To-Pixel Crosstalk */
    ecal->Enabled = SCI_Frame_Dequeue08u(frame);
    ecal->KcFactorS = SCI_Frame_Dequeue16s(frame);
    ecal->KcFactorC = SCI_Frame_Dequeue16s(frame);
    ecal->KcFactorSRefPx = SCI_Frame_Dequeue16s(frame);
    ecal->KcFactorCRefPx = SCI_Frame_Dequeue16s(frame);
    ecal->RelativeThreshold = SCI_Frame_Dequeue08u(frame);
    ecal->AbsoluteTreshold = SCI_Frame_Dequeue16u(frame);

    /* Optical Pixel-To-Pixel Crosstalk */
    argus_cal_optical_p2pxtalk_t * ocal = &cal->Optical;
    ocal->Enabled = SCI_Frame_Dequeue08u(frame);
    ocal->CouplingCoeffS = SCI_Frame_Dequeue16s(frame);
    ocal->CouplingCoeffC = SCI_Frame_Dequeue16s(frame);
}

/*******************************************************************************
 * Command Functions
 ******************************************************************************/

static status_t RxCmd_CalGlobalRangeOffsets(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        q0_15_t offset_low = SCI_Frame_Dequeue16s(frame);
        q0_15_t offset_high = SCI_Frame_Dequeue16s(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationGlobalRangeOffsets(argus, offset_low, offset_high);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_GLOBAL_RANGE_OFFSET, 0, 0);
    }
}
static status_t TxCmd_CalGlobalRangeOffsets(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    q0_15_t offset_high, offset_low;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationGlobalRangeOffsets(argus, &offset_low, &offset_high);
    SCI_Frame_Queue16s(frame, offset_low);
    SCI_Frame_Queue16s(frame, offset_high);
    return status;
}

static status_t RxCmd_CalPixelRangeOffsets(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_cal_offset_table_t offsets;
        for (uint_fast8_t p = 0; p < ARGUS_DCA_POWER_STAGE_COUNT; ++p)
        {
            for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
            {
                for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
                {
                    offsets.Table[p][x][y] = SCI_Frame_Dequeue16s(frame);
                }
            }
        }
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationPixelRangeOffsets(argus, &offsets);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;

    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_PIXEL_RANGE_OFFSETS, 0, 0);
    }
}
static status_t TxCmd_CalPixelRangeOffsets(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    argus_cal_offset_table_t offsets;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationPixelRangeOffsets(argus, &offsets);
    for (uint_fast8_t p = 0; p < ARGUS_DCA_POWER_STAGE_COUNT; ++p)
    {
        for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
        {
            for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
            {
                SCI_Frame_Queue16s(frame, offsets.Table[p][x][y]);
            }
        }
    }
    return status;
}

static status_t RxCmd_CalResetPixelRangeOffsets(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return Argus_ResetCalibrationPixelRangeOffsets(argus);
}

static status_t RxCmd_CalRangeOffsetSeqSampleTime(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        uint16_t time = SCI_Frame_Dequeue16u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationRangeOffsetSequenceSampleTime(argus, time);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;

    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_RANGE_OFFSET_SAMPLE_TIME, 0, 0);
    }
}
static status_t TxCmd_CalRangeOffsetSeqSampleTime(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    uint16_t time;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationRangeOffsetSequenceSampleTime(argus, &time);
    SCI_Frame_Queue16u(frame, time);
    return status;
}

static status_t RxCmd_CalXtalkPixel2Pixel(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_cal_p2pxtalk_t cal = { 0 };
        Deserialize_Cal_P2PXtalk(frame, &cal);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationCrosstalkPixel2Pixel(argus, &cal);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_XTALK_PIXEL_2_PIXEL, 0, 0);
    }
}
static status_t TxCmd_CalXtalkPixel2Pixel(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    argus_cal_p2pxtalk_t cal = { 0 };
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationCrosstalkPixel2Pixel(argus, &cal);
    Serialize_Cal_P2PXtalk(frame, &cal);
    return status;
}

static status_t RxCmd_CalXtalkVectorTable(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_cal_xtalk_table_t xtalk;
        for (uint_fast8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; ++f)
        {
            for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
            {
                for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
                {
                    xtalk.Table[f][x][y].dS = SCI_Frame_Dequeue16s(frame);
                    xtalk.Table[f][x][y].dC = SCI_Frame_Dequeue16s(frame);
                }
            }
        }
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationCrosstalkVectorTable(argus, &xtalk);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;

    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_XTALK_VECTOR_TABLE, 0, 0);
    }
}
static status_t TxCmd_CalXtalkVectorTable(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    argus_cal_xtalk_table_t xtalk;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationCrosstalkVectorTable(argus, &xtalk);
    for (uint_fast8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; ++f)
    {
        for (uint_fast8_t x = 0; x < ARGUS_PIXELS_X; ++x)
        {
            for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
            {
                SCI_Frame_Queue16s(frame, xtalk.Table[f][x][y].dS);
                SCI_Frame_Queue16s(frame, xtalk.Table[f][x][y].dC);
            }
        }
    }
    return status;
}

static status_t RxCmd_CalXtalkResetVectorTable(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)frame;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    return Argus_ResetCalibrationCrosstalkVectorTable(argus);
}

static status_t RxCmd_CalXtalkSeqSampleTime(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        uint16_t time = SCI_Frame_Dequeue16u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationCrosstalkSequenceSampleTime(argus, time);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;

    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_XTALK_SAMPLE_TIME, 0, 0);
    }
}
static status_t TxCmd_CalXtalkSeqSampleTime(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    uint16_t time;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationCrosstalkSequenceSampleTime(argus, &time);
    SCI_Frame_Queue16u(frame, time);
    return status;
}

static status_t RxCmd_CalXtalkSeqMaxAmplitude(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        uq12_4_t ampl = SCI_Frame_Dequeue16u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(argus, ampl);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CALIBRATION_XTALK_MAX_AMPLITUDE, 0, 0);
    }
}
static status_t TxCmd_CalXtalkSeqMaxAmplitude(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)data;
    (void)param;

    status_t status = STATUS_OK;
    uq12_4_t ampl;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status = Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold(argus, &ampl);
    SCI_Frame_Queue16u(frame, ampl);
    return status;
}

/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitCal()
{
    status_t status;
    status = SCI_SetRxTxCommand(CMD_CALIBRATION_GLOBAL_RANGE_OFFSET, RxCmd_CalGlobalRangeOffsets, TxCmd_CalGlobalRangeOffsets);
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

