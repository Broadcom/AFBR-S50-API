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
#include "explorer_api_cfg.h"
#include "core/core_device.h"
#include "core/core_utils.h"
#include "core/core_cfg.h"
#include "driver/uart.h"
#include "core/explorer_status.h"

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

static void Serialize_Cfg_DCA(sci_frame_t * frame, argus_cfg_dca_t const * dcacfg)
{
    SCI_Frame_Queue08u(frame, (uint8_t)((dcacfg->Enabled > 0 ? 1U : 0U)
                                      | (dcacfg->Enabled < 0 ? 2U : 0U)));
    SCI_Frame_Queue08u(frame, dcacfg->SatPxThLin);
    SCI_Frame_Queue08u(frame, dcacfg->SatPxThExp);
    SCI_Frame_Queue08u(frame, dcacfg->SatPxThRst);
    SCI_Frame_Queue16u(frame, dcacfg->Atarget);
    SCI_Frame_Queue16u(frame, dcacfg->AthLow);
    SCI_Frame_Queue16u(frame, dcacfg->AthHigh);
    SCI_Frame_Queue08u(frame, dcacfg->AmplitudeMode);
    SCI_Frame_Queue16u(frame, dcacfg->DepthNom);
    SCI_Frame_Queue16u(frame, dcacfg->DepthMin);
    SCI_Frame_Queue16u(frame, dcacfg->DepthMax);
    SCI_Frame_Queue08u(frame, dcacfg->Power);
    SCI_Frame_Queue08u(frame, dcacfg->GainNom);
    SCI_Frame_Queue08u(frame, dcacfg->GainMin);
    SCI_Frame_Queue08u(frame, dcacfg->GainMax);
    SCI_Frame_Queue08u(frame, dcacfg->PowerSavingRatio);
}
static void Deserialize_Cfg_DCA(sci_frame_t * frame, argus_cfg_dca_t * dcacfg)
{
    /* Dynamic Configuration Adaption. */
    uint8_t tmp = SCI_Frame_Dequeue08u(frame);
    dcacfg->Enabled = (tmp & 1U) ? 1 : (tmp & 2U) ? -1 : 0;
    dcacfg->SatPxThLin = SCI_Frame_Dequeue08u(frame);
    dcacfg->SatPxThExp = SCI_Frame_Dequeue08u(frame);
    dcacfg->SatPxThRst = SCI_Frame_Dequeue08u(frame);
    dcacfg->Atarget = SCI_Frame_Dequeue16u(frame);
    dcacfg->AthLow = SCI_Frame_Dequeue16u(frame);
    dcacfg->AthHigh = SCI_Frame_Dequeue16u(frame);
    dcacfg->AmplitudeMode = SCI_Frame_Dequeue08u(frame);
    dcacfg->DepthNom = SCI_Frame_Dequeue16u(frame);
    dcacfg->DepthMin = SCI_Frame_Dequeue16u(frame);
    dcacfg->DepthMax = SCI_Frame_Dequeue16u(frame);
    dcacfg->Power = SCI_Frame_Dequeue08u(frame);
    dcacfg->GainNom = SCI_Frame_Dequeue08u(frame);
    dcacfg->GainMin = SCI_Frame_Dequeue08u(frame);
    dcacfg->GainMax = SCI_Frame_Dequeue08u(frame);
    dcacfg->PowerSavingRatio = SCI_Frame_Dequeue08u(frame);
}

static void Serialize_Cfg_PBA(sci_frame_t * frame, argus_cfg_pba_t const * pba)
{
    assert(frame != 0);
    assert(pba != 0);

    SCI_Frame_Queue08u(frame, !!(pba->Enabled & PBA_ENABLE));
    SCI_Frame_Queue08u(frame, !!(pba->Enabled & PBA_ENABLE_GOLDPX_FALLBACK_MODE));
    SCI_Frame_Queue08u(frame, !!(pba->Enabled & PBA_ENABLE_GOLDPX_PRIORITY_MODE));
    SCI_Frame_Queue08u(frame, !!(pba->Enabled & PBA_ENABLE_MIN_DIST_SCOPE));
    SCI_Frame_Queue08u(frame, pba->AveragingMode);
    SCI_Frame_Queue32u(frame, pba->PrefilterMask);
    SCI_Frame_Queue16u(frame, pba->AbsoluteAmplitudeExclusion);
    SCI_Frame_Queue16u(frame, pba->AbsoluteAmplitudeInclusion);
    SCI_Frame_Queue08u(frame, pba->RelativeAmplitudeExclusion);
    SCI_Frame_Queue08u(frame, pba->RelativeAmplitudeInclusion);
    SCI_Frame_Queue32s(frame, pba->AbsoluteMinimumDistanceThreshold);
    SCI_Frame_Queue16u(frame, pba->AbsoluteDistanceScopeExclusion);
    SCI_Frame_Queue16u(frame, pba->AbsoluteDistanceScopeInclusion);
    SCI_Frame_Queue08u(frame, pba->RelativeDistanceScopeExclusion);
    SCI_Frame_Queue08u(frame, pba->RelativeDistanceScopeInclusion);
    SCI_Frame_Queue16u(frame, pba->GoldenPixelPriorityAmplitudeExclusion);
    SCI_Frame_Queue16u(frame, pba->GoldenPixelPriorityAmplitudeInclusion);
    SCI_Frame_Queue08u(frame, pba->GoldenPixelSaturationFilterPixelThreshold);
    SCI_Frame_Queue08u(frame, pba->GoldenPixelOutOfSyncAgeThreshold);
}
static void Deserialize_Cfg_PBA(sci_frame_t * frame, argus_cfg_pba_t * pba)
{
    assert(frame != 0);
    assert(pba != 0);

    pba->Enabled = 0;
    pba->Enabled |= SCI_Frame_Dequeue08u(frame) ? PBA_ENABLE : 0;
    pba->Enabled |= SCI_Frame_Dequeue08u(frame) ? PBA_ENABLE_GOLDPX_FALLBACK_MODE : 0;
    pba->Enabled |= SCI_Frame_Dequeue08u(frame) ? PBA_ENABLE_GOLDPX_PRIORITY_MODE : 0;
    pba->Enabled |= SCI_Frame_Dequeue08u(frame) ? PBA_ENABLE_MIN_DIST_SCOPE : 0;
    pba->AveragingMode = SCI_Frame_Dequeue08u(frame);
    pba->PrefilterMask = SCI_Frame_Dequeue32u(frame);
    pba->AbsoluteAmplitudeExclusion = SCI_Frame_Dequeue16u(frame);
    pba->AbsoluteAmplitudeInclusion = SCI_Frame_Dequeue16u(frame);
    pba->RelativeAmplitudeExclusion = SCI_Frame_Dequeue08u(frame);
    pba->RelativeAmplitudeInclusion = SCI_Frame_Dequeue08u(frame);
    pba->AbsoluteMinimumDistanceThreshold = SCI_Frame_Dequeue32s(frame);
    pba->AbsoluteDistanceScopeExclusion = SCI_Frame_Dequeue16u(frame);
    pba->AbsoluteDistanceScopeInclusion = SCI_Frame_Dequeue16u(frame);
    pba->RelativeDistanceScopeExclusion = SCI_Frame_Dequeue08u(frame);
    pba->RelativeDistanceScopeInclusion = SCI_Frame_Dequeue08u(frame);
    pba->GoldenPixelPriorityAmplitudeExclusion = SCI_Frame_Dequeue16u(frame);
    pba->GoldenPixelPriorityAmplitudeInclusion = SCI_Frame_Dequeue16u(frame);
    pba->GoldenPixelSaturationFilterPixelThreshold = SCI_Frame_Dequeue08u(frame);
    pba->GoldenPixelOutOfSyncAgeThreshold = SCI_Frame_Dequeue08u(frame);
}


/*******************************************************************************
 * Command Functions
 ******************************************************************************/

static status_t RxCmd_CfgMeasurementMode(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
        if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

        argus_mode_t mode = SCI_Frame_Dequeue08s(frame);
        return ExplorerApp_SetDeviceMeasurementMode(explorer, mode);
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_MEASUREMENT_MODE, 0, 0);
    }
}
static status_t TxCmd_CfgMeasurementMode(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;

    argus_mode_t mode;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetMeasurementMode(argus, &mode);
    SCI_Frame_Queue08u(frame, mode);

    return status;
}

static status_t RxCmd_CfgDataOutputMode(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
        if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

        explorer_cfg_t cfg;
        ExplorerApp_GetConfiguration(explorer, &cfg);
        cfg.DataOutputMode = SCI_Frame_Dequeue08u(frame);
        return ExplorerApp_SetConfiguration(explorer, &cfg);
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_DATA_OUTPUT_MODE, 0, 0);
    }
}
static status_t TxCmd_CfgDataOutputMode(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;

    explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
    if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

    explorer_cfg_t cfg;
    ExplorerApp_GetConfiguration(explorer, &cfg);
    SCI_Frame_Queue08u(frame, cfg.DataOutputMode);
    return STATUS_OK;
}

static status_t RxCmd_CfgFrameTime(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        uint32_t frameTime = SCI_Frame_Dequeue32u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationFrameTime(argus, frameTime);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_FRAME_TIME, 0, 0);
    }
}
static status_t TxCmd_CfgFrameTime(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) param;
    (void) data;

    uint32_t frameTime = 0;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetConfigurationFrameTime(argus, &frameTime);
    SCI_Frame_Queue32u(frame, frameTime);
    return status;
}

static status_t RxCmd_CfgSmartPowerSaveEnabled(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        bool enabled = SCI_Frame_Dequeue08u(frame) != 0;
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationSmartPowerSaveEnabled(argus, enabled);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_SMART_POWER_SAVE, 0, 0);
    }
}
static status_t TxCmd_CfgSmartPowerSaveEnabled(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) data;
    (void) param;

    bool enabled = 0;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetConfigurationSmartPowerSaveEnabled(argus, &enabled);
    SCI_Frame_Queue08u(frame, enabled);

    return status;
}

static status_t RxCmd_CfgDualFrequencyMode(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_dfm_mode_t dfm = SCI_Frame_Dequeue08u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationDFMMode(argus, dfm);
        ExplorerApp_DisplayUnambiguousRange(argus);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_DUAL_FREQUENCY_MODE, 0, 0);
    }
}
static status_t TxCmd_CfgDualFrequencyMode(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) data;
    (void) param;

    argus_dfm_mode_t dfm;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetConfigurationDFMMode(argus, &dfm);
    SCI_Frame_Queue08u(frame, dfm);

    return status;
}

static status_t RxCmd_CfgShotNoiseMonitor(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_snm_mode_t snm = SCI_Frame_Dequeue08u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationShotNoiseMonitorMode(argus, snm);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_SHOT_NOISE_MONITOR_MODE, 0, 0);
    }
}
static status_t TxCmd_CfgShotNoiseMonitor(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) data;
    (void) param;

    argus_snm_mode_t snm;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetConfigurationShotNoiseMonitorMode(argus, &snm);
    SCI_Frame_Queue08u(frame, snm);

    return status;
}

static status_t RxCmd_CfgXtalkMonitor(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        bool xtm = SCI_Frame_Dequeue08u(frame);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationCrosstalkMonitorMode(argus, xtm);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_XTALK_MONITOR_MODE, 0, 0);
    }
}
static status_t TxCmd_CfgXtalkMonitor(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) data;
    (void) param;

    bool xtm;
    argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
    if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
    status_t status = Argus_GetConfigurationCrosstalkMonitorMode(argus, &xtm);
    SCI_Frame_Queue08u(frame, xtm);

    return status;
}
static status_t RxCmd_CfgDca(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_cfg_dca_t dca = { 0 };
        Deserialize_Cfg_DCA(frame, &dca);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationDynamicAdaption(argus, &dca);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_DCA, 0, 0);
    }
}
static status_t TxCmd_CfgDca(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) param;
    status_t status = STATUS_OK;
    if (data == 0)
    {
        argus_cfg_dca_t dca = { 0 };
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        status = Argus_GetConfigurationDynamicAdaption(argus, &dca);
        Serialize_Cfg_DCA(frame, &dca);
    }
    else
    {
        Serialize_Cfg_DCA(frame, (argus_cfg_dca_t*) data);
    }

    return status;
}

static status_t RxCmd_CfgPba(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        argus_cfg_pba_t pba = { 0 };
        Deserialize_Cfg_PBA(frame, &pba);
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        bool resume = ExplorerApp_SuspendTimerMeasurement(argus);
        status_t status = Argus_SetConfigurationPixelBinning(argus, &pba);
        if (resume) ExplorerApp_StartTimerMeasurement(argus);
        return status;

    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_PBA, 0, 0);
    }
}
static status_t TxCmd_CfgPba(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, void const * data)
{
    (void) param;
    status_t status = STATUS_OK;
    if (data == 0)
    {
        argus_cfg_pba_t pba = { 0 };
        argus_hnd_t * argus = ExplorerApp_GetArgusPtr(deviceID);
        if (argus == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;
        status = Argus_GetConfigurationPixelBinning(argus, &pba);
        Serialize_Cfg_PBA(frame, &pba);
    }
    else
    {
        Serialize_Cfg_PBA(frame, (argus_cfg_pba_t*) data);
    }

    return status;
}

static status_t RxCmd_CfgSpi(sci_device_t deviceID, sci_frame_t * frame)
{
    if (SCI_Frame_BytesToRead(frame) > 1)
    {
        /* Master sending data... */
        explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
        if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

        explorer_cfg_t cfg;
        ExplorerApp_GetConfiguration(explorer, &cfg);
        cfg.SPIBaudRate = SCI_Frame_Dequeue32u(frame);
        return ExplorerApp_SetConfiguration(explorer, &cfg);
    }
    else
    {
        /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_SPI, 0, 0);
    }
}
static status_t TxCmd_CfgSpi(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void)param;
    (void)data;

    explorer_t * explorer = ExplorerApp_GetExplorerPtr(deviceID);
    if (explorer == NULL) return ERROR_EXPLORER_UNINITIALIZED_DEVICE_ADDRESS;

    explorer_cfg_t cfg;
    ExplorerApp_GetConfiguration(explorer, &cfg);
    SCI_Frame_Queue32u(frame, cfg.SPIBaudRate);
    return STATUS_OK;
}

static status_t RxCmd_CfgUart(sci_device_t deviceID, sci_frame_t * frame)
{
#if AFBR_SCI_USB
    (void) deviceID;
    (void) frame;
    return ERROR_NOT_SUPPORTED;
#else
    if(SCI_Frame_BytesToRead(frame) > 1)
    {   /* Master sending data... */
        const uint32_t baudRate = SCI_Frame_Dequeue32u(frame);
        return UART_CheckBaudRate(baudRate);
    }
    else
    {   /* Master is requesting data... */
        return SCI_SendCommand(deviceID, CMD_CONFIGURATION_UART, 0, 0);
    }
#endif
}
static status_t PrxCmd_CfgUart(sci_device_t deviceID, sci_frame_t *frame)
{
    (void) deviceID;

#if AFBR_SCI_USB
    (void) frame;
    return ERROR_NOT_SUPPORTED;
#else
    if (SCI_Frame_BytesToRead(frame) > 1)
    { /* Master sending data... */
        const uint32_t baudRate = SCI_Frame_Dequeue32u(frame);
        status_t status = STATUS_BUSY;
        while (status == STATUS_BUSY)
        {
            status = UART_SetBaudRate(baudRate);
        }
        assert(status == STATUS_OK);
    }
    return STATUS_OK;
#endif
}
static status_t TxCmd_CfgUart(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_data_t data)
{
    (void) deviceID;
    (void) param;
    (void) data;
#if AFBR_SCI_USB
    (void) frame;
    return ERROR_NOT_SUPPORTED;
#else
    const uint32_t baudRate = UART_GetBaudRate();
    SCI_Frame_Queue32u(frame, baudRate);
    return STATUS_OK;
#endif
}

/*******************************************************************************
 * Init Code
 ******************************************************************************/
status_t ExplorerAPI_InitCfg(void)
{
    status_t status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_DATA_OUTPUT_MODE, RxCmd_CfgDataOutputMode, TxCmd_CfgDataOutputMode);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_MEASUREMENT_MODE, RxCmd_CfgMeasurementMode, TxCmd_CfgMeasurementMode);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_FRAME_TIME, RxCmd_CfgFrameTime, TxCmd_CfgFrameTime);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_DUAL_FREQUENCY_MODE, RxCmd_CfgDualFrequencyMode, TxCmd_CfgDualFrequencyMode);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_SHOT_NOISE_MONITOR_MODE, RxCmd_CfgShotNoiseMonitor, TxCmd_CfgShotNoiseMonitor);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_XTALK_MONITOR_MODE, RxCmd_CfgXtalkMonitor, TxCmd_CfgXtalkMonitor);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_SMART_POWER_SAVE, RxCmd_CfgSmartPowerSaveEnabled, TxCmd_CfgSmartPowerSaveEnabled);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_DCA, RxCmd_CfgDca, TxCmd_CfgDca);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_PBA, RxCmd_CfgPba, TxCmd_CfgPba);
    if (status < STATUS_OK) return status;
    status = SCI_SetRxTxCommand(CMD_CONFIGURATION_SPI, RxCmd_CfgSpi, TxCmd_CfgSpi);
    if (status < STATUS_OK) return status;
    status = SCI_SetCommand(CMD_CONFIGURATION_UART, RxCmd_CfgUart, TxCmd_CfgUart, PrxCmd_CfgUart);
    if(status < STATUS_OK) return status;


    return status;
}


