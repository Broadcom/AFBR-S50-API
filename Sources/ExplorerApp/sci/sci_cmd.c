/*************************************************************************//**
 * @file
 * @brief       SCI Layer 3: Command (Message) Module
 * @details     This file provides command definitions for the systems
 *              communication interface.
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
#include "sci_cmd.h"
#include "sci.h"
#include "sci_datalink.h"

#include "board/board.h"
#include "utility/time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t GetSystemStatus(sci_device_t deviceID);

/*!***************************************************************************
 * @brief   Initialize the serial commands module.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_CMD_Init(void);

/*!***************************************************************************
 * @brief   Receiving Ping Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_Ping(sci_device_t deviceID, sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Receiving Test Message Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_TestMessage(sci_device_t deviceID, sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Sending Test Message Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @param   param No used!
 * @param   msg Pointer to the message to be sent.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t TxCmd_TestMessage(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_frame_t * msg);

/*!***************************************************************************
 * @brief   Receiving Status Report Request Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_StatusReport(sci_device_t deviceID, sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Receiving Test Message Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @param   param No used!
 * @param   status Pointer to status to be sent.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t TxCmd_StatusReport(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, status_t const * status);

/*!***************************************************************************
 * @brief   Receiving MCU Reset Command
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   frame Pointer to data frame.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_SystemReset(sci_device_t deviceID, sci_frame_t * frame);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * Ping Message Command
 ******************************************************************************/
static status_t RxCmd_Ping(sci_device_t deviceID, sci_frame_t * frame)
{
    (void) frame;
    (void)deviceID;

    return STATUS_OK; // do nothing, just send an acknowledge
}

/*******************************************************************************
 * Status Report Command
 ******************************************************************************/
status_t SCI_SendStatusReport(sci_device_t deviceID, status_t status)
{
    return SCI_SendCommand(deviceID, CMD_STATUS_REPORT, 0, &status);
}
static status_t RxCmd_StatusReport(sci_device_t deviceID, sci_frame_t * frame)
{
    (void) frame; // unused;

    status_t status = GetSystemStatus(deviceID);
    return SCI_SendCommand(deviceID, CMD_STATUS_REPORT, 0, &status);
}
static status_t TxCmd_StatusReport(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, status_t const * status)
{
    (void)param;
    (void)deviceID;

    if(!status) return ERROR_INVALID_ARGUMENT;
    ltc_t t_now;
    Time_GetNow(&t_now);
    SCI_Frame_Queue_Time(frame, &t_now);
    SCI_Frame_Queue32s(frame, (int32_t)(*status));
    return STATUS_OK;
}

/*******************************************************************************
 * Test Message Command
 ******************************************************************************/
static status_t RxCmd_TestMessage(sci_device_t deviceID, sci_frame_t * frame)
{
    return SCI_SendCommand(deviceID, CMD_TEST_MESSAGE, 0, frame);
}
static status_t TxCmd_TestMessage(sci_device_t deviceID, sci_frame_t * frame, sci_param_t param, sci_frame_t * msg)
{
    (void)param;
    (void)deviceID;

    while (SCI_Frame_BytesToRead(msg) > 1)
        SCI_Frame_Queue08u(frame, SCI_Frame_Dequeue08u(msg));

    return STATUS_OK;
}

/*******************************************************************************
 * Reset System Command
 ******************************************************************************/
#include "utility/time.h"
static status_t RxCmd_SystemReset(sci_device_t deviceID, sci_frame_t * frame)
{
    /* No need to consider device ID, this is a system-wide reset */
    (void)deviceID;

    if(0xDEADC0DE != SCI_Frame_Dequeue32u(frame))
    {
        return ERROR_SCI_INVALID_CMD_PARAMETER;
    }
    return STATUS_OK;
}
static status_t PrxCmd_SystemReset(sci_device_t deviceID, sci_frame_t * frame)
{
    (void)deviceID;
    (void)frame;

    Time_DelayMSec(10);
    Board_Reset();
    return STATUS_OK;
}


/*******************************************************************************
 * Initialization
 ******************************************************************************/
status_t SCI_CMD_Init(void)
{
    status_t status = STATUS_OK;

    status = SCI_SetRxCommand(CMD_PING, RxCmd_Ping);
    if (status < STATUS_OK) return status;

    status = SCI_SetRxTxCommand(CMD_TEST_MESSAGE, RxCmd_TestMessage, (sci_tx_cmd_fct_t)TxCmd_TestMessage);
    if (status < STATUS_OK) return status;

    status = SCI_SetPostRxCommand(CMD_SYSTEM_RESET, RxCmd_SystemReset, PrxCmd_SystemReset);
    if (status < STATUS_OK) return status;

    status = SCI_SetRxTxCommand(CMD_STATUS_REPORT, RxCmd_StatusReport, (sci_tx_cmd_fct_t)TxCmd_StatusReport);
    if (status < STATUS_OK) return status;

    return status;
}
