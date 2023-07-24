/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Application.
 * @details     This file implements the API of the systems communication interface.
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
#include "sci.h"
#include "sci_datalink.h"
#include "sci_cmd.h"
#include "sci_handshaking.h"

#include "debug.h"
#include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Max. number of commands. */
#ifndef SCI_MAX_COMMANDS
#define SCI_MAX_COMMANDS 128u
#endif

/*! Command functions definition. */
typedef struct sci_cmd_ctrl_block_t
{
    /*! The binary command code. */
    sci_cmd_t cmd;

    /*! The callback function to execute the sending command. */
    sci_tx_cmd_fct_t txfct;

    /*! The callback function to execute the received command. */
    sci_rx_cmd_fct_t rxfct;

    /*! The callback function to execute the received command
     *  after the message has been handled, i.e. after ACK
     *  has been sent. Note that the ACK is sent in blocking
     *  manner such that the function is called with TX line idle. */
    sci_rx_cmd_fct_t pfct;

} sci_cmd_ctrl_block_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Initialize the serial commands module.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
extern status_t SCI_CMD_Init(void);


/******************************************************************************
 * Variables
 ******************************************************************************/

/*! Callback function pointer for received frame event. */
extern sci_rx_cmd_cb_t SCI_RxCallback;

/*! Callback function pointer for error event. */
extern sci_error_cb_t SCI_ErrorCallback;

/*! The command control block buffer. */
static sci_cmd_ctrl_block_t myCCB[SCI_MAX_COMMANDS] = {{0}};

///*! Indicator whether the last transmission was in new protocol mode. */
//static bool SCI_isNewProtocolMode = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t SCI_Init(void)
{
    status_t status = STATUS_OK;

    for(uint_fast8_t i = 0; i < SCI_MAX_COMMANDS; ++i)
    {
        myCCB[i].cmd = CMD_INVALID;
        myCCB[i].rxfct = 0;
        myCCB[i].txfct = 0;
    }

    status = SCI_DataLink_Init();
    if(status < STATUS_OK)  return status;

    status = SCI_CMD_Init();
    if(status < STATUS_OK)  return status;

    return status;
}

void SCI_SetRxCommandCallback(sci_rx_cmd_cb_t cb)
{
    SCI_RxCallback = cb;
}
void SCI_RemoveRxCommandCallback(void)
{
    SCI_RxCallback = 0;
}
void SCI_SetErrorCallback(sci_error_cb_t cb)
{
    SCI_ErrorCallback = cb;
}
void SCI_RemoveErrorCallback(void)
{
    SCI_ErrorCallback = 0;
}

status_t SCI_SetRxCommand(sci_cmd_t cmd, sci_rx_cmd_fct_t rxfct)
{
    return SCI_SetCommand(cmd, rxfct, 0, 0);
}
status_t SCI_SetPostRxCommand(sci_cmd_t cmd, sci_rx_cmd_fct_t rxfct, sci_rx_cmd_fct_t pfct)
{
    return SCI_SetCommand(cmd, rxfct, 0, pfct);
}
status_t SCI_SetTxCommand(sci_cmd_t cmd, sci_tx_cmd_fct_t txfct)
{
    return SCI_SetCommand(cmd, 0, txfct, 0);
}
status_t SCI_SetRxTxCommand(sci_cmd_t cmd,
                            sci_rx_cmd_fct_t rxfct,
                            sci_tx_cmd_fct_t txfct)
{
    return SCI_SetCommand(cmd, rxfct, txfct, 0);
}
status_t SCI_SetCommand(sci_cmd_t cmd,
                        sci_rx_cmd_fct_t rxfct,
                        sci_tx_cmd_fct_t txfct,
                        sci_rx_cmd_fct_t pfct)
{
    if ((cmd == CMD_INVALID) || SCI_CMD_IS_EXTENDED_CMD(cmd))
        return ERROR_SCI_INVALID_CMD_CODE;

    if (!rxfct && !txfct && !pfct)
        return ERROR_INVALID_ARGUMENT;

    for (uint_fast8_t i = 0; i < SCI_MAX_COMMANDS; ++i)
    {
        if (myCCB[i].cmd == CMD_INVALID ||
            myCCB[i].cmd == cmd)
        {
            myCCB[i].cmd = cmd;
            if (rxfct != 0) myCCB[i].rxfct = rxfct;
            if (txfct != 0) myCCB[i].txfct = txfct;
            if (pfct != 0) myCCB[i].pfct = pfct;
            return STATUS_OK;
        }
    }
    return ERROR_SCI_BUFFER_FULL;
}
status_t SCI_UnsetCommand(sci_cmd_t cmd)
{
    if ((cmd == CMD_INVALID) || SCI_CMD_IS_EXTENDED_CMD(cmd))
        return ERROR_SCI_INVALID_CMD_CODE;

    for (uint_fast8_t i = 0; i < SCI_MAX_COMMANDS; ++i)
    {
        if (myCCB[i].cmd == cmd)
        {
            myCCB[i].cmd = CMD_INVALID;
            myCCB[i].rxfct = 0;
            myCCB[i].txfct = 0;
            return STATUS_OK;
        }
    }
    return ERROR_SCI_UNKNOWN_COMMAND;
}
status_t SCI_InvokeRxCommand(sci_frame_t * frame)
{
    assert(frame != 0);

    /* Check CRC checksum. */
    status_t status = SCI_DataLink_CheckRxFrame(frame);

    /* Get command code. */
    sci_cmd_t cmd = (sci_cmd_t) SCI_Frame_Dequeue08u(frame);

    /* Check whether it's an extended or basic command:
     * extended = with device ID, for multi-device addressing
     * basic = simple, just consider the default device */
    sci_device_t deviceID = DEVICEID_DEFAULT;
    if (SCI_CMD_IS_EXTENDED_CMD(cmd))
    {
        /* New Protocol Mode:
         * Determine the device that must process the command.
         * Assume the ID is valid - only app has the device count and can check if necessary */
        deviceID = SCI_Frame_Dequeue08u(frame);

        /* If the device ID is 0 (default), use the first available device. This is required
         * to ensure proper DeviceID in all command/response packets from this point onwards */
        if (deviceID == DEVICEID_DEFAULT)
            deviceID = DEVICEID_FIRST_VALID;
    }

    /* There is no validity check for the DeviceID because the SCI layer doesn't
     * have the required information on the valid Devices connected. Thus it will
     * simply forward the received DeviceID to the upper layers. */

    /* If CRC fails */
    if (status < STATUS_OK)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, status);
        return status;
    }

    /* If command is unknown. */
    if ((cmd & 0x7FU) == CMD_INVALID)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, ERROR_SCI_INVALID_CMD_CODE);
        return ERROR_SCI_INVALID_CMD_CODE;
    }

    /* Find the command function. */
    sci_rx_cmd_fct_t rxfct = 0;
    sci_rx_cmd_fct_t pfct = 0;
    for (uint_fast8_t i = 0; i < SCI_MAX_COMMANDS; ++i)
    {
        if (myCCB[i].cmd == (cmd & 0x7FU))
        {
            rxfct = myCCB[i].rxfct;
            pfct = myCCB[i].pfct;
            break;
        }
    }

    /* If command function was not found. */
    if (!rxfct)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, ERROR_SCI_UNKNOWN_COMMAND);
        return ERROR_SCI_UNKNOWN_COMMAND;
    }

    /* Invoke the RX command function. */
    status = rxfct(deviceID, frame);
    if (status < STATUS_OK)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, status);
        return status;
    }

    /* Check if all data has been consumed. (only CRC must be left) */
    uint32_t bytesToRead = SCI_Frame_BytesToRead(frame);
    if (bytesToRead < 1)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, ERROR_SCI_FRAME_TOO_SHORT);
        return ERROR_SCI_FRAME_TOO_SHORT;
    }

    else if (bytesToRead > 1)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, ERROR_SCI_FRAME_TOO_LONG);
        return ERROR_SCI_FRAME_TOO_LONG;
    }

    /* Send acknowledge. */
    status = SCI_SendAcknowledge(deviceID, cmd);
    if (status < STATUS_OK)
    {
        SCI_DataLink_ReleaseFrames(frame);
        SCI_SendNotAcknowledge(deviceID, cmd, ERROR_FAIL);
        return status;
    }

    /* Run post function if available. */
    if (pfct)
    {
        SCI_DataLink_ResetRxFrames(frame);
        SCI_Frame_Dequeue08u(frame); // skip command
        if (SCI_CMD_IS_EXTENDED_CMD(cmd))
            SCI_Frame_Dequeue08u(frame); // skip address
        while (SCI_DataLink_IsTxBusy()) __asm("nop");
        status = pfct(deviceID, frame);
    }

    /* Release frames. */
    SCI_DataLink_ReleaseFrames(frame);

    return status;
}

status_t SCI_SendCommand(sci_device_t deviceID, sci_cmd_t cmd, sci_param_t param, sci_data_t data)
{
    if ((cmd == CMD_INVALID) || SCI_CMD_IS_EXTENDED_CMD(cmd))
        return ERROR_SCI_INVALID_CMD_CODE;

    /* Find the command function. */
    sci_tx_cmd_fct_t fct = 0;
    for (uint_fast8_t i = 0; i < SCI_MAX_COMMANDS; ++i)
    {
        if (myCCB[i].cmd == cmd)
        {
            fct = myCCB[i].txfct;
            break;
        }
    }

    /* If command function was not found. */
    if (!fct)
    {
        return ERROR_SCI_UNKNOWN_COMMAND;
    }

    sci_frame_t * frame = SCI_DataLink_RequestTxFrame(true);
    if (!frame) return ERROR_SCI_BUFFER_FULL;

    if (deviceID > 0)
    {
        SCI_Frame_Queue08u(frame, cmd | 0x80);
        SCI_Frame_Queue08u(frame, deviceID);
    }
    else
    {
        SCI_Frame_Queue08u(frame, cmd);
    }

    /* Invoke the TX command function. */
    status_t status = fct(deviceID, frame, param, data);
    if (status < STATUS_OK)
    {
        SCI_DataLink_ReleaseFrames(frame);
        return status;
    }

    return SCI_DataLink_SendTxFrame(frame, false);
}
