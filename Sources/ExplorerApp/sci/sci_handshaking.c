/*************************************************************************//**
 * @file
 * @brief       SCI Handshaking Commands
 * @details     This file provides an interface for the data link layer of the
 *              systems communication interface.
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
#include "sci_handshaking.h"
#include "sci_datalink.h"
#include "sci_cmd.h"

#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t SCI_SendAcknowledge(sci_device_t deviceID, sci_cmd_t cmd)
{
    sci_frame_t * frame = SCI_DataLink_RequestTxFrame(true);
    if (!frame) return ERROR_SCI_BUFFER_FULL;

    if (SCI_CMD_IS_EXTENDED_CMD(cmd))
    {
        SCI_Frame_Queue08u(frame, CMD_ACKNOWLEDGE | 0x80);
        SCI_Frame_Queue08u(frame, deviceID);
        SCI_Frame_Queue08u(frame, cmd | 0x80);
    }
    else
    {
        SCI_Frame_Queue08u(frame, CMD_ACKNOWLEDGE);
        SCI_Frame_Queue08u(frame, cmd);
    }
    return SCI_DataLink_SendTxFrame(frame, true);
}

status_t SCI_SendNotAcknowledge(sci_device_t deviceID, sci_cmd_t cmd, status_t reason)
{
    sci_frame_t * frame = SCI_DataLink_RequestTxFrame(true);
    if (!frame) return ERROR_SCI_BUFFER_FULL;

    if (SCI_CMD_IS_EXTENDED_CMD(cmd))
    {
        SCI_Frame_Queue08u(frame, CMD_NOT_ACKNOWLEDGE | 0x80);
        SCI_Frame_Queue08u(frame, deviceID);
        SCI_Frame_Queue08u(frame, cmd | 0x80);
    }
    else
    {
        SCI_Frame_Queue08u(frame, CMD_NOT_ACKNOWLEDGE);
        SCI_Frame_Queue08u(frame, cmd);
    }
    SCI_Frame_Queue16s(frame, (int16_t)reason);
    return SCI_DataLink_SendTxFrame(frame, true);
}
