/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Application.
 * @details		This file implements the API of the systems communication interface.
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
#include "sci.h"
#include "sci_datalink.h"
#include "sci_cmd.h"
#include "sci_handshaking.h"

#include "utility/debug.h"
#include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAX_COMMANDS 128u	/*!< Max. number of commands. */

/*! Command functions definition. */
typedef struct
{
	/*! The binary command code. */
	sci_cmd_t cmd;

	/*! The callback function to execute the received command. */
	sci_rx_cmd_fct_t rxfct;

	/*! The callback function to execute the sending command. */
	sci_tx_cmd_fct_t txfct;

} sci_cmd_ctrl_block_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief	Initialize the serial commands module.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
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
static sci_cmd_ctrl_block_t myCCB[MAX_COMMANDS] = {{0}};


/*******************************************************************************
 * Code
 ******************************************************************************/
status_t SCI_Init(void)
{
	status_t status = STATUS_OK;

	for(uint_fast8_t i = 0; i < MAX_COMMANDS; ++i)
	{
		myCCB[i].cmd = CMD_INVALID;
		myCCB[i].rxfct = 0;
		myCCB[i].txfct = 0;
	}

	status = SCI_DataLink_Init();
	if(status < STATUS_OK)	return status;

	status = SCI_CMD_Init();
	if(status < STATUS_OK)	return status;

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
	return SCI_SetCommand(cmd, rxfct, 0);
}
status_t SCI_SetTxCommand(sci_cmd_t cmd, sci_tx_cmd_fct_t txfct)
{
	return SCI_SetCommand(cmd, 0, txfct);
}
status_t SCI_SetCommand(sci_cmd_t cmd,
						sci_rx_cmd_fct_t rxfct,
						sci_tx_cmd_fct_t txfct)
{
	if(cmd == CMD_INVALID)
		return ERROR_SCI_INVALID_CMD_CODE;

	if(!rxfct && !txfct)
		return ERROR_INVALID_ARGUMENT;

	for(uint_fast8_t i = 0; i < MAX_COMMANDS; ++i)
	{
		if(myCCB[i].cmd == CMD_INVALID ||
		   myCCB[i].cmd == cmd)
		{
			myCCB[i].cmd = cmd;
			if(rxfct != 0) myCCB[i].rxfct = rxfct;
			if(txfct != 0) myCCB[i].txfct = txfct;
			return STATUS_OK;
		}
	}
	return ERROR_SCI_BUFFER_FULL;
}
status_t SCI_UnsetCommand(sci_cmd_t cmd)
{
	if(cmd == CMD_INVALID)
		return ERROR_SCI_INVALID_CMD_CODE;

	for(uint_fast8_t i = 0; i < MAX_COMMANDS; ++i)
	{
		if(myCCB[i].cmd == cmd)
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
	sci_cmd_t cmd = (sci_cmd_t)SCI_Frame_Dequeue08u(frame);

	/* If CRC fails */
	if (status < STATUS_OK)
	{
		SCI_DataLink_ReleaseFrames(frame);
		SCI_SendNotAcknowledge(cmd, status);
		return status;
	}

	/* If command is unknown. */
	if (cmd == CMD_INVALID)
	{
		SCI_DataLink_ReleaseFrames(frame);
		SCI_SendNotAcknowledge(cmd, ERROR_SCI_INVALID_CMD_CODE);
		return ERROR_SCI_INVALID_CMD_CODE;
	}

    /* Find the command function. */
	sci_rx_cmd_fct_t fct = 0;
	for(uint_fast8_t i = 0; i < MAX_COMMANDS; ++i)
	{
		if(myCCB[i].cmd == cmd)
		{
			fct = myCCB[i].rxfct;
			break;
		}
	}

	/* If command function was not found. */
	if(!fct)
	{
		SCI_DataLink_ReleaseFrames(frame);
		SCI_SendNotAcknowledge(cmd, ERROR_SCI_UNKNOWN_COMMAND);
		error_log("received unknown command 0x%02x!", cmd);
		return ERROR_SCI_UNKNOWN_COMMAND;
	}

	/* Invoke the rx command function. */
	status = fct(frame);
    if(status < STATUS_OK)
    {
    	SCI_DataLink_ReleaseFrames(frame);
    	SCI_SendNotAcknowledge(cmd, status);
    	error_log("received command 0x%02x, decoding/execution failed!\ncommand error code: %d", cmd, status);
    	return status;
    }

    /* Check if all data has been consumed. (only CRC must be left) */
    if(SCI_Frame_BytesToRead(frame) < 1)
    {
//    	SCI_DataLink_ReleaseFrames(frame);
//    	SCI_SendNotAcknowledge(cmd, ERROR_SCI_FRAME_TOO_LONG);
    	error_log("received command 0x%02x, frame too short!", cmd);
//    	return status;
    }

    if(SCI_Frame_BytesToRead(frame) > 1)
    {
//    	SCI_DataLink_ReleaseFrames(frame);
//    	SCI_SendNotAcknowledge(cmd, ERROR_SCI_FRAME_TOO_LONG);
    	error_log("received command 0x%02x, frame too long!", cmd);
//    	return status;
    }


    /* Send acknowledge. */
    status = SCI_SendAcknowledge(cmd);
    if(status < STATUS_OK)
    {
    	SCI_DataLink_ReleaseFrames(frame);
    	SCI_SendNotAcknowledge(cmd, ERROR_FAIL);
    	error_log("received command 0x%02x, acknowledge failed!\nACK error code: %d", cmd, status);
    	return status;
    }

    SCI_DataLink_ReleaseFrames(frame);
    return status;
}

status_t SCI_SendCommand(sci_cmd_t cmd, sci_param_t param, sci_data_t data)
{
    /* Find the command function. */
	sci_tx_cmd_fct_t fct = 0;
	for(uint_fast8_t i = 0; i < MAX_COMMANDS; ++i)
	{
		if(myCCB[i].cmd == cmd)
		{
			fct = myCCB[i].txfct;
			break;
		}
	}

	/* If command function was not found. */
	if(!fct)
	{
		error_log("unknown command 0x%02x!", cmd);
		return ERROR_SCI_UNKNOWN_COMMAND;
	}

	sci_frame_t * frame = SCI_DataLink_RequestTxFrame(true);
	if(!frame) return ERROR_SCI_BUFFER_FULL;

	SCI_Frame_Queue08u(frame, cmd);

	/* Invoke the tx command function. */
	status_t status = fct(frame, param, data);
    if(status < STATUS_OK)
    {
		SCI_DataLink_ReleaseFrames(frame);
    	error_log("command 0x%02x, encoding failed!\ncommand error code: %d", cmd, status);
    	return status;
    }

	return SCI_DataLink_SendTxFrame(frame);
}
