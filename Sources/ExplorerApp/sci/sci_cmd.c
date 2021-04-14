/*************************************************************************//**
 * @file
 * @brief    	SCI Layer 3: Command (Message) Module
 * @details		This file provides command definitions for the systems
 * 				communication interface.
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
#include "sci_cmd.h"
#include "sci.h"
#include "sci_datalink.h"

#include "utility/time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief	Returns the current system status.
 * @return	Returns the current system status.
 *****************************************************************************/
extern status_t GetSystemStatus(void);


/*!***************************************************************************
 * @brief	Initialize the serial commands module.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_CMD_Init(void);

/*!***************************************************************************
 * @brief	Receiving Test Message Command
 * @param	frame Pointer to data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_TestMessage(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Sending Test Message Command
 * @param	frame Pointer to data frame.
 * @param	param No used!
 * @param	data Pointer to the message to be sent.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t TxCmd_TestMessage(sci_frame_t * frame, sci_param_t param, sci_frame_t * msg);

/*!***************************************************************************
 * @brief	Receiving Status Report Request Command
 * @param	frame Pointer to data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_StatusReport(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Receiving Test Message Command
 * @param	frame Pointer to data frame.
 * @param	param No used!
 * @param	status Pointer to status to be sent.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t TxCmd_StatusReport(sci_frame_t * frame, sci_param_t param, status_t const * status);

/*!***************************************************************************
 * @brief	Receiving MCU Reset Command
 * @param	frame Pointer to data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_SystemReset(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Enters the bootloader mode.
 * @param	frame Pointer to data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t RxCmd_EnterBootloader(sci_frame_t * frame);


/*******************************************************************************
 * Code
 ******************************************************************************/
status_t SCI_CMD_Init(void)
{
	status_t status = STATUS_OK;

	status = SCI_SetCommand(CMD_TEST_MESSAGE, RxCmd_TestMessage, (sci_tx_cmd_fct_t)TxCmd_TestMessage);
	if(status < STATUS_OK) return status;

	status = SCI_SetRxCommand(CMD_SYSTEM_RESET, RxCmd_SystemReset);
	if(status < STATUS_OK) return status;

	status = SCI_SetRxCommand(CMD_BOOTLOADER, RxCmd_EnterBootloader);
	if(status < STATUS_OK) return status;

	status = SCI_SetCommand(CMD_STATUS_REPORT, RxCmd_StatusReport, (sci_tx_cmd_fct_t)TxCmd_StatusReport);
	if(status < STATUS_OK) return status;

	return status;
}

/*******************************************************************************
 * Status Report Command
 ******************************************************************************/
status_t SCI_SendStatusReport(status_t status)
{
	return SCI_SendCommand(CMD_STATUS_REPORT, 0, &status);
}
static status_t RxCmd_StatusReport(sci_frame_t * frame)
{
	(void) frame; // unused;
	return SCI_SendStatusReport(GetSystemStatus());
}
static status_t TxCmd_StatusReport(sci_frame_t * frame, sci_param_t param, status_t const * status)
{
	(void)param;
	if(!status) return ERROR_INVALID_ARGUMENT;
	ltc_t t_now;
	Time_GetNow(&t_now);
	SCI_Frame_Queue_Time(frame, &t_now);
	SCI_Frame_Queue32s(frame, (int32_t)(*status));
	return STATUS_OK;
}
__attribute__((weak)) status_t GetSystemStatus(void)
{
	return ERROR_NOT_IMPLEMENTED;
}

/*******************************************************************************
 * Test Message Command
 ******************************************************************************/
static status_t RxCmd_TestMessage(sci_frame_t * frame)
{
	return SCI_SendCommand(CMD_TEST_MESSAGE, 0, frame);
}
static status_t TxCmd_TestMessage(sci_frame_t * frame, sci_param_t param, sci_frame_t * msg)
{
	(void)param;
	while(SCI_Frame_BytesToRead(msg) > 1)
		SCI_Frame_Queue08u(frame, SCI_Frame_Dequeue08u(msg));

	return STATUS_OK;
}

/*******************************************************************************
 * Reset System Command
 ******************************************************************************/
#include "driver/cop.h" // system reset command
#include "utility/time.h"
static status_t RxCmd_SystemReset(sci_frame_t * frame)
{
	if(0xDEADC0DE != SCI_Frame_Dequeue32u(frame))
	{
		return ERROR_SCI_INVALID_CMD_PARAMETER;
	}

	ltc_t start = {0};
	Time_GetNow(&start);

	// send the acknowledge before resetting
	frame = SCI_DataLink_RequestTxFrame(1);
	if(frame)
	{
		SCI_Frame_Queue08u(frame, CMD_ACKNOWLEDGE);
		SCI_Frame_Queue08u(frame, CMD_SYSTEM_RESET);
		SCI_DataLink_SendTxFrame(frame);
		while(SCI_Frame_BytesToRead(frame)
				&& !Time_CheckTimeoutMSec(&start, 1000))
		{
			Time_DelayMSec(1);
		}
		Time_DelayMSec(10);
	}
	COP_ResetSystem();

	return STATUS_OK;
}


/*******************************************************************************
 * Enter the bootloader mode
 ******************************************************************************/
#if defined(CPU_MKL17Z256VFM4)
#include "driver/MKL17Z/bootloader.h"
#endif
static status_t RxCmd_EnterBootloader(sci_frame_t * frame)
{
	if(0x6B636667 != SCI_Frame_Dequeue32u(frame))
	{
		return ERROR_SCI_INVALID_CMD_PARAMETER;
	}

#if defined(CPU_MKL17Z256VFM4)
	ltc_t start = {0};
	Time_GetNow(&start);

	// send the acknowledge before resetting
	frame = SCI_DataLink_RequestTxFrame(1);
	if(frame)
	{
		SCI_Frame_Queue08u(frame, CMD_ACKNOWLEDGE);
		SCI_Frame_Queue08u(frame, CMD_BOOTLOADER);
		SCI_DataLink_SendTxFrame(frame);
		while(SCI_Frame_BytesToRead(frame)
				&& !Time_CheckTimeoutMSec(&start, 1000))
		{
			Time_DelayMSec(1);
		}
		Time_DelayMSec(10);
	}

	return EnterBootloader();
#else
	return ERROR_NOT_SUPPORTED;
#endif
}
