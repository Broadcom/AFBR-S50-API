/*************************************************************************//**
 * @file
 * @brief    	SCI: The main interface.
 * @details		This file provides an interface for the systems communication
 * 				interface.
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

#ifndef SCI_H
#define SCI_H

/*!***************************************************************************
 * @defgroup	sci SCI: Systems Communication Interface
 * @brief		Systems Communication Interface
 * @details		The systems communication interface module provides a high
 * 				level protocol for information exchange between several systems.
 * 				Basically it is a two point interface containing a single master
 * 				and a single slave device.
 *
 * 				The protocol is a frame based hardware interface. Each data
 * 				frame consists of a dedicate start and stop byte that determines
 * 				the bounds of the frame. After the start byte, the first byte is
 * 				considered to be the command identifier that specifies the
 * 				following data. After that, an arbitrary number of data bytes
 * 				can follow. In order to provide the full range per byte, the
 * 				start and stop bytes are escaped and inverted by escape bytes.
 * 				Thus the data bytes can have any value. Finally, the frame is
 * 				finished with an 8-bit CRC value before the stop byte is send.
 *
 * 				Apart from the CRC value, that assures the correct data integrity,
 * 				there are acknowledge messages sent from the device in order
 * 				to verify the correct receiving and execution of the commands
 * 				within the slave system. I.e. every received command will cause
 * 				either an acknowledge or an not-acknowledge with a corresponding
 * 				status byte that informs the master about the error cause.
 *				On the other hand, the transmitted messages do not have to be
 *				acknowledged by the master.
 *
 * @addtogroup 	sci
 * @{
 *****************************************************************************/

#include "sci_status.h"
#include "sci_frame.h"

/*!***************************************************************************
 *  The SCI command type identifier byte.
 *****************************************************************************/
typedef uint8_t sci_cmd_t;

/*!***************************************************************************
 *  The SCI parameter type. An abstract (integer) parameter to be used in the
 *  function.
 *****************************************************************************/
typedef uint32_t sci_param_t;

/*!***************************************************************************
 *  The SCI data pointer type. An abstract void pointer to any data object.
 *****************************************************************************/
typedef void const * sci_data_t;

/*!***************************************************************************
 * @brief 	Received command invocation function definition.
 * @details This function pointer represents a command that is invoked whenever
 * 			the corresponding data frame has been received via the SCI module.
 * @param	frame Pointer to the received data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*sci_rx_cmd_fct_t)(sci_frame_t * frame);

/*!***************************************************************************
 * @brief 	Transmitting command function definition.
 * @details This function pointer represents a command that is invoked whenever
 * 			the corresponding data frame will be sent via the SCI module. It
 * 			contains a pointer to the data that must serialized into the data
 * 			frame structure.
 * @param	frame Pointer to the data frame that will be transmitted.
 * @param	param An optional abstract parameter to be used in the command
 *                  function.
 * @param	data An optional abstract pointer to the data to be serialize.
 *                 The pointer van be null.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*sci_tx_cmd_fct_t)(sci_frame_t * frame, sci_param_t param,
		sci_data_t data);

/*!***************************************************************************
 * @brief 	Callback function type for received SCI data frames.
 * @param	frame Pointer to the received data frame.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*sci_rx_cmd_cb_t)(sci_frame_t * frame);

/*!***************************************************************************
 * @brief 	Callback function type for SCI error.
 * @param	status The corresponding error code.
 *****************************************************************************/
typedef void (*sci_error_cb_t)(status_t satus);

/*!***************************************************************************
 * @brief	Initialize the SCI module.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_Init(void);

/*!***************************************************************************
 * @brief	Installs a callback routine for command received event.
 * @details	Installs a callback function that will be called after a command
 * 			has been received successfully. This call is determined to inform
 * 			about an new Rx data frame has been received and the corresponding
 * 			command shall be invoked. Thus, after the callback, invoke the
 * 			#SCI_InvokeRxCommand() function.
 * 			If no callback is set, the commands are invoked directly from the
 * 			interrupt services routine. Therefore, it is highly recommended to
 * 			install a callback and invoke the commands from the main thread/task.
 * @warning	The callback function is called	from the interrupt service routine
 * 			and should return within an appropriate time!
 * @param	cb The callback functions to be called
 *****************************************************************************/
void SCI_SetRxCommandCallback(sci_rx_cmd_cb_t cb);

/*!***************************************************************************
 * @brief	Removes the previously installed callback function.
 *****************************************************************************/
void SCI_RemoveRxCommandCallback(void);

/*!***************************************************************************
 * @brief	Installs a callback routine for the error event.
 * @details	Installs a callback function that will be called after an error
 * 			has occurred.
 *			TODO: Describe how to handle errors...
 *
 * @warning	The callback function is called	from the interrupt service routine
 * 			and should return within an appropriate time!
 * @param	cb The callback functions to be called
 *****************************************************************************/
void SCI_SetErrorCallback(sci_error_cb_t cb);

/*!***************************************************************************
 * @brief	Removes the previously installed callback function.
 *****************************************************************************/
void SCI_RemoveErrorCallback(void);

/*!***************************************************************************
 * @brief 	Sends a command via the SCI module.
 * @details	The corresponding command function will be called in order to
 * 			serialize the data from into a data frame. The frame is sent
 * 			afterwards.
 * @param	cmd  The command code / keyword.
 * @param	param An optional abstract parameter to be used in the command
 *                  function.
 * @param	data An optional abstract pointer to the data to be serialize.
 *                 The pointer van be null.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SendCommand(sci_cmd_t cmd, sci_param_t param, sci_data_t data);

/*!***************************************************************************
 * @brief	Sets a Rx command function in the list of available commands.
 * @details	Registers the command code to the SCI module. If already set, the
 * 			corresponding Rx function is replaced by the specified one. The Tx
 * 			function will not be changed.
 * @param	cmd	The command code / keyword.
 * @param	fct The function to be called when a command is received.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SetRxCommand(sci_cmd_t cmd, sci_rx_cmd_fct_t fct);

/*!***************************************************************************
 * @brief	Sets a Tx command function in the list of available commands.
 * @details	Registers the command code to the SCIs module. If already set, the
 * 			corresponding Tx function is replaced by the specified one. The Rx
 * 			function will not be changed.
 * @param	cmd	The command code / keyword.
 * @param	fct The function to be called when a command is sent.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SetTxCommand(sci_cmd_t cmd, sci_tx_cmd_fct_t fct);

/*!***************************************************************************
 * @brief	Sets the Rx and Tx command functions in the list of available commands.
 * @details	Registers the command code to the SCI module. If already set, the
 * 			corresponding Tx and Rx functions are replaced by the specified ones.
 * @param	cmd	  The command code / keyword.
 * @param	rxfct The function to be called when a command is received.
 * @param	txfct The function to be called when a command is sent.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SetCommand(sci_cmd_t cmd, sci_rx_cmd_fct_t rxfct, sci_tx_cmd_fct_t txfct);

/*!***************************************************************************
 * @brief	Unsets a command from the list of available commands.
 * @details	Removes the command code from the SCI module, i.e. it deletes the
 * 			corresponding Tx and Rx functions. If not set, the function returns
 * 			with #ERROR_SCI_UNKNOWN_COMMAND.
 * @param	cmd	The command code / keyword.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_UnsetCommand(sci_cmd_t cmd);

/*!***************************************************************************
 * @brief	Invokes the previously received user command.
 * @details	Parses and executes the previously received command. To be called
 * 			after Command Received Handler has been invoked.
 * @param	frame The previously received SCI frame (parameter of
 * 					"RxFrameCallback") with the command code and parameter
 * 					 buffer.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_InvokeRxCommand(sci_frame_t * frame);

/*! @} */
#endif // SCI_H
