/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Demo Application.
 * @details		This file contains the main functionality of the AFBR-S50
 * 				Explorer Application.
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

#ifndef EXPLORER_APP_H
#define EXPLORER_APP_H

/*!***************************************************************************
 * @defgroup	explorermain AFBR-S50 Explorer Application
 * @ingroup		explorerapp
 * @brief		AFBR-S50 Explorer Application
 * @details		An example application that utilizes the Argus API.
 *
 * 				A simple task scheduler is used to host the Argus API and a
 * 				simple systems communication interface is implemented to
 * 				connect to the AFBR-S50 Explorer GUI. The latter is an evaluation
 * 				software for the Argus time-of-flight devices.
 *
 * @addtogroup 	explorermain
 * @{
 *****************************************************************************/

#include "explorer_status.h"
#include "api/explorer_api.h"



/*! Explorer Application Task numbers (and priority!). */
typedef enum
{
	TASK_ERROR		= 7U,	/*!< ID and Priority of error handling task. */
	TASK_HNDL_CMD	= 6U,	/*!< ID and Priority of command handling task. */
	TASK_SEND_DAT 	= 2U,	/*!< ID and Priority of send results task. */
	TASK_EVAL_DAT 	= 1U,	/*!< ID and Priority of evaluate data task. */
	TASK_IDLE		= 0U	/*!< ID and Priority of the idle task. */
} explorer_task_t;


/*! Measurement Results Output Streaming Mode. */
typedef enum
{
	STREAM_FULL_DATA = 0x00,	/*!< Stream full data set. */
	STREAM_RAW_DATA = 0x01,		/*!< Stream raw data (i.e. samples) only. */
	STREAM_EVAL_DATA = 0x02,	/*!< Stream evaluated data only. */
	STREAM_BINNDED_DATA = 0x03	/*!< Stream pixel binning results data only. */
} data_streaming_mode_t;


/*! AFBR-S50 Explorer Software configuration data. */
typedef struct
{
	/*! Determines the SPI slave. */
	int8_t SPISlave;

	/*! Determines the SPI baud rate in bps. */
	uint32_t SPIBaudRate;

	/*! Determines the software is in debug mode. */
	volatile uint8_t DebugMode;

	/*! Determines the current data streaming mode. */
	volatile data_output_mode_t DataOutputMode;

} explorer_cfg_t;


 /*!***************************************************************************
 * @brief	Initializes all the state machine.
 * @details	This function does initialization of the state machine.
 * 			- Initialization of the task scheduler.
 * 			- Adding all task to the scheduler.
 * 			- Set internal data to a known state.
 * 			- Hardware driver initialization.
 * 			- Send software information via serial interface.
 * 			.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_Init(void);

/*!***************************************************************************
 * @brief	This runs the state machine.
 * @details	It runs the task scheduler and never return.
 * @return	This function never returns.
 *****************************************************************************/
void ExplorerApp_Run(void);

/*!***************************************************************************
 * @brief	Enqueues a specified task to the task scheduler queue.
 * @param	task The specified task to enqueue.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_QueueTask(explorer_task_t task);

/*!***************************************************************************
 * @brief	Getter for the software configuration.
 * @param	cfg A pointer to the \link #explorer_cfg_t software
 * 				  configuration\endlink data structure that will be filled with
 * 				  the current configuration data.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_GetConfiguration(explorer_cfg_t * cfg);

/*!***************************************************************************
 * @brief	Setter for the software configuration.
 * @param	cfg A pointer to the \link #explorer_cfg_t software
 * 				  configuration\endlink data structure that contains the new
 * 				  configuration data.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SetConfiguration(explorer_cfg_t * cfg);


/*!***************************************************************************
 * @brief	Getter for the default software configuration.
 * @param	cfg A pointer to the \link #explorer_cfg_t software
 * 				  configuration\endlink data structure that will be filled with
 * 				  the current configuration data.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_GetDefaultConfiguration(explorer_cfg_t * cfg);


/*!***************************************************************************
 * @brief	Starts the measurements.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_StartTimerMeasurement(void);

/*!***************************************************************************
 * @brief	Stops the measurements and goes to idle state.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_StopTimerMeasurement(void);

/*!***************************************************************************
 * @brief	Triggers a single measurement and goes to idle afterwards.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_SingleMeasurement(void);

/*!***************************************************************************
 * @brief	Triggers a crosstalk calibration measurement sequence.
 * @param	mode The measurement mode to use for calibration.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ExecuteXtalkCalibrationSequence(argus_mode_t mode);

/*!***************************************************************************
 * @brief	Triggers a crosstalk calibration measurement sequence.
 * @param	mode The measurement mode to use for calibration.
 * @param	targetDistance The calibration target distance in meter and Q9.22
 * 						   format. Pass non-positive (0) value to execute
 * 						   relative calibration sequence only.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_ExecuteOffsetsCalibrationSequence(argus_mode_t mode,
													   q9_22_t targetDistance);

/*!***************************************************************************
 * @brief	Reinitializes the device w/ default settings w/o MCU reset.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_DeviceReinit(void);

/*!***************************************************************************
 * @brief	Displays the unambiguous range in meter on the SLCD display.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
void ExplorerApp_DisplayUnambiguousRange(void);

/*! @} */
#endif /* EXPLORER_APP_H */
