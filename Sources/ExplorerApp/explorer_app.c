/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Demo Application.
 * @details		This file contains the main functionality of the Explorer Application.
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
#include "explorer_app.h"
#include "explorer_hardware.h"
#include "explorer_flash.h"

#include "api/explorer_api.h"
#include "tasks/task_scheduler.h"
#include "sci/sci.h"
#include "argus.h"

#include "board/board_config.h" // for SPI slave select definition

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#include "usb/usb_sci.h" // status definitions
#include "driver/gpio.h" // debug only
#endif

#include "driver/s2pi.h"
#include "utility/debug.h"

#include <assert.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! Buffer status type. */
typedef enum
{
	/*! Data buffer empty. Ready to write. */
	BUFFER_EMTPY = 0,

	/*! Data buffer is currently processed. */
	BUFFER_BUSY = 1,

	/*! Data buffer is full. Ready to read. */
	BUFFER_FULL = 2,

	/*! An error occurred and needs to be handled. */
	BUFFER_ERROR = -1,

} buffer_status_t;

/*! Buffer structure for measurement results. */
typedef struct
{
	/*! The current buffer status. */
	buffer_status_t Status;

	/*! The measurement results data structure. */
	argus_results_t Result;

} argus_resultsbuffer_t;

/*! Size of the Argus results data buffer. */
#define ARGUSRESULTBUFFER_SIZE 2U

/*! Size of the event queue. */
#define EVENTQ_SIZE 4U

/*! The period to trigger a SPI ping signal to the device. */
#define PING_PERIOD_MS	333U

/*!***************************************************************************
 * @brief	Determines the SPI slave setting: use external (wired) or internal
 * 			(soldered) device.
 * @details	-1: Auto detect.
 * 			1: The intern S2PI slave (connected via adapter board).
 * 			2: The extern S2PI slave (connected via cable).
 *			3: The experimental intern S2PI slave w/ GPIO CS (connected via adapter board).
 *			4: The experimental extern S2PI slave w/ GPIO CS (connected via cable).
 *****************************************************************************/
#ifndef SPI_DEFAULT_SLAVE
#define SPI_DEFAULT_SLAVE -1
#endif

/*! The default baud rate for SPI. */

#ifndef SPI_BAUDRATE
#ifdef SPI_MAX_BAUDRATE
#define SPI_BAUDRATE SPI_MAX_BAUDRATE
#else
#define SPI_BAUDRATE 1000000U
#endif
#endif

/*!@cond */
//#if 0
//#define DEBUG_TASK_IDLE_ENTER 					GPIO_ClearPinOutput(  Pin_PTB0)
//#define DEBUG_TASK_IDLE_LEAVE	 				GPIO_SetPinOutput(Pin_PTB0)
//#define DEBUG_TASK_HANDLECMD_ENTER 				GPIO_ClearPinOutput(  Pin_PTB1)
//#define DEBUG_TASK_HANDLECMD_LEAVE	 			GPIO_SetPinOutput(Pin_PTB1)
//#define DEBUG_TASK_EVALUATEDATA_ENTER 			GPIO_ClearPinOutput(  Pin_PTB2)
//#define DEBUG_TASK_EVALUATEDATA_LEAVE	 		GPIO_SetPinOutput(Pin_PTB2)
//#define DEBUG_TASK_SENDRESULTS_ENTER			GPIO_ClearPinOutput(  Pin_PTB2)
//#define DEBUG_TASK_SENDRESULTS_LEAVE			GPIO_SetPinOutput(Pin_PTB2)
//#else
#define DEBUG_TASK_IDLE_ENTER
#define DEBUG_TASK_IDLE_LEAVE
#define DEBUG_TASK_HANDLECMD_ENTER
#define DEBUG_TASK_HANDLECMD_LEAVE
#define DEBUG_TASK_EVALUATEDATA_ENTER
#define DEBUG_TASK_EVALUATEDATA_LEAVE
#define DEBUG_TASK_SENDRESULTS_ENTER
#define DEBUG_TASK_SENDRESULTS_LEAVE
//#endif
/*!@endcond */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t ExplorerApp_InitDevice(explorer_cfg_t * cfg);
static status_t ExplorerApp_FindConnectedDevices(int8_t * slave, uint32_t * maxBaudRate);
static status_t ExplorerApp_CheckConnectedSlave(uint8_t slave);

/*!***************************************************************************
 * @brief	Initializes the scheduler tasks.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t ExplorerApp_InitTasks(void);

/* Task function prototypes */
static status_t Task_EvaluateMeasurementData(task_event_t e);
static status_t Task_SendMeasurementData(task_event_t e);
static status_t Task_HandleCommand(task_event_t e);
static status_t Task_Error(task_event_t e);
static status_t Task_Idle(task_event_t e);

/* Prototypes for callback and interrupt service routines */

/*! Callback function after measurement cycle was finished. */
static status_t MeasurementDoneCallback(status_t status, void * param);
/*! Callback function for new command received from SCI module. */
static status_t SCI_RxCommandCallback(sci_frame_t * cmd);
/*! Callback function for error occurred within the SCI/UART module. */
static void SCI_ErrorCallback(status_t status);


/******************************************************************************
 * Variables
 ******************************************************************************/
/* Event Queues */
static task_event_t EventQ_Error[EVENTQ_SIZE] = {0};
static task_event_t EventQ_Idle[EVENTQ_SIZE] = {0};
static task_event_t EventQ_SendResults[EVENTQ_SIZE] = {0};
static task_event_t EventQ_EvalData[EVENTQ_SIZE] = {0};
static task_event_t EventQ_HandleCommand[2*EVENTQ_SIZE] = {0};

static struct
{
	status_t Status;
	ltc_t TimeStamp;
	char String[128];
} myError = {0 };

/*! The AFBR-S50 Explorer configuration. */
explorer_cfg_t ExplorerConfiguration;

/*! The Argus data structure. */
static argus_hnd_t * myArgusPtr;

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t ExplorerApp_InitDevice(explorer_cfg_t * cfg)
{
	/* Check for connected devices. */
	int8_t slave = cfg->SPISlave;
	uint32_t baudRate = cfg->SPIBaudRate;
	status_t status = ExplorerApp_FindConnectedDevices(&slave, &baudRate);
	if (status < STATUS_OK)
	{
		error_log("No suitable device connected, error code: %d", status);

		/* Clear the Argus memory object. */
		Argus_Deinit(myArgusPtr);

		return status;
	}

	/* Device initialization */
	status = Argus_Init(myArgusPtr, slave);
	if (status < STATUS_OK)
	{
		error_log("Failed to initialize AFBR-S50 API, error code: %d", status);
		return status;
	}

	return status;
}

status_t ExplorerApp_Init(void)
{
	status_t status;
	myArgusPtr = Argus_CreateHandle();
	assert(myArgusPtr != 0);

	status = ExplorerApp_GetDefaultConfiguration(&ExplorerConfiguration);
	if(status < STATUS_OK)
	{
		assert(0);
		return status;
	}

	/* Initialize the board hardware /w watchdog timer disabled */
	status = ExplorerApp_InitHardware(&ExplorerConfiguration);
	if(status < STATUS_OK)
	{
		assert(0);
		return status;
	}

	/* Initialize the systems communication interface. */
	status = ExplorerApp_InitCommands(myArgusPtr);
	if(status < STATUS_OK)
	{
		assert(0);
		return status;
	}

	/* Initialize the AFBR-S50 Explorer task scheduler. */
	status = ExplorerApp_InitTasks();
	if(status < STATUS_OK)
	{
		assert(0);
		return status;
	}

	/* printout reset cause for debugging. */
	Debug_CheckReset();

	/* Check for connected devices. */
	status = ExplorerApp_InitDevice(&ExplorerConfiguration);
	if(status < STATUS_OK)
	{
		error_log("Device Initialization failed: force S2PI pins to low state!");
		S2PI_PinsLow();
	}

	status = ExplorerApp_SetConfiguration(&ExplorerConfiguration);
	if(status < STATUS_OK)
	{
//		assert(0);
		return status;
	}

	Scheduler_PostEvent(TASK_IDLE, 0);
	return status;
}

static status_t ExplorerApp_InitTasks(void)
{
	status_t status;

	/* Initialize the task scheduler module. */
	Scheduler_Init();

	/* Add tasks. */
	status = Scheduler_AddTask(&Task_Error, TASK_ERROR, EventQ_Error, EVENTQ_SIZE, "Error");
	if(status < STATUS_OK) return status;
	status = Scheduler_AddTask(&Task_EvaluateMeasurementData, TASK_EVAL_DAT, EventQ_EvalData, EVENTQ_SIZE, "Evaluate");
	if(status < STATUS_OK) return status;
	status = Scheduler_AddTask(&Task_SendMeasurementData, TASK_SEND_DAT, EventQ_SendResults, EVENTQ_SIZE, "Data Streaming");
	if(status < STATUS_OK) return status;
	status = Scheduler_AddTask(&Task_HandleCommand, TASK_HNDL_CMD, EventQ_HandleCommand, 2*EVENTQ_SIZE, "Handle SCI Command");
	if(status < STATUS_OK) return status;
	status = Scheduler_AddTask(&Task_Idle, TASK_IDLE, EventQ_Idle, EVENTQ_SIZE, "Idle");
	if(status < STATUS_OK) return status;

	/* Install SCI callbacks. */
	SCI_SetRxCommandCallback(SCI_RxCommandCallback);
	SCI_SetErrorCallback(SCI_ErrorCallback);

	return status;
}

void ExplorerApp_Run(void)
{
//#if DEBUG
//	ExplorerApp_StartTimerMeasurement();
//#endif
	Scheduler_Run(); // never returns
}

void ExplorerApp_DisplayUnambiguousRange(void)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	assert(myArgusPtr != 0);

	/* Display clock frequency on LCD. */
	uint32_t measRange;
	if(Argus_GetConfigurationUnambiguousRange(myArgusPtr, &measRange) == STATUS_OK)
	{
		uint8_t decPos = 1U;
		while (measRange > 9999U)
		{
			measRange /= 10U;
			decPos++;
		}
		SLCD_DisplayDecimalUnsigned(measRange);
		SLCD_SetDecimalPointPosition(decPos);
	}
#else
	(void)0;
#endif
}

status_t ExplorerApp_GetDefaultConfiguration(explorer_cfg_t * cfg)
{
	assert(cfg != 0);

	cfg->SPIBaudRate = SPI_BAUDRATE;
	cfg->SPISlave = SPI_DEFAULT_SLAVE;

#if defined(DEBUG)
	cfg->DebugMode = true;
#else
	cfg->DebugMode = false;
#endif
	cfg->DataOutputMode = DATA_OUTPUT_STREAMING_FAST;

	return STATUS_OK;
}

void ExplorerApp_GetConfiguration(explorer_cfg_t * cfg)
{
	memcpy(cfg, &ExplorerConfiguration, sizeof(explorer_cfg_t));
}

status_t ExplorerApp_SetConfiguration(explorer_cfg_t * cfg)
{
	if (cfg->DataOutputMode > DATA_OUTPUT_STREAMING_1D_FAST)
	{
		error_log("Explorer configuration failed: the data output mode (%d) is unknown.",
				  cfg->DataOutputMode);
		return ERROR_INVALID_ARGUMENT;
	}

	if (cfg->SPIBaudRate > SPI_MAX_BAUDRATE)
	{
		error_log("Explorer configuration failed: the SPI baud rate (%d) is too large.\n"
				  "It is reset to maximum value of %d bps.",
				  cfg->SPIBaudRate, SPI_MAX_BAUDRATE);
		cfg->SPIBaudRate = SPI_MAX_BAUDRATE;
	}

	if (ExplorerConfiguration.SPIBaudRate != cfg->SPIBaudRate)
	{
		const status_t status = S2PI_SetBaudRate(cfg->SPIBaudRate);
		if (status != STATUS_OK)
		{
			/* Check if the actual baud rate is within 10 % of the desired baud rate. */
			if (status == ERROR_S2PI_INVALID_BAUDRATE)
				error_log("S2PI: The requested baud rate (%d bps) is not supported! "
						  "The actual baud rate is %d bps.",
						  cfg->SPIBaudRate, S2PI_GetBaudRate());
			else
				error_log("S2PI: Setting the new baud rate failed, "
						  "error code: %d", status);

			/* Reset baud rate to last setting. */
			S2PI_SetBaudRate(ExplorerConfiguration.SPIBaudRate);
			return status;
		}
		cfg->SPIBaudRate = S2PI_GetBaudRate();
		//print("S2PI: Baud Rate set to %d bps.", cfg->SPIBaudRate);
	}

	const s2pi_slave_t slave = Argus_GetSPISlave(myArgusPtr);

	if ((cfg->SPISlave < 0 && slave <= 0) ||
		(cfg->SPISlave > 0 && slave != cfg->SPISlave))
	{
		if (Argus_GetStatus(myArgusPtr) != STATUS_IDLE)
		{
			error_log("S2PI: Configuration failed, cannot set new SPI slave while device is busy!");
			return ERROR_FAIL;
		}

		status_t status = ExplorerApp_InitDevice(cfg);
		if (status != STATUS_OK)
		{
			error_log("S2PI: Configuration failed, cannot set new SPI slave, "
					  "error code: %d", status);

			ExplorerApp_InitDevice(&ExplorerConfiguration);
			return status;
		}

		//print("S2PI: Re-initialized with Slave %d.", cfg->SPISlave);
	}

	memcpy(&ExplorerConfiguration, cfg, sizeof(explorer_cfg_t));

	return STATUS_OK;
}

status_t ExplorerApp_StartTimerMeasurement(void)
{
	status_t status = Argus_StartMeasurementTimer(myArgusPtr, MeasurementDoneCallback);
	ExplorerApp_DisplayUnambiguousRange();
	return status;
}
status_t ExplorerApp_StopTimerMeasurement(void)
{
	return Argus_StopMeasurementTimer(myArgusPtr);
}
status_t ExplorerApp_SingleMeasurement(void)
{
	status_t status = STATUS_OK;
	do
	{
		status = Argus_TriggerMeasurement(myArgusPtr, MeasurementDoneCallback);
	} while (status == STATUS_ARGUS_POWERLIMIT);
	return status;
}
status_t ExplorerApp_ExecuteXtalkCalibrationSequence(argus_mode_t mode)
{
	status_t status = Argus_ExecuteXtalkCalibrationSequence(myArgusPtr, mode);
	if (status < STATUS_OK) return status;

	do
	{
		status = Argus_GetStatus(myArgusPtr);
	} while (status > STATUS_IDLE);
	return status;
}
status_t ExplorerApp_ExecuteOffsetsCalibrationSequence(argus_mode_t mode, q9_22_t targetRange)
{
	status_t status = STATUS_OK;
	if (targetRange <= 0)
	{
		status = Argus_ExecuteRelativeRangeOffsetCalibrationSequence(myArgusPtr, mode);
	}
	else
	{
		status = Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence(myArgusPtr, mode, targetRange);
	}
	if (status < STATUS_OK) return status;

	do
	{
		status = Argus_GetStatus(myArgusPtr);
	} while (status > STATUS_IDLE);
	return status;
}
status_t ExplorerApp_DeviceReinit(void)
{
    status_t status = ExplorerApp_InitDevice(&ExplorerConfiguration);
	ExplorerApp_DisplayUnambiguousRange();
	return status;
}

static inline status_t OnError(status_t status, char * message)
{
	myError.Status = status;
	Time_GetNow(&myError.TimeStamp);
	strcpy(myError.String, message);
	Scheduler_PostEvent(TASK_ERROR, 0);
	return status;
}

static inline void PingDevice(void)
{
	status_t status = Argus_Ping(myArgusPtr);
	if(status < STATUS_OK)
	{
		S2PI_PinsLow();
		OnError(status, "Ping failed! Device has been disconnected! SPI connection is disabled.");
	}
}

status_t GetSystemStatus(void)
{
	status_t status = Argus_GetStatus(myArgusPtr);

	if (status == STATUS_IDLE)
	{
		status = Argus_Ping(myArgusPtr);
	}

	return status;
}

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static status_t Task_EvaluateMeasurementData(task_event_t e)
{
	// e is ads_value_buf_t *

	static argus_resultsbuffer_t buffer[ARGUSRESULTBUFFER_SIZE] = {{0}};

	DEBUG_TASK_EVALUATEDATA_ENTER;
	assert(e);

	/* Find free data buffer. */
    argus_resultsbuffer_t * buf = 0;
	for(uint8_t i = 0; i < ARGUSRESULTBUFFER_SIZE; ++i)
	{
		if(buffer[i].Status == BUFFER_EMTPY)
		{
			buf = &buffer[i];
			break;
		}
	}
	assert(buf != 0); // no buffer found! should never happen
	buf->Status = BUFFER_BUSY;

	/* Evaluate data. */
	Argus_EvaluateData(myArgusPtr, &(buf->Result), e);

	buf->Status = BUFFER_FULL;

	Scheduler_PostEvent(TASK_SEND_DAT, (task_event_t)buf);

	DEBUG_TASK_EVALUATEDATA_LEAVE;
	return STATUS_OK;
}
static status_t Task_SendMeasurementData(task_event_t e)
{
	DEBUG_TASK_SENDRESULTS_ENTER;
	assert(e);
	argus_resultsbuffer_t * buf = (argus_resultsbuffer_t *)e;

	switch(ExplorerConfiguration.DataOutputMode)
	{
		//case DATA_OUTPUT_ON_REQUEST:
		//	break; // do nothing, on request only!
		case DATA_OUTPUT_STREAMING_RAW:
			SendMeasurementData(CMD_MEASUREMENT_DATA_RAW, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_FULL:
			SendMeasurementData(CMD_MEASUREMENT_DATA_DEBUG, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_FAST:
			SendMeasurementData(CMD_MEASUREMENT_DATA_FULL, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_3D_FULL:
			SendMeasurementData(CMD_MEASUREMENT_DATA_3D_DEBUG, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_3D_FAST:
			SendMeasurementData(CMD_MEASUREMENT_DATA_3D, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_1D_FULL:
			SendMeasurementData(CMD_MEASUREMENT_DATA_1D_DEBUG, &(buf->Result));
			break;
		case DATA_OUTPUT_STREAMING_1D_FAST:
			SendMeasurementData(CMD_MEASUREMENT_DATA_1D, &(buf->Result));
			break;
		default:
			OnError(ERROR_FAIL, "Invalid Data Output Mode!");
	}

	buf->Status = BUFFER_EMTPY;
	DEBUG_TASK_SENDRESULTS_LEAVE;
	return STATUS_OK;
}
static status_t Task_HandleCommand(task_event_t e)
{
	DEBUG_TASK_HANDLECMD_ENTER;
	assert(e);
	SCI_InvokeRxCommand(e);
	DEBUG_TASK_HANDLECMD_LEAVE;
	return STATUS_OK;
}

static status_t Task_Error(task_event_t e)
{
	(void)e; // unused parameter;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	if(myError.Status != ERROR_USB_TIMEOUT)
	{
#endif
		error_log("%s, error code: %d", myError.String, myError.Status);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		SLCD_DisplayDecimalSigned((int16_t)myError.Status);
	}
#endif
	return STATUS_OK; // ERROR_FAIL;
}

static status_t Task_Idle(task_event_t e)
{
	DEBUG_TASK_IDLE_ENTER;

	// Idle task does check if the device is idle for a given period
	// if the period elapses, the devices is triggered in order to prevent
	// latchup effect.

	status_t status = Argus_GetStatus(myArgusPtr);

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	// Show error states by red led
	if (status < STATUS_OK)
	{
		GPIO_ClearPinOutput(Pin_LED_Red);
	}
	else
	{
		GPIO_SetPinOutput(Pin_LED_Red);
	}
#endif

	// trigger a ping from time to time
	if(!ExplorerConfiguration.DebugMode && status == STATUS_IDLE)
	{
		static ltc_t t_start = {0};
		if(e)
		{
			if(Time_CheckTimeoutMSec(&t_start, PING_PERIOD_MS))
			{
				Time_GetNow(&t_start);
				PingDevice();
			}
		}
		else
		{
			Time_GetNow(&t_start);
			e = &t_start;
		}
	}
	else
	{
		e = 0;
	}

	Scheduler_PostEvent(TASK_IDLE, e);

	DEBUG_TASK_IDLE_LEAVE;
	return STATUS_OK;
}


/*******************************************************************************
 * Callback functions
 ******************************************************************************/
static status_t MeasurementDoneCallback(status_t status, void * param)
{
	if(status < STATUS_OK)
	{
//		if(status == ERROR_TIMEOUT)
//		{
//			S2PI_PinsLow();
//		}
		return OnError(status, "The measurement task execution failed");
	}

	/* post event for evaluating results */
	task_event_t event = (task_event_t)param;
	status = Scheduler_PostEvent(TASK_EVAL_DAT, event);
	if(status < STATUS_OK)
	{
		return OnError(status, "Posting Evaluation Task Error");
	}
	return status;
}

static void SCI_ErrorCallback(status_t status)
{
	OnError(status, "Serial Error");
}
static status_t SCI_RxCommandCallback(sci_frame_t * cmd)
{
	assert(cmd);
	task_event_t event = (task_event_t)cmd;
	status_t status = Scheduler_PostEvent(TASK_HNDL_CMD, event);
	if(status < STATUS_OK)
	{
		OnError(status, "Command Receive Task Error");
	}
	return status;
}

static status_t ExplorerApp_CheckConnectedSlave(uint8_t slave)
{
	status_t status = STATUS_OK;

	uint8_t data[17U] = {0};
	for(uint8_t i = 1; i < 17U; ++i) data[i] = i;

	for(uint8_t n = 0; n < 2; n++)
	{
		data[0] = 0x04;
		status = S2PI_TransferFrame(slave, data, data, 17U, 0, 0);
		if(status < STATUS_OK)
		{
			return status;
		}

		ltc_t start;
		Time_GetNow(&start);
		do
		{
			status = S2PI_GetStatus();
			if(Time_CheckTimeoutMSec(&start, 100))
			{
				status = ERROR_TIMEOUT;
			}
		}
		while(status == STATUS_BUSY);

		if(status < STATUS_OK)
		{
			S2PI_Abort();
			return status;
		}
	}

	bool hasData = true;
	for (uint8_t i = 1; i < 17U; ++i)
	{
		uint8_t j = ~i; // devices w/ inverted MISO
		if ((data[i] != i) && (data[i] != j))
			hasData = false;
	}

	if (hasData)
		return STATUS_OK;

	return ERROR_ARGUS_NOT_CONNECTED;
}

static status_t ExplorerApp_FindConnectedDevices(int8_t * slave, uint32_t * maxBaudRate)
{
	assert(slave != 0);
	assert(maxBaudRate != 0);

	status_t status = STATUS_OK;
	uint8_t r_max = 0;

	if (*slave == 0)
	{
		return ERROR_ARGUS_NOT_CONNECTED;
	}
	else if (*slave > 0)
	{
		return ExplorerApp_CheckConnectedSlave(*slave);
	}
	else
	{
		/* Reduce baud rate for search. */
		while (((*maxBaudRate) >> r_max) > 100000U)	r_max++;
		S2PI_SetBaudRate((*maxBaudRate) >> r_max);

		/* Auto detect slave. */
		*slave = 0;
		for (uint8_t s = 1; s <= 4; s++)
		{
			status = ExplorerApp_CheckConnectedSlave(s);
			if (status == STATUS_OK)
			{
				*slave = s;
				break;
			}
			else if (status != ERROR_ARGUS_NOT_CONNECTED)
			{
				return status;
			}
		}
		if (*slave == 0)
		{
			return ERROR_ARGUS_NOT_CONNECTED;
		}
	}

	/* Auto detect max baud rate. */
	for (uint8_t r = 0; r < r_max; ++r)
	{
		S2PI_SetBaudRate((*maxBaudRate) >> r);
		status = ExplorerApp_CheckConnectedSlave(*slave);
		if (status == STATUS_OK)
		{
			(*maxBaudRate) = (*maxBaudRate) >> r;
			return STATUS_OK;
		}
		else if (status != ERROR_ARGUS_NOT_CONNECTED)
		{
			return status;
		}
	}

	return ERROR_ARGUS_NOT_CONNECTED;
}
