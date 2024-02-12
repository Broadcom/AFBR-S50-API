/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the main functionality of the Explorer Application.
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
#include "core/core_device.h"
#include "core/core_cfg.h"
#include "core/explorer_config.h"
#include "api/explorer_api.h"
#include "explorer_tasks.h"
#include "explorer_app.h"
#include "argus.h"
#include "sci/sci.h"
#include "tasks/task_scheduler.h"
#include "debug.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#include "driver/gpio.h" // debug only
#endif

#if AFBR_SCI_USB
#include "usb/usb_sci.h" // status definitions
#endif

#include <assert.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Buffer status type. */
typedef enum buffer_status_t
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
typedef struct argus_resultsbuffer_t
{
    /*! The device ID associated with the buffer. */
    sci_device_t deviceID;

    /*! The current buffer status. */
    buffer_status_t Status;

    /*! The data output mode to be used for this buffer. */
    data_output_mode_t DataOutputMode;

    /*! The measurement results data structure. */
    argus_results_t Result;

    /*! The debug measurement results data structure. */
    argus_results_debug_t DebugResults;

} argus_resultsbuffer_t;

/*! Size of the Argus results data buffer. */
#define ARGUSRESULTBUFFER_SIZE (2U * EXPLORER_DEVICE_COUNT)

/*! Size of the event queue. */
#define EVENTQ_SIZE (2U + 2U * EXPLORER_DEVICE_COUNT)

/*! The period to trigger a SPI ping signal to the device. */
#define PING_PERIOD_MS  333U


/*!@cond */
//#if 0
//#define DEBUG_TASK_IDLE_ENTER                     GPIO_ClearPinOutput(  Pin_PTB0)
//#define DEBUG_TASK_IDLE_LEAVE                 GPIO_SetPinOutput(Pin_PTB0)
//#define DEBUG_TASK_HANDLECMD_ENTER                GPIO_ClearPinOutput(  Pin_PTB1)
//#define DEBUG_TASK_HANDLECMD_LEAVE                GPIO_SetPinOutput(Pin_PTB1)
//#define DEBUG_TASK_EVALUATEDATA_ENTER             GPIO_ClearPinOutput(  Pin_PTB2)
//#define DEBUG_TASK_EVALUATEDATA_LEAVE         GPIO_SetPinOutput(Pin_PTB2)
//#define DEBUG_TASK_SENDRESULTS_ENTER          GPIO_ClearPinOutput(  Pin_PTB2)
//#define DEBUG_TASK_SENDRESULTS_LEAVE          GPIO_SetPinOutput(Pin_PTB2)
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

typedef struct idle_event_t
{
    ltc_t PingTime;
    status_t Status;

} idle_event_t;


typedef struct error_event_t
{
    status_t Status;
    ltc_t TimeStamp;
    char String[128];

} error_event_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* Task function prototypes */
static void Task_EvaluateMeasurementData(argus_hnd_t * argus);
static void Task_SendMeasurementData(argus_resultsbuffer_t * buffer);
static void Task_HandleCommand(sci_frame_t * frame);
static void Task_Error(error_event_t * e);
static void Task_Idle(idle_event_t * e);

/* Prototypes for callback and interrupt service routines */

/*! Callback function for new command received from SCI module. */
static status_t SCI_RxCommandCallbackHandler(sci_frame_t * cmd);

/*! Callback function for error occurred within the SCI/UART module. */
static void SCI_ErrorCallbackHandler(status_t status);


/******************************************************************************
 * Variables
 ******************************************************************************/

static scheduler_t * myScheduler = NULL;

/* Event Queues */
static task_event_t EventQ_Error[EVENTQ_SIZE] = {0};
static task_event_t EventQ_Idle[EVENTQ_SIZE] = {0};
static task_event_t EventQ_SendResults[EVENTQ_SIZE] = {0};
static task_event_t EventQ_EvalData[EVENTQ_SIZE] = {0};
static task_event_t EventQ_HandleCommand[2*EVENTQ_SIZE] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t ExplorerApp_InitTasks()
{
    /* Initialize the task scheduler module. */
    myScheduler = Scheduler_Init();
    assert(myScheduler != NULL);
    if (myScheduler == NULL) return ERROR_FAIL;

    /* Add tasks. */
    status_t
    status = Scheduler_AddTask(myScheduler, (task_function_t)Task_Error, TASK_ERROR, EventQ_Error,
                               sizeof(EventQ_Error) / sizeof(EventQ_Error[0]), "Error");
    if (status < STATUS_OK) return status;

    status = Scheduler_AddTask(myScheduler, (task_function_t)Task_EvaluateMeasurementData, TASK_EVAL_DAT, EventQ_EvalData,
                               sizeof(EventQ_EvalData) / sizeof(EventQ_EvalData[0]), "Evaluate");
    if (status < STATUS_OK) return status;

    status = Scheduler_AddTask(myScheduler, (task_function_t)Task_SendMeasurementData, TASK_SEND_DAT, EventQ_SendResults,
                               sizeof(EventQ_SendResults) / sizeof(EventQ_SendResults[0]), "Data Streaming");
    if (status < STATUS_OK) return status;

    status = Scheduler_AddTask(myScheduler, (task_function_t)Task_HandleCommand, TASK_HNDL_CMD, EventQ_HandleCommand,
                               sizeof(EventQ_HandleCommand) / sizeof(EventQ_HandleCommand[0]), "Handle SCI Command");
    if (status < STATUS_OK) return status;

    status = Scheduler_AddTask(myScheduler, (task_function_t)Task_Idle, TASK_IDLE, EventQ_Idle,
                               sizeof(EventQ_Idle) / sizeof(EventQ_Idle[0]), "Idle");
    if (status < STATUS_OK) return status;

    /* Install SCI callbacks. */
    SCI_SetRxCommandCallback(SCI_RxCommandCallbackHandler);
    SCI_SetErrorCallback(SCI_ErrorCallbackHandler);

    static idle_event_t idle_event = { 0 };
    status = Scheduler_PostEvent(myScheduler, TASK_IDLE, &idle_event);
    if (status < STATUS_OK) return status;

    return status;
}

void ExplorerApp_Run(void)
{
//#if DEBUG
//  ExplorerApp_StartTimerMeasurement();
//#endif
    Scheduler_Run(myScheduler); // never returns
}

void ExplorerApp_SwitchContext(void)
{
    Scheduler_SwitchContext(myScheduler);
}

static status_t OnError(status_t status, char *message)
{
    static error_event_t event = { 0 };
    event.Status = status;
    Time_GetNow(&event.TimeStamp);
    strcpy(event.String, message);
    Scheduler_PostEvent(myScheduler, TASK_ERROR, &event);
    return status;
}


/*******************************************************************************
 * Tasks
 ******************************************************************************/
static void Task_EvaluateMeasurementData(argus_hnd_t * argus)
{
    assert(argus != NULL);
    DEBUG_TASK_EVALUATEDATA_ENTER;

    static argus_resultsbuffer_t buffer[ARGUSRESULTBUFFER_SIZE] = {{0}};

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
    explorer_t * explorer = ExplorerApp_GetExplorerPtrFromArgus(argus);
    buf->DataOutputMode = ExplorerApp_GetDataOutputMode(explorer);
    const bool isDebugStreamingMode = !(buf->DataOutputMode & 0x01);
    argus_results_t * res = &buf->Result;
    argus_results_debug_t * dbg = isDebugStreamingMode ? &buf->DebugResults : NULL;

    status_t status = Argus_EvaluateDataDebug(argus, res, dbg);
    if (status < STATUS_OK) OnError(status, "Evaluation Task failed");

    buf->Status = BUFFER_FULL;
    buf->deviceID = ExplorerApp_GetDeviceID(explorer);

    Scheduler_PostEvent(myScheduler, TASK_SEND_DAT, buf);

    DEBUG_TASK_EVALUATEDATA_LEAVE;
}

static void Task_SendMeasurementData(argus_resultsbuffer_t * buffer)
{
    DEBUG_TASK_SENDRESULTS_ENTER;
    assert(buffer != 0);

    assert((buffer->DataOutputMode == DATA_OUTPUT_STREAMING_FULL) ||
           (buffer->DataOutputMode == DATA_OUTPUT_STREAMING_FULL_DEBUG) ||
           (buffer->DataOutputMode == DATA_OUTPUT_STREAMING_3D) ||
           (buffer->DataOutputMode == DATA_OUTPUT_STREAMING_3D_DEBUG) ||
           (buffer->DataOutputMode == DATA_OUTPUT_STREAMING_1D) ||
           (buffer->DataOutputMode == DATA_OUTPUT_STREAMING_1D_DEBUG));

    /* For message modes w/ DEBUG, the Result.Debug structure pointer must be available!
     * For message modes w/o DEBUG, the Result.Debug structure pointer must be null!
     * DEBUG modes are even, i.e. check for !(mode & 0x01).
     * Not DEBUG modes are odd, i.e. check for (mode & 0x01). */
    assert((!(buffer->DataOutputMode & 0x01) && (buffer->Result.Debug != 0)) ||
           ((buffer->DataOutputMode & 0x01) && (buffer->Result.Debug == 0)));

    switch (buffer->DataOutputMode)
    {
        case DATA_OUTPUT_STREAMING_FULL:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_FULL, 0, &(buffer->Result));
            break;
        case DATA_OUTPUT_STREAMING_FULL_DEBUG:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_FULL_DEBUG, 0, &(buffer->Result));
            break;
        case DATA_OUTPUT_STREAMING_3D:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_3D, 0, &(buffer->Result));
            break;
        case DATA_OUTPUT_STREAMING_3D_DEBUG:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_3D_DEBUG, 0, &(buffer->Result));
            break;
        case DATA_OUTPUT_STREAMING_1D:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_1D, 0, &(buffer->Result));
            break;
        case DATA_OUTPUT_STREAMING_1D_DEBUG:
            SCI_SendCommand(buffer->deviceID, CMD_MEASUREMENT_DATA_1D_DEBUG, 0, &(buffer->Result));
            break;
        default:
            OnError(ERROR_FAIL, "Invalid Data Output Mode!");
    }

    buffer->Status = BUFFER_EMTPY;
    DEBUG_TASK_SENDRESULTS_LEAVE;
}

static void Task_HandleCommand(sci_frame_t * frame)
{
    DEBUG_TASK_HANDLECMD_ENTER;
    assert(frame != NULL);
    SCI_InvokeRxCommand(frame);
    DEBUG_TASK_HANDLECMD_LEAVE;
}

static void Task_Error(error_event_t * e)
{
#if AFBR_SCI_USB
    if (e->Status == ERROR_USB_TIMEOUT) return;
#endif

    error_log("%s, error code: %d", e->String, e->Status);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    SLCD_DisplayDecimalSigned((int16_t)e->Status);
#endif
}

static void Task_Idle(idle_event_t * e)
{
    assert(e != NULL);
    DEBUG_TASK_IDLE_ENTER;

    /* Idle Task:
     * Called when no other events/tasks are pending.
     * Checks the device status and enables the red LED in case of any error.
     * If the device is idle for a longer period of time, a ping is sent to
     * verify if the device is still connected. */

    uint8_t devCount = ExplorerApp_GetInitializedExplorerCount();
    bool foundActiveDevice = false;

    for (uint8_t i = 0; i < devCount; i++)
    {
        explorer_t * explorer = ExplorerApp_GetInitializedExplorer(i);
        status_t status = Argus_GetStatus(explorer->Argus);
        if (e->Status != status)
        {
            e->Status = status;

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
            // Show error states by red led
            if (status < STATUS_OK)
            {
                GPIO_ClearPinOutput(Pin_LED_RED);
            }
            else
            {
                GPIO_SetPinOutput(Pin_LED_RED);
            }
#endif
        }

        /* Trigger a ping from time to time if the device is idle
         * Disable the ping if in DEBUG mode. */
        bool isDbgModeEnabled = ExplorerApp_GetDebugModeEnabled(explorer);
        if ((!isDbgModeEnabled) && (status == STATUS_IDLE))
        {
            uint32_t timeout = PING_PERIOD_MS / devCount;
            if (Time_CheckTimeoutMSec(&e->PingTime, timeout))
            {
                Time_GetNow(&e->PingTime);

                status = Argus_Ping(explorer->Argus);
                if (status < STATUS_OK)
                {
                    OnError(status, "Ping failed! Device has been disconnected!");
                }
            }
        }
        else
        {
            Time_GetNow(&e->PingTime);
        }

        if (status != ERROR_NOT_INITIALIZED) foundActiveDevice = true;
    }

    if (foundActiveDevice)
        Scheduler_PostEvent(myScheduler, TASK_IDLE, e);

    DEBUG_TASK_IDLE_LEAVE;
}

/*******************************************************************************
 * Callback functions
 ******************************************************************************/
status_t ExplorerApp_MeasurementReadyCallback(status_t status, argus_hnd_t * argus)
{
    if(status < STATUS_OK)
    {
        return OnError(status, "The measurement task execution failed");
    }

    /* post event for evaluating results */
    status = Scheduler_PostEvent(myScheduler, TASK_EVAL_DAT, argus);

    if(status < STATUS_OK)
    {
        return OnError(status, "Posting Evaluation Task Error");
    }
    return status;
}

static void SCI_ErrorCallbackHandler(status_t status)
{
    OnError(status, "SCI Error");
}

static status_t SCI_RxCommandCallbackHandler(sci_frame_t * cmd)
{
    assert(cmd);
    task_event_t * const event = (task_event_t * const)cmd;
    status_t status = Scheduler_PostEvent(myScheduler, TASK_HNDL_CMD, event);
    if(status < STATUS_OK)
    {
        OnError(status, "Command Receive Task Error");
    }
    return status;
}

