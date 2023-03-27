/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the task definitions of the AFBR-S50
 *              Explorer Application.
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

#ifndef EXPLORER_TASKS_H
#define EXPLORER_TASKS_H

/*!***************************************************************************
 * @defgroup    explorer_tasks AFBR-S50 Explorer Application - Tasks
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer Application - Tasks
 * @details     Contains the scheduler task definitions for the AFBR-S50 Explorer
 *              Application.
 *
 *              A simple task scheduler is used to host the Argus API and a
 *              simple systems communication interface is implemented to
 *              connect to the AFBR-S50 Explorer GUI. The latter is an evaluation
 *              software for the Argus time-of-flight devices.
 *
 * @addtogroup  explorer_tasks
 * @{
 *****************************************************************************/

#include "api/argus_status.h"
#include "explorer_app.h"
#include "argus.h"


/*! Explorer Application Task numbers (and priority!). */
typedef enum explorer_task_t
{
    TASK_ERROR      = 7U,   /*!< ID and Priority of error handling task. */
    TASK_HNDL_CMD   = 6U,   /*!< ID and Priority of command handling task. */
    TASK_SEND_DAT   = 2U,   /*!< ID and Priority of send results task. */
    TASK_EVAL_DAT   = 1U,   /*!< ID and Priority of evaluate data task. */
    TASK_IDLE       = 0U    /*!< ID and Priority of the idle task. */
} explorer_task_t;

/*!***************************************************************************
 * @brief   Initializes the scheduler tasks.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_InitTasks();

/*!***************************************************************************
 * @brief   Callback function after measurement cycle was finished.
 *
 * @details Pass this to the #Argus_StartMeasurementTimer and
 *          #Argus_TriggerMeasurement functions.
 *
 * @param   status The current API measurement status.
 *
 * @param   argus The AFBR-S50-API instance that invokes the callback.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_MeasurementReadyCallback(status_t status, argus_hnd_t * argus);

/*!***************************************************************************
 * @brief   Switches the scheduler task context.
 * @details Pauses the execution of the current task and lets the lower priority
 *          tasks run. Useful when current task depends on any lower priority
 *          task.
 *****************************************************************************/
void ExplorerApp_SwitchContext(void);

/*! @} */
#endif /* EXPLORER_TASKS_H */
