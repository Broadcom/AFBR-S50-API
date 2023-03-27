/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer example application.
 * @details     This file implements an simple task scheduler.
 * @warning     Confidential under NDA!
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

#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

/*!***************************************************************************
 * @defgroup    scheduler Task Scheduler
 * @ingroup     explorer_app
 *
 * @brief       A simple cooperative task scheduler with prioritized tasks.
 *
 * @details     A simple cooperative task scheduler with prioritized tasks.
 *
 * @addtogroup  scheduler
 * @{
 *****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "task_status.h"

/*!***************************************************************************
 * @brief   Maximum Number of tasks.
 * @warning Maximum number is 32 due to internal data structures of size uint32.
 *          If larger number of tasks are desired, the internal data structures
 *          need to be adjusted.
 *****************************************************************************/
#define SCHEDULER_MAX_TASKS 8U

/*!***************************************************************************
 * @brief   Definition of the task priority (and unique task ID).
 * @details Higher values mean higher urgency.
 *          Priority 0 can be used for IDLE task.
 *****************************************************************************/
typedef uint8_t task_prio_t;

/*!***************************************************************************
 * @brief   Task scheduler type definition.
 *****************************************************************************/
typedef struct scheduler_t scheduler_t;

/*!***************************************************************************
 * @brief   Task event type definition.
 *****************************************************************************/
typedef void * task_event_t;

/*!***************************************************************************
 * @brief   Task functions definition.
 * @param   e Task event pointer. An abstract pointer to an task intern data
 *            structure.
 *****************************************************************************/
typedef void (*task_function_t)(task_event_t e);

/*!***************************************************************************
 * @brief   Initializes the task scheduler.
 * @details Resets internal data structures to a known state.
 * @return  Returns the abstract pointer to the scheduler handle object.
 *****************************************************************************/
scheduler_t* Scheduler_Init(void);

/*!***************************************************************************
 * @brief   Runs the task scheduler.
 * @details This is the main routine for the scheduler module. It schedules all
 *          the pending tasks in order of priority in an endless loop.
 *          If an error occurs within an task, the error is logged, but not
 *          handled!
 * @param   me The instance handle of the task scheduler.
 *****************************************************************************/
void Scheduler_Run(scheduler_t * const me);

/*!***************************************************************************
 * @brief   Suspends the current task and runs another task.
 * @details The function can be called from within a task in order to suspend
 *          the current task and run other task (to completion). The function
 *          return when the other task is finished. The function runs exactly
 *          one other task, even if more are pending. The function can be
 *          called another time to run another task. If no tasks are pending,
 *          the function does nothing.
 * @note    This function never returns!
 * @param   me The instance handle of the task scheduler.
 *****************************************************************************/
void Scheduler_SwitchContext(scheduler_t * const me);

/*!***************************************************************************
 * @brief   Adds an new task to the scheduler.
 * @param   me The instance handle of the task scheduler.
 * @param   task A pointer to the task function to be executed.
 * @param   priority The priority level for the task. Every priority level
 *                   can only have one task!
 * @param   eventQ A pointer to the task event queue.
 * @param   eventQSize The size of the task event queue.
 * @param   name A descriptive name of the task.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Scheduler_AddTask(scheduler_t * const me,
                           task_function_t task,
                           task_prio_t priority,
                           task_event_t eventQ,
                           size_t eventQSize,
                           const char * name);

/*!***************************************************************************
 * @brief   Posts an event to the scheduler and executes it as soon as possible
 * @param   me The instance handle of the task scheduler.
 * @param   priority The priority of the task to be executed.
 * @param   event A void* pointer to and task event parameter.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Scheduler_PostEvent(scheduler_t * const me,
                             task_prio_t priority,
                             task_event_t event);

/*!***************************************************************************
 * @brief   Checks whether a specified task is pending for execution.
 * @param   me The instance handle of the task scheduler.
 * @param   priority The priority of the task to be checked.
 * @return  Returns true if the task is pending, false otherwise.
 *****************************************************************************/
bool Scheduler_IsTaskPending(scheduler_t * const me,
                             task_prio_t priority);

/*! @} */
#endif /* TASK_SCHEDULER_H */
