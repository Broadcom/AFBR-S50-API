/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer example application.
 * @details		This file implements an simple task scheduler.
 * @warning 	Confidential under NDA!
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

#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

/*!***************************************************************************
 * @defgroup	scheduler Task Scheduler
 * @ingroup		explorer_app
 *
 * @brief		A simple cooperative task scheduler with prioritized tasks.
 *
 * @details		A simple cooperative task scheduler with prioritized tasks.
 *
 * @addtogroup 	scheduler
 * @{
 *****************************************************************************/


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "task_status.h"

/*!***************************************************************************
 * @brief	Maximum Number of tasks.
 * @warning	Maximum number is 32 due to internal data structures of size uint32.
 * 			If larger number of tasks are desired, the internal data structures
 * 			need to be adjusted.
 *****************************************************************************/
#define SCHEDULER_MAX_TASKS 8U

/*!***************************************************************************
 * @brief	Task event definition.
 *****************************************************************************/
typedef void * task_event_t;

/*!***************************************************************************
 * @brief	Task functions definition.
 * @param	p Task event pointer. An abstract pointer to an task intern data
 * 				structure.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*task_function_t)(task_event_t p);

/*!***************************************************************************
 * @brief 	Initializes the task scheduler.
 * @details	Resets internal data structures to a known state.
 *****************************************************************************/
void Scheduler_Init(void);

/*!***************************************************************************
 * @brief	Runs the task scheduler.
 * @details	This is the main routine for the scheduler module. It schedules all
 * 			the pending tasks in order of priority in an endless loop.
 * 			If an error occurs within an task, the error is logged, but not
 * 			handled!
 *****************************************************************************/
void Scheduler_Run(void);

/*!***************************************************************************
 * @brief	Adds an new task to the scheduler.
 * @param	task       A pointer to the task function to be executed.
 * @param	priority   The priority level for the task. Every priority level
 * 						 can only have one task!
 * @param	eventQ     A pointer to the task event queue.
 * @param	eventQSize The size of the task event queue.
 * @param	name 	   A descriptive name of the task.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Scheduler_AddTask(task_function_t task,
						   uint32_t priority,
						   task_event_t eventQ,
						   size_t eventQSize,
						   char * name);

/*!***************************************************************************
 * @brief	Posts an event to the scheduler and executes it as soon as possible
 * @param	priority The priority of the task to be executed.
 * @param	event    A void* pointer to and task event parameter.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Scheduler_PostEvent(uint32_t priority, task_event_t event);

/*!***************************************************************************
 * @brief	Checks whether a specified task is pending for execution.
 * @param	priority The priority of the task to be checked.
 * @return	Returns true if the task is pending, false otherwise.
 *****************************************************************************/
bool Scheduler_IsTaskPending(uint32_t priority);

/*! @} */
#endif /* TASK_SCHEDULER_H */
