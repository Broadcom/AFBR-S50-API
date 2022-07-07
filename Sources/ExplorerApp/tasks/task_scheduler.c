/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer example application.
 * @details		This file implements an simple task scheduler.
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
#include "task_scheduler.h"
#include "task_profiler.h"

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "utility/debug.h"
#include "driver/irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!***************************************************************************
 * @brief Task control block definition.
 *****************************************************************************/
typedef struct
{
	task_function_t  Task;		/*!< Function to execute. */
	task_event_t 	*EQ_Buff;	/*!< Event queue buffer. */
	task_event_t	*EQ_Head;	/*!< Head of the queue. */
	task_event_t	*EQ_Tail;	/*!< Tail of the queue. */
	size_t			 EQ_Size;	/*!< Total buffer size of the queue. */
	size_t			 EQ_Load;	/*!< Currently used buffer size. */
	char			*Name;		/*!< Task descriptive name. */
} taskcontrolblock_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static inline status_t ScheduleNext(void);
//extern void Scheduler_WatchdogKick(void);

#if PROFILING
void OnTaskStart(uint32_t priority);
void OnTaskFinished(uint32_t priority, status_t status);
void OnTaskQueued(uint32_t priority);
#endif

/******************************************************************************
 * Variables
 ******************************************************************************/
static taskcontrolblock_t myTCB[SCHEDULER_MAX_TASKS] = {{0}};
static volatile uint32_t myPendingFlags = 0;
static volatile uint32_t myMaskingFlags = 0;

static uint8_t myLog2Lookup[] = {0xff, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3};
static uint32_t myIdleCounter = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void Scheduler_Init(void)
{
	myPendingFlags = 0;
	myMaskingFlags = 0;
	myIdleCounter = 0;
}

status_t Scheduler_AddTask(task_function_t task, uint32_t priority, task_event_t eventQ, size_t eventQSize, char * name)
{
	if(!task) return ERROR_INVALID_ARGUMENT;
	if(!eventQ) return ERROR_INVALID_ARGUMENT;
	if(!(eventQSize > 0)) return ERROR_INVALID_ARGUMENT;
	if(!(priority < SCHEDULER_MAX_TASKS))	return ERROR_INVALID_ARGUMENT;

	taskcontrolblock_t * TCB = &myTCB[priority];

	if(TCB->Task != 0) return ERROR_INVALID_ARGUMENT; // check if task isn't used right now

	TCB->Task = task;
	TCB->EQ_Buff = eventQ;
	TCB->EQ_Head = eventQ;
	TCB->EQ_Tail = eventQ;
	TCB->EQ_Size = eventQSize;
	TCB->EQ_Load = 0;
	TCB->Name = name;

	return STATUS_OK;
}

status_t Scheduler_PostEvent(uint32_t priority, task_event_t event)
{
	if(!(priority < SCHEDULER_MAX_TASKS)) return ERROR_INVALID_ARGUMENT;

	taskcontrolblock_t * TCB = &myTCB[priority];
	if(TCB->Task == 0) return ERROR_NOT_INITIALIZED;

	IRQ_LOCK();
	if(TCB->EQ_Load < TCB->EQ_Size) // check if queue is not full
	{
		*TCB->EQ_Head = event;
		if((++TCB->EQ_Head) == TCB->EQ_Buff + TCB->EQ_Size) TCB->EQ_Head = TCB->EQ_Buff;
		if((++TCB->EQ_Load) == (uint8_t)1U) myPendingFlags |= (1U << priority);
#if PROFILING
		OnTaskQueued(priority);
#endif
	}
	else
	{
		IRQ_UNLOCK();
		return ERROR_TASK_QUEUE_FULL;
	}

	IRQ_UNLOCK();
	return STATUS_OK;
}

bool Scheduler_IsTaskPending(uint32_t priority)
{
	return !((myPendingFlags & (1U << priority)) == 0);
}

void Scheduler_Run(void)
{
	for(;;)
	{
		ScheduleNext();
	}
}

static inline status_t ScheduleNext(void)
{
	uint32_t pending = (uint32_t)(myPendingFlags & ~myMaskingFlags);
	status_t status = STATUS_OK;

	if(pending)
	{
		IRQ_LOCK();

		/* Get highest pending and not masked priority. */
		uint32_t prio = 0;
		while(pending & ~0xFU)
		{
			pending>>=4U;
			prio += 4U;
		}
		prio += myLog2Lookup[pending];
		assert(prio < SCHEDULER_MAX_TASKS);

		taskcontrolblock_t * TCB = &myTCB[prio];
		assert(TCB != 0);

		task_event_t event = *TCB->EQ_Tail;

		/* Get next event from queue. */
		if((++TCB->EQ_Tail) == TCB->EQ_Buff + TCB->EQ_Size)
			TCB->EQ_Tail = TCB->EQ_Buff;

		/* Clear pending flag if event queue is empty. */
		if((--TCB->EQ_Load) == (size_t)0)
			myPendingFlags &= (uint8_t)(~(1U << prio));

		IRQ_UNLOCK();

#if PROFILING
		OnTaskStart(prio);
#endif

		/* Execute the taks. */
		status = TCB->Task(event);

#if PROFILING
		OnTaskFinished(prio, status);
#endif

		if(status < STATUS_OK)
			error_log("Scheduler: task (name: \"%s\" - prio %d) execution failed, error code: %d", TCB->Name, prio, status);

		/* Check the tasks status. */
		if(status == STATUS_TASK_POSTPONE)				/* Task needs to be postponed. */
		{
			Scheduler_PostEvent(prio, event);			/* repost the current task */
			myMaskingFlags |= (uint8_t)(1U << prio); 	/* mask current task */
			status = ScheduleNext();					/* execute pending lower priority tasks */
			myMaskingFlags &= (uint8_t)(~(1U << prio));	/* unmask current task */
		}
	}
	else // idle
	{
		myIdleCounter++;
	}
	return status;
}



#if PROFILING
__attribute__((weak)) void OnTaskStart(uint32_t priority)
{
	(void)priority;
}
__attribute__((weak)) void OnTaskFinished(uint32_t priority, status_t status)
{
	(void)priority;
	(void)status;
}
__attribute__((weak)) void OnTaskQueued(uint32_t priority)
{
	(void)priority;
}
#endif
