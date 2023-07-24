/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer example application.
 * @details     This file implements an simple task scheduler.
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
#include "task_scheduler.h"
#include "task_profiler.h"

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "debug.h"
#include "driver/irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!***************************************************************************
 * @brief Task control block definition.
 *****************************************************************************/
typedef struct taskcontrolblock_t
{
    task_function_t  Task;      /*!< Function to execute. */
    task_event_t    *EQ_Buff;   /*!< Event queue buffer. */
    task_event_t    *EQ_Head;   /*!< Head of the queue. */
    task_event_t    *EQ_Tail;   /*!< Tail of the queue. */
    size_t           EQ_Size;   /*!< Total buffer size of the queue. */
    size_t           EQ_Load;   /*!< Currently used buffer size. */
    char const      *Name;      /*!< Task descriptive name. */
} taskcontrolblock_t;

typedef struct scheduler_t
{
    volatile uint32_t PendingFlags;
    volatile uint32_t MaskingFlags;
    volatile task_prio_t CurrentTask;

    taskcontrolblock_t TCB[SCHEDULER_MAX_TASKS];

} scheduler_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static inline void ScheduleNext(scheduler_t * const me);

#if PROFILING
void OnTaskStart(uint32_t priority);
void OnTaskFinished(uint32_t priority, status_t status);
void OnTaskQueued(uint32_t priority);
#endif

/******************************************************************************
 * Variables
 ******************************************************************************/

static const uint8_t myLog2Lookup[] = { 0xff, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };

/*******************************************************************************
 * Code
 ******************************************************************************/
scheduler_t * Scheduler_Init(void)
{
    // static instance of the scheduler; might be replaced by malloc.
    static scheduler_t me;
    memset(&me, 0, sizeof(scheduler_t));
    return &me;
}

status_t Scheduler_AddTask(scheduler_t * const me,
                           task_function_t task,
                           task_prio_t priority,
                           task_event_t eventQ,
                           size_t eventQSize,
                           const char * name)
{
    assert(me != NULL);
    if (!task) return ERROR_INVALID_ARGUMENT;
    if (!eventQ) return ERROR_INVALID_ARGUMENT;
    if (!(eventQSize > 0)) return ERROR_INVALID_ARGUMENT;
    if (!(priority < SCHEDULER_MAX_TASKS)) return ERROR_INVALID_ARGUMENT;

    taskcontrolblock_t * tcb = &me->TCB[priority];

    if (tcb->Task != 0) return ERROR_INVALID_ARGUMENT; // check if task isn't used right now

    tcb->Task = task;
    tcb->EQ_Buff = eventQ;
    tcb->EQ_Head = eventQ;
    tcb->EQ_Tail = eventQ;
    tcb->EQ_Size = eventQSize;
    tcb->EQ_Load = 0;
    tcb->Name = name;

    return STATUS_OK;
}

status_t Scheduler_PostEvent(scheduler_t * const me,
                             task_prio_t priority,
                             task_event_t event)
{
    assert(me != NULL);
    assert(event != NULL);

    if (!(priority < SCHEDULER_MAX_TASKS)) return ERROR_INVALID_ARGUMENT;

    taskcontrolblock_t * tcb = &me->TCB[priority];
    if (tcb->Task == 0) return ERROR_NOT_INITIALIZED;

    IRQ_LOCK();
    if (tcb->EQ_Load < tcb->EQ_Size) // check if queue is not full
    {
        *tcb->EQ_Head = event;
        if ((++tcb->EQ_Head) == tcb->EQ_Buff + tcb->EQ_Size) tcb->EQ_Head = tcb->EQ_Buff;
        if ((++tcb->EQ_Load) == (uint32_t)1U) me->PendingFlags |= (1U << priority);
        IRQ_UNLOCK();

#if PROFILING
        OnTaskQueued(priority);
#endif
    }
    else
    {
        IRQ_UNLOCK();
        return ERROR_TASK_QUEUE_FULL;
    }

    return STATUS_OK;
}

bool Scheduler_IsTaskPending(scheduler_t * const me, task_prio_t priority)
{
    assert(me != NULL);
    return !((me->PendingFlags & (1U << priority)) == 0);
}

void Scheduler_Run(scheduler_t * const me)
{
    assert(me != NULL);
    for (;;)
    {
        ScheduleNext(me);
    }
}

void Scheduler_SwitchContext(scheduler_t * const me)
{
    assert(me != NULL);
    task_prio_t prio = me->CurrentTask;
    me->MaskingFlags |= (uint32_t)(1U << prio); /* mask current task */
    ScheduleNext(me); /* execute pending lower priority tasks */
    me->MaskingFlags &= (uint32_t)(~(1U << prio)); /* unmask current task */
    me->CurrentTask = prio;
}

static inline void ScheduleNext(scheduler_t * const me)
{
    assert(me != NULL);
    uint32_t pending = (uint32_t)(me->PendingFlags & ~(me->MaskingFlags));

    if (pending)
    {
        /* Get highest pending and not masked priority. */
        uint8_t prio = 0;
        while (pending & ~0xFU)
        {
            pending >>= 4U;
            prio += 4U;
        }
        prio += myLog2Lookup[pending];
        assert(prio < SCHEDULER_MAX_TASKS);

        taskcontrolblock_t * tcb = &me->TCB[prio];
        assert(tcb->Task != 0);

        task_event_t event = *tcb->EQ_Tail;

        /* Get next event from queue. */
        if ((++tcb->EQ_Tail) == tcb->EQ_Buff + tcb->EQ_Size)
            tcb->EQ_Tail = tcb->EQ_Buff;

        IRQ_LOCK();

        /* Clear pending flag if event queue is empty. */
        if ((--tcb->EQ_Load) == (size_t)0)
            me->PendingFlags &= (uint32_t)(~(1U << prio));

        me->CurrentTask = prio;

        IRQ_UNLOCK();

#if PROFILING
        OnTaskStart(prio);
#endif

        /* Execute the task. */
        tcb->Task(event);

#if PROFILING
        OnTaskFinished(prio, status);
#endif
    }
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
