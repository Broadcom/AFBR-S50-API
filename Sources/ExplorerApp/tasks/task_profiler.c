/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer example application.
 * @details     A utility module that measures execution times of tasks.
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
#include "task_profiler.h"

#if PROFILING

#include "task_scheduler.h"

#include <assert.h>
#include <stdint.h>
#include "task_status.h"
#include "debug.h"
#include "utility/time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LOG_COUNT 254

/*!***************************************************************************
 * @brief Task profiler information definition.
 *****************************************************************************/
typedef struct taskprofileinfo_t
{
    uint32_t ExecutionCount;
    uint32_t FailedExecutionCount;
    ltc_t ExecutionTime;
    ltc_t LastStartTimeStamp;
} taskprofileinfo_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void OnTaskStart(uint32_t priority);
void OnTaskFinished(uint32_t priority, status_t status);
void OnTaskQueued(uint32_t priority);

static void ProfilerLog(char type, char id);

/******************************************************************************
 * Variables
 ******************************************************************************/
static taskprofileinfo_t myTPI[SCHEDULER_MAX_TASKS] = {{0}};
static char myLogStr[LOG_COUNT+2] = {0};
static char * myLogPtr = myLogStr;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void ProfilerLog(char type, char id)
{
//  *(myLogPtr++) = type;
//  *(myLogPtr++) = id;
//  if(myLogPtr - myLogStr > LOG_COUNT)
//  {
//      print(myLogStr);
//      myLogPtr = myLogStr;
//  }
}

void OnTaskStart(uint32_t priority)
{
    ProfilerLog('S', (char)('a' + priority));
    Time_GetNow(&(myTPI[priority].LastStartTimeStamp));
}
void OnTaskFinished(uint32_t priority, status_t status)
{
    if(status == STATUS_OK)
    {
        ltc_t t = {0};
        Time_GetElapsed(&t, &(myTPI[priority].LastStartTimeStamp));
        Time_Add(&(myTPI[priority].ExecutionTime), &(myTPI[priority].ExecutionTime), &t);
        myTPI[priority].ExecutionCount++;
        ProfilerLog('P', (char)('a' + priority));
    }
    else
    {
        myTPI[priority].FailedExecutionCount++;
        ProfilerLog('F', (char)('a' + priority));
    }
}
void OnTaskQueued(uint32_t priority)
{
    ProfilerLog('Q', (char)('a' + priority));
    if(myLogPtr - myLogStr > LOG_COUNT)
    {
        print(myLogStr);
        myLogPtr = myLogStr;
    }
}



#endif /* PROFILING */
