/*************************************************************************//**
 * @file
 * @brief       SCI: Command definitions for log messages in printf style.
 * @details     This file provides an implementation for log messages/commands
 *              in the printf style of the C standard library that can be used
 *              along with the systems communication interface.
 * @details     This file provides command definitions for the systems
 *              communication interface.
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
#include "sci_cmd.h"
#include "sci_frame.h"

#include <stdarg.h>

#include "printf/printf.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief   A `vprintf()` function for the uart connection.
 * @details This function is similar to #print except that, instead of
 *          taking a variable number of arguments directly, it takes an
 *          argument list pointer `ap`. Requires a call to `va_start(ap, fmt_s);`
 *          before and `va_end(ap);` after this function.
 * @param   fmt_s The printf() format string.
 * @param   ap The argument list.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static inline status_t vprint(const char *fmt_s, va_list ap);

/*! @cond */
#if SCI_LOG_TIMESTAMP
#include "utility/time.h"
#include "utility/fp_rnd.h"
//void Time_GetNow(ltc_t * t_now) __attribute__((weak));
//uint32_t Time_GetSec(ltc_t const * t) __attribute__((weak));
//uint32_t Time_GetUSec(ltc_t const * t) __attribute__((weak));
#endif

extern status_t SCI_DataLink_SendTxFrame(sci_frame_t * frame, bool high_priority);
extern sci_frame_t * SCI_DataLink_RequestTxFrame(bool queueStartByte);
/*! @endcond */

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/



status_t SCI_SendLogEntry(const char *fmt_s, ...)
{
    va_list  ap;
    va_start(ap, fmt_s);
    status_t status = vprint(fmt_s, ap);
    va_end(ap);
    return status;
}


status_t print(const char  *fmt_s, ...)
{
    va_list  ap;
    va_start(ap, fmt_s);
    status_t status = vprint(fmt_s, ap);
    va_end(ap);
    return status;
}
static inline status_t vprint(const char *fmt_s, va_list ap)
{
    /* sending a log message in formated printf style */

    sci_frame_t * frame = SCI_DataLink_RequestTxFrame(true);
    if(!frame) return ERROR_SCI_BUFFER_FULL;

    SCI_Frame_Queue08u(frame, CMD_LOG_MESSAGE);

#if SCI_LOG_TIMESTAMP
    ltc_t t_now;
    Time_GetNow(&t_now);
    SCI_Frame_Queue_Time(frame, &t_now);
#endif

    int len = vfctprintf(SCI_Frame_PutChar, frame, fmt_s, ap);
    if (len < 0) return ERROR_FAIL;

    return SCI_DataLink_SendTxFrame(frame, false);
}

///*! @cond */
//#if SCI_LOG_TIMESTAMP
//void Time_GetNow(ltc_t * t_now) { t_now->sec = 0; t_now->usec = 0; }
//uint32_t Time_GetSec(ltc_t const * t) { (void)t;  return 0; }
//uint32_t Time_GetUSec(ltc_t const * t) { (void)t; return 0; }
//#endif
///*! @endcond */
