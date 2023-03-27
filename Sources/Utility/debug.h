/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides debug functionality.
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
 *
 *****************************************************************************/

#ifndef DEBUG_H
#define DEBUG_H

/*!***************************************************************************
 * @defgroup    debug Debug Utility
 * @ingroup     platform
 * @brief       Debug Utility Module
 * @details     This module provides debugging utility functions such as
 *              logging, print, asserts and breakpoints.
 * @addtogroup  debug
 * @{
 *****************************************************************************/

#include "platform/argus_print.h"

/*!***************************************************************************
 * @brief   Error logging function.
 *
 * @details Redirected to print()
 *
 * @param   fmt format string (prinft style)
 * @param   ... parameters
 *****************************************************************************/
#define error_log(fmt, ...) print("ERROR: " fmt "\n", ##__VA_ARGS__)

/*******************************************************************************
 * Debug Utility
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Software breakpoint.
 * @details Stops the debugger at the corresponding line of code.
 *          Only active in debug configuration.
 *****************************************************************************/
#ifdef NDEBUG           /* required by ANSI standard */
#define BREAKPOINT() ((void)0)
#else
#define BREAKPOINT() __asm__ __volatile__ ("bkpt #0")
#endif

/*!***************************************************************************
 * @brief   Conditional software breakpoint.
 * @details Stops the debugger at the corresponding line of code if the
 *          condition is met.
 *          Only active in debug configuration.
 * @param   x The condition to be fulfilled (boolean value).
 *****************************************************************************/
#ifdef NDEBUG           /* required by ANSI standard */
#define BREAKPOINT_IF(x) ((void)0)
#else
#define BREAKPOINT_IF(x) if(x) { BREAKPOINT(); }
#endif

/*!***************************************************************************
 * @brief   Assert function definition.
 *
 * @details Called by the standard library.
 *
 * @param   file
 * @param   line
 * @param   func
 * @param   failedExpr
 *****************************************************************************/
void __assert_func(const char *file, int line, const char *func, const char *failedExpr);


/*!***************************************************************************
 * @brief   Returns the maximum stack usage since the last call.
 * @details Sets an pattern to the RAM/Stack memory and returns the size where
 *          the pattern has been overwritten since the last call.
 * @return  Returns the maximum stack size since the last call.
 *****************************************************************************/
uint32_t Debug_GetStackUsage(void);

/*!***************************************************************************
 * @brief   Resets the unused stack memory to a given pattern.
 * @details Reset the unused stack memory in RAM for debugging purposes.
 *          The memory from bottom of the stack until the current stack pointer
 *          is cleared to a 0xAAA... pattern.
 *****************************************************************************/
void Debug_ResetStackUsage(void);


/*! @} */
#endif /* DEBUG_H */
