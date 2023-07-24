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
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "debug.h"
#include "board/board.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/gpio.h"
#include "driver/MKL46Z/slcd.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! Definitions of stack top/bottom addresses from the linker file. */
#if defined(BSP_MCU_GROUP_RA4M2) && (BSP_MCU_GROUP_RA4M2 == 1)
// The RA4M2 has a different naming scheme for the stack boundaries
extern uint32_t __StackTop;
extern uint32_t __StackLimit;

#define STACK_TOP   __StackTop
#define STACK_BASE  __StackLimit
#else
extern int _vStackTop;
extern int _vStackBase;

#define STACK_TOP   _vStackTop
#define STACK_BASE  _vStackBase
#endif

/*! The pattern to be filled to unused stack space. */
#define STACK_PATTERN 0xAAAAAAAAU

uint32_t Debug_GetStackUsage(void)
{
    uint32_t size = 0;
    uint32_t * top = (uint32_t*)&STACK_TOP;
    uint32_t * base = (uint32_t*)&STACK_BASE;

    for (const uint32_t * p = base; p < top; p++)
    {
        if (*p != STACK_PATTERN)
        {
            size = (uint32_t)top - (uint32_t)p;
            break;
        }
    }

    /* reset for next readout */
    Debug_ResetStackUsage();

    return size;
}

void Debug_ResetStackUsage(void)
{
    /* Reset the unused stack memory in RAM for debugging purposes.
     * The memory from top of the stack until the current stack pointer
     *  is cleared to a 0xAAA... pattern. */

    uint32_t * sp = 0;
    __asm__ __volatile__ ("mov %0, sp" : "=r"(sp));
    uint32_t * base = (uint32_t*)&STACK_BASE;

    while (--sp >= base)
    {
        *sp = STACK_PATTERN;
    }
}

__attribute__((naked)) // avoid stack usage (no return from assert!)
void __assert_func(const char *file, int line, const char *func, const char *failedExpr)
{
    static bool is_asserting = false;
    if (!is_asserting)
    {
        BREAKPOINT();
        is_asserting = true;

        /* ***************************************************************** *
         * WARNING: In case of stack overflow (and stack is at beginning of  *
         * RAM), the following will fail an reset the device immediately     *
         * (system core lockup).                                             *
         * ***************************************************************** */

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        GPIO_ClearPinOutput(Pin_LED_RED);
        /* Display error on LCD. */
        SLCD_DisplayError(0xAA);
#endif

        /* Try to send message to master.
         * Note that sending via USB fails if the assert is called e.g. from
         * the HardFault handler. */
        print("ASSERT: expression \"%s\" failed;\n"
              "file \"%s\";\n"
              "line \"%d\";\n"
              "function \"%s\";\n",
              failedExpr, file, line, func);

        /* Wait for sending print statement */
        static volatile uint32_t i = 0;
        for (i = 0; i < 1000000; i++) __asm("nop");
    }

    /* Stop or Reset. */
    BREAKPOINT();
    Board_Reset();
    for (;;) __asm("nop");
}
