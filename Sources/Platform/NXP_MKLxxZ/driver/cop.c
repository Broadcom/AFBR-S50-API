/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides watchdog driver functionality.
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


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "cop.h"

#include "board/board_config.h"
#include "driver/irq.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (DISABLE_WDOG) /* Watchdog disabled warning. The startup code will disable COP by default, if not DISABLE_WDOG=0. */
#warning "COP Watchdog is disabled! Compile with '-DDISABLE_WDOG=0' in order to enable it."
#endif

#define COP SIM /*!< Register alias. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef DEBUG
static volatile bool isInitialized = false;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void COP_Init(void)
{
    assert(!isInitialized); // COP can only be initialized/disabled once!

    COP->COPC = SIM_COPC_COPW(0)        /*!< Set COP watchdog run mode to Normal mode (not Window mode) */
              | SIM_COPC_COPCLKS(0)     /*!< LPO clock, 1kHZ. */
              | SIM_COPC_COPT(3);       /*!< 2 to 10 clock cycles when clock source is LPO or in short timeout mode otherwise 2 to 18 clock cycles. */
#ifdef DEBUG
    isInitialized = true;
#endif
}

void COP_Disable(void)
{
    assert(!isInitialized); // COP can only be initialized/disabled once!

    COP->COPC &= ~SIM_COPC_COPT_MASK;

#ifdef DEBUG
    isInitialized = true;
#endif
}

void COP_Refresh(void)
{
    assert(isInitialized);

    /* Disable the global interrupt to protect refresh sequence */
    IRQ_LOCK();
    COP->SRVCOP = 0x55U;    /*!< First byte of refresh sequence */
    COP->SRVCOP = 0xAAU;    /*!< Second byte of refresh sequence */
    IRQ_UNLOCK();
}

void COP_ResetSystem(void)
{
    NVIC_SystemReset();
}
