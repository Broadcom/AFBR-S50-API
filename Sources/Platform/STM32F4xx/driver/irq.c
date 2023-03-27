/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details
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

#include "irq.h"
#include "main.h"
#include <assert.h>

/*! Global lock level counter value. */
static volatile int g_irq_lock_ct;


/*!***************************************************************************
 * @brief   Enable IRQ Interrupts
 *
 * @details Enables IRQ interrupts by clearing the I-bit in the CPSR.
 *          Can only be executed in Privileged modes.
 *
 * @return  -
 *****************************************************************************/
void IRQ_UNLOCK(void)
{
    assert(g_irq_lock_ct > 0);
    if (--g_irq_lock_ct <= 0)
    {
        g_irq_lock_ct = 0;
        __enable_irq();
    }
}


/*!***************************************************************************
 * @brief   Disable IRQ Interrupts
 *
 * @details Disables IRQ interrupts by setting the I-bit in the CPSR.
 *          Can only be executed in Privileged modes.
 *
 * @return  -
 *****************************************************************************/
void IRQ_LOCK(void)
{
    __disable_irq();
    ++g_irq_lock_ct;
}
