/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides driver functionality for PIT (periodic interrupt timer).
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
#include "timer.h"

#include "board/board_config.h"
#include "driver/fsl_clock.h"
#include "driver/irq.h"
#include "debug.h"

#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! Disables the Argus API PIT implementation. */
#ifndef DISABLE_PIT
#define DISABLE_PIT 0
#endif

/*! PIT clock source frequency; PIT is used as lifetime timer. */
#define PIT_Freq            24000000U

/*! Number of clock cycles per microsecond for PIT; PIT is used as lifetime timer. */
#define PIT_ClocksPerUSec   24U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! The system timer exception handler. */
void SysTick_Handler(void);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*! External definition of the system timer core clock. */
extern uint32_t SystemCoreClock;

#if !DISABLE_PIT

/*! A pointer to the ISR function to be called when the timer has elapsed. */
static timer_cb_t myISR = 0;

/*! The parameter to be passed to the callback. */
static void * myParam = 0;

#endif

#ifdef DEBUG
static volatile bool isInitialized = false;
#endif


/*******************************************************************************
 * Code
 *******************************************************************************/
void Timer_Init(void)
{
    assert(!isInitialized);

    /************************************************
     ***  Initialize the PIT as lifetime counter. ***
     ************************************************/

    /* Un-gate pit clock*/
    CLOCK_EnableClock(kCLOCK_Pit0);

    /* Enable PIT module clock*/
    PIT->MCR &= ~PIT_MCR_MDIS_MASK;

    /* Set timer to stop in debug mode*/
    PIT->MCR |= PIT_MCR_FRZ_MASK;

    /* Check the pit source clock frequency. */
    while (PIT_Freq != CLOCK_GetFreq(kCLOCK_BusClk)) assert(0);

    // Initialize PIT timer instance for channel 0 and 1
    PIT->CHANNEL[1].TCTRL = 0x0;                    // Disable timer 1 interrupts
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_CHN_MASK;    // Set chain mode
    PIT->CHANNEL[1].LDVAL = 0xffffffff;             // Timer 1 counts sec
    PIT->CHANNEL[0].TCTRL = 0x0;                    // Disable timer 0 interrupts
    PIT->CHANNEL[0].LDVAL = (PIT_Freq - 1U);        // Timer 0 counts us/24

    // Start timer channels
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;

#if !DISABLE_PIT
    /******************************************************************
     ***  Initialize the SysTick timer as periodic interrupt timer. ***
     ******************************************************************/

    myISR = 0;
    myParam = 0;

    /* Reset reload register and stop timer. */
    SysTick->LOAD = 0;

    /* Load the SysTick Counter Value */
    SysTick->VAL = 0;

    /* Setup but do not start yet. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk;

    /* Set Priority. */
    NVIC_SetPriority(SysTick_IRQn, IRQPRIO_SYSTICK);
#endif

#ifdef DEBUG
    isInitialized = true;
#endif
}

void Timer_GetCounterValue(uint32_t * hct, uint32_t * lct)
{
    assert(isInitialized);
    assert(hct != 0);
    assert(lct != 0);

    /* PIT Issue Workaround:
     *
     * Source: https://www.nxp.com/docs/en/errata/IMXRT1050CE.pdf
     *
     * ERR050130 PIT: Temporary incorrect value reported in LMTR64H register in
     * lifetimer mode
     *
     * Description:
     *
     * When the Programmable interrupt timer (PIT) module is used in lifetimer
     * mode, timer 0 and timer 1 are chained and the timer load start value
     * (LDVAL0[TSV] and LDVAL1[TSV]) are set according to the application need
     * for both timers. When timer 0 current time value (CVAL0[TVL]) reaches
     * 0x0 and subsequently reloads to LDVAL0[TSV], then timer 1 CVAL1[TVL]
     * should decrement by 0x1.
     *
     * However this decrement does not occur until one cycle later, therefore a
     * read of the PIT upper lifetime timer register (LTMR64H) is followed by a
     * read of the PIT lower lifetime timer register (LTMR64L) at the instant
     * when timer 0 has reloaded to LDVAL0[TSV] and timer 1 is yet to be
     * decremented in next cycle then an incorrect timer value in LTMR64H[LTH]
     * is expected.
     *
     * Workarounds:
     * In lifetimer mode, if the read value of LTMR64L[LTL] is equal to LDVAL0[TSV],
     * then read both LTMR64H and LTMR64L registers for one additional time to
     * obtain the correct lifetime value. */

    uint32_t LTMR64H, LTMR64L;
    do
    {
        IRQ_LOCK();
        /* To use LTMR64H and LTMR64L, timer 0 and timer 1 need to be chained. To obtain the
         * correct value, first read LTMR64H and then LTMR64L. LTMR64H will have the value
         * of CVAL1 at the time of the first access, LTMR64L will have the value of CVAL0 at the
         * time of the first access, therefore the application does not need to worry about carry-over
         * effects of the running counter. */
        LTMR64H = ~(PIT->LTMR64H);
        LTMR64L = (PIT_Freq - 1U) - PIT->LTMR64L;
        IRQ_UNLOCK();
    }
    while (LTMR64L == 0);

    *hct = LTMR64H;
    /* Approx. Division by 24;
     *
     * Idea: Use division by factor of 2 and approximate the
     * remaining 1/3 factor by multiplication
     *
     * Error Range: -37 <= x <= 0 µsec
     * Values are always a little smaller than the actual values. */
    *lct = ((LTMR64L >> 9U) * 21845U) >> 10U;
}

#if !DISABLE_PIT
status_t Timer_SetCallback(timer_cb_t f)
{
    assert(isInitialized);

    if (f == 0)
    {
        Timer_SetInterval(0, myParam);
    }

    myISR = f;
    return STATUS_OK;
}

status_t Timer_SetInterval(uint32_t dt_microseconds, void * param)
{
    assert(isInitialized);
    assert(!(dt_microseconds != 0 && myISR == 0)); // Timer must not be enabled without IRS

    if (dt_microseconds)
    {
        const uint32_t ticks = SystemCoreClock / 1000000U;
        assert(dt_microseconds > 100); // check reasonable minimum interval
        assert(dt_microseconds < (SysTick_VAL_CURRENT_Msk + 1U) / ticks); // check maximum interval

        IRQ_LOCK();
        myParam = param;

        SysTick->VAL = 0;
        SysTick->LOAD = (uint32_t) ((ticks * dt_microseconds) - 1);
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        IRQ_UNLOCK();
    }
    else
    {
        IRQ_LOCK();
        myParam = 0;

        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        IRQ_UNLOCK();
    }
    return STATUS_OK;
}

void SysTick_Handler(void)
{
    if (myISR) myISR(myParam);
}
#endif
