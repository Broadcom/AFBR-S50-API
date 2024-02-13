/*************************************************************************//**
 * @file
 * @brief       This file is part of the RA4M2 platform layer.
 * @details     This file provides driver functionality for PIT (periodic interrupt timer).
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
 *
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "driver/timer.h"
#include "driver/irq.h"
#include "bsp_api.h"
#include "hal_data.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! A pointer to the ISR function to be called when the timer has elapsed. */
static timer_cb_t myISR = 0;

/*! The parameter to be passed to the callback. */
static void * myParam = 0;

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
     ***  Initialize timer 0 as lifetime counter. ***
     ************************************************/
    fsp_err_t err = R_GPT_Open(&g_ltc_ctrl, &g_ltc_cfg);
    assert(err == FSP_SUCCESS);

    err = R_GPT_Start(&g_ltc_ctrl);
    assert(err == FSP_SUCCESS);

    /********************************************************
     ***  Initialize timer 1 as periodic interrupt timer. ***
     ********************************************************/
    err = R_GPT_Open(&g_pit_ctrl, &g_pit_cfg);
    assert(err == FSP_SUCCESS);

#if AFBR_SCI_USB
    /********************************************************
     ***  Initialize timer 2 as USB polling timer.        ***
     ********************************************************/
    err = R_GPT_Open(&g_upt_ctrl, &g_upt_cfg);
    assert(err == FSP_SUCCESS);

    err = R_GPT_Start(&g_upt_ctrl);
    assert(err == FSP_SUCCESS);
#endif

#ifdef DEBUG
    isInitialized = true;
#else
    /* Avoid compiler warning about unused variable */
    (void)err;
#endif
}


void Timer_GetCounterValue(uint32_t * hct, uint32_t * lct)
{
    assert(isInitialized);

    /* Note: the current timer setting wraps after exactly 4000 seconds.
     * In order to handle the wrap around (without chaining another timer)
     * a static variable is used to check for wrap around: if the new timer
     * value is smaller than the previously one, the timer must have wrapped
     * around.
     * Note that this requires that the function is called at least once
     * every 4000 seconds which shouldn't be of any concern. */
    static uint32_t prev_cnt = 0; // last counter read value
    static uint32_t offset_sec = 0; // offset in seconds (4000 per wrap around)

    IRQ_LOCK();
    const uint32_t cnt = g_ltc_ctrl.p_reg->GTCNT;
    if (cnt < prev_cnt) offset_sec += 4000;
    prev_cnt = cnt;

    *lct = (uint32_t)((float)(cnt % 390625U) * 2.56f);
    *hct = cnt / 390625U + offset_sec;

    IRQ_UNLOCK();
}

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
        /* GPT is clocked by PCLKD. */
        uint32_t pclkd = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
        uint64_t ticks = ((uint64_t)dt_microseconds * (uint64_t)pclkd) / 1000000;
        assert((ticks > 0) && (ticks < UINT32_MAX) && (g_pit_ctrl.variant == TIMER_VARIANT_32_BIT));

        IRQ_LOCK();
        myParam = param;

        fsp_err_t err = R_GPT_PeriodSet(&g_pit_ctrl, (uint32_t)ticks);
        if (FSP_SUCCESS != err)
        {
            IRQ_UNLOCK();
            return ERROR_FAIL;
        }

        /* Starts GPT timer0 */
        err = R_GPT_Start(&g_pit_ctrl);
        if (FSP_SUCCESS != err)
        {
            IRQ_UNLOCK();
            return ERROR_FAIL;
        }

        IRQ_UNLOCK();
    }
    else
    {
        IRQ_LOCK();
        myParam = 0;

        fsp_err_t err = R_GPT_Stop(&g_pit_ctrl);
        if (FSP_SUCCESS != err)
        {
            IRQ_UNLOCK();
            return ERROR_FAIL;
        }
        IRQ_UNLOCK();
    }
    return STATUS_OK;
}

void user_timer1_callback(timer_callback_args_t * p_args)
{
    (void)p_args; //unused
    if (myISR) myISR(myParam);
}
