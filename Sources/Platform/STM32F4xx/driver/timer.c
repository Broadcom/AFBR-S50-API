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

#include "timer.h"
#include "tim.h"
#include <assert.h>


/*!***************************************************************************
 * @brief   Initializes the timer hardware.
 * @return  -
 *****************************************************************************/
void Timer_Init(void)
{
    /* Initialize the timers, see generated main.c */
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    /* Start the timers relevant for the LTC */
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim5);

    __HAL_DBGMCU_FREEZE_TIM2();
    __HAL_DBGMCU_FREEZE_TIM4();
    __HAL_DBGMCU_FREEZE_TIM5();
}

#define DEBUG_TIMER 0
#if DEBUG_TIMER

/* For debugging: Increment counter only once */

void Timer_GetCounterValue(uint32_t * hct, uint32_t * lct)
{
    static uint32_t cnt = 0;
    cnt += 1;
    *lct = cnt % 1000000U;
    *hct = cnt / 1000000U;
}

#else

/*!***************************************************************************
 * @brief   Obtains the lifetime counter value from the timers.
 *
 * @details The function is required to get the current time relative to any
 *          point in time, e.g. the startup time. The returned values \p hct and
 *          \p lct are given in seconds and microseconds respectively. The current
 *          elapsed time since the reference time is then calculated from:
 *
 *          t_now [µsec] = hct * 1000000 µsec + lct * 1 µsec
 *
 * @param   hct A pointer to the high counter value bits representing current
 *                time in seconds.
 * @param   lct A pointer to the low counter value bits representing current
 *                time in microseconds. Range: 0, .., 999999 µsec
 * @return  -
 *****************************************************************************/
void Timer_GetCounterValue(uint32_t * hct, uint32_t * lct)
{
    /* The loop makes sure that there are no glitches
     * when the counter wraps between htim2 and htim5 reads. */
    do {
        *lct = __HAL_TIM_GET_COUNTER(&htim2);
        *hct = __HAL_TIM_GET_COUNTER(&htim5);
    }
    while (*lct > __HAL_TIM_GET_COUNTER(&htim2));
}

#endif

#if TIMER_PIT_ENABLED
/*! Storage for the callback parameter */
static void * callback_param_;

/*! Callback function for PIT timer */
static timer_cb_t timer_callback_;


/*!***************************************************************************
 * @brief   Sets the timer interval for a specified callback parameter.
 * @details Sets the callback interval for the specified parameter and starts
 *          the timer with a new interval. If there is already an interval with
 *          the given parameter, the timer is restarted with the given interval.
 *          If the same time interval as already set is passed, nothing happens.
 *          Passing a interval of 0 disables the timer.
 * @param   dt_microseconds The callback interval in microseconds.
 * @param   param An abstract parameter to be passed to the callback. This is
 *                  also the identifier of the given interval.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Timer_SetInterval(uint32_t dt_microseconds, void * param)
{
    assert(dt_microseconds == 0 || dt_microseconds > 100);

    /* Disable interrupt and timer */
    callback_param_ = 0;
    HAL_TIM_Base_Stop_IT(&htim4);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (dt_microseconds)
    {
        /* Determine the prescaler and counter period values such that
         * the period fits into 16-bits. */
        uint32_t prescaler = SystemCoreClock / 1000000U;
        uint32_t period = dt_microseconds;

        while (period > 0xFFFF)
        {
            period >>= 1U;
            prescaler <<= 1U;
        }

        assert(prescaler < 0x10000U);

        /* Set prescaler and period values and reset counter. */
        __HAL_TIM_SET_PRESCALER(&htim4, prescaler - 1);
        __HAL_TIM_SET_AUTORELOAD(&htim4, period - 1);
        __HAL_TIM_SET_COUNTER(&htim4, period - 1);

        /* The following generates an update event that triggers and update
         * of the auto-reload into the internal shadow registers. This is
         * required to update the timer configuration before the next update
         * event (i.e. under/overflow). Unfortunately this also generates
         * and immediate interrupt which is cleared in the next statement. */
        HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE); // clear interrupt

        /* Enable interrupt and timer */
        callback_param_ = param;
        HAL_TIM_Base_Start_IT(&htim4);
    }

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Installs an periodic timer callback function.
 * @details Installs an periodic timer callback function that is invoked whenever
 *          an interval elapses. The callback is the same for any interval,
 *          however, the single intervals can be identified by the passed
 *          parameter.
 *          Passing a zero-pointer removes and disables the callback.
 * @param   f The timer callback function.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Timer_SetCallback(timer_cb_t f)
{
    timer_callback_ = f;
    return STATUS_OK;
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Trigger callback if the interrupt belongs to TIM4 and there is a callback */
    if (htim == &htim4 && timer_callback_)
    {
        timer_callback_(callback_param_);
    }
}

#endif
