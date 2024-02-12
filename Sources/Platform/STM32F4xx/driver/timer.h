/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
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

#ifndef TIMER_H
#define TIMER_H

/*!***************************************************************************
 * @defgroup    timer Timer Module
 * @ingroup     driver
 * @brief       Timer Hardware Driver Module
 * @details     Provides driver functionality for the timer peripherals.
 *              This module actually implements the #argus_timer interface that
 *              is required for the Argus API. It contains two timing
 *              functionalities: A periodic interrupt/callback timer and
 *              an lifetime counter.
 *
 *              Note that the current implementation only features a single
 *              callback timer interval and does not yet support the feature
 *              of multiple intervals at a time.
 *
 * @addtogroup  timer
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "platform/argus_timer.h"

/*! Enable the PIT timer implementation.
 *  - 0: The PIT timer implementation is disabled. The dummy functions
 *       in the AFBR-S50 API are used.
 *  - 1: The PIT timer implementation is enabled. */
#ifndef TIMER_PIT_ENABLED
#define TIMER_PIT_ENABLED 1
#endif

/*!***************************************************************************
 * @brief   Initializes the timer hardware.
 *****************************************************************************/
void Timer_Init(void);


/*! @} */
#endif /* TIMER_H */
