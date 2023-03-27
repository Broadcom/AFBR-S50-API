/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the main functionality of the AFBR-S50
 *              Explorer Application.
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

#ifndef EXPLORER_APP_H
#define EXPLORER_APP_H

/*!***************************************************************************
 * @defgroup    explorer_main AFBR-S50 Explorer Application
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer Application
 * @details     An example application that utilizes the Argus API.
 *
 *              A simple task scheduler is used to host the Argus API and a
 *              simple systems communication interface is implemented to
 *              connect to the AFBR-S50 Explorer GUI. The latter is an evaluation
 *              software for the Argus time-of-flight devices.
 *
 * @addtogroup  explorer_main
 * @{
 *****************************************************************************/

#include "core/explorer_types.h"
#include "api/argus_status.h"
#include "argus.h"
#include <stdbool.h>

 /*!***************************************************************************
 * @brief   Initializes all the state machine.
 * @details This function does initialization of the state machine.
 *          - Initialization of the task scheduler.
 *          - Adding all task to the scheduler.
 *          - Set internal data to a known state.
 *          - Hardware driver initialization.
 *          - Send software information via serial interface.
 *          .
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_Init(void);

/*!***************************************************************************
 * @brief   This runs the state machine.
 * @details It runs the task scheduler and never return.
 *****************************************************************************/
void ExplorerApp_Run(void);


/*! @} */
#endif /* EXPLORER_APP_H */
