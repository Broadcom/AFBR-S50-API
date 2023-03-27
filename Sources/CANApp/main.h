/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Demo Application
 *
 * @copyright
 *
 * Copyright (c) 2022, Broadcom Inc
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

#ifndef MAIN_H_
#define MAIN_H_

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus.h"
#include "debug.h"

#if !defined(_RENESAS_RA_)
#error No target specified or target not supported!
#endif

/*!***************************************************************************
 * @defgroup    can_app CAN Application
 * @ingroup     demo_apps
 *
 * @brief       AFBR-S50 Reference Board CAN-bus example application.
 *
 * @details     This is the CAN interface example application that can be run
 *              in the \ref reference_board by **MikroElektronika** based on
 *              the **Renesas RA4M2** micro controller.
 *
 *              The application provides a simple CAN interface as an reference
 *              for the customers own implementation.
 *
 *              See the following documentation for more information:
 *              - @ref can_app
 *              - @ref reference_board
 *
 * @addtogroup  can_app
 * @{
 *****************************************************************************/


/*!***************************************************************************
 * @brief   A very brief example for error handling.
 *
 * @details Checks the specified status for errors (i.e. negative values) and
 *          prints a specified error message if any. An endless loop is entered
 *          to halt program execution.
 *
 * @param   status The specified status to be checked for errors.
 * @param   msg The associated error message to be printed in case of errors.
 *****************************************************************************/
void handle_error(status_t status, char const * msg);


/*!***************************************************************************
 * @brief   Starts measurements via the AFBR-S50 API.
 *
 * @details The measurements are started via the AFBR-S50 API using the current
 *          device configuration. Nothing happens if the device is already
 *          executing measurements.
 *
 *          In case of error, the #handle_error method is called.
 *****************************************************************************/
void start_measurements(void);


/*!***************************************************************************
 * @brief   Stops the currently ongoing measurements via the AFBR-S50 API.
 *
 * @details The measurements are stopped via the AFBR-S50 API. Nothing happens
 *          if the device does not currently execute measurements.
 *
 *          In case of error, the #handle_error method is called.
 *****************************************************************************/
void stop_measurements(void);

/*! @} */
#endif /* MAIN_H_ */
