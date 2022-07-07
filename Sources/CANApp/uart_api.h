/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 UART Interface
 * @details     This file defines an UART interface to communicate with the
 *              AFBR-S50 Time-Of-Flight sensor API.
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
 *
 *****************************************************************************/


#ifndef UART_API_H_
#define UART_API_H_

#include "argus.h"

/*!***************************************************************************
 * @defgroup    uart_intf UART API interface
 * @ingroup     can_app
 *
 * @brief       UART interface for the AFBR-S50 Reference Board CAN application.
 *
 * @details     A simple UART interface to demonstrate the basic usage of the
 *              UART bus that comes with the AFBR-S50 Reference Board.
 *
 * @addtogroup  uart_intf
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief   Initializes the UART API module.
 *****************************************************************************/
void UART_API_Init(void);


/*!***************************************************************************
 * @brief   Handles incoming UART commands by invoking the corresponding methods.
 *
 * @details Checks the incomming data queue and handles/invokes commands
 *          accordingly.
 *
 *          Note that this function must be called from thread level (not from
 *          interrupt service routines) in order to be executed correctly.
 *****************************************************************************/
void UART_HandleCommand(void);

/*!***************************************************************************
 * @brief   Prints measurement results via UART.
 *
 * @details Print the recent measurement results:
 *
 *          1. Time stamp in seconds since the last MCU reset.
 *          2. Range in mm (converting the Q9.22 value to mm)
 *          3. Amplitude in LSB (converting the UQ12.4 value to LSB)
 *          4. Signal Quality in % (100% = good signal).
 *          4. Status (0: OK, <0: Error, >0: Warning
 *
 * @note    Sending data via UART creates a large delay which might prevent
 *          the API from reaching the full frame rate. This example sends
 *          approximately 80 characters per frame at 115200 bps which limits
 *          the max. frame rate of 144 fps:
 *          115200 bps / 10 [bauds-per-byte] / 80 [bytes-per-frame] = 144 fps
 *
 * @param   res A pointer to the latest measurement results structure.
 *****************************************************************************/
void UART_Send1D(argus_results_t const * res);

/*! @} */
#endif /* UART_API_H_ */
