/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Interface
 * @details     This file defines an CAN interface to communicate with the
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

#ifndef CAN_API_H_
#define CAN_API_H_

#include "argus.h"


/*!***************************************************************************
 * @defgroup    can_intf CAN API interface
 * @ingroup     can_app
 *
 * @brief       CAN interface for the AFBR-S50 Reference Board CAN application.
 *
 * @details     A simple CAN interface to demonstrate the basic usage of the
 *              CAN bus that comes with the AFBR-S50 Reference Board.
 *
 * @addtogroup  can_intf
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief   Initializes the CAN API module.
 *****************************************************************************/
void CAN_Init(void);


/*!***************************************************************************
 * @brief   Deinitializes the CAN module.
 *****************************************************************************/
void CAN_Deinit(void);


/*!***************************************************************************
 * @brief   Handles incoming CAN commands by invoking the corresponding methods.
 *
 * @details Checks the incomming data queue and handles/invokes commands
 *          accordingly.
 *
 *          Note that this function must be called from thread level (not from
 *          interrupt service routines) in order to be executed correctly.
 *****************************************************************************/
void CAN_HandleCommand(void);


/*!***************************************************************************
 * @brief   Prints measurement results via CAN bus.
 *
 * @details Prints 1D measurement data via CAN-bus as data frame with ID 28.
 *
 *          The following values are included in the CAN data frame payload:
 *
 *          - 0..2: 1D Range [mm] (24bit, unsigned, MSB first)
 *          - 3..4: 1D Amplitude [LSB] (16-bit, unsigned, MSB first)
 *          - 5: Signal Quality [%] (8-bit, 0%-100% )
 *          - 6-7: Status (16-bit, signed, MSB first, see #status_t for details)
 *          .
 *
 * @param   res A pointer to the latest measurement results structure.
 *****************************************************************************/
void CAN_Transmit1D(argus_results_t const * res);

/*! @} */
#endif /* CAN_API_H_ */
