/*************************************************************************//**
 * @file
 * @brief       SCI Handshaking Commands
 * @details     This file provides an interface for the data link layer of the
 *              systems communication interface.
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

#ifndef SCI_HANDSHAKING_H
#define SCI_HANDSHAKING_H
/*!***************************************************************************
 * @defgroup    sci_handshaking SCI: Handshaking
 * @ingroup     sci
 * @brief       SCI Handshaking Module
 * @details     Implements the handshaking command messages:
 *               - Acknowledge
 *               - Not-Acknowledge
 *               .
 * @addtogroup  sci_handshaking
 * @{
 *****************************************************************************/

#include "sci.h"


/*!***************************************************************************
 * @brief   Sends an Acknowledge message.
 * @param   deviceID The ID (index) of the SPI device that should process
 *          the received frame.
 *          This is only used for extended commands!
 * @param   cmd The command that is acknowledged.
 *          If extended command is passed, the frame will contain the deviceID.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SendAcknowledge(sci_device_t deviceID, sci_cmd_t cmd);

/*!***************************************************************************
 * @brief   Sends an Not-Acknowledge message.
 * @param   deviceID The ID (index) of the SPI device that should process
 *          the received frame.
 *          This is only used for extended commands!
 * @param   cmd The command that is not-acknowledged.
 *          If extended command is passed, the frame will contain the deviceID.
 * @param   reason The reason/status that caused the not-acknowledged.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SendNotAcknowledge(sci_device_t deviceID, sci_cmd_t cmd, status_t reason);


/*! @} */
#endif // SCI_HANDSHAKING_H
