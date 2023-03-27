/*************************************************************************//**
 * @file
 * @brief       SCI Layer 3: Command (Message) Interface
 * @details     This file provides an interface for the command or message layer
 *              of the systems communication interface.
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

#ifndef SCI_CMD_H
#define SCI_CMD_H

/*!***************************************************************************
 * @defgroup    sci_cmd SCI: Generic Command Definitions
 * @ingroup     sci
 * @brief       Generic Command Definitions
 * @details     Generic command definitions for the systems communication interface.
 * @addtogroup  sci_cmd
 * @{
 *****************************************************************************/

#include "sci.h"

/*! Determines whether to include a time stamp into log messages. */
#define SCI_LOG_TIMESTAMP 1


/*! Generic commands for the SCI module. */
enum GenericSerialCommandCodes
{
    CMD_INVALID             = 0x00, /*!< Program internal marker for invalid/erroneous commands. */

    /* Generic commands. */
    CMD_PING                = 0x01, /*! Simple ping message that will just acknowledge if successfully received. */
    CMD_LOG_MESSAGE         = 0x06, /*!< An event/debug log message. */
    CMD_STATUS_REPORT       = 0x07, /*!< Software status/error report (running, pause, error, ...) from MCU or request from PC. */
    CMD_SYSTEM_RESET        = 0x08, /*!< Command to reset the MCU. */
    CMD_ACKNOWLEDGE         = 0x0A, /*!< Acknowledge of the previous command. */
    CMD_NOT_ACKNOWLEDGE     = 0x0B, /*!< Not-acknowledge of the previous command. */

    /* Misc. commands. */
    CMD_TEST_MESSAGE        = 0x04, /*!< Test message send to the slave. The slave will reflect the message back to the master. */

};

/*!***************************************************************************
 * @brief   Sends an status report.
 * @param   deviceID The slave ID of the sensor handler to process the command.
 * @param   status The status value to be reported.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SendStatusReport(sci_device_t deviceID, status_t status);

/*!***************************************************************************
 * @brief   Sends a log message.
 * @param   fmt_s The printf() format string.
 * @param   ...   The printf() arguments.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_SendLogEntry(const char *fmt_s, ...);

/*!***************************************************************************
 * @brief   Sends a log message.
 * @param   fmt_s The printf() format string.
 * @param   ...   The printf() arguments.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_Printf(const char  *fmt_s, ...);

/*! @} */
#endif /* SCI_CMD_H */
