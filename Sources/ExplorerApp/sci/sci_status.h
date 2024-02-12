/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Application.
 * @details     This file provides common (e.g. status) definitions for the
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

#ifndef SCI_STATUS_H
#define SCI_STATUS_H

/*!***************************************************************************
 * @addtogroup  sci
 * @{
 *****************************************************************************/

#include "utility/status.h"

/*! @brief The SCI module specific status and error return codes. */
enum StatusSCI
{
    /*! Unknown/not initialized command. */
    ERROR_SCI_UNKNOWN_COMMAND = -211,

    /*! The execution of a command returned status != #STATUS_OK. */
    ERROR_SCI_CMD_EXECUTION_FAILURE = -212,

    /*! Invalid command code: e.g. is not tx/rx command. */
    ERROR_SCI_INVALID_CMD_CODE = -213,

    /*! Invalid command parameters/data/decoding failed. */
    ERROR_SCI_INVALID_CMD_PARAMETER = -214,

    /*! CRC failed -> frame invalid. */
    ERROR_SCI_CRC_FAILED = -215,

    /*! Status for buffer full. */
    ERROR_SCI_BUFFER_FULL = -216,

    /*! Received a start byte when no one has been expected. */
    ERROR_SCI_INVALID_START_BYTE = -217,

    /*! Received a stop byte when no one has been expected. */
    ERROR_SCI_INVALID_STOP_BYTE = -218,

    /*! Status for buffer full. */
    ERROR_SCI_RX_BUFFER_FULL = -219,

    /*! Received wrong escape byte. */
    ERROR_SCI_INVALID_ESCAPE_BYTE = -220,

    /*! Frame too short, i.e. too less data received for the corresponding message. */
    ERROR_SCI_FRAME_TOO_SHORT = -221,

    /*! Frame too long, i.e. too much data received for the corresponding message. */
    ERROR_SCI_FRAME_TOO_LONG = -222,

    /*! The data that was requested to be sent exceeds the maximum output data buffers size.*/
    ERROR_SCI_TX_BUFFER_EXCEEDANCE = -223,
};

/*! @} */
#endif /* SCI_STATUS_H */
