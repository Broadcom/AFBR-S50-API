/*************************************************************************//**
 * @file
 * @brief       SCI Layer 2: Data Link Interface
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

#ifndef SCI_DATALINK_H
#define SCI_DATALINK_H

/*!***************************************************************************
 * @defgroup    sci_datalink SCI: Data Link Layer
 * @ingroup     sci
 * @brief       SCI Data Link Layer
 * @details     Implements the data link layer protocol for systems communication
 *              interface that connects to an external device. It takes care of
 *              sending and receiving data frames and thus byte stuffing and
 *              CRC check.
 *
 *              Remarks:
 *                  - Transmitting frames:
 *                      - Two buffers: one for preparing data, one for sending
 *                        at the same time.
 *                      - If UART TX Line is still busy when trying to send new
 *                        data, the program is delayed until TX is idle.
 *                      .
 *                  - Receiving frames:
 *                      - Special frame buffers collect data from RX (w/o
 *                        escape bytes)
 *                      - Only data between START and STOP is collected.
 *                      - After STOP was received, the FrameReceivedCallback
 *                        is invoked.
 *                      .
 *
 * @addtogroup  sci_datalink
 * @{
 *****************************************************************************/

#include "sci_status.h"
#include "sci_internal_types.h"

/*!***************************************************************************
 * @brief   Whether to allow newline (\n) in print / log messages.
 *****************************************************************************/
#define SCI_ALLOW_NEWLINE 1

/*!***************************************************************************
 * @brief   Checks whether the command byte is an extended command (= MSB set).
 *****************************************************************************/
#define SCI_CMD_IS_EXTENDED_CMD(cmd) ((cmd) & 0x80)

/*!***************************************************************************
 * @brief   Initialize the data link module.
 * @details Initialization implies the following steps:
 *              - Initialization of the SCI Hardware Layer, i.e. UART/LPSCI.
 *              - Starts to listen to incoming UART data and calls the frame
 *                received callback.
 *              - Initialization of the CRC module.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_DataLink_Init(void);

/*!***************************************************************************
 * @brief   Checks the CRC checksum for a RX frame.
 * @param   frame The RX frame which requires CRC checking.
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK (0) on success.
 *          - #ERROR_SCI_CRC_FAILED if the CRC failed.
 *****************************************************************************/
status_t SCI_DataLink_CheckRxFrame(sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Releases the frame queue.
 * @param   frame The frame queue to be released.
 *****************************************************************************/
void SCI_DataLink_ReleaseFrames(sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Resets the frame read pointer to the very first byte (= command byte).
 * @param   frame The frame queue to reset.
 *****************************************************************************/
void SCI_DataLink_ResetRxFrames(sci_frame_t * frame);

/*!***************************************************************************
 * @brief   Returns whether the TX line is currently busy and data is being sent.
 * @return  Returns true if the TX line is busy sending data.
 *****************************************************************************/
bool SCI_DataLink_IsTxBusy(void);

/*!***************************************************************************
 * @brief   Trigger the data transfer and releases the TX buffers.
 * @details Before the frame is transferred, a stop byte is added to the end
 *          of the data buffer.
 * @param   frame The frame to be sent.
 * @param   high_priority If set, the frame is queued right after the current
 *                        frame. If not set, the frame is queued at the very
 *                        end of the frame queue.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SCI_DataLink_SendTxFrame(sci_frame_t * frame, bool high_priority);

/*!***************************************************************************
 * @brief   Find an unused TX buffer from the queue and prepare it with a start
 *          byte.
 * @details If sending data over the SCI, an new and empty TX frame needs to be
 *          claimed for usage of the command. This functions finds one and
 *          prepares it with an start byte. A pointer to the frame is returned
 *          if one is found. Otherwise null, so checking for null pointer is
 *          recommended!
 * @param   queueStartByte Whether to queue a start byte into the buffer.
 * @return  Returns a pointer to an free TX frame, zero if no one is currently
 *          available.
 *****************************************************************************/
sci_frame_t * SCI_DataLink_RequestTxFrame(bool queueStartByte);

/*! @} */
#endif /* SCI_DATALINK_H */
