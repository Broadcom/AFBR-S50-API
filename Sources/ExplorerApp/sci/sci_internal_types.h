/*************************************************************************//**
 * @file
 * @brief       SCI: The internal type definitions.
 * @details     This file provides an interface for the systems communication
 *              interface.
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

#ifndef SCI_INTERNAL_TYPES_H
#define SCI_INTERNAL_TYPES_H

/*!***************************************************************************
 * @addtogroup  sci
 * @{
 *****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*! Max. byte size of transmitting/receiving SCI data frames. */
#ifndef SCI_FRAME_SIZE
#define SCI_FRAME_SIZE      64
#endif

/*! The number of SCI data frames for RX. */
#ifndef SCI_FRAME_BUF_RX_CT
#define SCI_FRAME_BUF_RX_CT 32
#endif

/*! The number of SCI data frames for TX. */
#ifndef SCI_FRAME_BUF_TX_CT
#define SCI_FRAME_BUF_TX_CT 32
#endif

/*! The timeout for a TX frame request in milliseconds. */
#ifndef SCI_TX_TIMEOUT_MSEC
#define SCI_TX_TIMEOUT_MSEC 1000
#endif

/*! The total number of SCI data frames. */
#define SCI_FRAME_BUF_CT (SCI_FRAME_BUF_RX_CT + SCI_FRAME_BUF_TX_CT)

/*!*****************************************************************************
 * @brief   Data buffer for outgoing frames.
 * @details A frame needs to be initialize with an data buffer and read/write
 *          pointers equal to zero.
 *          Status of the frame can be determined by the following conditions:
 *           - Idle: WrPtr == 0 && RdPtr == 0
 *           - Write/Read: WrPtr != 0 (|| RdPtr == 0)
 *           .
 *          The total amount of stored data is given by (size_t)(WrPtr - Buffer)
 *          The amount that is still to read is given by (size_t)(WrPtr - RdPtr)
 *
 *          \code
 *          Idle:
 *          Buffer:   0000000000000000
 *          Write:  | (0x00)
 *          Read:   | (0x00)
 *
 *
 *          Write:
 *          Buffer:   xxxxxxxx00000000
 *          Write:            |
 *          Read:     | (&Buffer)
 *
 *
 *          Read:
 *          Buffer:   0000xxxxxxxx0000
 *          Write:                |
 *          Read:         |
 *
 *          (0 = empty; x = full)
 *          \endcode
 *
 *          In order to accomplish flexible frame length, the frames might link
 *          to another frame which will be sent right after the current one has
 *          completely sent.
 ******************************************************************************/
typedef struct sci_frame_t
{
    /*! Frame write pointer */
    uint8_t * WrPtr;

    /*! Frame read pointer */
    uint8_t * RdPtr;

    /*! Data buffer. */
    uint8_t * Buffer;

    /*! Pointer to the next frame in the chain. */
    struct sci_frame_t * Next;

} sci_frame_t;

/*! SCI frames ring buffer.*/
typedef struct sci_frame_queue_t
{
    /*! Command queue buffer. */
    sci_frame_t     *Buff;

    /*! Head of the queue. */
    sci_frame_t     *Head;

    /*! Currently used queue load. */
    volatile size_t  Load;

    /*! Total size of the queue. */
    size_t           Size;

} sci_frame_queue_t;

/*! @} */
#endif // SCI_INTERNAL_TYPES_H
