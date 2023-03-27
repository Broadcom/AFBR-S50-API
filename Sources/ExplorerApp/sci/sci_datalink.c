/*************************************************************************//**

 * @brief       SCI Layer 2: Data Link and Data Frame Module
 * @details     This file provides a data link layer for the systems
 *              communication interface. It also contains the implementation of
 *              the data frame interface.
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


/*! Preprocessor flag to choose between USB and non-USB (UART) connection. */
#ifndef AFBR_SCI_USB
#define AFBR_SCI_USB 0
#endif


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "sci_cmd.h"
#include "sci_datalink.h"
#include "sci_byte_stuffing.h"
#include "sci_crc8.h"
#include "sci_handshaking.h"

#include "driver/irq.h"
#include "debug.h"

#if AFBR_SCI_USB
#include "usb/usb_sci.h"
#else
#include "driver/uart.h"
#endif

#include <stddef.h>
#include <assert.h>

#include "utility/time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Setup for SCI Rx and Tx Buffers:
 *
 * Buffer Size: 64 byte * 16 = 1KByte for Rx and Tx
 *
 * With respect to the 3 special bytes:
 * => Average maximum frame length: ~ 1024 * (1 - 3/256) = 1012 bytes
 *
 *******************************************************************************
 * Timing for a single frame: t = FRAME_SIZE * BAUDS_PER_BYTE / BAUDRATE
 *
 * where
 *  - BAUDS_PER_BYTE = 10 @ 8N1 UART settings (8 Bits, 1 Start, 1 Stop Byte, no parity)
 *
 * e.g.:
 *  - BAUDRATE: 115200
 *  - FRAME_SIZE: 64
 *
 *  => t = 5.56 msec/frame
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static inline void RaiseError(status_t error);
static void RxCallback(uint8_t const * data, uint32_t size);
static void TxCallback(status_t status, sci_frame_t * frame);

static sci_frame_t * SCI_DataLink_RequestFrame(sci_frame_queue_t * queue);
static inline status_t SCI_DataLink_SendFrame(sci_frame_t * frame);
static inline void SCI_DataLink_ReleaseFrame(sci_frame_t * frame);

static uint8_t SCI_DataLink_GetCRC(sci_frame_t const * frame);
static uint8_t SCI_DataLink_CalcCRC(sci_frame_t const * frame);

/*!***************************************************************************
 * @brief   Inserts a checksum into the Tx frame;
 * @details Given a Tx frame with data filled, this function calculates and
 *          appends the CRC to the frame buffer. It assumes that the first byte
 *          in the TX buffer is start byte which is usually ensured by the
 *          SCI_DataLink_RequestTxFrame function.
 * @param   frame The frame for which a checksum is calculated.
 *****************************************************************************/
static void SCI_Frame_QueueCRC(sci_frame_t * frame);

/******************************************************************************
 * Variables
 ******************************************************************************/
/*! The data buffer for rx and tx frames. */
static uint8_t SCI_DataBuffer[SCI_FRAME_SIZE * SCI_FRAME_BUF_CT] = { 0 };

/*! The frame buffer for rx and tx frames. */
static sci_frame_t SCI_FrameBuffer[SCI_FRAME_BUF_CT];

/*! The data frame queue for rx frames. */
static sci_frame_queue_t SCI_RxFrameQueue;

/*! The data frame queue for tx frames. */
static sci_frame_queue_t SCI_TxFrameQueue;

/*! Callback function pointer for received frame event. */
sci_rx_cmd_cb_t SCI_RxCallback = 0;

/*! Callback function pointer for error event. */
sci_error_cb_t SCI_ErrorCallback = 0;

/*! A pointer to the frame that is currently sent.
 *  It is used to enqueue more frames if the UART is busy. */
static volatile sci_frame_t * SCI_CurrentTxFrame = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t SCI_DataLink_Init(void)
{
    status_t status = STATUS_OK;
    SCI_CurrentTxFrame = 0;

    for (uint8_t i = 0; i < SCI_FRAME_BUF_CT; ++i)
    {
        SCI_FrameBuffer[i].Buffer = SCI_DataBuffer + (i * SCI_FRAME_SIZE);
        SCI_FrameBuffer[i].RdPtr = 0;
        SCI_FrameBuffer[i].WrPtr = 0;
        SCI_FrameBuffer[i].Next = 0;
    }

    SCI_RxFrameQueue.Load = 0;
    SCI_RxFrameQueue.Head = SCI_FrameBuffer;
    SCI_RxFrameQueue.Buff = SCI_FrameBuffer;
    SCI_RxFrameQueue.Size = SCI_FRAME_BUF_RX_CT;

    SCI_TxFrameQueue.Load = 0;
    SCI_TxFrameQueue.Head = SCI_FrameBuffer + SCI_FRAME_BUF_RX_CT;
    SCI_TxFrameQueue.Buff = SCI_FrameBuffer + SCI_FRAME_BUF_RX_CT;
    SCI_TxFrameQueue.Size = SCI_FRAME_BUF_TX_CT;

    SCI_CRC8_Init();

#if AFBR_SCI_USB
    USB_DeviceApplicationInit();
    USB_SetRxCallback(RxCallback);
    USB_SetErrorCallback(RaiseError);
    Time_DelayMSec(100);
#else
    status = UART_Init();
    UART_SetRxCallback(RxCallback);
    UART_SetErrorCallback(RaiseError);
#endif

    return status;
}


/*******************************************************************************
 * IRQ handler
 ******************************************************************************/
static inline void RaiseError(status_t error)
{
    if (SCI_ErrorCallback)
        SCI_ErrorCallback(error);
}


/* Interrupt service routine for the receiving data from the serial port; */
static void RxCallback(uint8_t const *data, uint32_t size)
{
    static bool escapeNextByte = false; /*!< Flag for byte stuffing */
    static sci_frame_t *f = 0; /*!< the current buffer in the queue. */
    static sci_frame_t *f0 = 0; /*!< the first buffer in the queue. */

    for (uint8_t const *d = data; d < data + size; ++d)
    {
        uint8_t rx = *d;
        if (escapeNextByte)
        {
            escapeNextByte = false;

            /* Error Handling for the escape byte */
            rx = (uint8_t)(~rx);
            if (f0)
            {
                if ((rx != SCI_ESCAPE_BYTE) &&
                    (rx != SCI_START_BYTE) &&
                    (rx != SCI_STOP_BYTE))
                {
                    /* Reset f0 */
                    SCI_DataLink_ReleaseFrames(f0);
                    f0 = 0;
                    f = 0;

                    /* send an NAK with error code to the host */
                    SCI_SendNotAcknowledge(0, CMD_INVALID, ERROR_SCI_INVALID_ESCAPE_BYTE);
                    RaiseError(ERROR_SCI_INVALID_ESCAPE_BYTE);
                    continue;
                }
            }
        }
        else
        {
            if (rx == SCI_ESCAPE_BYTE)
            {
                /* Escape byte: ignore following control bytes. */
                escapeNextByte = true;
                continue;
            }
            else if (rx == SCI_START_BYTE)
            {
                /* Start byte: start a new command. */
                if (f0)
                {
                    /* Invalid Start Byte: already within an active frame -> reset f0 */
                    SCI_DataLink_ReleaseFrames(f0);
                    f0 = 0;
                    f = 0;

                    /* send an NAK with error code to the host */
                    SCI_SendNotAcknowledge(0, CMD_INVALID, ERROR_SCI_INVALID_START_BYTE);
                    RaiseError(ERROR_SCI_INVALID_START_BYTE);
                }

                f0 = SCI_DataLink_RequestFrame(&SCI_RxFrameQueue);

                if (!f0)
                {
                    /* send an NAK with error code to the host */
                    SCI_SendNotAcknowledge(0, CMD_INVALID, ERROR_SCI_RX_BUFFER_FULL);
                    RaiseError(ERROR_SCI_RX_BUFFER_FULL);
                    continue;
                }

                f = f0;
                continue;
            }
            else if (rx == SCI_STOP_BYTE)
            {
                /* Stop byte: command completed. */
                if (!f0)
                {
                    /* Invalid stop byte outside of an active frame received: ignore */
                    continue;
                }

                /* Check data length: minimal 2 bytes required (Command + CRC) */
                if (f0->WrPtr - f0->Buffer < 2)
                {
                    /* Release the frame */
                    SCI_DataLink_ReleaseFrames(f0);
                    f0 = 0;
                    f = 0;

                    /* send an NAK with error code to the host */
                    SCI_SendNotAcknowledge(0, CMD_INVALID, ERROR_SCI_FRAME_TOO_SHORT);
                    RaiseError(ERROR_SCI_FRAME_TOO_SHORT);
                    continue;
                }

                if (SCI_RxCallback)
                {
                    /* Invoke callback. */
                    if (SCI_RxCallback(f0) != STATUS_OK)
                    {
                        SCI_DataLink_ReleaseFrames(f0);
                    }
                }
                else
                {
                    /* Invoke command directly if no callback. */
                    status_t status = SCI_InvokeRxCommand(f0);
                    if (status != STATUS_OK)
                    {
                        SCI_DataLink_ReleaseFrames(f0);
                        RaiseError(status);
                    }
                }

                f0 = 0;
                f = 0;
                continue;
            }
        }

        if (f)
        {
            /* Check if frame is full and queue another frame. */
            if (f->WrPtr - f->Buffer == SCI_FRAME_SIZE)
            {
                f->Next = SCI_DataLink_RequestFrame(&SCI_RxFrameQueue);

                if (!f->Next)
                {
                    /* Reset f0 */
                    SCI_DataLink_ReleaseFrames(f0);
                    f0 = 0;
                    f = 0;

                    /* send an NAK with error code to the host */
                    SCI_SendNotAcknowledge(0, CMD_INVALID, ERROR_SCI_RX_BUFFER_FULL);
                    RaiseError(ERROR_SCI_RX_BUFFER_FULL);
                    continue;
                }

                f = f->Next;
            }

            /* Save byte into command buffer. */
            assert(f->WrPtr - f->Buffer < SCI_FRAME_SIZE);
            *(f->WrPtr++) = rx;
        }
    }
}

status_t SCI_DataLink_CheckRxFrame(sci_frame_t * frame)
{
    assert(frame != 0);
    assert(frame->Buffer <= frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

    const uint32_t CRC_r = SCI_DataLink_GetCRC(frame);
    const uint32_t CRC_c = SCI_DataLink_CalcCRC(frame);

    if (CRC_c != CRC_r)
    {
//      error_log("received command %02x, CRC failed! %08x != %08x",
//                frame->Buffer[0], CRC_r, CRC_c);
        return ERROR_SCI_CRC_FAILED;
    }

    return STATUS_OK;
}

/* Releases only the current frame and but not the next frames. */
static inline void SCI_DataLink_ReleaseFrame(sci_frame_t * frame)
{
    assert(frame != 0);

    IRQ_LOCK();
    if (frame->WrPtr != 0)
    {
        frame->WrPtr = 0;
        frame->RdPtr = 0;
        frame->Next = 0;
        if (frame < SCI_FrameBuffer + SCI_FRAME_BUF_RX_CT)
        {
            assert(SCI_RxFrameQueue.Load);
            SCI_RxFrameQueue.Load--;
        }
        else
        {
            assert(SCI_TxFrameQueue.Load);
            SCI_TxFrameQueue.Load--;
        }
    }
    IRQ_UNLOCK();
}

void SCI_DataLink_ReleaseFrames(sci_frame_t * frame)
{
    sci_frame_t * fnext;
    while (frame != 0)
    {
        fnext = frame->Next;
        SCI_DataLink_ReleaseFrame(frame);
        frame = fnext;
    }
}

void SCI_DataLink_ResetRxFrames(sci_frame_t * frame)
{
    while (frame != 0)
    {
        frame->RdPtr = frame->Buffer;
        frame = frame->Next;
    }
}

static sci_frame_t * SCI_DataLink_RequestFrame(sci_frame_queue_t * queue)
{
    sci_frame_t * frame = 0;

    IRQ_LOCK();
    if (!(queue->Load < queue->Size))
    {
        IRQ_UNLOCK();
        return 0; // no free buffers!!
    }
    queue->Load++;

    /* scan for free buffer. */
    do
    {
        frame = queue->Head++;
        if (queue->Head == queue->Buff + queue->Size)
        {
            queue->Head = queue->Buff;
        }
    } while (frame->WrPtr != 0);

    /* setup buffer */
    frame->Next = 0;
    frame->RdPtr = frame->Buffer; //0;
    frame->WrPtr = frame->Buffer;

    IRQ_UNLOCK();
    return frame;
}

sci_frame_t * SCI_DataLink_RequestTxFrame(bool queueStartByte)
{
    sci_frame_t * frame = 0;
    ltc_t start = { 0 };
    Time_GetNow(&start);

    while ((frame = SCI_DataLink_RequestFrame(&SCI_TxFrameQueue)) == 0)
    {
#if AFBR_SCI_USB
        if (USB_CancelIfTimeOutElapsed())
        {
            /* USB send timeout occurred! */
            RaiseError(ERROR_USB_TIMEOUT);
            continue;
        }
#endif
        if (SCI_CurrentTxFrame == 0)
        {
//          BREAKPOINT();
            /* No buffers available and not sending!!! */
            RaiseError(ERROR_SCI_TX_BUFFER_EXCEEDANCE);
            return 0;
        }

        if (Time_CheckTimeoutMSec(&start, SCI_TX_TIMEOUT_MSEC))
        {
//          BREAKPOINT();
            /* Timeout: sending but no buffers available within given time!!! */
            RaiseError(ERROR_SCI_BUFFER_FULL);
            return 0;
        }
    }

    if (queueStartByte)
    {
        /* add start byte */
        SCI_Frame_SetByte(frame, SCI_START_BYTE);
    }

    return frame;
}

static void TxCallback(status_t status, sci_frame_t * frame)
{
    assert(frame != 0);
    assert(frame->WrPtr != 0);
    assert(SCI_CurrentTxFrame == frame);

    /* Release the frame at the beginning of the queue.
     * If an error has occurred, also remove the subsequent
     * frames that belong to the current message. */
    do
    {
        sci_frame_t * next_frame = frame->Next;
        SCI_DataLink_ReleaseFrame(frame);
        frame = next_frame;
    }
    while ((status < STATUS_OK) &&
           (frame != 0) &&
           (!SCI_Frame_IsStartFrame(frame)));

    /* Check for errors and invoke the error callback. */
    if (status < STATUS_OK && status != ERROR_ABORTED)
    {
        RaiseError(status);
    }

    SCI_CurrentTxFrame = frame;

    /* Send the next frame in the queue. */
    if (frame != 0)
    {
        status = SCI_DataLink_SendFrame(frame);
        if (status < STATUS_OK)
        {
            SCI_CurrentTxFrame = 0;
            SCI_DataLink_ReleaseFrames(frame);
            RaiseError(status);
        }
    }
}

static inline status_t SCI_DataLink_SendFrame(sci_frame_t * frame)
{
    assert(frame != 0);
    assert(frame->Buffer == frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);
#if AFBR_SCI_USB
    return USB_SendBuffer(frame->Buffer,
                          (size_t) (frame->WrPtr - frame->Buffer),
                          (usb_tx_callback_t) TxCallback,
                          frame);
#else
    return UART_SendBuffer(frame->Buffer,
                           (size_t) (frame->WrPtr - frame->Buffer),
                           (uart_tx_callback_t) TxCallback,
                           frame);
#endif
}

bool SCI_DataLink_IsTxBusy(void)
{
#if AFBR_SCI_USB
    return (SCI_CurrentTxFrame != 0) || USB_IsTxBusy();
#else
    return (SCI_CurrentTxFrame != 0) || UART_IsTxBusy();
#endif
}

status_t SCI_DataLink_SendTxFrame(sci_frame_t * frame, bool high_priority)
{
    assert(frame != 0);
    assert(frame->Buffer == frame->RdPtr);
    assert(frame->RdPtr < frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

    status_t status = STATUS_OK;

    /* queue CRC and stop byte */
    SCI_Frame_QueueCRC(frame);
    SCI_Frame_SetByte(frame, SCI_STOP_BYTE);

    /* Lock interrupts such that the current TX frame
     * does not finish while the new one is enqueued. */
    IRQ_LOCK();
    if (SCI_CurrentTxFrame != 0)
    {
        sci_frame_t * last_frame = (sci_frame_t*) SCI_CurrentTxFrame;

        if (high_priority)
        {
            /* find end of current frame */
            while ((last_frame->Next != 0) && !SCI_Frame_IsStartFrame(last_frame->Next))
            //while ((*(last_frame->WrPtr - 1) != SCI_STOP_BYTE) && (last_frame->Next != 0))
            {
                last_frame = last_frame->Next;
            }

            assert(last_frame != 0);
            assert(last_frame->Buffer == last_frame->RdPtr);
            assert(last_frame->RdPtr < last_frame->WrPtr);
            assert(last_frame->WrPtr <= last_frame->Buffer + SCI_FRAME_SIZE);

            sci_frame_t * next_frame = last_frame->Next;

            /* Enqueue frame to the end of the current frame. */
            last_frame->Next = frame;

            /* set remaining frame to be queued. */
            frame = next_frame;
        }


        /* find end of queue . */
        while (last_frame->Next != 0)
        {
            last_frame = last_frame->Next;
        }

        assert(last_frame != 0);
        assert(last_frame->Buffer == last_frame->RdPtr);
        assert(last_frame->RdPtr < last_frame->WrPtr);
        assert(last_frame->WrPtr <= last_frame->Buffer + SCI_FRAME_SIZE);

        /* Enqueue frame to the queue. */
        last_frame->Next = frame;

        IRQ_UNLOCK();
    }
    else
    {
        /* Send data if TX line is free. */
        SCI_CurrentTxFrame = frame;

        IRQ_UNLOCK();

        status = SCI_DataLink_SendFrame(frame);
        if (status < STATUS_OK)
        {
            SCI_CurrentTxFrame = 0;
            SCI_DataLink_ReleaseFrames(frame);
            BREAKPOINT();
            RaiseError(status);
        }
    }

    return status;
}

static uint8_t SCI_DataLink_GetCRC(sci_frame_t const * frame)
{
    assert(frame != 0);

    sci_frame_t const * frame2 = frame;

    while (frame->Next != 0)
    {
        frame2 = frame;
        frame = frame->Next;
    }

    if (frame->WrPtr > frame->Buffer)
        return *(frame->WrPtr - 1);
    else
        return *(frame2->WrPtr - 1);
}

static uint8_t SCI_DataLink_CalcCRC(sci_frame_t const * frame)
{
    assert(frame != 0);
    assert(frame->Buffer <= frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

    /* Total frame length (- CRC length). */
    int32_t len_data = SCI_Frame_TotalFrameLength(frame) - 1;

//  /* Consistency check for total frame length. */
//  if (len_data < 0)
//  {
//      error_log("received command %#02x, data size to short! %08x != %08x",
//                frame->Buffer[0]);
//  }

    /* Calculate the CRC for all frames in the queue. */
    uint8_t crc = 0;
    while (len_data > 0)
    {
        int32_t len_frame = frame->WrPtr - frame->Buffer;
        if (len_data > len_frame)
            crc = SCI_CRC8_Compute(crc, frame->Buffer, (size_t) len_frame);
        else
            crc = SCI_CRC8_Compute(crc, frame->Buffer, (size_t) len_data);

        len_data -= len_frame;
        frame = frame->Next;
    }
    return crc;
}

static void SCI_Frame_QueueCRC(sci_frame_t * frame)
{
    assert(frame != 0);
    assert(frame->Buffer == frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);
    assert(frame->Buffer[0] == SCI_START_BYTE);

    uint8_t crc = 0;

    uint_fast8_t escape = 0;

    /* Ignore the start byte in the first frame. */
    frame->RdPtr = frame->Buffer + 1;

    while (frame != 0)
    {
        /* Calculate byte-wise CRC and heed byte stuffing. */
        while (frame->RdPtr < frame->WrPtr)
        {
            if (!escape && *frame->RdPtr == SCI_ESCAPE_BYTE)
            {
                escape = !0;
            }
            else
            {
                if (escape)
                {
                    escape = 0;
                    uint8_t byte = (uint8_t) (~(*frame->RdPtr));
                    crc = SCI_CRC8_Compute(crc, &byte, 1);
                }
                else
                {
                    crc = SCI_CRC8_Compute(crc, frame->RdPtr, 1);
                }
            }
            frame->RdPtr++;
        }

        /* Iterate through queued frames and queue CRC in last frame. */
        frame->RdPtr = frame->Buffer;
        if (frame->Next != 0)
        {
            frame = frame->Next;
            frame->RdPtr = frame->Buffer;
        }
        else
        {
            /* Enqueue the CRC. */
            SCI_Frame_Queue08u(frame, crc);
            frame = 0;
        }
    }
}
