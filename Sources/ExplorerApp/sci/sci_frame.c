/*************************************************************************//**
 * @file
 * @brief       SCI Data Frame Interface.
 * @details     This file provides an interface for the data frame objects of
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


/*******************************************************************************
 * Code
 ******************************************************************************/

#include "sci_frame.h"
#include "sci_datalink.h"
#include "sci_byte_stuffing.h"
#include "sci_crc8.h"
#include "sci_status.h"
#include <assert.h>

int32_t SCI_Frame_TotalFrameLength(sci_frame_t const * frame)
{
    assert(frame != 0);
    assert(frame->WrPtr != 0);

    /* Find total frame length. */
    int32_t len_data = 0;
    while (frame != 0)
    {
        len_data += frame->WrPtr - frame->Buffer;
        frame = frame->Next;
    }
    return len_data;
}
uint32_t SCI_Frame_BytesToRead(sci_frame_t const * frame)
{
    assert(frame != 0);
    uint32_t length = 0;
    while (frame != 0)
    {
        length += (uint32_t)(frame->WrPtr - frame->RdPtr);
        frame = frame->Next;
    }
    return length;
}
uint8_t SCI_Frame_IsStartFrame(sci_frame_t const * frame)
{
    assert(frame != 0);
    return frame->Buffer[0] == SCI_START_BYTE;
}

void SCI_Frame_SetByte(sci_frame_t * frame, uint8_t byte)
{
    assert(frame != 0);

    /* Iterate to the last frame in the queue. */
    while (frame->Next != 0)
    {
        frame = frame->Next;
    }

    /* Check if frame is full and enqueue another one. */
    if (frame->WrPtr - frame->Buffer == SCI_FRAME_SIZE)
    {
        frame->Next = SCI_DataLink_RequestTxFrame(false);
        frame = frame->Next;
        assert(frame != 0);
        if (!frame) return;
    }

    assert(frame != 0);
    assert(frame->Buffer == frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

    *(frame->WrPtr++) = byte;
}

void SCI_Frame_Queue08u(sci_frame_t * frame, uint8_t data)
{
    if (data == SCI_START_BYTE || data == SCI_STOP_BYTE || data == SCI_ESCAPE_BYTE)
    {
        SCI_Frame_SetByte(frame, SCI_ESCAPE_BYTE);
        SCI_Frame_SetByte(frame, (uint8_t)(~data));
    }
    else
    {
        SCI_Frame_SetByte(frame, (uint8_t)data);
    }
}
void SCI_Frame_Queue16u(sci_frame_t * frame, uint16_t data)
{
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 8));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 0));
}
void SCI_Frame_Queue24u(sci_frame_t * frame, uint32_t data)
{
    assert(data < 0x01000000U);
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 16));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 8));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 0));
}
void SCI_Frame_Queue32u(sci_frame_t * frame, uint32_t data)
{
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 24));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 16));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 8));
    SCI_Frame_Queue08u(frame, (uint8_t)(data >> 0));
}
void SCI_Frame_Queue08s(sci_frame_t * frame, int8_t data)
{
    if (data == SCI_START_BYTE || data == SCI_STOP_BYTE || data == SCI_ESCAPE_BYTE)
    {
        SCI_Frame_SetByte(frame, SCI_ESCAPE_BYTE);
        SCI_Frame_SetByte(frame, (uint8_t)(~data));
    }
    else
    {
        SCI_Frame_SetByte(frame, (uint8_t)data);
    }
}
void SCI_Frame_Queue16s(sci_frame_t * frame, int16_t data)
{
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 8));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 0));
}
void SCI_Frame_Queue24s(sci_frame_t * frame, int32_t data)
{
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 16));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 8));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 0));
}
void SCI_Frame_Queue32s(sci_frame_t * frame, int32_t data)
{
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 24));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 16));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 8));
    SCI_Frame_Queue08s(frame, (int8_t)(data >> 0));
}

void SCI_Frame_Queue_Time(sci_frame_t * frame, ltc_t const * t)
{
    assert(frame != 0);
    assert(t != 0);
    SCI_Frame_Queue32u(frame, t->sec);          // s
    SCI_Frame_Queue16u(frame, (uint16_t)(t->usec >> 4U));   // µs / 16
}

static inline uint8_t SCI_Frame_GetByte(sci_frame_t * frame)
{
    assert(frame != 0);
    assert(frame->Buffer <= frame->RdPtr);
    assert(frame->RdPtr <= frame->WrPtr);
    assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

    /* Skip all completely read frames. */
    while (frame->WrPtr == frame->RdPtr)
    {
        assert(frame->Next != 0);
        frame = frame->Next;
    }
    return *(frame->RdPtr++);
}
int8_t SCI_Frame_Dequeue08s(sci_frame_t * frame)
{
    return (int8_t)SCI_Frame_GetByte(frame);
}
int16_t SCI_Frame_Dequeue16s(sci_frame_t * frame)
{
    uint_fast16_t
    val  = (uint_fast16_t)(SCI_Frame_GetByte(frame) << 8U);
    val |= (uint_fast16_t)(SCI_Frame_GetByte(frame) << 0U);
    return (int16_t)val;
}
int32_t SCI_Frame_Dequeue24s(sci_frame_t * frame)
{
    uint32_t
    val  = (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
    return (int32_t)val;
}
int32_t SCI_Frame_Dequeue32s(sci_frame_t * frame)
{
    uint32_t
    val  = (uint32_t)(SCI_Frame_GetByte(frame) << 24U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
    return (int32_t)val;
}
uint8_t SCI_Frame_Dequeue08u(sci_frame_t * frame)
{
    return (uint8_t)SCI_Frame_GetByte(frame);
}
uint16_t SCI_Frame_Dequeue16u(sci_frame_t * frame)
{
    uint_fast16_t // uint16_t raises warning
    val  = (uint_fast16_t)(SCI_Frame_GetByte(frame) <<  8U);
    val |= (uint_fast16_t)(SCI_Frame_GetByte(frame) <<  0U);
    return (uint16_t)val;
}
uint32_t SCI_Frame_Dequeue24u(sci_frame_t * frame)
{
    uint32_t
    val  = (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
    return val;
}
uint32_t SCI_Frame_Dequeue32u(sci_frame_t * frame)
{
    uint32_t
    val  = (uint32_t)(SCI_Frame_GetByte(frame) << 24U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
    val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
    return val;
}

void SCI_Frame_PutChar(char c, void * frame)
{
    if (c == '\r') return;
    SCI_Frame_Queue08u((sci_frame_t*)frame, (uint8_t)c);
}
