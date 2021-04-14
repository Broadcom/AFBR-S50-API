/*************************************************************************//**
 * @file
 * @brief    	SCI Data Frame Interface.
 * @details		This file provides an interface for the data frame objects of
 * 				of the systems communication interface.
 * 
 * @copyright
 * 
 * Copyright (c) 2021, Broadcom Inc
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

#ifndef SCI_FRAME_H
#define SCI_FRAME_H

#include "sci_internal_types.h"
#include "utility/time.h"

/*!***************************************************************************
 * @defgroup	sci_frame SCI: Data Frames
 * @ingroup		sci
 * @brief		SCI Data Frames
 * @details		The SCI Data Frame Interface. Contains functionality to
 * 				enqueue/dequeue different data types to/from an specified
 * 				#sci_frame_t object. This is mainly used to define the SCI
 * 				commands and implement the data serialization.
 *
 * @addtogroup 	sci_frame
 * @{
 *****************************************************************************/



/*!***************************************************************************
 * @brief	Returns the total number of bytes within a specified frame.
 * @details	Does calculate the total number of bytes that have been written
 * 			into the buffer. Especially it does not heed the byte stuffing and
 * 			counts escape bytes as well as normal bytes.
 * @param	frame The frame to count the data bytes.
 * @return	The number of bytes to read.
 *****************************************************************************/
static inline int32_t SCI_Frame_TotalFrameLength (sci_frame_t const * frame);

/*!***************************************************************************
 * @brief	Determines whether the frame is a start frame.
 * @details If the specified frame is a start frame, i.e. the first frame of a
 * 			sequence of multiple frames that starts with a start byte, the
 * 			function returns true. False elsewise.
 * @param	frame The frame to check for the start byte.
 * @return	True if the frame is a start frame.
 *****************************************************************************/
static inline uint8_t SCI_Frame_IsStartFrame(sci_frame_t const * frame);

/*!***************************************************************************
 * @brief	Returns the bytes that have been written from the buffer and not
 * 			read yet.
 * @details	Does calculate the total number of bytes to be read in the buffer.
 * 			Especially it does not heed the byte stuffing and counts escape
 * 			bytes as well as normal bytes.
 * @param	frame The frame to count the data bytes.
 * @return	The number of bytes to read.
 *****************************************************************************/
static inline uint32_t SCI_Frame_BytesToRead(sci_frame_t const * frame);

/*!***************************************************************************
 * @brief	Inserts a char into the TX buffer.
 * @details	The char is checked for byte stuffing and thus it might
 * 			additionally add an escape byte to the buffer.
 * @note	No new line / carriage return chars are added to the buffer. They
 * 			will be removed/ignored.
 * @param	frame The frame to put the data.
 * @param	c     The char to append.
 * @return 	Ignore; returns always 0;
 *****************************************************************************/
static inline int SCI_Frame_PutChar(void * frame, int c);

/*!***************************************************************************
 * @brief	Function for inserting a byte in a SCI frame.
 * @param	frame The frame to insert the byte.
 * @param	byte The byte to insert.
 *****************************************************************************/
static inline void SCI_Frame_SetByte(sci_frame_t * frame, uint8_t byte);

/*!***************************************************************************
 * @brief	Inserts a signed byte (8-bit) into the TX buffer.
 * @details	The byte is checked for byte stuffing and thus it might additionally
 * 			add an escape byte to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The byte to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue08s(sci_frame_t * frame, int8_t data);

/*!***************************************************************************
 * @brief	Inserts a signed halfword (16-bit) into the TX buffer.
 * @details	The two bytes are checked for byte stuffing and thus it might
 * 			additionally add escape bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue16s(sci_frame_t * frame, int16_t data);

/*!***************************************************************************
 * @brief	Inserts a signed 3/4-word (24-bit) into the TX buffer.
 * @details	The three least significant bytes are added to the buffer. They are
 * 			checked for byte stuffing and thus it might additionally add escape
 * 			bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue24s(sci_frame_t * frame, int32_t data);

/*!***************************************************************************
 * @brief	Inserts a signed word (32-bit) into the TX buffer.
 * @details	The four bytes are checked for byte stuffing and thus it might
 * 			additionally add escape bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue32s(sci_frame_t * frame, int32_t data);

/*!***************************************************************************
 * @brief	Inserts a #ltc_t time stamp type into the TX buffer.
 * @details The time stamp is added as 6 byte wide value, i.e.
 * 			1. The seconds are added as uint32_t
 * 			2. The microseconds / 16 are added as uint16_t
 * 			.
 * 			The four bytes are checked for byte stuffing and thus it might
 * 			additionally add escape bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	t  The time stamp data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue_Time(sci_frame_t * frame, ltc_t const * t);


/*!***************************************************************************
 * @brief	Inserts a unsigned byte (8-bit) into the TX buffer.
 * @details	The byte is checked for byte stuffing and thus it might additionally
 * 			add an escape byte to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The byte to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue08u(sci_frame_t * frame, uint8_t data);

/*!***************************************************************************
 * @brief	Inserts a unsigned halfword (16-bit) into the TX buffer.
 * @details	The two bytes are checked for byte stuffing and thus it might
 * 			additionally add escape bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue16u(sci_frame_t * frame, uint16_t data);

/*!***************************************************************************
 * @brief	Inserts a unsigned 3/4-word (24-bit) into the TX buffer.
 * @details	The three least significant bytes are added to the buffer. They are
 * 			checked for byte stuffing and thus it might additionally add escape
 * 			bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue24u(sci_frame_t * frame, uint32_t data);

/*!***************************************************************************
 * @brief	Inserts a unsigned word (32-bit) into the TX buffer.
 * @details	The four bytes are checked for byte stuffing and thus it might
 * 			additionally add escape bytes to the buffer.
 * @param	frame The frame to put the data.
 * @param	data  The data to append.
 *****************************************************************************/
static inline void SCI_Frame_Queue32u(sci_frame_t * frame, uint32_t data);

/*!***************************************************************************
 * @brief	Takes a signed byte (8-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Signed byte.
 *****************************************************************************/
static inline int8_t SCI_Frame_Dequeue08s(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a signed halfword (16-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Signed 32-bit halfword.
 *****************************************************************************/
static inline int16_t SCI_Frame_Dequeue16s(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a signed 3/4-word (24-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Signed 24-bit data.
 *****************************************************************************/
static inline int32_t SCI_Frame_Dequeue24s(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a signed word (32-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Signed 32-bit word.
 *****************************************************************************/
static inline int32_t SCI_Frame_Dequeue32s(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a unsigned byte (8-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Unsigned byte.
 *****************************************************************************/
static inline uint8_t SCI_Frame_Dequeue08u(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a unsigned halfword (16-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Unsigned 16-bit halfword.
 *****************************************************************************/
static inline uint16_t SCI_Frame_Dequeue16u(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a unsigned 3/4-word (24-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Unsigned 24-bit 3/4-word.
 *****************************************************************************/
static inline uint32_t SCI_Frame_Dequeue24u(sci_frame_t * frame);

/*!***************************************************************************
 * @brief	Takes a unsigned word (32-bit) from the RX buffer.
 * @param	frame The frame to take the data from.
 * @return	Unsigned 32-bit word.
 *****************************************************************************/
static inline uint32_t SCI_Frame_Dequeue32u(sci_frame_t * frame);



/*******************************************************************************
 * Code
 ******************************************************************************/

#include "sci_datalink.h"
#include "sci_byte_stuffing.h"
#include "sci_crc8.h"
#include "sci_status.h"
#include <assert.h>

static inline int32_t SCI_Frame_TotalFrameLength (sci_frame_t const * frame)
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
static inline uint32_t SCI_Frame_BytesToRead(sci_frame_t const * frame)
{
	assert(frame != 0);
	uint32_t length = 0;
	while(frame != 0)
	{
		length += (uint32_t)(frame->WrPtr - frame->RdPtr);
		frame = frame->Next;
	}
	return length;
}
static inline uint8_t SCI_Frame_IsStartFrame(sci_frame_t const * frame)
{
	assert(frame != 0);
	return frame->Buffer[0] == SCI_START_BYTE;
}

static inline void SCI_Frame_SetByte(sci_frame_t * frame, uint8_t byte)
{
	assert(frame != 0);

	/* Iterate to the last frame in the queue. */
	while(frame->Next != 0)
	{
		frame = frame->Next;
	}

	/* Check if frame is full and enqueue another one. */
	if(frame->WrPtr - frame->Buffer == SCI_FRAME_SIZE)
	{
		frame->Next = SCI_DataLink_RequestTxFrame(false);
		frame = frame->Next;
		assert(frame != 0);
		if(!frame) return;
	}

	assert(frame != 0);
	assert(frame->Buffer == frame->RdPtr);
	assert(frame->RdPtr <= frame->WrPtr);
	assert(frame->WrPtr <= frame->Buffer + SCI_FRAME_SIZE);

	*(frame->WrPtr++) = byte;
}

static inline void SCI_Frame_Queue08u(sci_frame_t * frame, uint8_t data)
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
static inline void SCI_Frame_Queue16u(sci_frame_t * frame, uint16_t data)
{
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>8));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>0));
}
static inline void SCI_Frame_Queue24u(sci_frame_t * frame, uint32_t data)
{
	assert(data < 0x01000000U);
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>16));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>8));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>0));
}
static inline void SCI_Frame_Queue32u(sci_frame_t * frame, uint32_t data)
{
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>24));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>16));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>8));
	SCI_Frame_Queue08u(frame, (uint8_t)(data>>0));
}
static inline void SCI_Frame_Queue08s(sci_frame_t * frame, int8_t data)
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
static inline void SCI_Frame_Queue16s(sci_frame_t * frame, int16_t data)
{
	SCI_Frame_Queue08s(frame, (int8_t)(data>>8));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>0));
}
static inline void SCI_Frame_Queue24s(sci_frame_t * frame, int32_t data)
{
	SCI_Frame_Queue08s(frame, (int8_t)(data>>16));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>8));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>0));
}
static inline void SCI_Frame_Queue32s(sci_frame_t * frame, int32_t data)
{
	SCI_Frame_Queue08s(frame, (int8_t)(data>>24));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>16));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>8));
	SCI_Frame_Queue08s(frame, (int8_t)(data>>0));
}

static inline void SCI_Frame_Queue_Time(sci_frame_t * frame, ltc_t const * t)
{
	assert(frame != 0);
	assert(t != 0);
	SCI_Frame_Queue32u(frame, t->sec);			// s
	SCI_Frame_Queue16u(frame, t->usec >> 4U); 	// µs / 16
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
static inline int8_t SCI_Frame_Dequeue08s(sci_frame_t * frame)
{
	return (int8_t)SCI_Frame_GetByte(frame);
}
static inline int16_t SCI_Frame_Dequeue16s(sci_frame_t * frame)
{
	uint_fast16_t
	val  = (uint_fast16_t)(SCI_Frame_GetByte(frame) << 8U);
	val |= (uint_fast16_t)(SCI_Frame_GetByte(frame) << 0U);
	return (int16_t)val;
}
static inline int32_t SCI_Frame_Dequeue24s(sci_frame_t * frame)
{
	uint32_t
	val  = (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
	return (int32_t)val;
}
static inline int32_t SCI_Frame_Dequeue32s(sci_frame_t * frame)
{
	uint32_t
	val  = (uint32_t)(SCI_Frame_GetByte(frame) << 24U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
	return (int32_t)val;
}
static inline uint8_t SCI_Frame_Dequeue08u(sci_frame_t * frame)
{
	return (uint8_t)SCI_Frame_GetByte(frame);
}
static inline uint16_t SCI_Frame_Dequeue16u(sci_frame_t * frame)
{
	uint_fast16_t // uint16_t raises warning
	val  = (uint_fast16_t)(SCI_Frame_GetByte(frame) <<  8U);
	val |= (uint_fast16_t)(SCI_Frame_GetByte(frame) <<  0U);
	return (uint16_t)val;
}
static inline uint32_t SCI_Frame_Dequeue24u(sci_frame_t * frame)
{
	uint32_t
	val  = (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
	return val;
}
static inline uint32_t SCI_Frame_Dequeue32u(sci_frame_t * frame)
{
	uint32_t
	val  = (uint32_t)(SCI_Frame_GetByte(frame) << 24U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) << 16U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  8U);
	val |= (uint32_t)(SCI_Frame_GetByte(frame) <<  0U);
	return val;
}

static inline int SCI_Frame_PutChar(void * frame, int c)
{
	if(c == '\r') return 0;
	SCI_Frame_Queue08u((sci_frame_t*)frame, (uint8_t)c);
	return 0;
}

/*! @} */
#endif /* SCI_FRAME_H */
