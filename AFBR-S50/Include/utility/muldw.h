/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Provides algorithms for multiplying long data types
 * 				(uint32_t x uint32_t) on a 32-bit architecture.
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

#ifndef MULDW_H
#define MULDW_H

/*!***************************************************************************
 * @addtogroup 	misc
 * @{
 *****************************************************************************/

#include <stdint.h>

/*!***************************************************************************
 * @brief 	Long multiplication of two signed 32-bit into an 64-bit value on
 * 			32-bit architecture.
 *
 * @details	w (two words) gets the product of u and v (one word each).
 * 			w[0] is the most significant word of the result, w[1] the least.
 * 			(The words are in big-endian order).
 * 			It is derived from Knuth's Algorithm M from [Knu2] section 4.3.1.
 *
 * @see		http://www.hackersdelight.org/hdcodetxt/muldws.c.txt
 *
 * @param	w The result (u * v) value given as two signed 32-bit numbers:
 * 				w[0] is the most significant word of the result, w[1] the least.
 * 				(The words are in big-endian order).
 * @param	u Left hand side of the multiplication.
 * @param	v Right hand side of the multiplication.
 *****************************************************************************/
static inline void muldws(int32_t w[], int32_t u, int32_t v);

/*!***************************************************************************
 * @brief 	Long multiplication of two unsigned 32-bit into an 64-bit value on
 * 			32-bit architecture.
 *
 * @details	w (two words) gets the product of u and v (one word each).
 * 			w[0] is the most significant word of the result, w[1] the least.
 * 			(The words are in big-endian order).
 * 			It is Knuth's Algorithm M from [Knu2] section 4.3.1.
 * *
 * @see		http://www.hackersdelight.org/hdcodetxt/muldwu.c.txt
 *
 * @param	w The result (u * v) value given as two unsigned 32-bit numbers:
 * 				w[0] is the most significant word of the result, w[1] the least.
 * 				(The words are in big-endian order).
 * @param	u Left hand side of the multiplication.
 * @param	v Right hand side of the multiplication.
 *****************************************************************************/
static inline void muldwu(uint32_t w[], uint32_t u, uint32_t v);


static inline void muldws(int32_t w[], int32_t u, int32_t v)
{
   int32_t  u0, v0;
   uint32_t u1, v1, k, t;
   uint32_t w1, w2, w3;

   u0 = u >> 16U;
   u1 = u & 0xFFFF;
   v0 = v >> 16U;
   v1 = v & 0xFFFF;

   t = u1*v1;
   w3 = t & 0xFFFF;
   k = t >> 16U;

   t = (uint32_t)u0*v1 + k;
   w2 = t & 0xFFFF;
   w1 = (uint32_t)((int32_t)t >> 16U);

   t = u1*(uint32_t)v0 + w2;
   k = (uint32_t)((int32_t)t >> 16U);

   w[0] = u0*v0 + (int32_t)(w1 + k);
   w[1] = (int32_t)((t << 16U) + w3);
/* w[1] = u*v;                  // Alternative. */

   return;
}

static inline void muldwu(uint32_t w[], uint32_t u, uint32_t v)
{
	uint32_t u0, u1, v0, v1, k, t;
	uint32_t w1, w2, w3;

	u0 = u >> 16U;
	u1 = u & 0xFFFFU;
	v0 = v >> 16U;
	v1 = v & 0xFFFFU;

	t = u1*v1;
	w3 = t & 0xFFFFU;
	k = t >> 16U;

	t = u0*v1 + k;
	w2 = t & 0xFFFFU;
	w1 = t >> 16U;

	t = u1*v0 + w2;
	k = t >> 16U;

	w[0] = u0*v0 + w1 + k;
	w[1] = (t << 16U) + w3;
	/* w[1] = u*v;                  // Alternative. */

	return;
}

/*! @} */
#endif /* MULDW_H */
