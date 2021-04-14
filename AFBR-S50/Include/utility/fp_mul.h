/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Provides definitions and basic macros for fixed point data types.
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

#ifndef FP_MUL_H
#define FP_MUL_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include "utility/fp_rnd.h"
#include "utility/muldw.h"

/*!***************************************************************************
 * @brief	64-bit implementation of an multiplication with fixed point format.
 *
 * @details	Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 * 			number of 32-bit width. The algorithm ins considering the sign of
 * 			the input values. The multiplication is done in 64-bit and the
 * 			result is shifted down by the passed shift parameter. The shift
 * 			is executed with correct rounding.
 *
 * 			Note that the result must fit into the 32-bit value. An assertion
 * 			error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param	u The left parameter in Qx1.y1 format
 * @param	v The right parameter in Qx2.y2 format
 * @param	shift The final right shift (rounding) value.
 * @return	Result = (a*b)>>shift in Qx.(y1+y2-shift) format.
 *****************************************************************************/
static inline int32_t fp_muls(int32_t u, int32_t v, uint_fast8_t shift);

/*!***************************************************************************
 * @brief	64-bit implementation of an multiplication with fixed point format.
 *
 * @details	Algorithm to evaluate a*b, where a and b are arbitrary fixed point
 * 			number of 32-bit width. The multiplication is done in 64-bit and
 * 			the result is shifted down by the passed shift parameter. The shift
 * 			is executed with correct rounding.
 *
 * 			Note that the result must fit into the 32-bit value. An assertion
 * 			error occurs otherwise (or undefined behavior of no assert available).
 *
 * @param	u The left parameter in UQx1.y1 format
 * @param	v The right parameter in UQx2.y2 format
 * @param	shift The final right shift (rounding) value.
 * @return	Result = (a*b)>>shift in UQx.(y1+y2-shift) format.
 *****************************************************************************/
static inline uint32_t fp_mulu(uint32_t u, uint32_t v, uint_fast8_t shift);

static inline uint32_t fp_mul_u32_u16(uint32_t u, uint16_t v, uint_fast8_t shift);

static inline int32_t fp_mul_s32_u16(int32_t u, uint16_t v, uint_fast8_t shift);



static inline uint32_t fp_mul_u32_u16(uint32_t u, uint16_t v, uint_fast8_t shift)
{
	assert(shift <= 48);

	if (shift > 16)
	{
		uint32_t msk = 0xFFFFU;
		uint32_t a = (u >> 16U) * v;
		uint32_t b = (msk & u) * v;
		return fp_rndu(a, shift - 16) + fp_rndu(b, shift);
	}
	else
	{
		uint32_t msk = ~(0xFFFFFFFFU << shift);
		uint32_t a = (u >> shift) * v;
		uint32_t b = fp_rndu((msk & u) * v, shift);
		return a + b;
	}
}

static inline int32_t fp_mul_s32_u16(int32_t u, uint16_t v, uint_fast8_t shift)
{
	return u >= 0 ? fp_mul_u32_u16(u, v, shift) : - fp_mul_u32_u16(-u, v, shift);
}

static inline uint32_t fp_mulu(uint32_t u, uint32_t v, uint_fast8_t shift)
{
	assert(shift <= 32);

	uint32_t tmp[2] = {0};
	muldwu(tmp, u, v);

	assert(shift ? tmp[0] <= (UINT32_MAX >> (32-shift)) : tmp[0] == 0);

	if(32-shift)
		return ((tmp[0] << (32-shift)) + fp_rndu(tmp[1], shift));
	else
		return tmp[1] > (UINT32_MAX>>1) ? tmp[0] + 1 : tmp[0];
}

static inline int32_t fp_muls(int32_t u, int32_t v, uint_fast8_t shift)
{
	int_fast8_t sign = 1;

	uint32_t u2, v2;
	if(u < 0) { u2 = -u; sign = -sign; } else { u2 = u; }
	if(v < 0) { v2 = -v; sign = -sign; } else { v2 = v; }

	uint32_t res = fp_mulu(u2, v2, shift);

	assert(sign > 0 ? res <= 0x7FFFFFFFU : res <= 0x80000000U);

	return sign > 0 ? res : -res;
}

/*! @} */
#endif /* FP_MUL_H */
