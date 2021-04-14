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

#ifndef FP_DIV_H
#define FP_DIV_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include "int_math.h"

/*!***************************************************************************
 * @brief	32-bit implementation of an Q15.16 division.
 *
 * @details	Algorithm to evaluate a/b, where b is in Q15.16 format, on a 32-bit
 * 			architecture with maximum precision. This does the division manually,
 * 			and is therefore good for processors that do not have hardware division.
 * 			The result is correctly rounded and given as the input format.
 * 			Division by 0 yields 0 (assert in debug configuration).
 * 			Too high/low results are truncated to max/min values (and asserted
 * 			in debug configuration).
 *
 * 			@see https://code.google.com/archive/p/libfixmath
 *
 * @param	a Numerator in any Qx.y format
 * @param	b Denominator in Q15.16 format
 * @return	Result = a/b in the same Qx.y format as the input parameter a.
 *****************************************************************************/
static inline int32_t fp_div16(int32_t a, q15_16_t b);


static inline int32_t fp_div16(int32_t a, q15_16_t b)
{
	// This uses the basic binary restoring division algorithm.
	// It appears to be faster to do the whole division manually than
	// trying to compose a 64-bit divide out of 32-bit divisions on
	// platforms without hardware divide.

	if (b == 0)
	{
		//assert(0); // division by 0
		return 0; // division by 0
	}

	uint32_t remainder = absval(a);
	uint32_t divider   = absval(b);

	uint32_t quotient = 0;
	uint32_t bit = 0x10000U;

	/* The algorithm requires D >= R */
	while (divider < remainder)
	{
		divider <<= 1U;
		bit <<= 1U;
	}

	if (!bit)
	{
		if ((uint32_t)(a ^ b) & 0x80000000U) // return truncated values
		{
			return INT32_MIN;
		}
		else
		{
			return INT32_MAX;
		}
	}

	if (divider & 0x80000000U)
	{
		// Perform one step manually to avoid overflows later.
		// We know that divider's bottom bit is 0 here.
		if (remainder >= divider)
		{
			quotient |= bit;
			remainder -= divider;
		}
		divider >>= 1U;
		bit >>= 1U;
	}

	/* Main division loop */
	while (bit && remainder)
	{
		if (remainder >= divider)
		{
			quotient |= bit;
			remainder -= divider;
		}
		remainder <<= 1U;
		bit >>= 1U;
	}

	if (remainder >= divider)
	{
		quotient++;
	}

	uint32_t result = quotient;

	/* Figure out the sign of result */
	if ((uint32_t)(a ^ b) & 0x80000000U)
	{
		result = -result;
	}

	return (int32_t)result;
}

/*! @} */
#endif /* FP_DIV_H */
