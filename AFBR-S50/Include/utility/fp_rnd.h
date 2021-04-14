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

#ifndef FP_RND_H
#define FP_RND_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include <assert.h>

/*!***************************************************************************
 * @brief	Converting with rounding from UQx.n1 to UQx.n2.
 * @details	Equivalent to dividing by 2^n with correct rounding to unsigned
 * 			integer values.
 * @param	Q The number in (U)Qx.n1 fixed point format to be rounded.
 * @param	n The number of bits to be truncated/rounded,
 * 			    e.g. UQ8.8 -> UQ12.4 => n = 8 - 4 = 4.
 * @return	The rounded value in (U)Qx.n2 format.
 *****************************************************************************/
static inline uint32_t fp_rndu(uint32_t Q, uint_fast8_t n);
static inline uint32_t fp_rndu(uint32_t Q, uint_fast8_t n)
{
	if (n == 0)
		return Q;
	else if (n == 32U)
		return Q > 0x7FFFFFFFU ? 1U : 0U;
	else if (n > 32U)
		return 0;

	uint32_t tmp = (1U << (n - 1U));
	if (Q > UINT32_MAX - tmp)
		return (Q >> n) + 1U;
	else
		return ((Q + tmp) >> n);
}

/*!***************************************************************************
 * @brief	Converting with rounding from Qx.n1 to Qx.n2.
 * @details	Equivalent to dividing by 2^n with correct rounding to integer
 * 			values.
 * @param	Q The number in (U)Qx.n1 fixed point format to be rounded.
 * @param	n The number of bits to be truncated/rounded,
 * 			    e.g. Q7.8 -> Q11.4 => n = 8 - 4 = 4.
 * @return	The rounded value in (U)Qx.n2 format.
 *****************************************************************************/
static inline int32_t fp_rnds(int32_t Q, uint_fast8_t n);
static inline int32_t fp_rnds(int32_t Q, uint_fast8_t n)
{
	return (Q < 0) ? -fp_rndu(-Q, n) : fp_rndu(Q, n);
}

/*! @} */
#endif /* FP_RND_H */
