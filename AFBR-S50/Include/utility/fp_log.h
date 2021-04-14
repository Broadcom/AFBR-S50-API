/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an logarithm function for fixed point type.
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

#ifndef FP_LOG_H
#define FP_LOG_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include "utility/fp_rnd.h"
#include <assert.h>

/*! The natural logarithm of 2 in UQ08.24 representation. */
#define FP_LOG24_2 (0x0B17218)

/*!***************************************************************************
 * @brief	Calculates the natural logarithm (base e) of an fixed point number.
 *
 * @details Calculates y = ln(x) = log_e(x) in fixed point representation.
 *
 *			Note that the result might not be 100 % accurate and might contain
 *			a small error!
 *
 * 			@see https://www.quinapalus.com/efunc.html
 *
 * @param	x The input parameter in unsigned fixed point format Q15.16.
 * @return	Result y = ln(x) in the UQ16.16 format.
 *****************************************************************************/
static inline q15_16_t fp_log16(uq16_16_t x);

static inline q15_16_t fp_log16(uq16_16_t x)
{
	if (x == UQ16_16_ONE) return 0;
	if (x == UQ16_16_E) return Q15_16_ONE;

	/* https://www.quinapalus.com/efunc.html */

	int32_t t = 0;
	int32_t y = 0x0A65AF68; // log(2^15) in Q7.24

	/* Use ln(x/2) = ln(x)-ln(2) for large numbers. */
	if (x > 0x80000000U)
	{
		x = fp_rndu(x, 1);
		y += FP_LOG24_2;
	}

	if(x<0x00008000) x<<=16,               y-=0x0B17217F;
	if(x<0x00800000) x<<= 8,               y-=0x058B90C0;
	if(x<0x08000000) x<<= 4,               y-=0x02C5C860;
	if(x<0x20000000) x<<= 2,               y-=0x0162E430;
	if(x<0x40000000) x<<= 1,               y-=0x00B17218;
	t=x+(x>> 1); if((t&0x80000000)==0) x=t,y-=0x0067CC90;
	t=x+(x>> 2); if((t&0x80000000)==0) x=t,y-=0x00391FF0;
	t=x+(x>> 3); if((t&0x80000000)==0) x=t,y-=0x001E2707;
	t=x+(x>> 4); if((t&0x80000000)==0) x=t,y-=0x000F8518;
	t=x+(x>> 5); if((t&0x80000000)==0) x=t,y-=0x0007E0A7;
	t=x+(x>> 6); if((t&0x80000000)==0) x=t,y-=0x0003F815;
	t=x+(x>> 7); if((t&0x80000000)==0) x=t,y-=0x0001FE03;
	t=x+(x>> 8); if((t&0x80000000)==0) x=t,y-=0x0000FF80;
	t=x+(x>> 9); if((t&0x80000000)==0) x=t,y-=0x00007FE0;
	t=x+(x>>10); if((t&0x80000000)==0) x=t,y-=0x00003FF8;
	t=x+(x>>11); if((t&0x80000000)==0) x=t,y-=0x00001FFE;

	/* Use residual to apply a final correction. */
	x=0x80000000-x;
	y-=x>>7;

	return fp_rnds(y, 8);
}

/*! @} */
#endif /* FP_DIV_H */
