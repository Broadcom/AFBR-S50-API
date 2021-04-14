/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an exponential function for fixed point type.
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

#ifndef FP_EXP_H
#define FP_EXP_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include "utility/fp_rnd.h"
#include "utility/fp_mul.h"
#include "utility/fp_div.h"

/*! The maximum number for the input of the fp_exp16 function that gives a
 *  result still larger than 0 (in UQ16.16 representation). */
#define FP_EXP16_MAX (0x000B1721)

/*! The minimum number for the input of the fp_exp16 function that gives a
    result smaller than the maximum of the UQ16.16 format.  */
#define FP_EXP16_MIN (-0x000BC894)

/*! The maximum number for the input of the fp_exp24 function that gives a
 *  result still larger than 0 (in UQ08.24 representation). */
#define FP_EXP24_MAX (0x058B90BF)

/*!***************************************************************************
 * @brief	Calculates the exponential of an fixed point number.
 *
 * @details Calculates y = exp(x) in fixed point representation.
 *
 *			Note that the result might not be 100 % accurate and might contain
 *			a small error!
 *
 * 			@see https://www.quinapalus.com/efunc.html
 *
 * @param	x The input parameter in unsigned fixed point format Q15.16.
 * @return	Result y = exp(x) in the UQ16.16 format.
 *****************************************************************************/
static inline uq16_16_t fp_exp16(q15_16_t x);



static inline uint32_t fp_exp24(uint32_t x)
{
	/* https://www.quinapalus.com/efunc.html */
	int32_t t = 0;
	uint32_t y = 0x01000000;

	t=x-0x058B90C0; if(t>=0) x=t, y<<=8;
	t=x-0x02C5C860; if(t>=0) x=t, y<<=4;
	t=x-0x0162E430; if(t>=0) x=t, y<<=2;
	t=x-0x00B17218; if(t>=0) x=t, y<<=1;
	t=x-0x0067CC90; if(t>=0) x=t, y+=y>> 1;
	t=x-0x00391FF0; if(t>=0) x=t, y+=y>> 2;
	t=x-0x001E2707; if(t>=0) x=t, y+=y>> 3;
	t=x-0x000F8518; if(t>=0) x=t, y+=y>> 4;
	t=x-0x0007E0A7; if(t>=0) x=t, y+=y>> 5;
	t=x-0x0003F815; if(t>=0) x=t, y+=y>> 6;
	t=x-0x0001FE03; if(t>=0) x=t, y+=y>> 7;
	t=x-0x0000FF80; if(t>=0) x=t, y+=y>> 8;
	t=x-0x00007FE0; if(t>=0) x=t, y+=y>> 9;
	t=x-0x00003FF8; if(t>=0) x=t, y+=y>>10;
	t=x-0x00001FFE; if(t>=0) x=t, y+=y>>11;
	if(x&0x00001000)              y+=y>>12;
	if(x&0x00000800)              y+=y>>13;
	if(x&0x00000400)              y+=y>>14;
	if(x&0x00000200)              y+=y>>15;
	if(x&0x00000100)              y+=y>>16;
	if(x&0x00000080)              y+=y>>17;
	if(x&0x00000040)              y+=y>>18;
	if(x&0x00000020)              y+=y>>19;
	if(x&0x00000010)              y+=y>>20;
	if(x&0x00000008)              y+=y>>21;
	if(x&0x00000004)              y+=y>>22;
	if(x&0x00000002)              y+=y>>23;
	if(x&0x00000001)              y+=y>>24;

	return y;
}

static inline uq16_16_t fp_exp16(q15_16_t x)
{
	if (x == 0) return Q15_16_ONE;
	if (x == UQ16_16_ONE) return UQ16_16_E;
	if (x > FP_EXP16_MAX) return UQ16_16_MAX;
	if (x < FP_EXP16_MIN) return 0;

	if (x > 0)
	{
		if ((x << 8U) < FP_EXP24_MAX)
		{
			return fp_rndu(fp_exp24(x << 8U), 8U);
		}
		else
		{
			/* Use exp(2*x) = exp(x)^2 for large input number. */
			uint32_t y = fp_exp24(x << 7U);
			return fp_mulu(y, y, 32U); // 24 + 8
		}
	}
	else
	{
		/* Use exp(-x) = 1/exp(x) for negative numbers. */
		x = -x;

		if ((x << 8U) < FP_EXP24_MAX)
		{
			uint32_t y = fp_exp24(x << 8U);
			return fp_div16(Q15_16_ONE << 7U, fp_rndu(y, 1));
		}
		else
		{
			/* Use exp(2*x) = exp(x)^2 for large input number. */
			uint32_t y = fp_exp24(x << 7U);
			y = fp_div16(Q15_16_ONE << 7U, fp_rndu(y, 1));
			return fp_mulu(y, y, 16);
		}
	}
}

/*! @} */
#endif /* FP_DIV_H */
