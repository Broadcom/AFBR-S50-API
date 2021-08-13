/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides debug functionality.
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

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "board/board_config.h"
#include "debug.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void Debug_CheckReset(void)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4) || \
    defined(CPU_MKL17Z128VFM4) || defined(CPU_MKL17Z128VFT4) || defined(CPU_MKL17Z128VLH4) || defined(CPU_MKL17Z128VMP4)
	// Check what reset has occurred
    uint32_t srs0 = (uint32_t)RCM->SRS0;
    uint32_t srs1 = (uint32_t)RCM->SRS1;

    if(srs0 & RCM_SRS0_WAKEUP_MASK)
    {
		error_log(" >>> Reset due to a \"Low Leakage Wakeup Reset\"! <<< ");
    }
	if(srs0 &  RCM_SRS0_LVD_MASK)
	{
		error_log(" >>> Reset due to a \"Low-Voltage Detect Reset\"! <<< ");
	}
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	if(srs0 & RCM_SRS0_LOC_MASK)
	{
		error_log(" >>> Reset due to a \"Loss-of-Clock Reset\"! <<< ");
	}
	if(srs0 & RCM_SRS0_LOL_MASK)
	{
		error_log(" >>> Reset due to a \"Loss-of-Lock Reset\"! <<< ");
	}
#endif
	if(srs0 &  RCM_SRS0_WDOG_MASK)
	{
		error_log(" >>> Reset due to a \"Watchdog Reset\"! <<< ");
	}
	if(srs0 &  RCM_SRS0_PIN_MASK)
	{
		print(" >>> Reset due to a \"External Reset Pin\"! <<< ");
	}
	if(srs0 & RCM_SRS0_POR_MASK)
	{
		print(" >>> Reset due to a \"Power-On Reset\"! <<< ");
	}

    /*! @name SRS1 - System Reset Status Register 1 */
    if(srs1 & RCM_SRS1_LOCKUP_MASK)
    {
		error_log(" >>> Reset due to a \"Core Lockup\"! <<< ");
    }
    if(srs1 &  RCM_SRS1_SW_MASK)
    {
		print(" >>> Reset due to a \"Software Reset\"! <<< ");
    }
    if(srs1 &  RCM_SRS1_MDM_AP_MASK)
    {
    	// Reset after flashing
    	print(" >>> Reset due to a \"MDM-AP System Reset Request\"! <<< ");
    }
    if(srs1 &  RCM_SRS1_SACKERR_MASK)
    {
		error_log(" >>> Reset due to a \"Stop Mode Acknowledge Error Reset\"! <<< ");
    }
#endif
}

#ifndef NDEBUG
#if (defined(__CC_ARM)) || (defined(__ICCARM__))
void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Display error on LCD. */
	SLCD_DisplayError(0xAA);
#endif

	/* Try to send printf message. */
	print("ASSERT: expression \"%s\" failed;\n"
			"file \"%s\";\n"
			"line \"%d\";\n",
			failedExpr, file, line);

	/* Wait for sending print statement */
	for (volatile uint32_t i = 0; i < 1000000; i++) __asm("nop");

	/* Stop. */
	for (;;) BREAKPOINT();
}
#elif(defined(__REDLIB__))
void __assertion_failed(char *_Expr)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Display error on LCD. */
	SLCD_DisplayError(0xAA);
#endif

	/* Try to send printf message. */
	print("ASSERT: \"%s\"", _Expr);

	/* Wait for sending print statement */
	for (volatile uint32_t i = 0; i < 1000000; i++) __asm("nop");

	/* Stop. */
	for (;;) BREAKPOINT();
}
#elif(defined(__GNUC__))
void __assert_func(const char *file, int line, const char *func, const char *failedExpr)
{
	static bool is_asserting = false;
	if (!is_asserting)
	{
		is_asserting = true;

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		/* Display error on LCD. */
		SLCD_DisplayError(0xAA);
#endif

		/* Try to send printf message. */
		print("ASSERT: expression \"%s\" failed;\n"
			  "file \"%s\";\n"
			  "line \"%d\";\n"
			  "function \"%s\";\n",
			  failedExpr, file, line, func);

		/* Wait for sending print statement */
		for (volatile uint32_t i = 0; i < 1000000; i++) __asm("nop");
	}

	/* Stop. */
	for (;;) BREAKPOINT();

}
#endif /* (defined(__CC_ARM)) ||  (defined (__ICCARM__)) */
#endif /* NDEBUG */
