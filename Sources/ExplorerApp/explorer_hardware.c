/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer example application.
 * @details		This file contains hardware initialization code.
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

#include "explorer_hardware.h"

#include "board/clock_config.h"
#include "driver/cop.h"
#include "driver/gpio.h"
#include "driver/flash.h"
#include "driver/timer.h"
#include "driver/s2pi.h"
#include "utility/debug.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#endif

#include <assert.h>


#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#define DEFAULT_SLAVE S2PI_PINS_LOW
#else
#define DEFAULT_SLAVE S2PI_S1
#endif

#define DEFAULT_BAUD_RATE 1000000U

status_t ExplorerApp_InitHardware(void)
{
#if WATCHDOG_ENABLED
	COP_Init();
#else
	COP_Disable();
#endif

	BOARD_ClockInit();
	GPIO_Init();
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	SLCD_Init();
	SLCD_DisplayBar();
#endif
	Timer_Init();

	/* Initialize the S2PI hardware. */
	status_t status = S2PI_Init(DEFAULT_SLAVE, DEFAULT_BAUD_RATE);
	if (status < STATUS_OK)
	{
		error_log("SPI driver 'initialization' failed, "
				  "error code: %d", status);
		return status;
	}

	status = Flash_Init();
	if (status < STATUS_OK)
	{
		error_log("Flash driver 'initialization' failed, "
				  "error code: %d", status);
		return status;
	}

	return status;
}
