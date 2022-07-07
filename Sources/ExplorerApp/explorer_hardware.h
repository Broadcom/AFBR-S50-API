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

#ifndef EXPLORER_HARDWARE_H
#define EXPLORER_HARDWARE_H

/*!***************************************************************************
 * @defgroup	explorer_init Hardware Initialization Module
 * @ingroup		explorer_app
 * @brief		Hardware Initialization Module
 * @details		Functions to initialize all the required hardware.
 * @addtogroup 	explorer_init
 * @{
 *****************************************************************************/

#include "explorer_app.h"



/*!***************************************************************************
 * @brief	Initialization of hardware drivers.
 * @details	Initializes all the hardware of the given board.
 * 			- Watchdog timer
 * 			- Clocks
 * 			- GPIO
 * 			- LCD
 * 			- Timers
 * 			- SPI
 * 			- Flash
 * 			.
 * @param	cfg The configuration containing SPI parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_InitHardware(explorer_cfg_t * cfg);



/*! @} */
#endif /* EXPLORER_HARDWARE_H */
