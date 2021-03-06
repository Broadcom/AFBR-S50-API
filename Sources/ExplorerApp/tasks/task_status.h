/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer example application.
 * @details		This file provides status definitions.
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

#ifndef TASK_STATUS_H
#define TASK_STATUS_H

/*!***************************************************************************
 * @defgroup	status Status Codes
 * @brief		Status Codes Definitions
 * @details		Defines status codes for specific functions.
 * @addtogroup 	status
 * @{
 *****************************************************************************/

#include "utility/status.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Task scheduler status and error return codes.
 *  @ingroup status */
enum StatusTask
{
	/*! Task has not finished and needs to be postponed. Lower priority tasks
	 *  are executed meanwhile. */
	STATUS_TASK_POSTPONE		= 231,

	/*! Task queue is full. Event not queued. */
	ERROR_TASK_QUEUE_FULL		= -231,
};


/*! @} */
#endif /* TASK_STATUS_H */
