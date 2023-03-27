/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides drivers for SLCD display
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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
 *
 *****************************************************************************/

#ifndef SLCD_H
#define SLCD_H

/*!***************************************************************************
 * @defgroup    SLCD SLCD: Segment LCD
 * @ingroup     driver
 * @brief       SLCD Hardware Layer Module
 * @details     Driver functionality for SLCD Module.
 * @addtogroup  SLCD
 * @{
 *****************************************************************************/

#include <stdint.h>

/*!***************************************************************************
 * @brief   Initializes the SLCD driver and setup pin muxing.
 *****************************************************************************/
void SLCD_Init(void);

/*!***************************************************************************
 * @brief   Displays a unsigned integer in decimal mode.
 * @param   value The integer to display.
 *****************************************************************************/
void SLCD_DisplayDecimalUnsigned(uint16_t value);

/*!***************************************************************************
 * @brief   Displays a signed integer in decimal mode.
 * @param   value The integer to display.
 *****************************************************************************/
void SLCD_DisplayDecimalSigned(int16_t value);

/*!***************************************************************************
 * @brief   Displays the decimal point at a given position on the SLCD.
 * @param   pos The position of the decimal point:
 *                  - 0 = off;
 *                  - 1, 2, 3 = display at position 1, 2, 3;
 *                  - 4 = display colon at pos 2.
 *****************************************************************************/
void SLCD_SetDecimalPointPosition(uint8_t pos);

/*!***************************************************************************
 * @brief   Displays "----" on the SLCD.
 *****************************************************************************/
void SLCD_DisplayBar(void);

/*!***************************************************************************
 * @brief   Clear the SLCD Display.
 *****************************************************************************/
void SLCD_ClearDisplay(void);

/*!***************************************************************************
 * @brief   Displays an error number on the SLCD.
 * @param   error An 8bit unsigned integer that represents the error number.
 *****************************************************************************/
void SLCD_DisplayError(uint8_t error);



/*! @} */
#endif /* SLCD_H */
