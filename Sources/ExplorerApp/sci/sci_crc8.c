/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Application.
 * @details     This file provides a CRC8 algorithm for the systems
 *              communication interface.
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
 *****************************************************************************/



/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "sci_crc8.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! CRC8_SAE_J1850_ZERO */
#define CRC8GENERATOR   ((uint8_t)0x1D)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/

/*! CRC8 lookup table; filled on initialization of the module. */
static uint8_t myCRC8Lookup[256];

/*******************************************************************************
 * Code
 ******************************************************************************/
void SCI_CRC8_Init(void)
{
    // prepare CRC lookup table
    for (unsigned int i = 0; i < 256; i++)
    {
        uint8_t b = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((b & 0x80U) != 0)
            {
                b = (uint8_t)(b << 1U);
                b ^= CRC8GENERATOR;
            }
            else
            {
                b = (uint8_t)(b << 1U);
            }
        }
        /* store CRC value in lookup table */
        myCRC8Lookup[i] = b;
    }
}

uint8_t SCI_CRC8_Compute(uint8_t crc, const uint8_t * data, size_t length)
{
    // CRC8_SAE_J1850_ZERO
    while (length--)
        crc = myCRC8Lookup[crc ^ (*data++)];
    return crc;
}
