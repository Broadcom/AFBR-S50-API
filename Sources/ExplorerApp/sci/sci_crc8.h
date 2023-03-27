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

#ifndef SCI_CRC8_H
#define SCI_CRC8_H

/*!***************************************************************************
 * @defgroup    sci_crc SCI: CRC8
 * @ingroup     sci
 * @brief       SCI CRC8 (Cyclic Redundancy Check)
 * @details     CRC8 checksum calculation with an optimized algorithm that
 *              utilizes an look-up table and proceeds a whole byte at each step.
 * @see         http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
 * @addtogroup  sci_crc
 * @{
 *****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*!***************************************************************************
 * @brief   Initialization of the CRC8 module.
 * @details Prepares the CRC8 lookup table.
 *****************************************************************************/
void SCI_CRC8_Init(void);

/*!***************************************************************************
 * @brief   Calculation routine for the CRC8 checksum.
 * @details Uses an optimized algorithm that utilizes an look-up table with
 *          256 entries of 8-bit values (total: 256 bytes) and
 *          proceeds a whole byte at each step.
 * @see     http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
 * @param   crc          The previous CRC8. If starting, pass zero.
 * @param   data         Pointer to the send or receive array.
 * @param   length       Number of bytes in the frame array.
 * @return  Returns the CRC8 checksum.
 *****************************************************************************/
uint8_t SCI_CRC8_Compute(uint8_t crc, const uint8_t * data, size_t length);

/*! @} */
#endif /* SCI_CRC8_H */
