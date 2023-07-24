/*************************************************************************//**
 * @file
 * @brief       This file is part of the MKL46z/MKL17z platform layer.
 * @details     This file provides generic board abstraction.
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

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*!***************************************************************************
 * @defgroup    boardcfg Board Configuration
 * @ingroup     platform
 * @brief       Board Configuration
 * @details     Board/Platform Depended Definitions.
 * @addtogroup  boardcfg
 * @{
 *****************************************************************************/


/*! The board name. */
#define BOARD_NAME          "RA4M2"

/*****************************************************************************
 * Board UART configuration
 *****************************************************************************/

/*! The UART baud rate in bps */
#ifndef UART_BAUDRATE
#define UART_BAUDRATE       115200U
#endif


/*****************************************************************************
 * Board SPI configuration
 *****************************************************************************/

/*! Define the maximum SPI baud rate (to be used in the SPI module).
 *  This is dependent of the available peripheral. */
#ifndef SPI_MAX_BAUDRATE
#define SPI_MAX_BAUDRATE    21000000
#endif

/*! Define the current SPI baud rate (to be used in the SPI module).
 *  This is dependent of the available peripheral. */
#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE        SPI_MAX_BAUDRATE
#endif

/*! The number of available S2PI slaves for the Renesas board. */
#define S2PI_SLAVE_COUNT    1

/*! Dummy Slave. */
#define S2PI_SLAVE_NONE     0

/*! Multi-Device Board Slave 1 on SPI1; Default Slave. */
#define S2PI_SLAVE1         1

/*! Define the default SPI slave for device.
 *  The slave is used for SPI initialization only! */
#ifndef SPI_DEFAULT_SLAVE
#define SPI_DEFAULT_SLAVE   S2PI_SLAVE1
#endif

/*! @} */
#endif /* BOARD_CONFIG_H */
