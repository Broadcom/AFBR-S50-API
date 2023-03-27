/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the main functionality of the AFBR-S50
 *              Explorer Application.
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

#ifndef EXPLORER_APP_CONFIG_H
#define EXPLORER_APP_CONFIG_H

/*!***************************************************************************
 * @defgroup    explorer_config AFBR-S50 Explorer Application - Config
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer Application - Config
 * @details     Contains the static configuration of the Explorer App
 *
 * @addtogroup  explorer_config
 * @{
 *****************************************************************************/

#include "board/board_config.h"

/*!***************************************************************************
 *  The maximum number of instantiated time-of-flight sensor devices.
 *****************************************************************************/
#ifndef EXPLORER_DEVICE_COUNT
#define EXPLORER_DEVICE_COUNT    2
#endif

/*!***************************************************************************
 *  The minimum device ID supported;
 *  Note: skips the 0 as default device address.
 *****************************************************************************/
#define EXPLORER_DEVICE_ID_MIN   1

/*!***************************************************************************
 *  The maximum device ID supported; equivalent to the available S2PI slaves.
 *  Note: includes the 0 default device address.
 *****************************************************************************/
#define EXPLORER_DEVICE_ID_MAX   S2PI_SLAVE_COUNT

/*!***************************************************************************
 *  The default sensor that will be paired with the DEVICEID_DEFAULT.
 *  Note: the Explorer index starts from 0 (C array) and the DEVICE ID
 *  starts from 1, according to the SCI specs.
 *****************************************************************************/
#define DEFAULT_EXPLORER    (SPI_DEFAULT_SLAVE - 1U)


/*! @} */
#endif /* EXPLORER_APP_CONFIG_H */
