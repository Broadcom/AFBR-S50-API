/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Demo Application Version File
 *
 * @copyright
 *
 * Copyright (c) 2022, Broadcom Inc
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

#ifndef CAN_APP_VERSION_H
#define CAN_APP_VERSION_H

/*!***************************************************************************
 * @defgroup    can_version CAN Application Version
 * @ingroup     can_app
 *
 * @brief       AFBR-S50 CAN Application version number
 *
 * @details     Contains the AFBR-S50 CAN Application version number.
 *
 * @addtogroup  can_version
 * @{
 *****************************************************************************/

/*! Major version number of the CAN Application. */
#define CAN_APP_VERSION_MAJOR    0

/*! Minor version number of the CAN Application. */
#define CAN_APP_VERSION_MINOR    1

/*! Bugfix version number of the CAN Application. */
#define CAN_APP_VERSION_BUGFIX   0

/*****************************************************************************/

/*! Construct the version number for drivers. */
#define MAKE_VERSION(major, minor, bugfix) \
    (((major) << 24) | ((minor) << 16) | (bugfix))

/*! Version number of the CAN Application. */
#define CAN_APP_VERSION MAKE_VERSION((CAN_APP_VERSION_MAJOR), \
                                     (CAN_APP_VERSION_MINOR), \
                                     (CAN_APP_VERSION_BUGFIX))

/*! @} */
#endif /* CAN_APP_VERSION_H */
