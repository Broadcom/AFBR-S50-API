/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains current Explorer Application version number.
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

#ifndef EXPLORER_VERSION_H
#define EXPLORER_VERSION_H

/*!***************************************************************************
 * @defgroup    explorer_version Explorer Version
 * @ingroup     explorer_app
 *
 * @brief       Explorer Application Code Version.
 *
 * @details     Provides a version number for Explorer Application.
 *
 * @addtogroup  explorer_version
 * @{
 *****************************************************************************/

/*! @brief Major version number of the platform code. */
#define EXPLORER_VERSION_MAJOR    1

/*! @brief Minor version number of the platform code. */
#define EXPLORER_VERSION_MINOR    5

/*! @brief Bugfix version number of the platform code. */
#define EXPLORER_VERSION_BUGFIX   6

/*! Build version number of the AFBR-S50 API. */
#define EXPLORER_VERSION_BUILD    "20240208081753"

/*****************************************************************************/

/*! @brief Construct the version number for drivers. */
#define MAKE_VERSION(major, minor, bugfix) \
    (((major) << 24) | ((minor) << 16) | (bugfix))

/*! @brief the current version of the platform code. */
#define EXPLORER_VERSION MAKE_VERSION((EXPLORER_VERSION_MAJOR), \
                                      (EXPLORER_VERSION_MINOR), \
                                      (EXPLORER_VERSION_BUGFIX))

/*! @} */
#endif /* EXPLORER_VERSION_H */
