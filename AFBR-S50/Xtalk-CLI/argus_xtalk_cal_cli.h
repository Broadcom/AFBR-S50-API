/*************************************************************************//**
 * @file
 * @brief   Provides an interactive crosstalk calibration CLI to the AFBR-S50 API.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARGUS_XTALK_CAL_CLI_H
#define ARGUS_XTALK_CAL_CLI_H

/*!***************************************************************************
 * @defgroup    argus_xtk_cli Crosstalk Calibration CLI
 * @ingroup     argus
 *
 * @brief       A CLI module to interactively run crosstalk calibration.
 *
 * @details     This interactive procedure guides through the steps needed to
 *              compensate the application specific crosstalk (xtalk) using
 *              the AFBR-S50 API.
 *
 *              Please read application note
 *              [AFBR-S50-XTK-Crosstalk-Guide](https://docs.broadcom.com/docs/AFBR-S50-XTK-Crosstalk-Guide)
 *              to get more information.
 *
 * @addtogroup  argus_xtk_cli
 * @{
 *****************************************************************************/

#include "argus.h"

/*!***************************************************************************
 * @brief   Version number of the xtalk calibration CLI.
 *
 * @details Changes:
 *          * v1.0:
 *              - Initial release.
 *
 *****************************************************************************/
#define ARGUS_XTALK_CAL_CLI_VERSION "v1.0"

/*!***************************************************************************
 * @brief   The xtalk measurement amplitude limits in LSB for the electrical part.
 *
 * @note    These values can but should not be increased to meet target sensor
 *          performance.
 ******************************************************************************/
#define ELEC_XTK_AMPL_LMT (25U)

/*!***************************************************************************
 * @brief   The xtalk measurement amplitude limits in LSB for the optical part.
 *
 * @note    These values can but should not be increased to meet target sensor
 *          performance.
 ******************************************************************************/
#define OPT_ELEC_XTK_AMPL_LMT (200U)

/*!***************************************************************************
 * @brief   Wavelength dependent transmission of a cover glass.
 *
 * @details Wavelength dependent transmission or transmittance factor of the
 *          specific cover glass.
 *
 *          Example: Gorilla glass has 92% transmittance @ 850nm which is
 *          translated as UQ0.8 by `0.92 * 2^8 = 236`
 *
 * @note    * AFBR-S5xxx85x devises use 850nm,
 *          * AFBR-S5xxx68x use 680nm
 *
 ******************************************************************************/
#define T_GLASS (236U)

/*!***************************************************************************
 * @brief   Interactive Xtalk Calibration Procedure CLI
 *
 * @details This interactive procedure guides through the steps needed to
 *          compensate the application specific crosstalk (xtalk) using the
 *          AFBR-S50 API.
 *
 *          Please read application note
 *          [AFBR-S50-XTK-Crosstalk-Guide](https://docs.broadcom.com/docs/AFBR-S50-XTK-Crosstalk-Guide)
 *          to get more information.
 ******************************************************************************/
void Argus_XtalkCalibration_CLI(argus_hnd_t * hnd);

/*! @} */
#endif /* ARGUS_XTALK_CAL_CLI */
