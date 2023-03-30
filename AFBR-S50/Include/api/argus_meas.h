/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 hardware API.
 * @details     Defines the generic measurement parameters and data structures.
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

#ifndef ARGUS_MEAS_H
#define ARGUS_MEAS_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_meas Measurement/Device Control
 * @ingroup     argus_api
 *
 * @brief       Measurement/Device control module
 *
 * @details     This module contains measurement and device control specific
 *              definitions and methods.
 *
 * @addtogroup  argus_meas
 * @{
 *****************************************************************************/

#include "argus_dca.h"
#include "argus_def.h"

/*! Number of raw data values. */
#define ARGUS_RAW_DATA_VALUES 132U // 33 channels * 4 phases

/*! Size of the raw data in bytes. */
#define ARGUS_RAW_DATA_SIZE (3U * ARGUS_RAW_DATA_VALUES) // 3 bytes * 33 channels * 4 phases

/*! The number channels for auxiliary measurements readout. */
#define ARGUS_AUX_CHANNEL_COUNT (5U)

/*! Size of the auxiliary data in bytes. */
#define ARGUS_AUX_DATA_SIZE (3U * ARGUS_AUX_CHANNEL_COUNT) // 3 bytes * x channels * 1 phase

/*!***************************************************************************
 * @brief   The device measurement configuration structure.
 * @details The portion of the configuration data that belongs to the
 *          measurement cycle. I.e. the data that defines a measurement frame.
 *****************************************************************************/
typedef struct argus_meas_frame_t
{
    /*! Frame integration time in microseconds.
     *  The integration time determines the measured time between
     *  the start signal and the IRQ. Note that this value will be
     *  slightly larger than the actual integration time since the
     *  watch is started before the SPI transfer and stopped in the
     *  IRQ service routine which also might be delayed due to higher
     *  priority tasks. */
    uint32_t IntegrationTime;

    /*! Pixel enabled mask for the 32 pixels sorted
     *  by x-y-indices.
     *  See [pixel mapping](@ref argus_map) for more
     *  details on the pixel mask. */
    uint32_t PxEnMask;

    /*! ADS channel enabled mask for the remaining
     *  channels 31 .. 63 (miscellaneous values).
     *  See [pixel mapping](@ref argus_map) for more
     *  details on the ADC channel mask. */
    uint32_t ChEnMask;

    /*! The current state of the measurement frame:
     *  - Measurement Mode,
     *  - A/B Frame,
     *  - PLL_Locked Bit,
     *  - BGL Warning/Error,
     *  - DCA State,
     *  - ... */
    argus_state_t State;

    /*! Pattern count per sample in uq10.6 format.
     *  Determines the analog integration depth. */
    uq10_6_t AnalogIntegrationDepth;

    /*! Sample count per phase/frame.
     *  Determines the digital integration depth. */
    uint16_t DigitalIntegrationDepth;

    /*! Laser Modulation Current per sample in mA.
     *  Determines the optical output power. */
    uq12_4_t OutputPower;

    /*! Laser Bias Current Settings in LSB. */
    uint8_t BiasCurrent;

    /*! Charge pump voltage per sample in LSB.
     *  Determines the pixel gain.  */
    uint8_t PixelGain;

    /*! PLL Frequency Offset, caused by temperature
     *  compensation, in PLL_INT_PRD LSBs. */
    int8_t PllOffset;

    /*! The current PLL_CTRL_CUR value. */
    uint8_t PllCtrlCur;

} argus_meas_frame_t;

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_MEAS_H */
