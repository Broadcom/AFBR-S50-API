/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _CLOCK_CONFIG_MKL46Z_H_
#define _CLOCK_CONFIG_MKL46Z_H_

/*!***************************************************************************
 * @defgroup    clockcfg Clock Configuration
 * @ingroup     platform
 * @brief       Clock Configuration Module
 * @details     Platform Clock Configuration/Setup Module.
 *
 *              How to setup clock using clock driver functions:
 *
 *              1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *                  and flash clock are in allowed range during clock mode switch.
 *
 *              2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 *              3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *                  internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *                  1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *                  2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *                      correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *                      explicitly to setup MCGIRCLK.
 *
 *                  3). Don't need to configure FLL explicitly, because if target mode is FLL
 *                      mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *                      if the target mode is not FLL mode, the FLL is disabled.
 *
 *                  4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *                      setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *                      be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 *              4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 *
 * @addtogroup  clockcfg
 * @{
 *****************************************************************************/

/*! @brief Crystal Clock Frequency in Hz. */
#define BOARD_XTAL0_CLK_HZ 8000000U


/*!***************************************************************************
 * @brief   Function to change clock mode to VLPR mode.
 *****************************************************************************/
void BOARD_BootClockVLPR(void);


/*!***************************************************************************
 * @brief   Function to change clock mode to RUN mode.
 *****************************************************************************/
void BOARD_BootClockRUN(void);

/*! @} */
#endif /* _CLOCK_CONFIG_MKL46Z_H_ */
