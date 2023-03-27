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

#if defined(CPU_MKL17Z256VFM4)

#include "devices/MKL17Z/MKL17Z4.h"
#include "board/MKL17Z/board_config_MKL17Z.h"

#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

#include "devices/MKL46Z/MKL46Z4.h"
#include "devices/MKL46Z/MKL46Z4_features.h"
#include "board/MKL46Z/board_config_MKL46Z.h"

#endif


/*****************************************************************************
 * Interrupt Priority Configuration:
 * 4 priority levels: 0 means high urgency, 3 means low urgency.
 *****************************************************************************/
#define IRQPRIO_SYSTICK     1U      /*!< Interrupt priority level of SysTick Timer. */
#define IRQPRIO_DMA0        2U      /*!< Interrupt priority level of DMA0 IRQ: SPI Transmitter. */
#define IRQPRIO_DMA1        2U      /*!< Interrupt priority level of DMA1 IRQ: SPI Receiver. */
#define IRQPRIO_DMA2        0U      /*!< Interrupt priority level of DMA2 IRQ: UART Transmitter. High priority such that also messages from other interrupt service routines can be sent. */
#define IRQPRIO_DMA3        3U      /*!< Interrupt priority level of DMA3 IRQ: Not used. */
#define IRQPRIO_SPI         2U      /*!< Interrupt priority level of SPI IRQ. */
#define IRQPRIO_UART0       0U      /*!< Interrupt priority level of UART0 IRQ: Serial Rx IRQ. High Priority to prevent Rx Overrun. */
#define IRQPRIO_GPIOA       2U      /*!< Interrupt priority level of GPIOA IRQ. */
#define IRQPRIO_GPIOCD      2U      /*!< Interrupt priority level of GPIOCD IRQ. */
#define IRQPRIO_USB         0U      /*!< Interrupt priority level of USB IRQ. */

#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE        SPI_MAX_BAUDRATE
#endif

/*! @} */
#endif /* BOARD_CONFIG_H */
