/*************************************************************************//**
 * @file
 * @brief       This file is part of the STM32F4 platform layer.
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

#include "main.h"

/*! The board name. */
#define BOARD_NAME          "STM32F401-Nucleo-64"


/*****************************************************************************
 * Board SPI configuration
 *****************************************************************************/

/*! The number of available S2PI slaves for STM32F401RE board. */
#define S2PI_SLAVE_COUNT    4

/*! Dummy Slave. */
#define S2PI_SLAVE_NONE     0

/*! Multi-Device Board Slave 1 on SPI1; Default Slave. */
#define S2PI_SLAVE1         1

/*! Multi-Device Board Slave 2 on SPI1. */
#define S2PI_SLAVE2         2

/*! Multi-Device Board Slave 3 on SPI1. */
#define S2PI_SLAVE3         3

/*! Multi-Device Board Slave 4 on SPI1. */
#define S2PI_SLAVE4         4

/*! Define the maximum SPI baud rate (to be used in the SPI module).
 *  This is dependent of the available peripheral. */
#ifndef SPI_MAX_BAUDRATE
#define SPI_MAX_BAUDRATE 21000000
#endif

/*! Define the current SPI baud rate (to be used in the SPI module).
 *  This is dependent of the available peripheral. */
#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE SPI_MAX_BAUDRATE
#endif

/*! Define the default SPI slave for device.
 *  The slave is used for SPI initialization only! */
#ifndef SPI_DEFAULT_SLAVE
#define SPI_DEFAULT_SLAVE   (S2PI_SLAVE1)
#endif

/* SPI instances */
// #define S2PI_S1_BASE                SPI1_BASE           /*!< S2PI Slave 1 Instance Base Address. */
// #define S2PI_S2_BASE                SPI1_BASE           /*!< S2PI Slave 2 Instance Base Address. */
// #define S2PI_S3_BASE                SPI1_BASE           /*!< S2PI Slave 2 Instance Base Address. */
// #define S2PI_S4_BASE                SPI1_BASE           /*!< S2PI Slave 4 Instance Base Address. */
// #define S2PI_S5_BASE                SPI0_BASE           /*!< S2PI Slave 5 Instance Base Address. */
// #define S2PI_S6_BASE                SPI0_BASE           /*!< S2PI Slave 6 Instance Base Address. */

/* SPI0 Pins */


#define S2PI_MISO_GPIO              GPIOA               /*!< S2PI MOSI Line - GPIO. */
#define S2PI_MISO_GPIO_PIN          GPIO_PIN_6          /*!< S2PI MOSI Line - Pin Number. */

#define S2PI_MOSI_GPIO              GPIOA               /*!< S2PI MISO Line - GPIO. */
#define S2PI_MOSI_GPIO_PIN          GPIO_PIN_7          /*!< S2PI MISO Line - Pin Number. */

#define S2PI_CLK_GPIO               GPIOA               /*!< S2PI Clock Line - GPIO. */
#define S2PI_CLK_GPIO_PIN           GPIO_PIN_5          /*!< S2PI Clock Line - Pin Number. */

#define S2PI_CS1_GPIO               GPIOB               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS1_GPIO_PIN           GPIO_PIN_6          /*!< S2PI Chip Select Line - Pin Number. */
#define S2PI_CS1_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_IRQ1_GPIO              GPIOC               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ1_GPIO_PIN          GPIO_PIN_7          /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ1_EXTI              EXTI9_5_IRQn        /*!< S2PI Interrupt Line - ExtInt ID. */
#define S2PI_IRQ1_PORTCLK_ENB()     __HAL_RCC_GPIOC_CLK_ENABLE()

#define S2PI_PWR1_GPIO               GPIOB               /*!< Board Device Enable Line - GPIO. */
#define S2PI_PWR1_GPIO_PIN           GPIO_PIN_6          /*!< Board Device Enable Line - Pin Number. */
#define S2PI_PWR1_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_CS2_GPIO               GPIOA               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS2_GPIO_PIN           GPIO_PIN_9          /*!< S2PI Chip Select Line - Pin Number. */
#define S2PI_CS2_PORTCLK_ENB()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define S2PI_IRQ2_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ2_GPIO_PIN          GPIO_PIN_10         /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ2_EXTI              EXTI15_10_IRQn      /*!< S2PI Interrupt Line - ExtInt ID. */
#define S2PI_IRQ2_PORTCLK_ENB()     __HAL_RCC_GPIOA_CLK_ENABLE()

#define S2PI_PWR2_GPIO               GPIOB               /*!< Board Device Enable Line - GPIO. */
#define S2PI_PWR2_GPIO_PIN           GPIO_PIN_6          /*!< Board Device Enable Line - Pin Number. */
#define S2PI_PWR2_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_CS3_GPIO               GPIOA               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS3_GPIO_PIN           GPIO_PIN_8          /*!< S2PI Chip Select Line - Pin Number. */
#define S2PI_CS3_PORTCLK_ENB()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define S2PI_IRQ3_GPIO              GPIOB               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ3_GPIO_PIN          GPIO_PIN_3          /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ3_EXTI              EXTI3_IRQn          /*!< S2PI Interrupt Line - ExtInt ID. */
#define S2PI_IRQ3_PORTCLK_ENB()     __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_PWR3_GPIO               GPIOB               /*!< Board Device Enable Line - GPIO. */
#define S2PI_PWR3_GPIO_PIN           GPIO_PIN_6          /*!< Board Device Enable Line - Pin Number. */
#define S2PI_PWR3_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_CS4_GPIO               GPIOB               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS4_GPIO_PIN           GPIO_PIN_10         /*!< S2PI Chip Select Line - Pin Number. */
#define S2PI_CS4_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_IRQ4_GPIO              GPIOB               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ4_GPIO_PIN          GPIO_PIN_5          /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ4_EXTI              EXTI9_5_IRQn        /*!< S2PI Interrupt Line - ExtInt ID. */
#define S2PI_IRQ4_PORTCLK_ENB()     __HAL_RCC_GPIOB_CLK_ENABLE()

#define S2PI_EN4_GPIO               GPIOB               /*!< Board Device Enable Line - GPIO. */
#define S2PI_EN4_GPIO_PIN           GPIO_PIN_6          /*!< Board Device Enable Line - Pin Number. */
#define S2PI_EN4_PORTCLK_ENB()      __HAL_RCC_GPIOB_CLK_ENABLE()


#define S2PI_FUNCTION               GPIO_AF5_SPI1

/*! @} */
#endif /* BOARD_CONFIG_H */
