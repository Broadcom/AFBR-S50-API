/*************************************************************************//**
 * @file
 * @brief       This file is part of the MKL17z platform layer.
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

#ifndef BOARD_CONFIG_MKL17Z_H
#define BOARD_CONFIG_MKL17Z_H

/*!***************************************************************************
 * @defgroup    boardcfg Board Configuration
 * @ingroup     platform
 * @brief       Board Configuration
 * @details     Board/Platform Depended Definitions.
 * @addtogroup  boardcfg
 * @{
 *****************************************************************************/

/*! The board name. */
#define BOARD_NAME          "FRDM-KL17Z"

/*****************************************************************************
 * Board UART configuration
 *****************************************************************************/

/*! The UART baud rate in bps */
#ifndef UART_BAUDRATE
#define UART_BAUDRATE       115200U
#endif

/* The UART to use for communication with the AFBR-S50 Explorer. */
#define UART_INSTANCE               0U                  /*!< UART Instance Number. */
#define UART_BASEADDR               LPUART0             /*!< UART Instance Base Address. */
#define UART_IRQn                   LPUART0_IRQn        /*!< UART Interrupt Number. */
#define UART_IRQ_HANDLER            LPUART0_IRQHandler  /*!< UART Interrupt Handler. */
#define UART_CLKSRC                 kCLOCK_CoreSysClk   /*!< UART Clock Source (Core System Clock). */


/*****************************************************************************
 * Board SPI configuration
 *****************************************************************************/

/*! The number of available S2PI slaves for KL17z board. */
#define S2PI_SLAVE_COUNT    1

/*! Dummy Slave. */
#define S2PI_SLAVE_NONE     0

/*! Multi-Device Board Slave 1 on SPI1. */
#define S2PI_SLAVE1         1

/*! The maximum SPI baud rate in bps for KL17z board. */
#ifndef SPI_MAX_BAUDRATE
// TODO: test if this is still 6M or 12M;
// a now fixed error in set Baudrate (using wrong clock) may caused the issue...
#define SPI_MAX_BAUDRATE     6000000U
#endif

/*! Define the default SPI slave for device. */
#ifndef SPI_DEFAULT_SLAVE
#define SPI_DEFAULT_SLAVE   (S2PI_SLAVE1)
#endif

/* SPI instances */
#define S2PI_S1_BASE                SPI1_BASE           /*!< S2PI Slave 1 Instance Base Address. */

/* SPI1 Pins */
#define SPI1_MISO_PORT              PORTD               /*!< SPI1 MOSI Line - Port. */
#define SPI1_MISO_GPIO              GPIOD               /*!< SPI1 MOSI Line - GPIO. */
#define SPI1_MISO_GPIO_PIN          6U                  /*!< SPI1 MOSI Line - Pin Number. */
#define SPI1_MISO_MUX_GPIO          1U                  /*!< SPI1 MOSI Line - Pin Muxing for GPIO mode. */
#define SPI1_MISO_MUX_SPI           5U                  /*!< SPI1 MOSI Line - Pin Muxing for SPI mode. */

#define SPI1_MOSI_PORT              PORTD               /*!< SPI1 MISO Line - Port. */
#define SPI1_MOSI_GPIO              GPIOD               /*!< SPI1 MISO Line - GPIO. */
#define SPI1_MOSI_GPIO_PIN          7U                  /*!< SPI1 MISO Line - Pin Number. */
#define SPI1_MOSI_MUX_GPIO          1U                  /*!< SPI1 MISO Line - Pin Muxing for GPIO mode. */
#define SPI1_MOSI_MUX_SPI           5U                  /*!< SPI1 MISO Line - Pin Muxing for SPI mode. */

#define SPI1_CLK_PORT               PORTD               /*!< SPI1 Clock Line - Port. */
#define SPI1_CLK_GPIO               GPIOD               /*!< SPI1 Clock Line - GPIO. */
#define SPI1_CLK_GPIO_PIN           5U                  /*!< SPI1 Clock Line - Pin Number. */
#define SPI1_CLK_MUX_GPIO           1U                  /*!< SPI1 Clock Line - Pin Muxing for GPIO mode. */
#define SPI1_CLK_MUX_SPI            2U                  /*!< SPI1 Clock Line - Pin Muxing for SPI mode. */

/* S2PI Slave 1 on SPI1 */
#define S2PI_CS1_PORT               PORTD               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS1_GPIO               GPIOD               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS1_GPIO_PIN           4U                  /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS1_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS1_MUX_SPI            2U                  /*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

#define S2PI_IRQ1_PORT              PORTA               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ1_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ1_GPIO_PIN          18U                 /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ1_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */


/*****************************************************************************
 * Other Board GPIO configuration
 *****************************************************************************/

/* PTE16 (SPI0_CS, MUX) */
#define PTE16_PORT        PORTE               /*!< PTE16 - Port. */
#define PTE16_GPIO        GPIOE               /*!< PTE16  - GPIO. */
#define PTE16_GPIO_PIN    16U                 /*!< PTE16 - Pin Number. */
#define PTE16_MUX_GPIO    0U                  /*!< PTE16 - Pin Muxing for GPIO mode. */
#define PTE16_MUX_SPI     2U                  /*!< PTE16 - Pin Muxing for SPI mode. */

/* PTE17 (SPI0_CLK) */
#define PTE17_PORT        PORTE               /*!< PTE17 - Port. */
#define PTE17_GPIO        GPIOE               /*!< PTE17  - GPIO. */
#define PTE17_GPIO_PIN    17U                 /*!< PTE17 - Pin Number. */
#define PTE17_MUX_GPIO    0U                  /*!< PTE17 - Pin Muxing for GPIO mode. */
#define PTE17_MUX_SPI     2U                  /*!< PTE17 - Pin Muxing for SPI mode. */

/* PTE18 (SPI0_MOSI, I2C_SDA) */
#define PTE18_PORT        PORTE               /*!< PTE18 - Port. */
#define PTE18_GPIO        GPIOE               /*!< PTE18  - GPIO. */
#define PTE18_GPIO_PIN    18U                 /*!< PTE18 - Pin Number. */
#define PTE18_MUX_GPIO    0U                  /*!< PTE18 - Pin Muxing for GPIO mode. */
#define PTE18_MUX_SPI     2U                  /*!< PTE18 - Pin Muxing for SPI mode. */
#define PTE18_MUX_I2C     3U                  /*!< PTE18 - Pin Muxing for I2C mode. */

/* PTE19 (SPI0_MISO, I2C0_SCK) */
#define PTE19_PORT        PORTE               /*!< PTE19 - Port. */
#define PTE19_GPIO        GPIOE               /*!< PTE19  - GPIO. */
#define PTE19_GPIO_PIN    19U                 /*!< PTE19 - Pin Number. */
#define PTE19_MUX_GPIO    0U                  /*!< PTE19 - Pin Muxing for GPIO mode. */
#define PTE19_MUX_SPI     2U                  /*!< PTE19 - Pin Muxing for SPI mode. */
#define PTE19_MUX_I2C     3U                  /*!< PTE19 - Pin Muxing for I2C mode. */

/*******************************************************************************
 * TOF Device Specific Setup
 ******************************************************************************/


/*******************************************************************************
 * DMA Configuration
 ******************************************************************************/

/* DMA Channels (0..3) */
#define DMA_CHANNEL_SPI_TX          0U                  /*!< DMA Channel 0: SPI transmit */
#define DMA_CHANNEL_SPI_RX          1U                  /*!< DMA Channel 1: SPI receiver */
#define DMA_CHANNEL_UART_TX         2U                  /*!< DMA Channel 2: UART transmit */

#define DMA_REQUEST_MUX_UART_TX     3U                  /*!< DMAMUX Channel 2: LPUART0 transmit complete */
#define DMA_REQUEST_MUX_SPI1_TX     19U                 /*!< DMAMUX Channel 0: SPI1 transmit complete */
#define DMA_REQUEST_MUX_SPI1_RX     18U                 /*!< DMAMUX Channel 1: SPI1 receive complete */


/*! @} */
#endif /* BOARD_CONFIG_MKL17Z_H */
