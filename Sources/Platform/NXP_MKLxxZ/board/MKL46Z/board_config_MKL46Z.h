/*************************************************************************//**
 * @file
 * @brief       This file is part of the MKL46z platform layer.
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

#ifndef BOARD_CONFIG_MKL46Z_H
#define BOARD_CONFIG_MKL46Z_H

/*!***************************************************************************
 * @defgroup    boardcfg Board Configuration
 * @ingroup     platform
 * @brief       Board Configuration
 * @details     Board/Platform Depended Definitions.
 * @addtogroup  boardcfg
 * @{
 *****************************************************************************/

/*! The board name. */
#define BOARD_NAME          "FRDM-KL46Z"

/*****************************************************************************
 * Board UART configuration
 *****************************************************************************/

/*! The UART baud rate in bps */
#ifndef UART_BAUDRATE
#define UART_BAUDRATE       115200U
#endif

/* The UART to use for communication with the AFBR-S50 Explorer. */
#define UART_INSTANCE               0U                  /*!< UART Instance Number. */
#define UART_BASEADDR               UART0               /*!< UART Instance Base Address. */
#define UART_IRQn                   UART0_IRQn          /*!< UART Interrupt Number. */
#define UART_IRQ_HANDLER            UART0_IRQHandler    /*!< UART Interrupt Handler. */
#define UART_CLKSRC                 kCLOCK_CoreSysClk   /*!< UART Clock Source (Core System Clock). */


/*****************************************************************************
 * Board SPI configuration
 *****************************************************************************/

/*! The number of available S2PI slaves for KL46z board. */
#define S2PI_SLAVE_COUNT    6

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

/*! Legacy Board Slave 1 on SPI0; Internal Slave. */
#define S2PI_SLAVE5         5

/*! Legacy Board Slave 2 on SPI0; External Slave. */
#define S2PI_SLAVE6         6

/*! The maximum SPI baud rate in bps for KL46z board. */
#ifndef SPI_MAX_BAUDRATE
#define SPI_MAX_BAUDRATE    12000000U
#endif

/*! Define the default SPI slave for device. */
#ifndef SPI_DEFAULT_SLAVE
#define SPI_DEFAULT_SLAVE   (S2PI_SLAVE1)
#endif

/* SPI instances */
#define S2PI_S1_BASE                SPI1_BASE           /*!< S2PI Slave 1 Instance Base Address. */
#define S2PI_S2_BASE                SPI1_BASE           /*!< S2PI Slave 2 Instance Base Address. */
#define S2PI_S3_BASE                SPI1_BASE           /*!< S2PI Slave 2 Instance Base Address. */
#define S2PI_S4_BASE                SPI1_BASE           /*!< S2PI Slave 4 Instance Base Address. */
#define S2PI_S5_BASE                SPI0_BASE           /*!< S2PI Slave 5 Instance Base Address. */
#define S2PI_S6_BASE                SPI0_BASE           /*!< S2PI Slave 6 Instance Base Address. */

/* SPI0 Pins */
#define SPI0_MISO_PORT              PORTA               /*!< SPI0 MOSI Line - Port. */
#define SPI0_MISO_GPIO              GPIOA               /*!< SPI0 MOSI Line - GPIO. */
#define SPI0_MISO_GPIO_PIN          17U                 /*!< SPI0 MOSI Line - Pin Number. */
#define SPI0_MISO_MUX_GPIO          1U                  /*!< SPI0 MOSI Line - Pin Muxing for GPIO mode. */
#define SPI0_MISO_MUX_SPI           2U                  /*!< SPI0 MOSI Line - Pin Muxing for SPI mode. */

#define SPI0_MOSI_PORT              PORTA               /*!< SPI0 MISO Line - Port. */
#define SPI0_MOSI_GPIO              GPIOA               /*!< SPI0 MISO Line - GPIO. */
#define SPI0_MOSI_GPIO_PIN          16U                 /*!< SPI0 MISO Line - Pin Number. */
#define SPI0_MOSI_MUX_GPIO          1U                  /*!< SPI0 MISO Line - Pin Muxing for GPIO mode. */
#define SPI0_MOSI_MUX_SPI           2U                  /*!< SPI0 MISO Line - Pin Muxing for SPI mode. */

#define SPI0_CLK_PORT               PORTA               /*!< SPI0 Clock Line - Port. */
#define SPI0_CLK_GPIO               GPIOA               /*!< SPI0 Clock Line - GPIO. */
#define SPI0_CLK_GPIO_PIN           15U                 /*!< SPI0 Clock Line - Pin Number. */
#define SPI0_CLK_MUX_GPIO           1U                  /*!< SPI0 Clock Line - Pin Muxing for GPIO mode. */
#define SPI0_CLK_MUX_SPI            2U                  /*!< SPI0 Clock Line - Pin Muxing for SPI mode. */

/* SPI1 Pins */
#define SPI1_MISO_PORT              PORTD               /*!< SPI1 MOSI Line - Port. */
#define SPI1_MISO_GPIO              GPIOD               /*!< SPI1 MOSI Line - GPIO. */
#define SPI1_MISO_GPIO_PIN          7U                  /*!< SPI1 MOSI Line - Pin Number. */
#define SPI1_MISO_MUX_GPIO          1U                  /*!< SPI1 MOSI Line - Pin Muxing for GPIO mode. */
#define SPI1_MISO_MUX_SPI           2U                  /*!< SPI1 MOSI Line - Pin Muxing for SPI mode. */

#define SPI1_MOSI_PORT              PORTD               /*!< SPI1 MISO Line - Port. */
#define SPI1_MOSI_GPIO              GPIOD               /*!< SPI1 MISO Line - GPIO. */
#define SPI1_MOSI_GPIO_PIN          6U                  /*!< SPI1 MISO Line - Pin Number. */
#define SPI1_MOSI_MUX_GPIO          1U                  /*!< SPI1 MISO Line - Pin Muxing for GPIO mode. */
#define SPI1_MOSI_MUX_SPI           2U                  /*!< SPI1 MISO Line - Pin Muxing for SPI mode. */

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

#define S2PI_IRQ1_PORT              PORTD               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ1_GPIO              GPIOD               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ1_GPIO_PIN          2U                  /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ1_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 2 on SPI1 */
#define S2PI_CS2_PORT               PORTA               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS2_GPIO               GPIOA               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS2_GPIO_PIN           13U                 /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS2_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */

#define S2PI_IRQ2_PORT              PORTD               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ2_GPIO              GPIOD               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ2_GPIO_PIN          3U                  /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ2_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 3 on SPI1 */
#define S2PI_CS3_PORT               PORTC               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS3_GPIO               GPIOC               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS3_GPIO_PIN           9U                  /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS3_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */

#define S2PI_IRQ3_PORT              PORTA               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ3_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ3_GPIO_PIN          12U                 /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ3_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 4 on SPI1 */
#define S2PI_CS4_PORT               PORTC               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS4_GPIO               GPIOC               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS4_GPIO_PIN           8U                  /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS4_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */

#define S2PI_IRQ4_PORT              PORTA               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ4_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ4_GPIO_PIN          4U                  /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ4_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 5 on SPI0; legacy intern slave */
#define S2PI_CS5_PORT               PORTA               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS5_GPIO               GPIOA               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS5_GPIO_PIN           14U                 /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS5_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS5_MUX_SPI            2U                  /*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

#define S2PI_IRQ5_PORT              PORTA               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ5_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ5_GPIO_PIN          6U                  /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ5_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 6 on SPI0; legacy extern slave */
#define S2PI_CS6_PORT               PORTE               /*!< S2PI Chip Select Line - Port. */
#define S2PI_CS6_GPIO               GPIOE               /*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS6_GPIO_PIN           16U                 /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS6_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS6_MUX_SPI            2U                  /*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

#define S2PI_IRQ6_PORT              PORTA               /*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ6_GPIO              GPIOA               /*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ6_GPIO_PIN          7U                  /*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ6_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

///* S2PI Slave 7 on SPI0; legacy HTOL slave 3 */
//#define S2PI_CS7_PORT               PORTE               /*!< S2PI Chip Select Line - Port. */
//#define S2PI_CS7_GPIO               GPIOE               /*!< S2PI Chip Select Line - GPIO. */
//#define S2PI_CS3_GPIO_PIN           17U                 /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
//#define S2PI_CS7_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
//
//#define S2PI_IRQ7_PORT              PORTC               /*!< S2PI Interrupt Line - Port. */
//#define S2PI_IRQ7_GPIO              GPIOC               /*!< S2PI Interrupt Line - GPIO. */
//#define S2PI_IRQ7_GPIO_PIN          16U                 /*!< S2PI Interrupt Line - Pin Number. */
//#define S2PI_IRQ7_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

///* S2PI Slave 8 on SPI0; legacy HTOL slave 4 */
//#define S2PI_CS8_PORT               PORTE               /*!< S2PI Chip Select Line - Port. */
//#define S2PI_CS8_GPIO               GPIOE               /*!< S2PI Chip Select Line - GPIO. */
//#define S2PI_CS8_GPIO_PIN           18U                 /*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
//#define S2PI_CS8_MUX_GPIO           1U                  /*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
//
//#define S2PI_IRQ8_PORT              PORTC               /*!< S2PI Interrupt Line - Port. */
//#define S2PI_IRQ8_GPIO              GPIOC               /*!< S2PI Interrupt Line - GPIO. */
//#define S2PI_IRQ8_GPIO_PIN          13U                 /*!< S2PI Interrupt Line - Pin Number. */
//#define S2PI_IRQ8_MUX               1U                  /*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */


/*****************************************************************************
 * Other Board GPIO configuration
 *****************************************************************************/

// LED is connected to CLK of SPI interface!
//#define LED_GREEN_PORT      PORTD               /*!< Green LED - Port. */
//#define LED_GREEN_GPIO      GPIOD               /*!< Green LED - GPIO. */
//#define LED_GREEN_GPIO_PIN  5U                  /*!< Green LED - Pin Number. */

#define LED_RED_PORT          PORTE               /*!< Red LED - Port. */
#define LED_RED_GPIO          GPIOE               /*!< Red LED - GPIO. */
#define LED_RED_GPIO_PIN      29U                 /*!< Red LED - Pin Number. */

//#define SW1_PORT              PORTC               /*!< Switch 1 - Port. */
//#define SW1_GPIO              GPIOC               /*!< Switch 1 - GPIO. */
//#define SW1_GPIO_PIN          3U                  /*!< Switch 1 - Pin Number. */
//
//#define SW3_PORT              PORTC               /*!< Switch 2 - Port. */
//#define SW3_GPIO              GPIOC               /*!< Switch 2 - GPIO. */
//#define SW3_GPIO_PIN          12U                 /*!< Switch 2 - Pin Number. */

#define PTB0_PORT             PORTB               /*!< Debug Pin 0 Port B - Port. */
#define PTB0_GPIO             GPIOB               /*!< Debug Pin 0 Port B - GPIO. */
#define PTB0_GPIO_PIN         0U                  /*!< Debug Pin 0 Port B - Pin Number. */

#define PTB1_PORT             PORTB               /*!< Debug Pin 1 Port B - Port. */
#define PTB1_GPIO             GPIOB               /*!< Debug Pin 1 Port B - GPIO. */
#define PTB1_GPIO_PIN         1U                  /*!< Debug Pin 1 Port B - Pin Number. */

#define PTB2_PORT             PORTB               /*!< Debug Pin 2 Port B - Port. */
#define PTB2_GPIO             GPIOB               /*!< Debug Pin 2 Port B - GPIO. */
#define PTB2_GPIO_PIN         2U                  /*!< Debug Pin 2 Port B - Pin Number. */

#define PTB3_PORT             PORTB               /*!< Debug Pin 3 Port B - Port. */
#define PTB3_GPIO             GPIOB               /*!< Debug Pin 3 Port B - GPIO. */
#define PTB3_GPIO_PIN         3U                  /*!< Debug Pin 3 Port B - Pin Number. */

#define PTC1_PORT             PORTC               /*!< Debug Pin 1 Port C - Port. */
#define PTC1_GPIO             GPIOC               /*!< Debug Pin 1 Port C - GPIO. */
#define PTC1_GPIO_PIN         1U                  /*!< Debug Pin 1 Port C - Pin Number. */

#define PTC2_PORT             PORTC               /*!< Debug Pin 2 Port C - Port. */
#define PTC2_GPIO             GPIOC               /*!< Debug Pin 2 Port C - GPIO. */
#define PTC2_GPIO_PIN         2U                  /*!< Debug Pin 2 Port C - Pin Number. */

//#define JUMPER1_PORT          PORTA               /*!< Jumper 1 - Port. */
//#define JUMPER1_GPIO          GPIOA               /*!< Jumper 1 - GPIO. */
//#define JUMPER1_GPIO_PIN      14U                 /*!< Jumper 1 - Pin Number. */
//
//#define JUMPER2_PORT          PORTE               /*!< Jumper 2 - Port. */
//#define JUMPER2_GPIO          GPIOE               /*!< Jumper 2 - GPIO. */
//#define JUMPER2_GPIO_PIN      17U                 /*!< Jumper 2 - Pin Number. */


/*******************************************************************************
 * DMA Configuration
 ******************************************************************************/

/* DMA Channels (0..3) */
#define DMA_CHANNEL_SPI_TX          0U                  /*!< DMA Channel 0: SPI transmit */
#define DMA_CHANNEL_SPI_RX          1U                  /*!< DMA Channel 1: SPI receiver */
#define DMA_CHANNEL_UART_TX         2U                  /*!< DMA Channel 2: UART transmit */

#define DMA_REQUEST_MUX_UART_TX     3U                  /*!< DMAMUX Channel 2: UART0 transmit complete */
#define DMA_REQUEST_MUX_SPI0_TX     17U                 /*!< DMAMUX Channel 0: SPI0 transmit complete */
#define DMA_REQUEST_MUX_SPI0_RX     16U                 /*!< DMAMUX Channel 1: SPI0 receive complete */
#define DMA_REQUEST_MUX_SPI1_TX     19U                 /*!< DMAMUX Channel 0: SPI1 transmit complete */
#define DMA_REQUEST_MUX_SPI1_RX     18U                 /*!< DMAMUX Channel 1: SPI1 receive complete */


/*! @} */
#endif /* BOARD_CONFIG_MKL46Z_H */
