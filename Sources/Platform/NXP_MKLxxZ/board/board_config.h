/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides some example initialization code.
 * 
 * @copyright
 * 
 * Copyright (c) 2021, Broadcom Inc
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
 * @defgroup	boardcfg Board Configuration
 * @ingroup		platform
 * @brief		Board Configuration
 * @details		Board/Platform Depended Definitions.
 * @addtogroup 	boardcfg
 * @{
 *****************************************************************************/

#if defined(CPU_MKL17Z256VFM4)

#include "devices/MKL17Z/MKL17Z4.h"
//#include "devices/MKL17Z/MKL17Z4_features.h"

#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

#include "devices/MKL46Z/MKL46Z4.h"
#include "devices/MKL46Z/MKL46Z4_features.h"

#elif defined(STM32F401xE)



#endif


/*!***************************************************************************
 * @brief	The board name.
 *****************************************************************************/
#define BOARD_NAME			"FRDM-KL46Z"

/*****************************************************************************
 * Interrupt Priority Configuration:
 * 4 priority levels: 0 means high urgency, 3 means low urgency.
 *****************************************************************************/
#define IRQPRIO_SYSTICK		1U		/*!< Interrupt priority level of SysTick Timer. */
#define IRQPRIO_DMA0		2U		/*!< Interrupt priority level of DMA0 IRQ: SPI Transmitter. */
#define IRQPRIO_DMA1		2U		/*!< Interrupt priority level of DMA1 IRQ: SPI Receiver. */
#define IRQPRIO_DMA2		0U		/*!< Interrupt priority level of DMA2 IRQ: UART Transmitter. High priority such that also messages from other interrupt service routines can be sent. */
#define IRQPRIO_DMA3		3U		/*!< Interrupt priority level of DMA3 IRQ: Not used. */
#define IRQPRIO_SPI0		2U		/*!< Interrupt priority level of SPI0 IRQ. */
#define IRQPRIO_SPI1		2U		/*!< Interrupt priority level of SPI1 IRQ. */
#define IRQPRIO_UART0		0U		/*!< Interrupt priority level of UART0 IRQ: Serial Rx IRQ. High Priority to prevent Rx Overrun. */
#define IRQPRIO_UART2		1U		/*!< Interrupt priority level of UART2 IRQ: Serial Rx IRQ. High Priority to prevent Rx Overrun. */
//#define IRQPRIO_LPTMR0		3U		/*!< Interrupt priority level of LPTMR0 IRQ. */
#define IRQPRIO_GPIOA		2U		/*!< Interrupt priority level of GPIOA IRQ. */
#define IRQPRIO_GPIOCD		2U		/*!< Interrupt priority level of GPIOCD IRQ. */
#define IRQPRIO_USB  		0U		/*!< Interrupt priority level of USB IRQ. */


#if defined(CPU_MKL17Z256VFM4)

/*****************************************************************************
 * Board GPIO configuration
 *****************************************************************************/

///* PTA1 (LPUART0_RX, DATA_READY) */
//#define BOARD_PTA01_PORT		PORTA				/*!< PTA1 - Port. */
//#define BOARD_PTA01_GPIO		GPIOA				/*!< PTA1 - GPIO. */
//#define BOARD_PTA01_GPIO_PIN	1U					/*!< PTA1 - Pin Number. */
//#define BOARD_PTA01_MUX_GPIO	0U					/*!< PTA1 - Pin Muxing for GPIO mode. */
//#define BOARD_PTA01_MUX_UART	2U					/*!< PTA1 - Pin Muxing for UART mode. */
//
///* PTA2 (LPUART0_TX, MUX) */
//#define BOARD_PTA02_PORT		PORTA				/*!< PTA2 - Port. */
//#define BOARD_PTA02_GPIO		GPIOA				/*!< PTA2 - GPIO. */
//#define BOARD_PTA02_GPIO_PIN	2U					/*!< PTA2 - Pin Number. */
//#define BOARD_PTA01_MUX_GPIO	0U					/*!< PTA2 - Pin Muxing for GPIO mode. */
//#define BOARD_PTA01_MUX_UART	2U					/*!< PTA2 - Pin Muxing for UART mode. */

/* PTE16 (SPI0_CS, MUX) */
#define BOARD_PTE16_PORT		PORTE				/*!< PTE16 - Port. */
#define BOARD_PTE16_GPIO		GPIOE				/*!< PTE16  - GPIO. */
#define BOARD_PTE16_GPIO_PIN	16U					/*!< PTE16 - Pin Number. */
#define BOARD_PTE16_MUX_GPIO	0U					/*!< PTE16 - Pin Muxing for GPIO mode. */
#define BOARD_PTE16_MUX_SPI		2U					/*!< PTE16 - Pin Muxing for SPI mode. */

/* PTE17 (SPI0_CLK) */
#define BOARD_PTE17_PORT		PORTE				/*!< PTE17 - Port. */
#define BOARD_PTE17_GPIO		GPIOE				/*!< PTE17  - GPIO. */
#define BOARD_PTE17_GPIO_PIN	17U					/*!< PTE17 - Pin Number. */
#define BOARD_PTE17_MUX_GPIO	0U					/*!< PTE17 - Pin Muxing for GPIO mode. */
#define BOARD_PTE17_MUX_SPI		2U					/*!< PTE17 - Pin Muxing for SPI mode. */

/* PTE18 (SPI0_MOSI, I2C_SDA) */
#define BOARD_PTE18_PORT		PORTE				/*!< PTE18 - Port. */
#define BOARD_PTE18_GPIO		GPIOE				/*!< PTE18  - GPIO. */
#define BOARD_PTE18_GPIO_PIN	18U					/*!< PTE18 - Pin Number. */
#define BOARD_PTE18_MUX_GPIO	0U					/*!< PTE18 - Pin Muxing for GPIO mode. */
#define BOARD_PTE18_MUX_SPI		2U					/*!< PTE18 - Pin Muxing for SPI mode. */
#define BOARD_PTE18_MUX_I2C		3U					/*!< PTE18 - Pin Muxing for I2C mode. */

/* PTE19 (SPI0_MISO, I2C0_SCK) */
#define BOARD_PTE19_PORT		PORTE				/*!< PTE19 - Port. */
#define BOARD_PTE19_GPIO		GPIOE				/*!< PTE19  - GPIO. */
#define BOARD_PTE19_GPIO_PIN	19U					/*!< PTE19 - Pin Number. */
#define BOARD_PTE19_MUX_GPIO	0U					/*!< PTE19 - Pin Muxing for GPIO mode. */
#define BOARD_PTE19_MUX_SPI		2U					/*!< PTE19 - Pin Muxing for SPI mode. */
#define BOARD_PTE19_MUX_I2C		3U					/*!< PTE19 - Pin Muxing for I2C mode. */

/*******************************************************************************
 * TOF Device Specific Setup
 ******************************************************************************/

/* S2PI Pins */
#define S2PI_MISO_PORT				PORTD				/*!< S2PI MOSI Line - Port. */
#define S2PI_MISO_GPIO				GPIOD				/*!< S2PI MOSI Line - GPIO. */
#define S2PI_MISO_GPIO_PIN			6U					/*!< S2PI MOSI Line - Pin Number. */
#define S2PI_MISO_MUX_GPIO			1U					/*!< S2PI MOSI Line - Pin Muxing for GPIO mode. */
#define S2PI_MISO_MUX_SPI			5U					/*!< S2PI MOSI Line - Pin Muxing for SPI mode. */

#define S2PI_MOSI_PORT				PORTD				/*!< S2PI MISO Line - Port. */
#define S2PI_MOSI_GPIO				GPIOD				/*!< S2PI MISO Line - GPIO. */
#define S2PI_MOSI_GPIO_PIN			7U					/*!< S2PI MISO Line - Pin Number. */
#define S2PI_MOSI_MUX_GPIO			1U					/*!< S2PI MISO Line - Pin Muxing for GPIO mode. */
#define S2PI_MOSI_MUX_SPI			5U					/*!< S2PI MISO Line - Pin Muxing for SPI mode. */

#define S2PI_CLK_PORT				PORTD				/*!< S2PI Clock Line - Port. */
#define S2PI_CLK_GPIO				GPIOD				/*!< S2PI Clock Line - GPIO. */
#define S2PI_CLK_GPIO_PIN			5U					/*!< S2PI Clock Line - Pin Number. */
#define S2PI_CLK_MUX_GPIO			1U					/*!< S2PI Clock Line - Pin Muxing for GPIO mode. */
#define S2PI_CLK_MUX_SPI			2U					/*!< S2PI Clock Line - Pin Muxing for SPI mode. */

#define S2PI_CS1_PORT				PORTD				/*!< S2PI Chip Select Line - Port. */
#define S2PI_CS1_GPIO				GPIOD				/*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS1_GPIO_PIN			4U					/*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS1_MUX_GPIO			1U					/*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS1_MUX_SPI			2U					/*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

#define S2PI_IRQ1_PORT				PORTA				/*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ1_GPIO				GPIOA				/*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ1_GPIO_PIN			18U					/*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ1_MUX				1U					/*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* SPI instance */
#define S2PI_BASE					SPI1				/*!< S2PI Instance Base Address. */

#ifndef SPI_MAX_BAUDRATE
#define SPI_MAX_BAUDRATE 			6000000U			/*! S2PI Baud Rate Maximum in bps for KL17z. */
#endif

#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE		SPI_MAX_BAUDRATE	/*!< S2PI default Baud Rate in pbs for KL17z. */
#endif

/* DMA Channels (0..3) */
#define DMA_CHANNEL_SPI_TX 			0U					/*!< DMA Channel 0: SPI transmit */
#define DMA_CHANNEL_SPI_RX 			1U					/*!< DMA Channel 1: SPI receiver */
#define DMA_CHANNEL_UART_TX 		2U					/*!< DMA Channel 2: UART transmit */

/* DMA request MUX */
#define DMA_REQUEST_MUX_SPI_TX 		19U 				/*!< DMAMUX Channel 0: SPI1 transmit complete */
#define DMA_REQUEST_MUX_SPI_RX 		18U 				/*!< DMAMUX Channel 1: SPI1 receive complete */
#define DMA_REQUEST_MUX_UART_TX 	3U	 				/*!< DMAMUX Channel 2: LPUART0 transmit complete */


/* The UART to use for communication with the AFBR-S50 Explorer. */
#define UART_INSTANCE				0U					/*!< UART Instance Number. */
#define UART_BASEADDR				LPUART0				/*!< UART Instance Base Address. */
#define UART_IRQn					LPUART0_IRQn		/*!< UART Interrupt Number. */
#define UART_IRQ_HANDLER			LPUART0_IRQHandler	/*!< UART Interrupt Handler. */
#define UART_CLKSRC					kCLOCK_CoreSysClk	/*!< UART Clock Source (Core System Clock). */
#ifndef UART_BAUDRATE
#define UART_BAUDRATE				115200U 			/*!< UART Baud Rate. */
#endif


#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)


/*****************************************************************************
 * Board GPIO configuration
 *****************************************************************************/

// LED is connected to CLK of SPI interface!
//#define BOARD_LED_GREEN_PORT		PORTD				/*!< Green LED - Port. */
//#define BOARD_LED_GREEN_GPIO		GPIOD				/*!< Green LED - GPIO. */
//#define BOARD_LED_GREEN_GPIO_PIN	5U					/*!< Green LED - Pin Number. */

#define BOARD_LED_RED_PORT			PORTE				/*!< Red LED - Port. */
#define BOARD_LED_RED_GPIO			GPIOE				/*!< Red LED - GPIO. */
#define BOARD_LED_RED_GPIO_PIN		29U					/*!< Red LED - Pin Number. */

#define BOARD_SW1_PORT				PORTC				/*!< Switch 1 - Port. */
#define BOARD_SW1_GPIO				GPIOC				/*!< Switch 1 - GPIO. */
#define BOARD_SW1_GPIO_PIN			3U					/*!< Switch 1 - Pin Number. */

#define BOARD_SW3_PORT				PORTC				/*!< Switch 2 - Port. */
#define BOARD_SW3_GPIO				GPIOC				/*!< Switch 2 - GPIO. */
#define BOARD_SW3_GPIO_PIN			12U					/*!< Switch 2 - Pin Number. */

#define BOARD_PTB0_PORT				PORTB				/*!< Debug Pin 0 Port B - Port. */
#define BOARD_PTB0_GPIO				GPIOB				/*!< Debug Pin 0 Port B - GPIO. */
#define BOARD_PTB0_GPIO_PIN			0U					/*!< Debug Pin 0 Port B - Pin Number. */

#define BOARD_PTB1_PORT				PORTB				/*!< Debug Pin 1 Port B - Port. */
#define BOARD_PTB1_GPIO				GPIOB				/*!< Debug Pin 1 Port B - GPIO. */
#define BOARD_PTB1_GPIO_PIN			1U					/*!< Debug Pin 1 Port B - Pin Number. */

#define BOARD_PTB2_PORT				PORTB				/*!< Debug Pin 2 Port B - Port. */
#define BOARD_PTB2_GPIO				GPIOB				/*!< Debug Pin 2 Port B - GPIO. */
#define BOARD_PTB2_GPIO_PIN			2U					/*!< Debug Pin 2 Port B - Pin Number. */

#define BOARD_PTB3_PORT				PORTB				/*!< Debug Pin 3 Port B - Port. */
#define BOARD_PTB3_GPIO				GPIOB				/*!< Debug Pin 3 Port B - GPIO. */
#define BOARD_PTB3_GPIO_PIN			3U					/*!< Debug Pin 3 Port B - Pin Number. */

#define BOARD_PTC1_PORT				PORTC				/*!< Debug Pin 1 Port C - Port. */
#define BOARD_PTC1_GPIO				GPIOC				/*!< Debug Pin 1 Port C - GPIO. */
#define BOARD_PTC1_GPIO_PIN			1U					/*!< Debug Pin 1 Port C - Pin Number. */

#define BOARD_PTC2_PORT				PORTC				/*!< Debug Pin 2 Port C - Port. */
#define BOARD_PTC2_GPIO				GPIOC				/*!< Debug Pin 2 Port C - GPIO. */
#define BOARD_PTC2_GPIO_PIN			2U					/*!< Debug Pin 2 Port C - Pin Number. */

/*****************************************************************************
 * Argus Reference Adapter Board GPIO configuration
 *****************************************************************************/
#define BOARD_JUMPER1_PORT			PORTA				/*!< Jumper 1 - Port. */
#define BOARD_JUMPER1_GPIO			GPIOA				/*!< Jumper 1 - GPIO. */
#define BOARD_JUMPER1_GPIO_PIN		14U					/*!< Jumper 1 - Pin Number. */

#define BOARD_JUMPER2_PORT			PORTE				/*!< Jumper 2 - Port. */
#define BOARD_JUMPER2_GPIO			GPIOE				/*!< Jumper 2 - GPIO. */
#define BOARD_JUMPER2_GPIO_PIN		17U					/*!< Jumper 2 - Pin Number. */


/*******************************************************************************
 * TOF Device Specific Setup
 ******************************************************************************/
/* S2PI Pins */
#define S2PI_MISO_PORT				PORTA				/*!< S2PI MOSI Line - Port. */
#define S2PI_MISO_GPIO				GPIOA				/*!< S2PI MOSI Line - GPIO. */
#define S2PI_MISO_GPIO_PIN			17U					/*!< S2PI MOSI Line - Pin Number. */
#define S2PI_MISO_MUX_GPIO			1U					/*!< S2PI MOSI Line - Pin Muxing for GPIO mode. */
#define S2PI_MISO_MUX_SPI			2U					/*!< S2PI MOSI Line - Pin Muxing for SPI mode. */

#define S2PI_MOSI_PORT				PORTA				/*!< S2PI MISO Line - Port. */
#define S2PI_MOSI_GPIO				GPIOA				/*!< S2PI MISO Line - GPIO. */
#define S2PI_MOSI_GPIO_PIN			16U					/*!< S2PI MISO Line - Pin Number. */
#define S2PI_MOSI_MUX_GPIO			1U					/*!< S2PI MISO Line - Pin Muxing for GPIO mode. */
#define S2PI_MOSI_MUX_SPI			2U					/*!< S2PI MISO Line - Pin Muxing for SPI mode. */

#define S2PI_CLK_PORT				PORTA				/*!< S2PI Clock Line - Port. */
#define S2PI_CLK_GPIO				GPIOA				/*!< S2PI Clock Line - GPIO. */
#define S2PI_CLK_GPIO_PIN			15U					/*!< S2PI Clock Line - Pin Number. */
#define S2PI_CLK_MUX_GPIO			1U					/*!< S2PI Clock Line - Pin Muxing for GPIO mode. */
#define S2PI_CLK_MUX_SPI			2U					/*!< S2PI Clock Line - Pin Muxing for SPI mode. */

/* S2PI Slave 1: intern */
#define S2PI_CS1_PORT				PORTA				/*!< S2PI Chip Select Line - Port. */
#define S2PI_CS1_GPIO				GPIOA				/*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS1_GPIO_PIN			14U					/*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS1_MUX_GPIO			1U					/*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS1_MUX_SPI			2U					/*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

/* S2PI Slave 2: extern */
#define S2PI_CS2_PORT				PORTE				/*!< S2PI Chip Select Line - Port. */
#define S2PI_CS2_GPIO				GPIOE				/*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS2_GPIO_PIN			16U					/*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS2_MUX_GPIO			1U					/*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
#define S2PI_CS2_MUX_SPI			2U					/*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

/* S2PI Slave 3  */
#define S2PI_CS3_PORT				PORTE				/*!< S2PI Chip Select Line - Port. */
#define S2PI_CS3_GPIO				GPIOE				/*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS3_GPIO_PIN			17U					/*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS3_MUX_GPIO			1U					/*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
//#define S2PI_CS3_MUX_SPI			2U					/*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */

/* S2PI Slave 4 */
#define S2PI_CS4_PORT				PORTE				/*!< S2PI Chip Select Line - Port. */
#define S2PI_CS4_GPIO				GPIOE				/*!< S2PI Chip Select Line - GPIO. */
#define S2PI_CS4_GPIO_PIN			18U					/*!< S2PI Chip Select Line - Pin Number. Shares the same pin as SPI_PSC! */
#define S2PI_CS4_MUX_GPIO			1U					/*!< S2PI Chip Select Line - Pin Muxing for GPIO mode. */
//#define S2PI_CS4_MUX_SPI			2U					/*!< S2PI Chip Select Line - Pin Muxing for SPI mode. */


/* S2PI Slave 1: intern */
#define S2PI_IRQ1_PORT				PORTA				/*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ1_GPIO				GPIOA				/*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ1_GPIO_PIN			6U					/*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ1_MUX				1U					/*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 2: extern */
#define S2PI_IRQ2_PORT				PORTA				/*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ2_GPIO				GPIOA				/*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ2_GPIO_PIN			7U					/*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ2_MUX				1U					/*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 3 */
#define S2PI_IRQ3_PORT				PORTC				/*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ3_GPIO				GPIOC				/*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ3_GPIO_PIN			16U					/*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ3_MUX				1U					/*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */

/* S2PI Slave 4 */
#define S2PI_IRQ4_PORT				PORTC				/*!< S2PI Interrupt Line - Port. */
#define S2PI_IRQ4_GPIO				GPIOC				/*!< S2PI Interrupt Line - GPIO. */
#define S2PI_IRQ4_GPIO_PIN			13U					/*!< S2PI Interrupt Line - Pin Number. */
#define S2PI_IRQ4_MUX				1U					/*!< S2PI Interrupt Line - Pin Muxing for GPIO mode. */


/* SPI instance */
#define S2PI_BASE					SPI0				/*!< S2PI Instance Base Address. */
#ifndef SPI_MAX_BAUDRATE
#define SPI_MAX_BAUDRATE 			12000000U			/*!< S2PI Baud Rate Maximum in bps for KL46z. */
#endif
#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE		SPI_MAX_BAUDRATE	/*!< S2PI default Baud Rate in pbs for KL46z. */
#endif

/* DMA Channels (0..3) */
#define DMA_CHANNEL_SPI_TX 			0U					/*!< DMA Channel 0: SPI transmit */
#define DMA_CHANNEL_SPI_RX 			1U					/*!< DMA Channel 1: SPI receiver */
#define DMA_CHANNEL_UART_TX 		2U					/*!< DMA Channel 2: UART transmit */
#define DMA_CHANNEL_UART2_TX 		3U					/*!< DMA Channel 3: UART2 transmit */

#define DMA_REQUEST_MUX_SPI_TX 		17U 				/*!< DMAMUX Channel 0: SPI0 transmit complete */
#define DMA_REQUEST_MUX_SPI_RX 		16U 				/*!< DMAMUX Channel 1: SPI0 receive complete */
#define DMA_REQUEST_MUX_UART_TX 	3U	 				/*!< DMAMUX Channel 2: UART0 transmit complete */
#define DMA_REQUEST_MUX_UART2_TX 	7U 					/*!< DMAMUX Channel 3: UART2 transmit complete */


/* The UART to use for communication with the AFBR-S50 Explorer. */
#define UART_INSTANCE				0U					/*!< UART Instance Number. */
#define UART_BASEADDR				UART0				/*!< UART Instance Base Address. */
#define UART_IRQn					UART0_IRQn			/*!< UART Interrupt Number. */
#define UART_IRQ_HANDLER			UART0_IRQHandler	/*!< UART Interrupt Handler. */
#define UART_CLKSRC					kCLOCK_CoreSysClk	/*!< UART Clock Source (Core System Clock). */
#ifndef UART_BAUDRATE
#define UART_BAUDRATE				115200U 			/*!< UART Baud Rate. */
#endif


/* The UART2 to use for communication with the AFBR-S50 reference board . */
#define UART2_CLKSRC				kCLOCK_BusClk		/*!< UART2 Clock Source (bus Clock). */
#ifndef UART2_BAUDRATE
#define UART2_BAUDRATE				UART_BAUDRATE		/*!< UART2 Baud Rate. */
#endif

#endif

/*! @} */
#endif /* BOARD_CONFIG_H */
