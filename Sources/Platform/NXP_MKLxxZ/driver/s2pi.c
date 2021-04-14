/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides driver functionality for the S2PI interface.
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
 *
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "s2pi.h"

#include "board/board_config.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/irq.h"
#include "driver/fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*!***************************************************************************
 * @brief	Checks whether the SPI interface status is idle and sets it to busy.
 * @details	If the current status of the SPI interface is idle, the status is
 * 			set to busy. Else wise, a return status; statement is executed.
 * 			The interrupts are locked during the check of the current status.
 *****************************************************************************/
#define S2PI_SET_BUSY() do					\
{ 											\
	IRQ_LOCK();								\
	if(myS2PIHnd.Status != STATUS_IDLE)		\
	{										\
		IRQ_UNLOCK();						\
		return myS2PIHnd.Status;			\
	}										\
	myS2PIHnd.Status = STATUS_BUSY;			\
	IRQ_UNLOCK();							\
} while (0)

/*!***************************************************************************
 * @brief	Checks whether the SPI interface status is idle and sets it to GPIO mode.
 * @details	If the current status of the SPI interface is idle, the status is
 * 			set to GPIO mode. Else wise, a return status; statement is executed.
 * 			The interrupts are locked during the check of the current status.
 *****************************************************************************/
#define S2PI_SET_GPIO() do						\
{ 												\
	IRQ_LOCK();									\
	if(myS2PIHnd.Status != STATUS_IDLE)			\
	{											\
		IRQ_UNLOCK();							\
		return myS2PIHnd.Status;				\
	}											\
	myS2PIHnd.Status = STATUS_S2PI_GPIO_MODE;	\
	IRQ_UNLOCK();								\
} while (0)


/*!***************************************************************************
 * @brief	Resets the current SPI interface status to idle.
 *****************************************************************************/
#define S2PI_SET_IDLE() do				\
{										\
	myS2PIHnd.Status = STATUS_IDLE;		\
} while (0)

/*!***************************************************************************
 * @brief	Enables the SPI hardware trace via serial connection.
 * @warning	Logging decreases the measurement rate drastically!
 *****************************************************************************/
#define S2PI_LOGGING 0


/*! Pin muxing for disable state. */
#define S2PI_PIN_MUX_DISABLED 0

/*! Alias for SPI base address. */
#define S2PI (S2PI_BASE)

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct
{
	/*! Determines the current driver status. */
	volatile status_t Status;

	/*! Determines the current S2PI slave. */
	volatile s2pi_slave_t Slave;

	/*! A callback function to be called after transfer/run mode is completed. */
	s2pi_callback_t Callback;

	/*! A parameter to be passed to the callback function. */
	void * CallbackParam;

	/*! Dummy variable for unused Rx data. */
	uint8_t RxSink;

	/*! The actual SPI baud rate in bps. */
	uint32_t BaudRate;

} s2pi_handle_t;



/*! An additional delay to be added after each GBIO access in order to decrease
 *  the baud rate of the software EEPROM protocol. Increase the delay if timing
 *  issues occur while reading the EERPOM.
 *  e.g. Delay = 10 µsec => Baud Rate < 100 kHz */
#ifndef S2PI_GPIO_DELAY_US
#define S2PI_GPIO_DELAY_US 10
#endif


#if (S2PI_GPIO_DELAY_US == 0)
#define S2PI_GPIO_DELAY() ((void)0)
#else
#include "utility/time.h"
#define S2PI_GPIO_DELAY() Time_DelayUSec(S2PI_GPIO_DELAY_US)
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! Sets the current S2PI slave (i.e. CS and IRQ pins). */
static status_t S2PI_SetSlave(s2pi_slave_t spi_slave);

/*! Completes the current series of SPI transfers. */
static inline status_t S2PI_CompleteTransfer(status_t status);

/*! Callback for DMA Tx interrupts. */
static void S2PI_TxDmaCallbackFunction(status_t status);

/*! Callback for DMA Rx interrupts. */
static void S2PI_RxDmaCallbackFunction(status_t status);

/*! Initializes the SPI hardware. */
static inline void S2PI_InitSpi(uint32_t baudRate);

/*! Initializes the DMA hardware. */
static inline void S2PI_InitDma(void);

/*! Initializes the required pins. */
static inline void S2PI_InitPins(s2pi_slave_t defaultSlave);

/******************************************************************************
 * Variables
 ******************************************************************************/
/*! The S2PI data handle. */
static s2pi_handle_t myS2PIHnd = {0};


/*******************************************************************************
 * Code
 ******************************************************************************/

/*! @cond */
#include "utility/debug.h"
#if S2PI_LOGGING
#include <stdio.h>
#define S2PI_LOG_BUFFER_SIZE 0x200
static uint8_t const * myRxPtr;
static size_t myFrameSize;
static char myString[S2PI_LOG_BUFFER_SIZE];
static char * myWrPtr = myString;
static void s2pi_log_setup(int slave, uint8_t const * txData, uint8_t const * rxData, size_t frameSize)
{
	myRxPtr = rxData;
	myFrameSize = frameSize;

	if (!myFrameSize) return;

	myWrPtr = myString;
	myWrPtr += sprintf(myWrPtr, "S2PI Transfer @ Slave %d:", slave);

	if (myRxPtr != 0)
	{
		if (2 * (10 + 3 * myFrameSize) > (S2PI_LOG_BUFFER_SIZE - (size_t)(myWrPtr - myString) - 1))
		{
			myFrameSize = ((S2PI_LOG_BUFFER_SIZE - (size_t) (myWrPtr - myString) - 1) / 2 - 14) / 3;
			myFrameSize |= 0x80000000U;
		}
	}
	else
	{
		if ((25 + 3 * myFrameSize) > (S2PI_LOG_BUFFER_SIZE - (size_t)(myWrPtr - myString) - 1))
		{
			myFrameSize = ((S2PI_LOG_BUFFER_SIZE - (size_t) (myWrPtr - myString) - 1) - 14) / 3;
			myFrameSize |= 0x80000000U;
		}
	}

	myWrPtr += sprintf(myWrPtr, "\n - Tx: 0x");
	for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
	{
		myWrPtr += sprintf(myWrPtr, " %02X", txData[i]);
	}
	if(myFrameSize & 0x80000000U)
		myWrPtr += sprintf(myWrPtr, " ...");
}
static void s2pi_log_send(void)
{
	if (!myFrameSize) return;

	myWrPtr += sprintf(myWrPtr, "\n - Rx: 0x");
	if (myRxPtr != 0)
	{
		for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
		{
			myWrPtr += sprintf(myWrPtr, " %02X", myRxPtr[i]);
		}
	}
	else
	{
		for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
		{
			myWrPtr += sprintf(myWrPtr, " --");
		}
	}

	if (myFrameSize & 0x80000000U)
		myWrPtr += sprintf(myWrPtr, " ...");

	*myWrPtr = '\0';
	myFrameSize = 0;

	print(myString);
}
#else
#define s2pi_log_setup(...)	(void)0
#define s2pi_log_send(...)	(void)0
#endif
/*! @endcond */

status_t S2PI_Init(s2pi_slave_t defaultSlave,
				   uint32_t baudRate_Bps)
{
	static bool isInitialized = false;
	if(!isInitialized)
	{
		myS2PIHnd.Status = STATUS_IDLE;
		myS2PIHnd.BaudRate = 0;
		myS2PIHnd.Callback = 0;
		myS2PIHnd.CallbackParam = 0;

		S2PI_InitPins(defaultSlave);
		S2PI_InitDma();
		S2PI_InitSpi(baudRate_Bps);

		isInitialized = true;
	}
	return STATUS_OK;
}

static inline void S2PI_InitSpi(uint32_t baudRate_Bps)
{

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Enable clock for S2PI*/
	CLOCK_EnableClock(kCLOCK_Spi0);
	NVIC_SetPriority(SPI0_IRQn, IRQPRIO_SPI0);
#elif defined (CPU_MKL17Z256VFM4)
	/* Enable clock for S2PI*/
	CLOCK_EnableClock(kCLOCK_Spi1);
	NVIC_SetPriority(SPI1_IRQn, IRQPRIO_SPI1);
#endif

	/* Reset the S2PI module to it's default state, which includes S2PI disabled */
	S2PI->C1 = 0;
	S2PI->C2 = 0;
	S2PI->BR = 0;
	S2PI->MH = 0;
	S2PI->ML = 0;

	/* Set S2PI to master mode */
	S2PI->C1 |= SPI_C1_MSTR_MASK;

	/* Set slave select to automatic output mode */
	S2PI->C1 |= SPI_C1_SSOE_MASK;
	S2PI->C2 |= SPI_C2_MODFEN_MASK;

	/* Configure clock and data format. */
    S2PI->C1 |= SPI_C1_CPOL_MASK; // set polarity
    S2PI->C1 |= SPI_C1_CPHA_MASK; // set phase

	/* Configure baud rate.*/
    S2PI_SetBaudRate(baudRate_Bps);

	/* Enable the S2PI RX DMA Request */
    S2PI->C2 |= SPI_C2_RXDMAE_MASK;

	/* Enable the S2PI TX DMA Request */
    S2PI->C2 |= SPI_C2_TXDMAE_MASK;

	/* S2PI system Enable */
    S2PI->C1 |= SPI_C1_SPE_MASK;
}

static inline void S2PI_InitDma(void)
{
	/* Initialize DMA module for S2PI module. */
	DMA_Init();

	/* Request DMA channel for TX/RX */
	DMA_ClaimChannel(DMA_CHANNEL_SPI_TX, DMA_REQUEST_MUX_SPI_TX);
	DMA_ClaimChannel(DMA_CHANNEL_SPI_RX, DMA_REQUEST_MUX_SPI_RX);

	/* Register callback for DMA interrupt */
	DMA_SetTransferDoneCallback(DMA_CHANNEL_SPI_TX, S2PI_TxDmaCallbackFunction);
	DMA_SetTransferDoneCallback(DMA_CHANNEL_SPI_RX, S2PI_RxDmaCallbackFunction);

	/* Set up this channel's control which includes enabling the DMA interrupt */
	DMA_ConfigTransfer(DMA_CHANNEL_SPI_TX, 1, DMA_MEMORY_TO_PERIPHERAL, 0, (uint32_t)(&S2PI->DL), 0); /* dest is data register */
	DMA_ConfigTransfer(DMA_CHANNEL_SPI_RX, 1, DMA_PERIPHERAL_TO_MEMORY, (uint32_t)(&S2PI->DL), 0, 0); /* src is data register */

}

static inline void S2PI_InitPins(s2pi_slave_t defaultSlave)
{
	/* Setup GPIO module */
	GPIO_Init();

	/* Setup S2PI default slave */
	S2PI_SetSlave(defaultSlave);
}

static inline status_t S2PI_SetSlaveInternal(s2pi_slave_t spi_slave)
{
	if (spi_slave == S2PI_PINS_LOW)
	{
		/* Set all @ output and clear */
		GPIO_SetPinDir(Pin_S2PI_MOSI, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_MOSI);
		GPIO_SetPinDir(Pin_S2PI_CLK, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_CLK);
		GPIO_SetPinDir(Pin_S2PI_MISO, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_MISO);
		GPIO_SetPinDir(Pin_S2PI_CS1, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_CS1);
		GPIO_SetPinDir(Pin_S2PI_IRQ1, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_IRQ1);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_SetPinDir(Pin_S2PI_CS2, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_CS2);
		GPIO_SetPinDir(Pin_S2PI_IRQ2, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_IRQ2);
		GPIO_SetPinDir(Pin_S2PI_CS3, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_CS3);
		GPIO_SetPinDir(Pin_S2PI_IRQ3, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_IRQ3);
		GPIO_SetPinDir(Pin_S2PI_CS4, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_CS4);
		GPIO_SetPinDir(Pin_S2PI_IRQ4, Pin_Input);
		GPIO_SetPinPullDown(Pin_S2PI_IRQ4);
#endif
	}
	else
	{
		/* Set Pin Directions */
		GPIO_SetPinDir(Pin_S2PI_MOSI, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_CLK, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_MISO, Pin_Input);
		GPIO_SetPinDir(Pin_S2PI_CS1, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_IRQ1, Pin_Input);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_SetPinDir(Pin_S2PI_CS2, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_IRQ2, Pin_Input);
		GPIO_SetPinDir(Pin_S2PI_CS3, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_IRQ3, Pin_Input);
		GPIO_SetPinDir(Pin_S2PI_CS4, Pin_Output);
		GPIO_SetPinDir(Pin_S2PI_IRQ4, Pin_Input);
#endif

		/* Output Pins */
		GPIO_SetPinOutput(Pin_S2PI_MOSI);
		GPIO_SetPinOutput(Pin_S2PI_CLK);
		GPIO_SetPinOutput(Pin_S2PI_CS1);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_SetPinOutput(Pin_S2PI_CS2);
		GPIO_SetPinOutput(Pin_S2PI_CS3);
		GPIO_SetPinOutput(Pin_S2PI_CS4);
#endif

		/* Input Pins */
		GPIO_SetPinPullUp(Pin_S2PI_MISO);
		GPIO_SetPinPullUp(Pin_S2PI_IRQ1);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_SetPinPullUp(Pin_S2PI_IRQ2);
		GPIO_SetPinPullUp(Pin_S2PI_IRQ3);
		GPIO_SetPinPullUp(Pin_S2PI_IRQ4);
#endif
	}

	/* Pin Muxing */
	switch (spi_slave)
	{
		case S2PI_S1:
		{
			GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_MISO_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_MOSI_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CLK, S2PI_CLK_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
#endif
		}
		break;

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		case S2PI_S2:
		{
			GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_MISO_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_MOSI_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CLK, S2PI_CLK_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
		}
		break;

#if S2PI_GPIO_SLAVES
		case S2PI_S3:
		case S2PI_S4:
		case S2PI_S1_GPIO:
		case S2PI_S2_GPIO:
		{
			GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_MISO_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_MOSI_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CLK, S2PI_CLK_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
		}
		break;
#endif
#endif
		case S2PI_NONE:
		{
			GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_CLK, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_PIN_MUX_DISABLED);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_PIN_MUX_DISABLED);
			GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_PIN_MUX_DISABLED);
#endif
		}
		break;

		case S2PI_PINS_LOW:
		{
			GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_MISO_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_MOSI_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CLK, S2PI_CLK_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
			GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
			GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
#endif
		}
		break;

		default:
		{
			return ERROR_S2PI_INVALID_SLAVE;
		}
		break;
	}

	myS2PIHnd.Slave = spi_slave;
	return STATUS_OK;
}

static status_t S2PI_SetSlave(s2pi_slave_t spi_slave)
{
	if (myS2PIHnd.Slave == spi_slave) return STATUS_OK;

	/* Check if something is ongoing. */
	S2PI_SET_BUSY();

	status_t status = S2PI_SetSlaveInternal(spi_slave);

	S2PI_SET_IDLE();

	return status;
}

status_t S2PI_TransferFrame(s2pi_slave_t spi_slave,
							uint8_t const * txData,
							uint8_t * rxData,
							size_t frameSize,
							s2pi_callback_t callback,
							void * callbackData)
{
	/* Verify arguments. */
	if (!txData || !frameSize) return ERROR_INVALID_ARGUMENT;

	/* Check the driver status and set spi slave.*/
	S2PI_SET_BUSY();

	s2pi_log_setup(spi_slave, txData, rxData, frameSize);

	if (myS2PIHnd.Slave != spi_slave)
	{
		status_t status = S2PI_SetSlaveInternal(spi_slave);
		if (status < STATUS_OK)
		{
			S2PI_SET_IDLE();
			return status;
		}
	}

	myS2PIHnd.Callback = callback;
	myS2PIHnd.CallbackParam = callbackData;

	/* Set up the RX DMA channel */
	if (rxData)
	{
		/* Set up this channel's control which includes enabling the DMA interrupt */
		DMA0->DMA[DMA_CHANNEL_SPI_RX].DAR = (uint32_t) rxData;		// set dest. address

		/* Set source address increment. */
		DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR |= DMA_DCR_DINC_MASK;
	}
	else
	{
		/* Enable Rx DMA in order to get an reliable IRQ when all transfers are done.
		 * Reason: Tx DMA IRQ occurs, when last transfers is still in progress. */

		/* Set up this channel's control which includes enabling the DMA interrupt */
		DMA0->DMA[DMA_CHANNEL_SPI_RX].DAR = (uint32_t) (&myS2PIHnd.RxSink);		// set pseudo dest. address

		/* Unset source address increment. */
		DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR &= ~DMA_DCR_DINC_MASK;
	}

#if S2PI_GPIO_SLAVES
	/* Clear GPIO CS. */
	if(myS2PIHnd.Slave == S2PI_S1_GPIO)
	{
		GPIO_ClearPinOutput(Pin_S2PI_CS1);
	}
	else if(myS2PIHnd.Slave == S2PI_S2_GPIO)
	{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_ClearPinOutput(Pin_S2PI_CS2);
#endif
	}
	else if(myS2PIHnd.Slave == S2PI_S3)
	{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_ClearPinOutput(Pin_S2PI_CS3);
#endif
	}
	else if(myS2PIHnd.Slave == S2PI_S4)
	{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		GPIO_ClearPinOutput(Pin_S2PI_CS4);
#endif
	}
#endif

	/* Set up this channel's control which includes enabling the DMA interrupt */
	DMA0->DMA[DMA_CHANNEL_SPI_TX].SAR = (uint32_t) txData;

	/* Set up the Rx channel's control which includes enabling the DMA interrupt */
	DMA0->DMA[DMA_CHANNEL_SPI_RX].DSR_BCR = DMA_DSR_BCR_BCR(frameSize);

	/* Enable the DMA peripheral request */
	DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR |= DMA_DCR_ERQ_MASK;

	/* Set up the Tx channel's control which includes enabling the DMA interrupt */
	DMA0->DMA[DMA_CHANNEL_SPI_TX].DSR_BCR = DMA_DSR_BCR_BCR(frameSize);

	/* Enable the DMA peripheral request */
	DMA0->DMA[DMA_CHANNEL_SPI_TX].DCR |= DMA_DCR_ERQ_MASK;

	return STATUS_OK;
}

status_t S2PI_GetStatus(void)
{
	return myS2PIHnd.Status;
}

status_t S2PI_Abort(void)
{
	/* Check if something is ongoing. */
	IRQ_LOCK();
	if (myS2PIHnd.Status == STATUS_IDLE)
	{
		IRQ_UNLOCK();
		return STATUS_OK;
	}

	/* Abort SPI transfer. */
	if (myS2PIHnd.Status == STATUS_BUSY)
	{
		/* Disable the DMA peripheral request */
		DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR &= ~DMA_DCR_ERQ_MASK;
		DMA0->DMA[DMA_CHANNEL_SPI_TX].DCR &= ~DMA_DCR_ERQ_MASK;
	}

	IRQ_UNLOCK();

	status_t status = S2PI_CompleteTransfer(ERROR_ABORTED);
	if(status == ERROR_ABORTED) status = STATUS_OK;

	return status;
}

status_t S2PI_PinsOff(void)
{
	status_t status = S2PI_SetSlave(S2PI_NONE);
	if(status == STATUS_OK)
	{
		S2PI_SET_IDLE();
	}
	return status;
}

status_t S2PI_PinsLow(void)
{
	status_t status = S2PI_SetSlave(S2PI_PINS_LOW);
	if(status == STATUS_OK)
	{
		S2PI_SET_IDLE();
	}
	return status;
}

status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
	/* Check the driver status. */
	S2PI_SET_BUSY();

	switch(slave)
	{
		case S2PI_S1:
		{
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
			GPIO_ClearPinOutput(Pin_S2PI_CS1);
			GPIO_SetPinOutput(Pin_S2PI_CS1);
			GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
		} break;

		case S2PI_S2:
		{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
			GPIO_ClearPinOutput(Pin_S2PI_CS2);
			GPIO_SetPinOutput(Pin_S2PI_CS2);
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_SPI);
			GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_SPI);
#endif
		} break;

#if S2PI_GPIO_SLAVES
		case S2PI_S1_GPIO:
		{
			GPIO_ClearPinOutput(Pin_S2PI_CS1);
			GPIO_SetPinOutput(Pin_S2PI_CS1);
		} break;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		case S2PI_S2_GPIO:
		{
			GPIO_ClearPinOutput(Pin_S2PI_CS2);
			GPIO_SetPinOutput(Pin_S2PI_CS2);
		} break;
		case S2PI_S3:
		{
			GPIO_ClearPinOutput(Pin_S2PI_CS3);
			GPIO_SetPinOutput(Pin_S2PI_CS3);
		} break;
		case S2PI_S4:
		{
			GPIO_ClearPinOutput(Pin_S2PI_CS4);
			GPIO_SetPinOutput(Pin_S2PI_CS4);
		} break;
#endif
#endif
		default:
		{
			S2PI_SET_IDLE();
			return ERROR_INVALID_ARGUMENT;
		}
	}

	S2PI_SET_IDLE();
	return STATUS_OK;
}

status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
							 s2pi_irq_callback_t callback,
							 void * callbackData)
{
	switch(slave)
	{
		case S2PI_S1:
#if S2PI_GPIO_SLAVES
		case S2PI_S1_GPIO:
#endif
			GPIO_SetISR(Pin_S2PI_IRQ1, callback, callbackData);
		break;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		case S2PI_S2:
#if S2PI_GPIO_SLAVES
		case S2PI_S2_GPIO:
#endif
			GPIO_SetISR(Pin_S2PI_IRQ2, callback, callbackData);
			break;

#if S2PI_GPIO_SLAVES
		case S2PI_S3:
			GPIO_SetISR(Pin_S2PI_IRQ3, callback, callbackData);
			break;
		case S2PI_S4:
			GPIO_SetISR(Pin_S2PI_IRQ4, callback, callbackData);
			break;
#endif
#endif
		default:
			return ERROR_S2PI_INVALID_SLAVE;
	}
	return STATUS_OK;
}

uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
	switch (slave)
	{
		case S2PI_S1:
#if S2PI_GPIO_SLAVES
		case S2PI_S1_GPIO:
#endif
			return GPIO_ReadPinInput(Pin_S2PI_IRQ1);
		break;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		case S2PI_S2:
#if S2PI_GPIO_SLAVES
		case S2PI_S2_GPIO:
#endif
			return GPIO_ReadPinInput(Pin_S2PI_IRQ2);
		break;

#if S2PI_GPIO_SLAVES
		case S2PI_S3:
			return GPIO_ReadPinInput(Pin_S2PI_IRQ3);
		break;
		case S2PI_S4:
			return GPIO_ReadPinInput(Pin_S2PI_IRQ4);
		break;
#endif
#endif
		default:
			return 1U; // IQR pin is pull up
	}
}

status_t S2PI_CaptureGpioControl(void)
{
	/* Check if something is ongoing. */
	S2PI_SET_GPIO();

	/* Output Pins */
	GPIO_SetPinOutput(Pin_S2PI_CS1);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	GPIO_SetPinOutput(Pin_S2PI_CS2);
	GPIO_SetPinOutput(Pin_S2PI_CS3);
	GPIO_SetPinOutput(Pin_S2PI_CS4);
#endif
	GPIO_SetPinOutput(Pin_S2PI_MOSI);
	GPIO_SetPinOutput(Pin_S2PI_CLK);

	/* Input Pins */
	GPIO_SetPinPullUp(Pin_S2PI_MISO);
	GPIO_SetPinPullUp(Pin_S2PI_IRQ1);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	GPIO_SetPinPullUp(Pin_S2PI_IRQ2);
	GPIO_SetPinPullUp(Pin_S2PI_IRQ3);
	GPIO_SetPinPullUp(Pin_S2PI_IRQ4);
#endif

	/* Pin Muxing. */
	GPIO_SetPinMux(Pin_S2PI_MISO, S2PI_MISO_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_MOSI, S2PI_MOSI_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_CLK,  S2PI_CLK_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
	GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
	GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
	GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
#endif
	return STATUS_OK;
}

status_t S2PI_ReleaseGpioControl(void)
{
	/* Check if in GPIO mode. */
	IRQ_LOCK();
	if(myS2PIHnd.Status != STATUS_S2PI_GPIO_MODE)
	{
		IRQ_UNLOCK();
		return myS2PIHnd.Status;
	}
	IRQ_UNLOCK();

	status_t status = S2PI_SetSlaveInternal(myS2PIHnd.Slave);

	S2PI_SET_IDLE();
	return status;
}

status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
	/* Check if in GPIO mode. */
	IRQ_LOCK();
	if(myS2PIHnd.Status != STATUS_S2PI_GPIO_MODE)
	{
		IRQ_UNLOCK();
		return ERROR_S2PI_INVALID_STATE;
	}
	IRQ_UNLOCK();

	switch(pin)
	{
		case S2PI_CS:
		{
			switch(slave)
			{
				case S2PI_S1:
#if S2PI_GPIO_SLAVES
				case S2PI_S1_GPIO:
#endif
				{
					GPIO_WritePinOutput(Pin_S2PI_CS1, value);
				} break;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
				case S2PI_S2:
#if S2PI_GPIO_SLAVES
				case S2PI_S2_GPIO:
#endif
				{
					GPIO_WritePinOutput(Pin_S2PI_CS2, value);
				} break;
#if S2PI_GPIO_SLAVES
				case S2PI_S3:
				{
					GPIO_WritePinOutput(Pin_S2PI_CS3, value);
				} break;
				case S2PI_S4:
				{
					GPIO_WritePinOutput(Pin_S2PI_CS4, value);
				} break;
#endif
#endif
				default:
					return ERROR_INVALID_ARGUMENT;
			}
		} break;
//		case S2PI_IRQ:
//		{
//			switch(slave)
//			{
//				case S2PI_S1:
//#if S2PI_GPIO_SLAVES
//				case S2PI_S1_GPIO:
//#endif
//				{
//					GPIO_WritePinOutput(Pin_S2PI_IRQ1, value);
//				} break;
//#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
//				case S2PI_S2:
//#if S2PI_GPIO_SLAVES
//				case S2PI_S2_GPIO:
//#endif
//				{
//					GPIO_WritePinOutput(Pin_S2PI_IRQ2, value);
//				} break;
//#if S2PI_GPIO_SLAVES
//				case S2PI_S3:
//				{
//					GPIO_WritePinOutput(Pin_S2PI_IRQ3, value);
//				} break;
//				case S2PI_S4:
//				{
//					GPIO_WritePinOutput(Pin_S2PI_IRQ4, value);
//				} break;
//#endif
//#endif
//				default:
//					return ERROR_INVALID_ARGUMENT;
//			}
//		} break;
		case S2PI_CLK:
		{
			GPIO_WritePinOutput(Pin_S2PI_CLK, value);
		} break;
		case S2PI_MOSI:
		{
			GPIO_WritePinOutput(Pin_S2PI_MOSI, value);
		} break;
//		case S2PI_MISO:
//		{
//			GPIO_WritePinOutput(Pin_S2PI_MISO, value);
//		} break;
		default:
			return ERROR_INVALID_ARGUMENT;
	}

	/* Decrease SW Protocol Speed by adding a delay. */
	S2PI_GPIO_DELAY();

	return STATUS_OK;
}

status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t * value)
{
	/* Check if in GPIO mode. */
	IRQ_LOCK();
	if(myS2PIHnd.Status != STATUS_S2PI_GPIO_MODE)
	{
		IRQ_UNLOCK();
		return ERROR_S2PI_INVALID_STATE;
	}
	IRQ_UNLOCK();

	switch(pin)
	{
//		case S2PI_CS:
//		{
//			switch(slave)
//			{
//				case S2PI_S1:
//#if S2PI_GPIO_SLAVES
//				case S2PI_S1_GPIO:
//#endif
//					*value = GPIO_ReadPinInput(Pin_S2PI_CS1);
//					break;
//#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
//				case S2PI_S2:
//#if S2PI_GPIO_SLAVES
//				case S2PI_S2_GPIO:
//#endif
//					*value = GPIO_ReadPinInput(Pin_S2PI_CS2);
//					break;
//#if S2PI_GPIO_SLAVES
//				case S2PI_S3:
//					*value = GPIO_ReadPinInput(Pin_S2PI_CS3);
//					break;
//				case S2PI_S4:
//					*value = GPIO_ReadPinInput(Pin_S2PI_CS4);
//					break;
//#endif
//#endif
//				default:
//					return ERROR_INVALID_ARGUMENT;
//			}
//		} break;
		case S2PI_IRQ:
		{
			switch(slave)
				{
					case S2PI_S1:
#if S2PI_GPIO_SLAVES
					case S2PI_S1_GPIO:
#endif
						*value = GPIO_ReadPinInput(Pin_S2PI_IRQ1);
						break;
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
					case S2PI_S2:
#if S2PI_GPIO_SLAVES
					case S2PI_S2_GPIO:
#endif
						*value = GPIO_ReadPinInput(Pin_S2PI_IRQ2);
						break;
#if S2PI_GPIO_SLAVES
					case S2PI_S3:
						*value = GPIO_ReadPinInput(Pin_S2PI_IRQ3);
						break;
					case S2PI_S4:
						*value = GPIO_ReadPinInput(Pin_S2PI_IRQ4);
						break;
#endif
#endif
					default:
						return ERROR_INVALID_ARGUMENT;
				}
		} break;
//		case S2PI_CLK:
//		{
//			*value = GPIO_ReadPinInput(Pin_S2PI_CLK);
//		} break;
//		case S2PI_MOSI:
//		{
//			*value = GPIO_ReadPinInput(Pin_S2PI_MOSI);
//		} break;
		case S2PI_MISO:
		{
			*value = GPIO_ReadPinInput(Pin_S2PI_MISO);
		} break;
		default:
			return ERROR_INVALID_ARGUMENT;
	}
	return STATUS_OK;
}

uint32_t S2PI_GetBaudRate(void)
{
	return myS2PIHnd.BaudRate;
}

status_t S2PI_SetBaudRate(uint32_t baudRate_Bps)
{
    status_t status = STATUS_OK;

	if(baudRate_Bps > SPI_BAUD_RATE_MAX)
	{
		return ERROR_S2PI_INVALID_BAUD_RATE;
	}

    // Baud Rate Register can be written at any time
//	/* Check the driver status.*/
//	IRQ_LOCK();
//	if(myS2PIHnd.Status != STATUS_IDLE)
//	{
//		IRQ_UNLOCK();
//		return myS2PIHnd.Status;
//	}
//	myS2PIHnd.Status = STATUS_BUSY;
//	IRQ_UNLOCK();

    uint32_t srcClock_Hz = CLOCK_GetFreq(kCLOCK_BusClk);

    /* Find combination of prescaler and scaler resulting in baud rate
     * closest to the requested value */
    uint32_t min_diff = 0xFFFFFFFFU;

    /* Set the maximum divisor bit settings for each of the following divisors */
    uint32_t bestPrescaler = 7U;
    uint32_t bestDivisor = 8U;

    /* In all for loops, if min_diff = 0, the exit for loop*/
    for (uint32_t prescaler = 0; (prescaler <= 7) && min_diff; prescaler++)
    {
        for (uint32_t rateDivisor = 0; (rateDivisor <= 8U) && min_diff; rateDivisor++)
        {
        	uint32_t rateDivisorValue = 2U << rateDivisor;

            /* Calculate actual baud rate, note need to add 1 to prescaler */
        	uint32_t realBaudrate = ((srcClock_Hz) / ((prescaler + 1) * rateDivisorValue));

            /* Calculate the baud rate difference based on the conditional statement,
             * that states that the calculated baud rate must not exceed the desired
             * baud rate */

        	uint32_t diff = 0xFFFFFFFFU;
            if (baudRate_Bps >= realBaudrate)
            	diff = baudRate_Bps - realBaudrate;
            else
            	diff = realBaudrate - baudRate_Bps;

			if (min_diff > diff)
			{
				/* A better match found */
				min_diff = diff;
				bestPrescaler = prescaler;
				bestDivisor = rateDivisor;
				myS2PIHnd.BaudRate = realBaudrate;
			}
        }
    }

    /* Write the best prescaler and baud rate scalar */
    S2PI->BR = (uint8_t)(SPI_BR_SPR(bestDivisor) | SPI_BR_SPPR(bestPrescaler));

    /* Check if the actual baud rate is within 10 % of the desired baud rate. */
    if(min_diff > baudRate_Bps / 10) status = ERROR_S2PI_INVALID_BAUD_RATE;

//    myS2PIHnd.Status = STATUS_IDLE;
    return status;
}

static inline status_t S2PI_CompleteTransfer(status_t status)
{
#if S2PI_GPIO_SLAVES
	/* Reset GPIO CS */
	if(myS2PIHnd.Slave == S2PI_S1_GPIO)
	{
		GPIO_SetPinOutput(Pin_S2PI_CS1);
	}
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	else if(myS2PIHnd.Slave == S2PI_S2_GPIO)
	{
		GPIO_SetPinOutput(Pin_S2PI_CS2);
	}
	else if(myS2PIHnd.Slave == S2PI_S3)
	{
		GPIO_SetPinOutput(Pin_S2PI_CS3);
	}
	else if(myS2PIHnd.Slave == S2PI_S4)
	{
		GPIO_SetPinOutput(Pin_S2PI_CS4);
	}
#endif
#endif

	s2pi_log_send();

	S2PI_SET_IDLE();

	/* Invoke callback if there is one */
	if (myS2PIHnd.Callback != 0)
	{
		s2pi_callback_t callback = myS2PIHnd.Callback;
		myS2PIHnd.Callback = 0;
		status = callback(status, myS2PIHnd.CallbackParam);
	}
	return status;
}

/*******************************************************************************
 * IRQ handler
 ******************************************************************************/
static void S2PI_TxDmaCallbackFunction(status_t status)
{
	if(status < STATUS_OK)
	{
		/* DMA error occurred. */
		S2PI_CompleteTransfer(status);
	}
}

static void S2PI_RxDmaCallbackFunction(status_t status)
{
	S2PI_CompleteTransfer(status);
}
