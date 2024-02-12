/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides driver functionality for the S2PI interface.
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
 * @brief   Checks whether the SPI interface status is idle and sets it to busy.
 * @details If the current status of the SPI interface is idle, the status is
 *          set to busy. Else wise, a return status; statement is executed.
 *          The interrupts are locked during the check of the current status.
 *****************************************************************************/
#define S2PI_SET_BUSY(hnd) do          \
{                                      \
    IRQ_LOCK();                        \
    if(hnd->Status != STATUS_IDLE)     \
    {                                  \
        IRQ_UNLOCK();                  \
        return hnd->Status;            \
    }                                  \
    hnd->Status = STATUS_BUSY;         \
    IRQ_UNLOCK();                      \
} while (0)

/*!***************************************************************************
 * @brief   Checks whether the SPI interface status is idle and sets it to GPIO mode.
 * @details If the current status of the SPI interface is idle, the status is
 *          set to GPIO mode. Else wise, a return status; statement is executed.
 *          The interrupts are locked during the check of the current status.
 *****************************************************************************/
#define S2PI_SET_GPIO(hnd) do              \
{                                          \
    IRQ_LOCK();                            \
    if(hnd->Status != STATUS_IDLE)         \
    {                                      \
        IRQ_UNLOCK();                      \
        return hnd->Status;                \
    }                                      \
    hnd->Status = STATUS_S2PI_GPIO_MODE;   \
    IRQ_UNLOCK();                          \
} while (0)


/*!***************************************************************************
 * @brief   Resets the current SPI interface status to idle.
 *****************************************************************************/
#define S2PI_SET_IDLE(hnd) do      \
{                                  \
    hnd->Status = STATUS_IDLE;     \
} while (0)

/*!***************************************************************************
 * @brief   Enables the SPI hardware trace via serial connection.
 * @warning Logging decreases the measurement rate drastically!
 *****************************************************************************/
#define S2PI_LOGGING 0

/*! Pin muxing for disable state. */
#define S2PI_PIN_MUX_DISABLED 0


typedef struct s2pi_instance_t
{
    /*! The actual SPI baud rate in bps. */
    uint32_t BaudRate;

    /*! The used SPI hardware instance. */
    SPI_Type * SPI;

} s2pi_instance_t;

//static const s2pi_slave_hnd_t mySlaves[S2PI_SLAVE_COUNT + 1];

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct s2pi_hnd_t
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

    /*! A mutex used for queue operations. */
    volatile bool SpiMutexBlocked;

    s2pi_instance_t Instance[2];

} s2pi_hnd_t;


/*! An additional delay to be added after each GPIO access in order to decrease
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

/*! Return the S2PI handle pointer for a specified S2PI slave identifier. */
static inline s2pi_instance_t * S2PI_GetHandleFromSlave(s2pi_slave_t slave);

/*! Completes the current series of SPI transfers. */
static inline status_t S2PI_CompleteTransfer(s2pi_hnd_t * hnd, status_t status);

/*! Callback for DMA Tx interrupts. */
static void S2PI_TxDmaCallbackFunction(status_t status, void * param);

/*! Callback for DMA Rx interrupts. */
static void S2PI_RxDmaCallbackFunction(status_t status, void * param);

/*! Initializes the SPI hardware. */
static inline void S2PI_InitSpi(s2pi_instance_t * hnd, uint32_t baudRate_Bps);

/*! Initializes the required pins. */
static inline void S2PI_InitPins(SPI_Type * spi);

/*! Sets the SPI instance. */
static inline void S2PI_SetInstance(s2pi_instance_t * instance);

/*! Sets the SPI baud rate in bps. */
static status_t S2PI_SetBaudRateIntern(s2pi_instance_t * hnd, uint32_t baudRate_Bps);

/*!***************************************************************************
 * @brief   Gets the specified GPIO pin of the specified S2PI slave.
 * @param   slave The SPI slave to obtain the GPIO pin from.
 * @param   pin The SPI pin to be obtained (MOSI, MISO, CS, CLK, IRQ).
 * @return  Returns the actual GPIO pin type.
 *****************************************************************************/
static gpio_pin_t S2PI_GetGpioPin(s2pi_slave_t slave, s2pi_pin_t pin);

#if defined(CPU_MKL17Z256VFM4)

#define S2PI_AssertSoftwareCS(slave) ((void)0)
#define S2PI_ClearSoftwareCS(slave) ((void)0)

#else
/*! Helper function to set the current S2PI slave (i.e. CS and IRQ pins). */
static inline status_t S2PI_SetSlaveInternal(s2pi_hnd_t * hnd, s2pi_slave_t slave);

/*! Sets/asserts GPIO for Software CS to LOW state. */
static inline void S2PI_AssertSoftwareCS(s2pi_slave_t slave);

/*! Clears GPIO for Software CS to HIGH state. */
static inline void S2PI_ClearSoftwareCS(s2pi_slave_t slave);
#endif


/******************************************************************************
 * Variables
 ******************************************************************************/
/*! The S2PI data handle. */
static s2pi_hnd_t myS2PIHnd = { 0 };

#ifdef DEBUG
static volatile bool isInitialized = false;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! @cond */
#include "debug.h"
#if S2PI_LOGGING
#include "printf/printf.h"
#define S2PI_LOG_BUFFER_SIZE 0x300
static uint8_t const * myRxPtr;
static size_t myFrameSize;
static char myString[S2PI_LOG_BUFFER_SIZE];
static char * myWrPtr = myString;
static bool isLogging = false;
static void s2pi_log_setup(int slave, uint8_t const * txData, uint8_t const * rxData, size_t frameSize)
{
    if (!isLogging) return;

    myRxPtr = rxData;
    myFrameSize = frameSize;

    if (!myFrameSize) return;

    myWrPtr = myString;
    myWrPtr += sprintf_(myWrPtr, "S2PI Transfer @ Slave %d:", slave);

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

    myWrPtr += sprintf_(myWrPtr, "\n - Tx: 0x");
    for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
    {
        myWrPtr += sprintf_(myWrPtr, " %02X", txData[i]);
    }
    if(myFrameSize & 0x80000000U)
        myWrPtr += sprintf_(myWrPtr, " ...");
}
static void s2pi_log_send(void)
{
    if (!myFrameSize) return;

    myWrPtr += sprintf_(myWrPtr, "\n - Rx: 0x");
    if (myRxPtr != 0)
    {
        for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
        {
            myWrPtr += sprintf_(myWrPtr, " %02X", myRxPtr[i]);
        }
    }
    else
    {
        for (size_t i = 0; i < (myFrameSize & 0x7FFFFFFF); i++)
        {
            myWrPtr += sprintf_(myWrPtr, " --");
        }
    }

    if (myFrameSize & 0x80000000U)
        myWrPtr += sprintf_(myWrPtr, " ...");

    *myWrPtr = '\0';
    myFrameSize = 0;

    print(myString);
}
void S2PI_StartLogging(void){ isLogging = true;}
void S2PI_StopLogging(void){ isLogging = false;}
#else
#define s2pi_log_setup(...) (void)0
#define s2pi_log_send(...)  (void)0
#endif
/*! @endcond */

status_t S2PI_Init(s2pi_slave_t defaultSlave,
                   uint32_t baudRate_Bps)
{
    assert(!isInitialized);

    GPIO_Init();
    DMA_Init();

    /* Register callback for DMA interrupt */
    DMA_SetTransferDoneCallback(DMA_CHANNEL_SPI_TX, S2PI_TxDmaCallbackFunction, &myS2PIHnd);
    DMA_SetTransferDoneCallback(DMA_CHANNEL_SPI_RX, S2PI_RxDmaCallbackFunction, &myS2PIHnd);

    memset(&myS2PIHnd, 0, sizeof(myS2PIHnd));

#if defined(CPU_MKL17Z256VFM4)
    (void)defaultSlave;
    assert(defaultSlave == SPI_DEFAULT_SLAVE);

    myS2PIHnd.Instance[0].SPI = SPI1;
    S2PI_InitPins(myS2PIHnd.Instance[0].SPI);
    S2PI_InitSpi(&myS2PIHnd.Instance[0], baudRate_Bps);
    S2PI_SetInstance(&myS2PIHnd.Instance[0]);
#else
    myS2PIHnd.Instance[0].SPI = SPI1;
    myS2PIHnd.Instance[1].SPI = SPI0;

    s2pi_instance_t * instance = S2PI_GetHandleFromSlave(defaultSlave);
    if (instance == NULL) instance = S2PI_GetHandleFromSlave(SPI_DEFAULT_SLAVE);
    assert(instance != NULL);
    S2PI_InitPins(instance->SPI);
    S2PI_InitSpi(&myS2PIHnd.Instance[0], baudRate_Bps);
    S2PI_InitSpi(&myS2PIHnd.Instance[1], baudRate_Bps);

    S2PI_SetSlaveInternal(&myS2PIHnd, defaultSlave);
#endif

#ifdef DEBUG
    isInitialized = true;
#endif

    return STATUS_OK;
}

static inline void S2PI_InitSpi(s2pi_instance_t * hnd, uint32_t baudRate_Bps)
{
    assert(hnd != 0);
    assert(hnd->SPI == SPI0 || hnd->SPI == SPI1); // No correct S2PI_BASE definition!

    /* Enable clock for S2PI*/
    if (hnd->SPI == SPI0)
    {
        CLOCK_EnableClock(kCLOCK_Spi0);
        NVIC_SetPriority(SPI0_IRQn, IRQPRIO_SPI);
    }
    else
    {
        CLOCK_EnableClock(kCLOCK_Spi1);
        NVIC_SetPriority(SPI1_IRQn, IRQPRIO_SPI);
    }


    /* Reset the S2PI module to it's default state, which includes S2PI disabled */
    hnd->SPI->C1 = 0;
    hnd->SPI->C2 = 0;
    hnd->SPI->BR = 0;
    hnd->SPI->MH = 0;
    hnd->SPI->ML = 0;

    /* Set S2PI to master mode */
    hnd->SPI->C1 |= SPI_C1_MSTR_MASK;

    /* Set slave select to automatic output mode */
    hnd->SPI->C1 |= SPI_C1_SSOE_MASK;
    hnd->SPI->C2 |= SPI_C2_MODFEN_MASK;

    /* Configure clock and data format. */
    hnd->SPI->C1 |= SPI_C1_CPOL_MASK; // set polarity
    hnd->SPI->C1 |= SPI_C1_CPHA_MASK; // set phase

    /* Configure baud rate.*/
    S2PI_SetBaudRateIntern(hnd, baudRate_Bps);

    /* Enable the S2PI RX DMA Request */
    hnd->SPI->C2 |= SPI_C2_RXDMAE_MASK;

    /* Enable the S2PI TX DMA Request */
    hnd->SPI->C2 |= SPI_C2_TXDMAE_MASK;

    /* S2PI system Enable */
    hnd->SPI->C1 |= SPI_C1_SPE_MASK;
}

static inline void S2PI_ResetPins(SPI_Type * spi)
{
#if defined(CPU_MKL17Z256VFM4)
    (void)spi;

    /* Enable all SPI1 slaves. */
    GPIO_SetPinMux(Pin_SPI1_MISO, SPI1_MISO_MUX_SPI);
    GPIO_SetPinMux(Pin_SPI1_MOSI, SPI1_MOSI_MUX_SPI);
    GPIO_SetPinMux(Pin_SPI1_CLK, SPI1_CLK_MUX_SPI);
    GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
#else
    if (spi == SPI0)
    {
        /* Disable all SPI1 slaves. */
        GPIO_SetPinMux(Pin_SPI1_MISO, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI1_MOSI, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI1_CLK, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_PIN_MUX_DISABLED);

        /* Enable all SPI0 slaves. */
        GPIO_SetPinMux(Pin_SPI0_MISO, SPI0_MISO_MUX_SPI);
        GPIO_SetPinMux(Pin_SPI0_MOSI, SPI0_MOSI_MUX_SPI);
        GPIO_SetPinMux(Pin_SPI0_CLK, SPI0_CLK_MUX_SPI);
        GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_CS5_MUX_GPIO);
        GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_CS6_MUX_GPIO);
    }
    else
    {
        /* Enable all SPI1 slaves. */
        GPIO_SetPinMux(Pin_SPI1_MISO, SPI1_MISO_MUX_SPI);
        GPIO_SetPinMux(Pin_SPI1_MOSI, SPI1_MOSI_MUX_SPI);
        GPIO_SetPinMux(Pin_SPI1_CLK, SPI1_CLK_MUX_SPI);
        GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
        GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
        GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
        GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);

        /* Disable all SPI0 slaves. */
        GPIO_SetPinMux(Pin_SPI0_MISO, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI0_MOSI, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI0_CLK, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_PIN_MUX_DISABLED);
    }
#endif
}

static inline void S2PI_InitPins(SPI_Type * spi)
{
#if defined(CPU_MKL17Z256VFM4)
    /* Initialize Pins */
    GPIO_SetPinDir(Pin_SPI1_MOSI, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI1_MOSI);
    GPIO_SetPinDir(Pin_SPI1_CLK, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI1_CLK);
    GPIO_SetPinDir(Pin_SPI1_MISO, Pin_Input);
    GPIO_SetPinPullUp(Pin_SPI1_MISO);
    GPIO_SetPinDir(Pin_S2PI_CS1, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS1);
    GPIO_SetPinDir(Pin_S2PI_IRQ1, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ1);

    GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
#else

    /* Initialize Pins */
    GPIO_SetPinDir(Pin_SPI0_MOSI, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI0_MOSI);
    GPIO_SetPinDir(Pin_SPI0_CLK, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI0_CLK);
    GPIO_SetPinDir(Pin_SPI0_MISO, Pin_Input);
    GPIO_SetPinPullUp(Pin_SPI0_MISO);

    GPIO_SetPinDir(Pin_SPI1_MOSI, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI1_MOSI);
    GPIO_SetPinDir(Pin_SPI1_CLK, Pin_Output);
    GPIO_SetPinOutput(Pin_SPI1_CLK);
    GPIO_SetPinDir(Pin_SPI1_MISO, Pin_Input);
    GPIO_SetPinPullUp(Pin_SPI1_MISO);

    GPIO_SetPinDir(Pin_S2PI_CS1, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS1);
    GPIO_SetPinDir(Pin_S2PI_IRQ1, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ1);

    GPIO_SetPinDir(Pin_S2PI_CS2, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS2);
    GPIO_SetPinDir(Pin_S2PI_IRQ2, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ2);

    GPIO_SetPinDir(Pin_S2PI_CS3, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS3);
    GPIO_SetPinDir(Pin_S2PI_IRQ3, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ3);

    GPIO_SetPinDir(Pin_S2PI_CS4, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS4);
    GPIO_SetPinDir(Pin_S2PI_IRQ4, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ4);

    GPIO_SetPinDir(Pin_S2PI_CS5, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS5);
    GPIO_SetPinDir(Pin_S2PI_IRQ5, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ5);

    GPIO_SetPinDir(Pin_S2PI_CS6, Pin_Output);
    GPIO_SetPinOutput(Pin_S2PI_CS6);
    GPIO_SetPinDir(Pin_S2PI_IRQ6, Pin_Input);
    GPIO_SetPinPullUp(Pin_S2PI_IRQ6);

    GPIO_SetPinMux(Pin_S2PI_IRQ1, S2PI_IRQ1_MUX);
    GPIO_SetPinMux(Pin_S2PI_IRQ2, S2PI_IRQ2_MUX);
    GPIO_SetPinMux(Pin_S2PI_IRQ3, S2PI_IRQ3_MUX);
    GPIO_SetPinMux(Pin_S2PI_IRQ4, S2PI_IRQ4_MUX);
    GPIO_SetPinMux(Pin_S2PI_IRQ5, S2PI_IRQ5_MUX);
    GPIO_SetPinMux(Pin_S2PI_IRQ6, S2PI_IRQ6_MUX);
#endif

    S2PI_ResetPins(spi);
}

s2pi_instance_t * S2PI_GetHandleFromSlave(s2pi_slave_t slave)
{
#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    return &myS2PIHnd.Instance[0];
#else
    switch (slave)
    {
        case S2PI_SLAVE1:
        case S2PI_SLAVE2:
        case S2PI_SLAVE3:
        case S2PI_SLAVE4:
            return &myS2PIHnd.Instance[0];

        case S2PI_SLAVE5:
        case S2PI_SLAVE6:
            return &myS2PIHnd.Instance[1];

        default:
            return NULL;
    }
#endif
}


static inline void S2PI_SetInstance(s2pi_instance_t * instance)
{
    assert(instance != NULL);

#if defined(CPU_MKL17Z256VFM4)
    DMA_ClaimChannel(DMA_CHANNEL_SPI_TX, DMA_REQUEST_MUX_SPI1_TX);
    DMA_ClaimChannel(DMA_CHANNEL_SPI_RX, DMA_REQUEST_MUX_SPI1_RX);
#else
    /* Request DMA channel for TX/RX */
    if (instance->SPI == SPI0)
    {
        DMA_ClaimChannel(DMA_CHANNEL_SPI_TX, DMA_REQUEST_MUX_SPI0_TX);
        DMA_ClaimChannel(DMA_CHANNEL_SPI_RX, DMA_REQUEST_MUX_SPI0_RX);
    }
    else
    {
        DMA_ClaimChannel(DMA_CHANNEL_SPI_TX, DMA_REQUEST_MUX_SPI1_TX);
        DMA_ClaimChannel(DMA_CHANNEL_SPI_RX, DMA_REQUEST_MUX_SPI1_RX);
    }
#endif

    /* Set up this channel's control which includes enabling the DMA interrupt */
    DMA_ConfigTransfer(DMA_CHANNEL_SPI_TX, 1, DMA_MEMORY_TO_PERIPHERAL, 0, (uint32_t)(&instance->SPI->DL), 0); /* dest is data register */
    DMA_ConfigTransfer(DMA_CHANNEL_SPI_RX, 1, DMA_PERIPHERAL_TO_MEMORY, (uint32_t)(&instance->SPI->DL), 0, 0); /* src is data register */
}

#if defined(CPU_MKL17Z256VFM4)
#else
static inline status_t S2PI_SetSlaveInternal(s2pi_hnd_t * hnd, s2pi_slave_t slave)
{
    assert(hnd != 0);

    s2pi_instance_t * new_instance = S2PI_GetHandleFromSlave(slave);
    assert(new_instance != NULL);
    s2pi_instance_t * current_instance = S2PI_GetHandleFromSlave(hnd->Slave);
    if (new_instance != current_instance)
        S2PI_SetInstance(new_instance);

    S2PI_ResetPins(new_instance->SPI);

    switch (slave)
    {
        case S2PI_SLAVE1:
            GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
            break;

        case S2PI_SLAVE2:
//            GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_SPI);
            break;

        case S2PI_SLAVE3:
//            GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
            break;

        case S2PI_SLAVE4:
//            GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
            break;

        case S2PI_SLAVE5:
            GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_CS5_MUX_SPI);
            break;

        case S2PI_SLAVE6:
            GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_CS6_MUX_SPI);
            break;

        default:
            return ERROR_S2PI_INVALID_SLAVE;
            break;
    }

    hnd->Slave = slave;
    return STATUS_OK;
}
#endif

status_t S2PI_SetSlave(s2pi_slave_t slave)
{
#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    return STATUS_OK;
#else
    s2pi_hnd_t * hnd = &myS2PIHnd;

    if (hnd->Slave == slave) return STATUS_OK;

    /* Check if something is ongoing. */
    S2PI_SET_BUSY(hnd);

    status_t status = S2PI_SetSlaveInternal(hnd, slave);

    S2PI_SET_IDLE(hnd);

    return status;
#endif
}

status_t S2PI_TransferFrame(s2pi_slave_t slave,
                            uint8_t const * txData,
                            uint8_t * rxData,
                            size_t frameSize,
                            s2pi_callback_t callback,
                            void * callbackData)
{
    assert(isInitialized);

    /* Verify arguments. */
    if (!txData || !frameSize) return ERROR_INVALID_ARGUMENT;

    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check the driver status and set spi slave.*/
    S2PI_SET_BUSY(hnd);

    s2pi_log_setup(slave, txData, rxData, frameSize);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
#else
    if (hnd->Slave != slave)
    {
        status_t status = S2PI_SetSlaveInternal(hnd, slave);
        if (status < STATUS_OK)
        {
            S2PI_SET_IDLE(hnd);
            return status;
        }
    }
#endif

    hnd->Callback = callback;
    hnd->CallbackParam = callbackData;

    /* Set up the RX DMA channel */
    if (rxData)
    {
        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA0->DMA[DMA_CHANNEL_SPI_RX].DAR = (uint32_t) rxData;      // set dest. address

        /* Set source address increment. */
        DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR |= DMA_DCR_DINC_MASK;
    }
    else
    {
        /* Enable Rx DMA in order to get an reliable IRQ when all transfers are done.
         * Reason: Tx DMA IRQ occurs, when last transfers is still in progress. */

        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA0->DMA[DMA_CHANNEL_SPI_RX].DAR = (uint32_t) (&hnd->RxSink);     // set pseudo dest. address

        /* Unset source address increment. */
        DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR &= ~DMA_DCR_DINC_MASK;
    }

    S2PI_AssertSoftwareCS(slave);

    /* Set up the TX channel's control which includes enabling the DMA interrupt */
    DMA0->DMA[DMA_CHANNEL_SPI_TX].SAR = (uint32_t) txData;

    /* Set up the RX channel's control which includes enabling the DMA interrupt */
    DMA0->DMA[DMA_CHANNEL_SPI_RX].DSR_BCR = DMA_DSR_BCR_BCR(frameSize);

    /* Enable the RX channel's DMA peripheral request */
    DMA0->DMA[DMA_CHANNEL_SPI_RX].DCR |= DMA_DCR_ERQ_MASK;

    /* Set up the TX channel's control which includes enabling the DMA interrupt */
    DMA0->DMA[DMA_CHANNEL_SPI_TX].DSR_BCR = DMA_DSR_BCR_BCR(frameSize);

    /* Enable the TX channel's DMA peripheral request */
    DMA0->DMA[DMA_CHANNEL_SPI_TX].DCR |= DMA_DCR_ERQ_MASK;

    return STATUS_OK;
}

status_t S2PI_GetStatus(s2pi_slave_t slave)
{
    (void)slave;
    assert(isInitialized);

    s2pi_hnd_t * hnd = &myS2PIHnd;
    return hnd->Status;
}

status_t S2PI_TryGetMutex(s2pi_slave_t slave)
{
    (void)slave;
    assert(isInitialized);

    s2pi_hnd_t * hnd = &myS2PIHnd;

    status_t status;

    IRQ_LOCK();
    if (!hnd->SpiMutexBlocked)
    {
        hnd->SpiMutexBlocked = true;
        status = STATUS_OK;
    }
    else
    {
        status = STATUS_BUSY;
    }
    IRQ_UNLOCK();

    return status;
}

void S2PI_ReleaseMutex(s2pi_slave_t slave)
{
    (void)slave;
    assert(isInitialized);

    s2pi_hnd_t * hnd = &myS2PIHnd;
    hnd->SpiMutexBlocked = false;
}

status_t S2PI_Abort(s2pi_slave_t slave)
{
    (void)slave;
    assert(isInitialized);
    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check if something is ongoing. */
    IRQ_LOCK();
    if (hnd->Status == STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return STATUS_OK;
    }

    /* Abort SPI transfer. */
    if (hnd->Status == STATUS_BUSY)
    {
        /* Disable the DMA peripheral request */
        DMA_StopChannel(DMA_CHANNEL_SPI_RX);
        DMA_StopChannel(DMA_CHANNEL_SPI_TX);
        DMA_ClearStatus(DMA_CHANNEL_SPI_RX);
        DMA_ClearStatus(DMA_CHANNEL_SPI_TX);
    }

    IRQ_UNLOCK();

    status_t status = S2PI_CompleteTransfer(hnd, ERROR_ABORTED);
    if(status == ERROR_ABORTED) status = STATUS_OK;

    return status;
}

#if defined(CPU_MKL17Z256VFM4)
#else
static inline void S2PI_ClearSoftwareCS(s2pi_slave_t slave)
{
    /* Clears GPIO for Software CS to HIGH state. */
    if (slave == S2PI_SLAVE2)
    {
        GPIO_SetPinOutput(Pin_S2PI_CS2);
    }
    else if (slave == S2PI_SLAVE3)
    {
        GPIO_SetPinOutput(Pin_S2PI_CS3);
    }
    else if (slave == S2PI_SLAVE4)
    {
        GPIO_SetPinOutput(Pin_S2PI_CS4);
    }
}
static inline void S2PI_AssertSoftwareCS(s2pi_slave_t slave)
{
    /* Sets/asserts GPIO for Software CS to LOW state. */
    if (slave == S2PI_SLAVE2)
    {
        GPIO_ClearPinOutput(Pin_S2PI_CS2);
    }
    else if (slave == S2PI_SLAVE3)
    {
        GPIO_ClearPinOutput(Pin_S2PI_CS3);
    }
    else if (slave == S2PI_SLAVE4)
    {
        GPIO_ClearPinOutput(Pin_S2PI_CS4);
    }
}
#endif

status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
    assert(isInitialized);
    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check the driver status. */
    S2PI_SET_BUSY(hnd);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
    GPIO_ClearPinOutput(Pin_S2PI_CS1);
    GPIO_SetPinOutput(Pin_S2PI_CS1);
    GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
#else

    switch(slave)
    {
        case S2PI_SLAVE1:
        {
            GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS1);
            GPIO_SetPinOutput(Pin_S2PI_CS1);
            GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_SPI);
        } break;

        case S2PI_SLAVE2:
        {
            GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS2);
            GPIO_SetPinOutput(Pin_S2PI_CS2);
        } break;

        case S2PI_SLAVE3:
        {
            GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS3);
            GPIO_SetPinOutput(Pin_S2PI_CS3);
        } break;

        case S2PI_SLAVE4:
        {
            GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS4);
            GPIO_SetPinOutput(Pin_S2PI_CS4);
        } break;

        case S2PI_SLAVE5:
        {
            GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_CS5_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS5);
            GPIO_SetPinOutput(Pin_S2PI_CS5);
            GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_CS5_MUX_SPI);
        } break;

        case S2PI_SLAVE6:
        {
            GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_CS6_MUX_GPIO);
            GPIO_ClearPinOutput(Pin_S2PI_CS6);
            GPIO_SetPinOutput(Pin_S2PI_CS6);
            GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_CS6_MUX_SPI);
        } break;

        default:
        {
            S2PI_SET_IDLE(hnd);
            return ERROR_INVALID_ARGUMENT;
        }
    }
#endif

    S2PI_SET_IDLE(hnd);
    return STATUS_OK;
}

status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
                             s2pi_irq_callback_t callback,
                             void * callbackData)
{
    assert(isInitialized);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    GPIO_SetISR(Pin_S2PI_IRQ1, callback, callbackData);
#else
    switch(slave)
    {
        case S2PI_SLAVE1:
            GPIO_SetISR(Pin_S2PI_IRQ1, callback, callbackData);
            break;

        case S2PI_SLAVE2:
            GPIO_SetISR(Pin_S2PI_IRQ2, callback, callbackData);
            break;

        case S2PI_SLAVE3:
            GPIO_SetISR(Pin_S2PI_IRQ3, callback, callbackData);
            break;

        case S2PI_SLAVE4:
            GPIO_SetISR(Pin_S2PI_IRQ4, callback, callbackData);
            break;

        case S2PI_SLAVE5:
            GPIO_SetISR(Pin_S2PI_IRQ5, callback, callbackData);
            break;

        case S2PI_SLAVE6:
            GPIO_SetISR(Pin_S2PI_IRQ6, callback, callbackData);
            break;

        default:
            return ERROR_S2PI_INVALID_SLAVE;
    }
#endif
    return STATUS_OK;
}

uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
    assert(isInitialized);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ1);
#else
    switch (slave)
    {
        case S2PI_SLAVE1:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ1);

        case S2PI_SLAVE2:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ2);

        case S2PI_SLAVE3:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ3);

        case S2PI_SLAVE4:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ4);

        case S2PI_SLAVE5:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ5);

        case S2PI_SLAVE6:
            return !GPIO_GetInterruptStatus(Pin_S2PI_IRQ6);

        default:
            return 1U; // IQR pin is pull up
    }
#endif
}

status_t S2PI_CaptureGpioControl(s2pi_slave_t slave)
{
    assert(isInitialized);
    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check if something is ongoing. */
    S2PI_SET_GPIO(hnd);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);

    /* Enable SPI1 Pins as GPIO. */
    GPIO_SetPinOutput(Pin_SPI1_MOSI);
    GPIO_SetPinMux(Pin_SPI1_MOSI, SPI1_MOSI_MUX_GPIO);
    GPIO_SetPinPullUp(Pin_SPI1_MISO);
    GPIO_SetPinMux(Pin_SPI1_MISO, SPI1_MISO_MUX_GPIO);
    GPIO_SetPinOutput(Pin_SPI1_CLK);
    GPIO_SetPinMux(Pin_SPI1_CLK, SPI1_CLK_MUX_GPIO);
    GPIO_SetPinOutput(Pin_S2PI_CS1);
    GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
#else
    s2pi_instance_t * instance = S2PI_GetHandleFromSlave(slave);

    /* Output Pins */
    if (instance->SPI == SPI0)
    {
        /* Disable SPI1 Pins. */
        GPIO_SetPinMux(Pin_SPI1_MOSI, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI1_MISO, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI1_CLK, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_PIN_MUX_DISABLED);

        /* Enable SPI0 Pins as GPIO. */
        GPIO_SetPinOutput(Pin_SPI0_MOSI);
        GPIO_SetPinMux(Pin_SPI0_MOSI, SPI0_MOSI_MUX_GPIO);
        GPIO_SetPinPullUp(Pin_SPI0_MISO);
        GPIO_SetPinMux(Pin_SPI0_MISO, SPI0_MISO_MUX_GPIO);
        GPIO_SetPinOutput(Pin_SPI0_CLK);
        GPIO_SetPinMux(Pin_SPI0_CLK, SPI0_CLK_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS5);
        GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_CS5_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS6);
        GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_CS6_MUX_GPIO);
    }
    else
    {
        /* Disable SPI0 Pins. */
        GPIO_SetPinMux(Pin_SPI0_MOSI, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI0_MISO, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_SPI0_CLK, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS5, S2PI_PIN_MUX_DISABLED);
        GPIO_SetPinMux(Pin_S2PI_CS6, S2PI_PIN_MUX_DISABLED);

        /* Enable SPI1 Pins as GPIO. */
        GPIO_SetPinOutput(Pin_SPI1_MOSI);
        GPIO_SetPinMux(Pin_SPI1_MOSI, SPI1_MOSI_MUX_GPIO);
        GPIO_SetPinPullUp(Pin_SPI1_MISO);
        GPIO_SetPinMux(Pin_SPI1_MISO, SPI1_MISO_MUX_GPIO);
        GPIO_SetPinOutput(Pin_SPI1_CLK);
        GPIO_SetPinMux(Pin_SPI1_CLK, SPI1_CLK_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS1);
        GPIO_SetPinMux(Pin_S2PI_CS1, S2PI_CS1_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS2);
        GPIO_SetPinMux(Pin_S2PI_CS2, S2PI_CS2_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS3);
        GPIO_SetPinMux(Pin_S2PI_CS3, S2PI_CS3_MUX_GPIO);
        GPIO_SetPinOutput(Pin_S2PI_CS4);
        GPIO_SetPinMux(Pin_S2PI_CS4, S2PI_CS4_MUX_GPIO);
    }
#endif
    return STATUS_OK;
}

status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave)
{
    assert(isInitialized);
    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check if in GPIO mode. */
    IRQ_LOCK();
    if(hnd->Status != STATUS_S2PI_GPIO_MODE)
    {
        IRQ_UNLOCK();
        return hnd->Status;
    }
    IRQ_UNLOCK();

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    S2PI_ResetPins(NULL);
    status_t status = STATUS_OK;
#else
    status_t status = S2PI_SetSlaveInternal(hnd, slave);
#endif

    S2PI_SET_IDLE(hnd);
    return status;
}

static gpio_pin_t S2PI_GetGpioPin(s2pi_slave_t slave, s2pi_pin_t pin)
{
    assert(isInitialized);

#if defined(CPU_MKL17Z256VFM4)
    (void)slave;
    assert(slave == SPI_DEFAULT_SLAVE);
    switch (pin)
    {
        case S2PI_CS: return Pin_S2PI_CS1;
        case S2PI_CLK: return Pin_SPI1_CLK;
        case S2PI_MOSI: return Pin_SPI1_MOSI;
        case S2PI_IRQ: return Pin_S2PI_IRQ1;
        case S2PI_MISO: return Pin_SPI1_MISO;
    }
#else

    switch (pin)
    {
        case S2PI_CS:

            switch (slave)
            {
                case S2PI_SLAVE1: return Pin_S2PI_CS1;
                case S2PI_SLAVE2: return Pin_S2PI_CS2;
                case S2PI_SLAVE3: return Pin_S2PI_CS3;
                case S2PI_SLAVE4: return Pin_S2PI_CS4;
                case S2PI_SLAVE5: return Pin_S2PI_CS5;
                case S2PI_SLAVE6: return Pin_S2PI_CS6;
            }
            break;

        case S2PI_CLK:

            switch (slave)
            {
                case S2PI_SLAVE1:
                case S2PI_SLAVE2:
                case S2PI_SLAVE3:
                case S2PI_SLAVE4:
                    return Pin_SPI1_CLK;

                case S2PI_SLAVE5:
                case S2PI_SLAVE6:
                    return Pin_SPI0_CLK;
            }
            break;

        case S2PI_MOSI:

            switch (slave)
            {
                case S2PI_SLAVE1:
                case S2PI_SLAVE2:
                case S2PI_SLAVE3:
                case S2PI_SLAVE4:
                    return Pin_SPI1_MOSI;

                case S2PI_SLAVE5:
                case S2PI_SLAVE6:
                    return Pin_SPI0_MOSI;
            }
            break;

        case S2PI_IRQ:

            switch (slave)
            {
                case S2PI_SLAVE1: return Pin_S2PI_IRQ1;
                case S2PI_SLAVE2: return Pin_S2PI_IRQ2;
                case S2PI_SLAVE3: return Pin_S2PI_IRQ3;
                case S2PI_SLAVE4: return Pin_S2PI_IRQ4;
                case S2PI_SLAVE5: return Pin_S2PI_IRQ5;
                case S2PI_SLAVE6: return Pin_S2PI_IRQ6;
            }
            break;

        case S2PI_MISO:

            switch (slave)
            {
                case S2PI_SLAVE1:
                case S2PI_SLAVE2:
                case S2PI_SLAVE3:
                case S2PI_SLAVE4:
                    return Pin_SPI1_MISO;

                case S2PI_SLAVE5:
                case S2PI_SLAVE6:
                    return Pin_SPI0_MISO;
            }
            break;
    }
#endif

    return NULL;
}
status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
    assert(isInitialized);
    assert(pin == S2PI_CS || pin == S2PI_CLK || pin == S2PI_MOSI);

    if (!(pin == S2PI_CS || pin == S2PI_CLK || pin == S2PI_MOSI))
        return ERROR_INVALID_ARGUMENT;

    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check if in GPIO mode. */
    IRQ_LOCK();
    if (hnd->Status != STATUS_S2PI_GPIO_MODE)
    {
        IRQ_UNLOCK();
        return ERROR_S2PI_INVALID_STATE;
    }
    IRQ_UNLOCK();

    gpio_pin_t gpio_pin = S2PI_GetGpioPin(slave, pin);
    GPIO_WritePinOutput(gpio_pin, value);

    /* Decrease SW Protocol Speed by adding a delay. */
    S2PI_GPIO_DELAY();

    return STATUS_OK;
}

status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t * value)
{
    assert(isInitialized);
    assert(pin == S2PI_MISO || pin == S2PI_IRQ);

    if(!(pin == S2PI_MISO || pin == S2PI_IRQ))
        return ERROR_INVALID_ARGUMENT;

    s2pi_hnd_t * hnd = &myS2PIHnd;

    /* Check if in GPIO mode. */
    IRQ_LOCK();
    if (hnd->Status != STATUS_S2PI_GPIO_MODE)
    {
        IRQ_UNLOCK();
        return ERROR_S2PI_INVALID_STATE;
    }
    IRQ_UNLOCK();

    gpio_pin_t gpio_pin = S2PI_GetGpioPin(slave, pin);
    *value = GPIO_ReadPinInput(gpio_pin);

    return STATUS_OK;
}

uint32_t S2PI_GetBaudRate(s2pi_slave_t slave)
{
    assert(isInitialized);
    s2pi_instance_t * instance = S2PI_GetHandleFromSlave(slave);
    return instance->BaudRate;
}

static status_t S2PI_SetBaudRateIntern(s2pi_instance_t * hnd, uint32_t baudRate_Bps)
{
    assert(hnd != 0);
    assert(hnd->SPI == SPI0 || hnd->SPI == SPI1); // No correct S2PI_BASE definition!

    if(baudRate_Bps > SPI_MAX_BAUDRATE)
    {
        return ERROR_S2PI_INVALID_BAUDRATE;
    }

    const uint32_t srcClock_Hz = CLOCK_GetFreq(hnd->SPI == SPI0 ? kCLOCK_BusClk : kCLOCK_CoreSysClk);

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
                hnd->BaudRate = realBaudrate;
            }
        }
    }

    /* Write the best prescaler and baud rate scalar */
    hnd->SPI->BR = (uint8_t)(SPI_BR_SPR(bestDivisor) | SPI_BR_SPPR(bestPrescaler));

    /* Check if the actual baud rate is within 10 % of the desired baud rate. */
    if(min_diff > baudRate_Bps / 10)
        return ERROR_S2PI_INVALID_BAUDRATE;

    return STATUS_OK;
}
status_t S2PI_SetBaudRate(s2pi_slave_t slave, uint32_t baudRate_Bps)
{
    assert(isInitialized);
    s2pi_instance_t * instance = S2PI_GetHandleFromSlave(slave);

    /* Baud Rate Register can be written at any time, no need to block the SPI handle */
    return S2PI_SetBaudRateIntern(instance, baudRate_Bps);
}

static inline status_t S2PI_CompleteTransfer(s2pi_hnd_t * hnd, status_t status)
{
    S2PI_ClearSoftwareCS(hnd->Slave);

    s2pi_log_send();

    S2PI_SET_IDLE(hnd);

    /* Invoke callback if there is one */
    if (hnd->Callback != 0)
    {
        s2pi_callback_t callback = hnd->Callback;
        hnd->Callback = 0;
        status = callback(status, hnd->CallbackParam);
        assert(status == STATUS_OK || status == ERROR_ABORTED);
    }

    return status;
}

/*******************************************************************************
 * IRQ handler
 ******************************************************************************/
static void S2PI_TxDmaCallbackFunction(status_t status, void * param)
{
    assert(status == STATUS_OK);
    if (status < STATUS_OK)
    {
        /* DMA error occurred. */
        s2pi_hnd_t * hnd = (s2pi_hnd_t*)param;
        S2PI_CompleteTransfer(hnd, status);
    }
}

static void S2PI_RxDmaCallbackFunction(status_t status, void * param)
{
    s2pi_hnd_t * hnd = (s2pi_hnd_t*)param;
    S2PI_CompleteTransfer(hnd, status);
}
