/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides UART driver functionality.
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
#include "uart.h"

#include "board/board_config.h"
#include "driver/dma.h"
#include "driver/irq.h"
#include "driver/fsl_clock.h"
#include "driver/fsl_port.h"
#include "printf/printf.h"

#include <stdarg.h>
#include <stdlib.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Alias for UART base address. */
#define UART UART_BASEADDR

/*! Output buffer size for debug console. */
#define PRINTF_BUFFER_SIZE 1024

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t SetBaudRate(uart_baud_rates_t baudRate, uint32_t srcClock_Hz);
static void TxDMACallbackFunction(status_t status, void * param);
status_t print(const char  *fmt_s, ...);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uart_error_callback_t myErrorCallback = 0;
static uart_rx_callback_t myRxCallback = 0;
static uart_tx_callback_t myTxCallback = 0;
static void * myTxCallbackState = 0;

static volatile bool isTxOnGoing = false;
static char myBuffer[PRINTF_BUFFER_SIZE] = {0};
static uart_baud_rates_t myBaudRate = UART_INVALID_BPS;

static volatile bool isInitialized = false;


/*******************************************************************************
 * Code
 ******************************************************************************/

status_t UART_Init(void)
{
    if (isInitialized) return STATUS_OK;

    /*****************************************
     * Initialize pins
     *****************************************/
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    CLOCK_EnableClock(kCLOCK_PortA);            /* Ungate the port clock */
    PORT_SetPinMux(PORTA, 1u, kPORT_MuxAlt2);   /* PORTA_PCR1 */
    PORT_SetPinMux(PORTA, 2u, kPORT_MuxAlt2);   /* PORTA_PCR2 */
#elif defined (CPU_MKL17Z256VFM4)
    CLOCK_EnableClock(kCLOCK_PortA);            /* Ungate the port clock */
    PORT_SetPinMux(PORTA, 1u, kPORT_MuxAlt2);   /* PORTA_PCR1 */
    PORT_SetPinMux(PORTA, 2u, kPORT_MuxAlt2);   /* PORTA_PCR2 */
#endif

    /*****************************************
     * Setup DMA module
     *****************************************/
    DMA_Init();

    /* Request DMA channel for TX/RX */
    DMA_ClaimChannel(DMA_CHANNEL_UART_TX, DMA_REQUEST_MUX_UART_TX);

    /* Register callback for DMA interrupt */
    DMA_SetTransferDoneCallback(DMA_CHANNEL_UART_TX, TxDMACallbackFunction, myTxCallbackState);

    /* Set up this channel's control which includes enabling the DMA interrupt */
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    DMA_ConfigTransfer(DMA_CHANNEL_UART_TX, 1, DMA_MEMORY_TO_PERIPHERAL, 0, (uint32_t) (&UART->D), 0); /* dest is data register */
#elif defined (CPU_MKL17Z256VFM4)
    DMA_ConfigTransfer(DMA_CHANNEL_UART_TX, 1, DMA_MEMORY_TO_PERIPHERAL, 0, (uint32_t) (&UART->DATA), 0); /* dest is data register */
#endif

    /*****************************************
     * Setup hardware module
     *****************************************/
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    CLOCK_SetLpsci0Clock(0x1U);
    CLOCK_EnableClock(kCLOCK_Uart0);

    /* Disable TX RX before setting. */
    UART->C2 &= (uint8_t) (~(UART_C2_TE_MASK | UART0_C2_RE_MASK));

#elif defined (CPU_MKL17Z256VFM4)

    CLOCK_SetLpuart0Clock(0x1U);
    CLOCK_EnableClock(kCLOCK_Lpuart0);

    /* Disable LPUART TX RX before setting. */
    UART->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
#endif

    status_t status = SetBaudRate(UART_BAUDRATE, CLOCK_GetFreq(UART_CLKSRC));
    if (status != STATUS_OK) return status;

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

    /* disable parity mode */
    UART->C1 &= (uint8_t) (~(UART_C1_PE_MASK | UART_C1_PT_MASK | UART_C1_M_MASK));

    /* set one stop bit per char */
    UART->BDH &= (uint8_t) (~UART0_BDH_SBNS_MASK);
    UART->BDH |= UART0_BDH_SBNS(0U);

    /* enable error interrupts */
    UART->C3 |= UART0_C3_ORIE_MASK; /* Overrun IRQ enable. */
    UART->C3 |= UART0_C3_NEIE_MASK; /* Noise Error IRQ enable. */
    //      UART->C3 |= UART0_C3_FEIE_MASK; /* Framing Error enable. */

    /* enable rx interrupt */
    UART->C2 |= UART0_C2_RIE_MASK;

    /* Enable the LPSCI TX DMA Request */
    UART->C5 |= UART0_C5_TDMAE_MASK;

    /* Enable TX/RX. */
    UART->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;

    /* Enable interrupt in NVIC. */
    NVIC_SetPriority(UART0_IRQn, IRQPRIO_UART0);

#elif defined (CPU_MKL17Z256VFM4)

    /* disable parity mode */
    UART->CTRL &= (uint8_t)(~(LPUART_CTRL_PE_MASK | LPUART_CTRL_PT_MASK | LPUART_CTRL_M_MASK));

    /* set one stop bit per char */
    UART->BAUD &= (~LPUART_BAUD_SBNS_MASK);
    UART->BAUD |= LPUART_BAUD_SBNS(0U);

    /* enable error interrupts */
    UART->CTRL |= LPUART_CTRL_ORIE_MASK; /* Overrun IRQ enable. */
    UART->CTRL |= LPUART_CTRL_NEIE_MASK; /* Noise Error IRQ enable. */
    //      UART->C3 |= UART0_C3_FEIE_MASK; /* Framing Error enable. */
    /* enable rx interrupt */
    UART->CTRL |= LPUART_CTRL_RIE_MASK;

    /* Enable the LPSCI TX DMA Request */
    UART->BAUD |=LPUART_BAUD_TDMAE_MASK;

    /* enable Break Detect interrupt */
    //UART->BAUD |= LPUART_BAUD_LBKDIE_MASK;
    /* enable RX input Active Edge Interrupt Enable */
    //UART->BAUD |= LPUART_BAUD_RXEDGIE_MASK;
    /* Enable TX/RX. */
    UART->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;

    /* Enable interrupt in NVIC. */
    NVIC_SetPriority(LPUART0_IRQn, LPUART0_IRQn);
#endif

    isInitialized = true;
    return STATUS_OK;
}


uart_baud_rates_t UART_GetBaudRate(void)
{
    assert(isInitialized);
    return myBaudRate;
}
status_t UART_CheckBaudRate(uart_baud_rates_t baudRate)
{
    //assert(isInitialized);
    (void) baudRate;
    return ERROR_NOT_SUPPORTED;
#if 0
    switch (baudRate)
    {
        case UART_115200_BPS:
        case UART_500000_BPS:
        case UART_2000000_BPS:
            return STATUS_OK;

        case UART_INVALID_BPS:
        default:
            return ERROR_UART_BAUDRATE_NOT_SUPPORTED;
    }
#endif
}
status_t UART_SetBaudRate(uart_baud_rates_t baudRate)
{
    //assert(isInitialized);
    (void) baudRate;
    return ERROR_NOT_SUPPORTED;
#if 0

    status_t status = UART_CheckBaudRate(baudRate);
    if (status != STATUS_OK) return status;

    /* Check that we're not busy.*/
    IRQ_LOCK();
    if(isTxOnGoing)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }
    isTxOnGoing = true;
    IRQ_UNLOCK();

    /* remove callback and disable RX line. */
    uart_rx_callback_t callback = myRxCallback;
    UART_SetRxCallback(0);

    /* Disable TX RX before setting. */
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    UART->C2 &= (uint8_t) (~(UART_C2_TE_MASK | UART0_C2_RE_MASK));
#elif defined (CPU_MKL17Z256VFM4)
    UART->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
#endif

    status = SetBaudRate(baudRate, CLOCK_GetFreq(UART_CLKSRC));
    if (status != STATUS_OK) return status;

    /* Enable TX/RX. */
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
    UART->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;
#elif defined (CPU_MKL17Z256VFM4)
    UART->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
#endif

    /* Add callback and enable RX line again. */
    UART_SetRxCallback(callback);

    isTxOnGoing = false;

    return STATUS_OK;
#endif
}


static status_t SetBaudRate(uart_baud_rates_t baudRate, uint32_t srcClock_Hz)
{
    /* This LPSCI instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, OSR is typically hard-set to 16 in other LPSCI instantiations
     * loop to find the best OSR value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of OSR */

    uint16_t sbr = 0;
    uint32_t osr = 0;
    uint32_t baudDiff = baudRate;

    for (uint32_t osrTemp = 4; osrTemp <= 32; osrTemp++)
    {
        /* calculate the temporary sbr value   */
        uint16_t sbrTemp = (uint16_t) (srcClock_Hz / (baudRate * osrTemp));

        /* set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate */
        if (sbrTemp == 0) sbrTemp = 1;

        /* Calculate the baud rate based on the temporary OSR and SBR values */
        uint32_t calculatedBaud = (srcClock_Hz / (osrTemp * sbrTemp));

        uint32_t tempDiff = calculatedBaud - baudRate;

        /* Select the better value between srb and (sbr + 1) */
        if (tempDiff > (baudRate - (srcClock_Hz / (osrTemp * (sbrTemp + 1U)))))
        {
            tempDiff = baudRate - (srcClock_Hz / (osrTemp * (sbrTemp + 1U)));
            sbrTemp++;
        }

        if (tempDiff <= baudDiff)
        {
            baudDiff = tempDiff;
            osr = osrTemp; /* update and store the best OSR value calculated*/
            sbr = sbrTemp; /* update store the best SBR value calculated*/
        }
    }

    /* next, check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate OSR value */
    if (baudDiff > ((baudRate / 100U) * 3U))
    {
        /* Unacceptable baud rate difference of more than 3%*/
        return ERROR_UART_BAUDRATE_NOT_SUPPORTED;
    }

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

    /* Acceptable baud rate */
    /* Check if OSR is between 4x and 7x oversampling */
    /* If so, then "BOTHEDGE" sampling must be turned on*/
    if ((osr > 3U) && (osr < 8U))
    {
        UART->C5 |= UART0_C5_BOTHEDGE_MASK;
    }

    /* program the osr value (bit value is one less than actual value)*/
    UART->C4 = (uint8_t) ((UART->C4 & ~UART0_C4_OSR_MASK) | (osr - 1));

    /* program the sbr (divider) value obtained above*/
    UART->BDH = (uint8_t) ((UART->C4 & ~UART0_BDH_SBR_MASK) | (uint8_t) (sbr >> 8));
    UART->BDL = (uint8_t) sbr;

#elif defined (CPU_MKL17Z256VFM4)

    uint32_t temp = UART->BAUD;

    /* Acceptable baud rate, check if OSR is between 4x and 7x oversampling.
     * If so, then "BOTHEDGE" sampling must be turned on */
    if ((osr > 3) && (osr < 8))
    {
        temp |= LPUART_BAUD_BOTHEDGE_MASK;
    }

    /* program the osr value (bit value is one less than actual value) */
    temp &= ~LPUART_BAUD_OSR_MASK;
    temp |= LPUART_BAUD_OSR(osr - 1);

    /* write the sbr value to the BAUD registers */
    temp &= ~LPUART_BAUD_SBR_MASK;
    UART->BAUD = temp | LPUART_BAUD_SBR(sbr);

#endif

    myBaudRate = baudRate;
    return STATUS_OK;
}


status_t UART_SendBuffer(uint8_t const * txBuff, size_t txSize, uart_tx_callback_t f, void * state)
{
    assert(isInitialized);
    if (!isInitialized) return ERROR_NOT_INITIALIZED;

    /* Debug: only send log entries from threads with lower priority
     * than the UART irq priority. */
//  assert(GET_IPSR() == 0 || GET_IPSR() > 15); // cannot send from exceptions!!

    /* Verify arguments. */
    if(!txBuff || !txSize) return ERROR_INVALID_ARGUMENT;

    /* Check that we're not busy.*/
    IRQ_LOCK();
    if(isTxOnGoing)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }
    isTxOnGoing = true;
    IRQ_UNLOCK();

    myTxCallback = f;
    myTxCallbackState = state;

    /* Set up this channel's control which includes enabling the DMA interrupt */
    DMA0->DMA[DMA_CHANNEL_UART_TX].SAR = (uint32_t)txBuff;  // set source address

    /* Set up this channel's control which includes enabling the DMA interrupt */
    DMA0->DMA[DMA_CHANNEL_UART_TX].DSR_BCR = DMA_DSR_BCR_BCR(txSize); // set transfer count

    /* Enable the DMA peripheral request */
    DMA0->DMA[DMA_CHANNEL_UART_TX].DCR |= DMA_DCR_ERQ_MASK;

    return STATUS_OK;
}

bool UART_IsTxBusy(void)
{
    assert(isInitialized);
    return isTxOnGoing;
}

/*******************************************************************************
 * Debug Console Functions
 ******************************************************************************/

__attribute__((weak)) status_t print(const char *fmt_s, ...)
{
    assert(isInitialized);
    if (!isInitialized) return ERROR_NOT_INITIALIZED;

    /* Wait for IDLE line */
    while (UART_IsTxBusy()) __asm("nop");

    va_list ap;
    va_start(ap, fmt_s);
    int len = vsnprintf_(myBuffer, PRINTF_BUFFER_SIZE, fmt_s, ap);
    va_end(ap);

    if (len < 0 || len >= PRINTF_BUFFER_SIZE) return ERROR_FAIL;
    return UART_SendBuffer((uint8_t*)myBuffer, len, 0, 0);
}

/*******************************************************************************
 * IRQ handler
 ******************************************************************************/

static void TxDMACallbackFunction(status_t status, void * param)
{
    (void)param;
    isTxOnGoing = false;

    if (status < STATUS_OK)
    {
        if (myErrorCallback)
        {
            myErrorCallback(status);
        }
    }

    if (myTxCallback)
    {
        myTxCallback(status, myTxCallbackState);
    }
}




/* LPUART IRQ handler for Rx callback. */
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

void UART0_IRQHandler(void)
{
    /* Get status register. */
    uint8_t s1 = UART->S1;

    /* Handle Rx Data Register Full interrupt */
    if (s1 & UART0_S1_RDRF_MASK)
    {
        /* Get data and invoke callback. */
        const uint8_t data = UART->D;
        myRxCallback(&data, 1);
        return;
    }

//    /* Handle idle line detect interrupt */
//    if(s1 & UART0_S1_IDLE_MASK)
//    {
//        /* Clear the flag, or the rxDataRegFull will not be set any more. */
//      UART->S1 |= UART0_S1_IDLE_MASK;
//      if(myErrorCallback)
//      {
//          myErrorCallback(...);
//      }
//    }

    /* Handle receive overrun interrupt */
    if (s1 & UART0_S1_OR_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->S1 |= UART0_S1_OR_MASK;
        if (myErrorCallback)
        {
            myErrorCallback(ERROR_UART_RX_OVERRUN);
        }
    }

    /* Handle noise interrupt */
    if (s1 & UART0_S1_NF_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->S1 |= UART0_S1_NF_MASK;
        if (myErrorCallback)
        {
            myErrorCallback(ERROR_UART_RX_NOISE);
        }
    }

    /* Handle framing error interrupt */
    if (s1 & UART0_S1_FE_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->S1 |= UART0_S1_FE_MASK;
        if (myErrorCallback)
        {
            myErrorCallback(ERROR_UART_FRAMING_ERR);
        }
    }

//    /* Handle parity error interrupt */
//    if(s1 & UART0_S1_PF_MASK)
//    {
//        /* Clear the flag, or the rxDataRegFull will not be set any more. */
//      UART->S1 |= UART0_S1_PF_MASK;
//      if(myErrorCallback)
//      {
//          myErrorCallback(...);
//      }
//    }
}

#elif defined (CPU_MKL17Z256VFM4)
void LPUART0_IRQHandler(void)
{
    /* Get status register. */
    uint32_t s1 = UART->STAT;

    /* Handle Rx Data Register Full interrupt */
    if(s1 & LPUART_STAT_RDRF_MASK)
    {
        /* Get data and invoke callback. */
        uint8_t data = UART->DATA;
        myRxCallback(&data, 1);
        return;
    }

    //    /* Handle idle line detect interrupt */
    //    if(s1 & LPUART_STAT_IDLE_MASK)
    //    {
    //        /* Clear the flag, or the rxDataRegFull will not be set any more. */
    //      UART->S1 |= LPUART_STAT_IDLE_MASK;
    //      if(myErrorCallback)
    //      {
    //          myErrorCallback(...);
    //      }
    //    }

    /* Handle receive overrun interrupt */
    if(s1 & LPUART_STAT_OR_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->STAT |= LPUART_STAT_OR_MASK;
        if(myErrorCallback)
        {
            myErrorCallback(ERROR_UART_RX_OVERRUN);
        }
    }

    /* Handle noise interrupt */
    if(s1 & LPUART_STAT_NF_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->STAT |= LPUART_STAT_NF_MASK;
        if(myErrorCallback)
        {
            myErrorCallback(ERROR_UART_RX_NOISE);
        }
    }

    /* Handle framing error interrupt */
    if(s1 & LPUART_STAT_FE_MASK)
    {
        /* Clear the flag, or the rxDataRegFull will not be set any more. */
        UART->STAT |= LPUART_STAT_FE_MASK;
        if(myErrorCallback)
        {
            myErrorCallback(ERROR_UART_FRAMING_ERR);
        }
    }

    //    /* Handle parity error interrupt */
    //    if(s1 & UART0_S1_PF_MASK)
    //    {
    //        /* Clear the flag, or the rxDataRegFull will not be set any more. */
    //      UART->S1 |= UART0_S1_PF_MASK;
    //      assert(0);
    //    }
}
#endif


void UART_SetRxCallback(uart_rx_callback_t f)
{
    assert(isInitialized);
    IRQ_LOCK();
    myRxCallback = f;
    if (f != 0)
    {
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        EnableIRQ(UART0_IRQn);
#elif defined (CPU_MKL17Z256VFM4)
        EnableIRQ(LPUART0_IRQn);
#endif
    }
    else
    {
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        DisableIRQ(UART0_IRQn);
#elif defined (CPU_MKL17Z256VFM4)
        DisableIRQ(LPUART0_IRQn);
#endif
    }
    IRQ_UNLOCK();
}
void UART_RemoveRxCallback(void)
{
    assert(isInitialized);
    UART_SetRxCallback(0);
}
void UART_SetErrorCallback(uart_error_callback_t f)
{
    assert(isInitialized);
    myErrorCallback = f;
}
void UART_RemoveErrorCallback(void)
{
    assert(isInitialized);
    UART_SetErrorCallback(0);
}
