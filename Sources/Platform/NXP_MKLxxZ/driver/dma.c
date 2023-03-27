/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides DMA hardware support.
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
#include "dma.h"

#include <stdbool.h>


/* CMSIS-style register definitions */
//#include "devices/MKL46Z4.h"
/* CPU specific feature definitions */
//#include "devices/MKL46Z4_features.h"

#include "board/board_config.h"

#include "driver/fsl_clock.h"
#include "board/board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DMA_CHANNEL_COUNT 4

static const IRQn_Type irqNumbers[DMA_CHANNEL_COUNT] = {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void DMA0_IRQHandler(void);                 /*!< ISR for DMA0 IRQ. */
void DMA1_IRQHandler(void);                 /*!< ISR for DMA1 IRQ. */
void DMA2_IRQHandler(void);                 /*!< ISR for DMA2 IRQ. */
void DMA3_IRQHandler(void);                 /*!< ISR for DMA3 IRQ. */
static inline void DMA_IRQhandler(uint32_t channel);

/******************************************************************************
 * Variables
 ******************************************************************************/
static dma_callback_t myCallbacks[DMA_CHANNEL_COUNT] = {0};
static void * myCallbackParams[DMA_CHANNEL_COUNT] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

void DMA_Init(void)
{
    static bool isInitialized = false;
    if(!isInitialized)
    {
        memset(myCallbacks, 0, sizeof(myCallbacks));
        memset(myCallbackParams, 0, sizeof(myCallbackParams));

        /* Enable DMA clock. */
        CLOCK_EnableClock(kCLOCK_Dma0);

        /* Enable DMAMUX clock and init. */
        CLOCK_EnableClock(kCLOCK_Dmamux0);

        /* Set IRQ priorities. */
        NVIC_SetPriority(DMA0_IRQn, IRQPRIO_DMA0);
        NVIC_SetPriority(DMA1_IRQn, IRQPRIO_DMA1);
        NVIC_SetPriority(DMA2_IRQn, IRQPRIO_DMA2);
        NVIC_SetPriority(DMA3_IRQn, IRQPRIO_DMA3);

        /* Initialize the dmamux module to the reset state. */
        for (int i = 0; i < FSL_FEATURE_DMAMUX_MODULE_CHANNEL; i++)
        {
            DMAMUX0->CHCFG[i] &= (uint8_t)(~DMAMUX_CHCFG_ENBL_MASK);
            DMAMUX0->CHCFG[i] = (uint8_t)((DMAMUX0->CHCFG[i] & ~DMAMUX_CHCFG_SOURCE_MASK) | DMAMUX_CHCFG_SOURCE(0));
        }

        isInitialized = true;
    }
}

void DMA_SetTransferDoneCallback(uint32_t channel, dma_callback_t f, void * param)
{
    assert(channel < DMA_CHANNEL_COUNT);
    myCallbacks[channel] = f;
    myCallbackParams[channel] = param;
}

void DMA_RemoveTransferDoneCallback(uint32_t channel)
{
    assert(channel < DMA_CHANNEL_COUNT);
    myCallbacks[channel] = 0;
    myCallbackParams[channel] = 0;
}

void DMA_ClaimChannel(uint32_t channel, uint8_t source)
{
    assert(channel < DMA_CHANNEL_COUNT);

    /* Enable NVIC interrupt. */
    EnableIRQ(irqNumbers[channel]);

    /* Configure DMAMUX channel */
    DMAMUX0->CHCFG[channel] &= (uint8_t)(~DMAMUX_CHCFG_ENBL_MASK); // Disables the DMAMUX channel.
    DMAMUX0->CHCFG[channel] = (uint8_t)((DMAMUX0->CHCFG[channel] & ~DMAMUX_CHCFG_SOURCE_MASK) |
            DMAMUX_CHCFG_SOURCE(source)); // Configure the DMA request for the DMAMUX channel.
    DMAMUX0->CHCFG[channel] |= DMAMUX_CHCFG_ENBL_MASK; // Enables the DMAMUX channel.
}
void DMA_StartChannel(uint32_t channel)
{
    assert(channel < DMA_CHANNEL_COUNT);
    DMA0->DMA[channel].DCR |= DMA_DCR_ERQ_MASK;
}

void DMA_SetSource(uint32_t channel, uint32_t sourceAddr, uint32_t transferCount)
{
    assert(channel < DMA_CHANNEL_COUNT);
    DMA0->DMA[channel].SAR = sourceAddr;                            // set source address
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(transferCount);    // set transfer count
}
void DMA_SetDestination(uint32_t channel, uint32_t destAddr, uint32_t transferCount)
{
    assert(channel < DMA_CHANNEL_COUNT);
    DMA0->DMA[channel].DAR = destAddr;                              // set destination address
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(transferCount);    // set transfer count
}
void DMA_StopChannel(uint32_t channel)
{
    assert(channel < DMA_CHANNEL_COUNT);
    DMA0->DMA[channel].DCR &= ~DMA_DCR_ERQ_MASK;
}
void DMA_ClearStatus(uint32_t channel)
{
    assert(channel < DMA_CHANNEL_COUNT);
    DMA0->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true);
}
uint32_t DMA_GetUnfinishedBytes(uint32_t channel)
{
    assert(channel < DMA_CHANNEL_COUNT);
    return (DMA0->DMA[channel].DSR_BCR & DMA_DSR_BCR_BCR_MASK) >> DMA_DSR_BCR_BCR_SHIFT;
}

void DMA_ConfigTransfer(uint32_t channel, uint32_t size, dma_transfer_type_t type,
        uint32_t sourceAddr, uint32_t destAddr, uint32_t length)
{
    assert(channel < DMA_CHANNEL_COUNT);

    uint8_t transfersize;
    uint8_t sinc, dinc;
    switch (size)
    {
        case 1:
            transfersize = 1; // 8-bit
            break;
        case 2:
            transfersize = 2; // 16-bit
            break;
        case 4:
            transfersize = 0; // 32-bit
            break;
        default:
            transfersize = 1; // 8-bit
    }

    switch (type)
    {
      case DMA_MEMORY_TO_PERIPHERAL:
          sinc = 1;
          dinc = 0;
          break;
      case DMA_PERIPHERAL_TO_MEMORY:
          sinc = 0;
          dinc = 1;
          break;
      case DMA_MEMORY_TO_MEMORY:
          sinc = 1;
          dinc = 1;
          break;
      case DMA_PERIPHERAL_TO_PERIPHERAL:
      default:
          sinc = 0;
          dinc = 0;
          break;
    }


    /* Clear the DMA status. */
    DMA0->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true);

    /* Common configuration. */

    /* Set source address */
    DMA0->DMA[channel].SAR = sourceAddr;
    /* Set destination address */
    DMA0->DMA[channel].DAR = destAddr;
    /* Set transfer bytes */
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(length);
    /* Set DMA Control Register */
    DMA0->DMA[channel].DCR = DMA_DCR_AA(0) |
                             DMA_DCR_CS(1) |
                             DMA_DCR_EADREQ(0) |
                             DMA_DCR_D_REQ(1) |
                             DMA_DCR_LINKCC(0) |
                             DMA_DCR_EINT(1) |
                             DMA_DCR_SMOD(0) |
                             DMA_DCR_DMOD(0) |
                             DMA_DCR_SSIZE(transfersize) |
                             DMA_DCR_DSIZE(transfersize) |
                             DMA_DCR_SINC(sinc) |
                             DMA_DCR_DINC(dinc);

}

static inline void DMA_IRQhandler(uint32_t channel)
{
    const uint32_t dcr_bcr = DMA0->DMA[channel].DSR_BCR; // get status
    DMA_ClearStatus(channel);

    if (myCallbacks[channel])
    {
        status_t status;

        if (dcr_bcr & DMA_DSR_BCR_CE_MASK)
            status = ERROR_DMA_CONFIG_ERR;
        else if(dcr_bcr & DMA_DSR_BCR_BED_MASK)
            status = ERROR_DMA_DEST_BUS_ERR;
        else if (dcr_bcr & DMA_DSR_BCR_BES_MASK)
            status = ERROR_DMA_SRC_BUS_ERR;
        else
            status = STATUS_OK;

        myCallbacks[channel](status, myCallbackParams[channel]);
    }
}

/* DMA IRQ handler with the same name in startup code*/
void DMA0_IRQHandler(void) { DMA_IRQhandler(0); }
void DMA1_IRQHandler(void) { DMA_IRQhandler(1); }
void DMA2_IRQHandler(void) { DMA_IRQhandler(2); }
void DMA3_IRQHandler(void) { DMA_IRQhandler(3); }
