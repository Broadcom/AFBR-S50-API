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
 *****************************************************************************/

#ifndef DMA_H
#define DMA_H

/*!***************************************************************************
 * @defgroup    DMA DMA: Direct Memory Access
 * @ingroup     driver
 * @brief       DMA Hardware Module
 * @details     Direct memory access driver layer.
 * @addtogroup  DMA
 * @{
 *****************************************************************************/

#include "utility/status.h"


/*! @brief Type for DMA transfer. */
typedef enum dma_transfer_type_t
{
    /*! Transfer from the peripheral to memory */
    DMA_PERIPHERAL_TO_MEMORY,

    /*! Transfer from the memory to peripheral */
    DMA_MEMORY_TO_PERIPHERAL,

    /*! Transfer from the memory to memory */
    DMA_MEMORY_TO_MEMORY,

    /*! Transfer from the peripheral to peripheral */
    DMA_PERIPHERAL_TO_PERIPHERAL

} dma_transfer_type_t;


/*! @brief Return status for the DMA driver.
 *  @ingroup status */
enum StatusDMA
{
    /*! DMA configuration error. */
    ERROR_DMA_CONFIG_ERR    = -61,

    /*! DMA source bus error. */
    ERROR_DMA_SRC_BUS_ERR   = -62,

    /*! DMA destination bus error. */
    ERROR_DMA_DEST_BUS_ERR  = -63,
};


/*! @brief Argus DMA layer callback function type. */
typedef void (* dma_callback_t)(status_t status, void * param);

/*!***************************************************************************
 * @brief   Initialize the DMA module to a defined state.
 *****************************************************************************/
void DMA_Init(void);

/*!***************************************************************************
 * @brief   Installs a callback function for DMA transfer done interrupts.
 * @param   channel DMA channel (0, ..., 3)
 * @param   f The interrupt service routine to be called.
 * @param   param An abstract parameter to be passed to the callback.
 *****************************************************************************/
void DMA_SetTransferDoneCallback(uint32_t channel, dma_callback_t f, void * param);

/*!***************************************************************************
 * @brief   Removes the previously installed callback function for DMA transfer done interrupts.
 * @param   channel DMA channel (0, ..., 3)
 *****************************************************************************/
void DMA_RemoveTransferDoneCallback(uint32_t channel);

/*!***************************************************************************
 * @brief   Claims a DMA channel for a given source.
 * @details Check hardware documentation for exact channel and source numbers.
 * @param   channel DMA channel (0, ..., 3)
 * @param   source Source number (see hardware documentations).
 *****************************************************************************/
void DMA_ClaimChannel(uint32_t channel, uint8_t source);

/*!***************************************************************************
 * @brief   Start DMA transfer for a given channel.
 * @param   channel DMA channel (0, ..., 3)
 *****************************************************************************/
void DMA_StartChannel(uint32_t channel);

/*!***************************************************************************
 * @brief   Stops an ongoing DMA transfer for a given channel.
 * @param   channel DMA channel (0, ..., 3)
 *****************************************************************************/
void DMA_StopChannel(uint32_t channel);

/*!***************************************************************************
 * @brief   Set the source address and length for a DMA transfer.
 * @param   channel DMA channel (0, ..., 3)
 * @param   sourceAddr The initial source address.
 * @param   transferCount The number of DMA transfers.
 *****************************************************************************/
void DMA_SetSource(uint32_t channel, uint32_t sourceAddr, uint32_t transferCount);

/*!***************************************************************************
 * @brief   Set the destination address and length for a DMA transfer.
 * @param   channel DMA channel (0, ..., 3)
 * @param   destAddr The initial destination address.
 * @param   transferCount The number of DMA transfers.
 *****************************************************************************/
void DMA_SetDestination(uint32_t channel, uint32_t destAddr, uint32_t transferCount);

/*!***************************************************************************
 * @brief   Clear the current status flags for a given DMA channel.
 * @param   channel DMA channel (0, ..., 3)
 *****************************************************************************/
void DMA_ClearStatus(uint32_t channel);

/*!***************************************************************************
 * @brief   Returns the number of unfinished DMA transfers for a given channel.
 * @param   channel DMA channel (0, ..., 3)
 * @return  The unfinished byte count.
 *****************************************************************************/
uint32_t DMA_GetUnfinishedBytes(uint32_t channel);

/*!***************************************************************************
 * @brief   Configures a DMA channel transfer.
 * @param   channel DMA channel (0, ..., 3)
 * @param   size The data size in bytes: 1 = 8-bit; 2 = 16-bit; 4 = 32-bit;
 * @param   type The DMA transfer type.
 * @param   sourceAddr The initial source address.
 * @param   destAddr The initial destination address.
 * @param   length The number of DMA transfers.
 *****************************************************************************/
void DMA_ConfigTransfer(uint32_t channel, uint32_t size, dma_transfer_type_t type,
        uint32_t sourceAddr, uint32_t destAddr, uint32_t length);

/*! @} */
#endif /* DMA_H */
