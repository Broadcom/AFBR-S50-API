/*************************************************************************//**
 * @file
 * @brief       This file is part of the STM32F401RE platform layer.
 * @details     This file provides UART driver functionality.
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

#include "uart.h"
#include "irq.h"
#include "dma.h"
#include "usart.h"

#include "printf/printf.h"
#include <assert.h>
#include <string.h>
#include <stdarg.h>

/*! The busy indication for the uart */
static volatile bool isTxBusy_ = false;

/*! The buffer for the uart print */
static uint8_t buffer_[1024];

/*! The callback for the uart */
static uart_tx_callback_t txCallback_ = 0;

/*! The callback state for the uart */
static void * txCallbackState_ = 0;

/*! The callback for the uart */
static uart_rx_callback_t rxCallback_ = 0;

/*! The callback for the uart */
static uart_error_callback_t errorCallback_ = 0;

/*! The RX data buffer size. */
#define RX_BUFFER_SIZE 128

/*! The RX data buffer UART. */
static uint8_t rxBuffer1[RX_BUFFER_SIZE];
static uint8_t rxBuffer2[RX_BUFFER_SIZE];


/*!***************************************************************************
 * @brief   Initialize the Universal Asynchronous Receiver/Transmitter
 *          (UART or LPSCI) bus and DMA module
 * @param   -
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t UART_Init(void)
{
    MX_DMA_Init();
    MX_USART2_UART_Init();

    return STATUS_OK;
}

#if !NO_DIRECT_UART_PRINT
#define UART_Print print
#endif

uart_baud_rates_t UART_GetBaudRate(void)
{
    return huart2.Init.BaudRate;
}
status_t UART_CheckBaudRate(uart_baud_rates_t baudRate)
{
    if (!IS_UART_BAUDRATE(baudRate))
        return ERROR_UART_BAUDRATE_NOT_SUPPORTED;

    switch (baudRate)
    {
        case UART_115200_BPS:
        case UART_500000_BPS:
        case UART_1000000_BPS:
        case UART_2000000_BPS:
            return STATUS_OK;

        case UART_INVALID_BPS:
        default:
            return ERROR_UART_BAUDRATE_NOT_SUPPORTED;
    }
}
status_t UART_SetBaudRate(uart_baud_rates_t baudRate)
{
    status_t status = UART_CheckBaudRate(baudRate);
    if (status != STATUS_OK) return status;

    /* Check module state; TX line must be idle to rest baud rate... */
    IRQ_LOCK();
    if (isTxBusy_)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }
    isTxBusy_ = true;
    IRQ_UNLOCK();

    /* remove callback and disable RX line. */
    uart_rx_callback_t callback = rxCallback_;
    UART_SetRxCallback(0);

    /* Obtain correct baud rate setting value. */
    uint32_t pclk = 0;
    if ((huart2.Instance == USART1) || (huart2.Instance == USART6))
    {
        pclk = HAL_RCC_GetPCLK2Freq();
    }
    else
    {
        pclk = HAL_RCC_GetPCLK1Freq();
    }

    huart2.Instance->CR1 &= ~(USART_CR1_UE);
    if (huart2.Init.OverSampling == UART_OVERSAMPLING_8)
    {
        huart2.Instance->BRR = UART_BRR_SAMPLING8(pclk, baudRate);
    }
    else
    {
        huart2.Instance->BRR = UART_BRR_SAMPLING16(pclk, baudRate);
    }
    huart2.Init.BaudRate = baudRate;
    huart2.Instance->CR1 |= USART_CR1_UE;


    /* Add callback and enable RX line again. */
    UART_SetRxCallback(callback);

    isTxBusy_ = false;

    return STATUS_OK;
}

bool UART_IsTxBusy(void)
{
    return isTxBusy_;
}

/*!***************************************************************************
 * @brief   printf-like function to send print messages via UART.
 *
 * @details Defined in "driver/uart.c" source file.
 *
 *          Open an UART connection with 8N1, no handshake to
 *          receive the data on a computer.
 *
 *          The baud rate is specified in the project configuration via
 *          UART_BAUDRATE define. Usually its either 115200 or 2000000 bps.
 *
 * @param   fmt_s The usual printf parameters.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t UART_Print(const char *fmt_s, ...)
{
    while (UART_IsTxBusy()) __asm("nop");

    va_list ap;
    va_start(ap, fmt_s);
    int len = vsnprintf_((char *) buffer_, sizeof(buffer_), fmt_s, ap);
    va_end(ap);

    if (len < 0) return ERROR_FAIL;

    status_t status = STATUS_BUSY;
    do
    {
        status = UART_SendBuffer(buffer_, len, 0, 0);
    } while (status == STATUS_BUSY);

    return status;
}

#if !NO_DIRECT_UART_PRINT
#undef UART_Print
#endif

/*!***************************************************************************
 * @brief   Writes several bytes to the UART connection.
 * @param   txBuff Data array to write to the uart connection
 * @param   txSize The size of the data array
 * @param   f Callback function after tx is done, set 0 if not needed;
 * @param   state Optional user state that will be passed to callback
 *                  function; set 0 if not needed.
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK (0) on success.
 *           - #STATUS_BUSY on Tx line busy
 *           - #ERROR_NOT_INITIALIZED
 *           - #ERROR_INVALID_ARGUMENT
 *****************************************************************************/
status_t UART_SendBuffer(uint8_t const *txBuff, size_t txSize, uart_tx_callback_t f, void *state)
{
    /* Verify arguments. */
    if (!txBuff || txSize == 0)
        return ERROR_INVALID_ARGUMENT;

    /* Lock interrupts to prevent completion interrupt before setup is complete */
    IRQ_LOCK();
    if (isTxBusy_)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }

    /* Set Tx Busy Status. */
    isTxBusy_ = true;
    txCallback_ = f;
    txCallbackState_ = state;

    HAL_StatusTypeDef hal_error = HAL_UART_Transmit_DMA(&huart2, (uint8_t*) txBuff, txSize);
    IRQ_UNLOCK(); // this must come after HAL_UART_Transmit_DMA to avoid race conditions w/ IRQs

    if (hal_error != HAL_OK)
    {
        //return ERROR_FAIL;
        return -1000 - hal_error;
    }

    return STATUS_OK;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    void *state = txCallbackState_;
    const uart_tx_callback_t callback = txCallback_;
    const status_t status = huart->gState == HAL_UART_STATE_ERROR ? ERROR_FAIL : STATUS_OK;

    isTxBusy_ = false;

    if (callback) callback(status, state);
}

void UART_SetRxCallback(uart_rx_callback_t f)
{
    rxCallback_ = f;

    /* Start receiving */
    if (f)
    {
        HAL_UART_Receive_DMA(&huart2, rxBuffer1, RX_BUFFER_SIZE); // Start receiving via DMA
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  // Enable serial port idle interrupt
    }
    else
    {
        HAL_UART_AbortReceive(&huart2);
        HAL_UART_RxCpltCallback(&huart2);
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);  // Disable serial port idle interrupt
    }
}

void UART_SetErrorCallback(uart_error_callback_t f)
{
    errorCallback_ = f;
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        // On idle interruption
        __HAL_UART_CLEAR_IDLEFLAG(huart); // Clear idle interrupt sign
        HAL_UART_AbortReceive(huart);
        HAL_UART_RxCpltCallback(huart);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    assert(huart->pRxBuffPtr == rxBuffer1 || huart->pRxBuffPtr == rxBuffer2);

    uint32_t size = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint8_t * curr = huart->pRxBuffPtr;
    uint8_t * next = huart->pRxBuffPtr == rxBuffer1 ? rxBuffer2 : rxBuffer1;

    HAL_StatusTypeDef rtn = HAL_UART_Receive_DMA(huart, next, RX_BUFFER_SIZE);
    if (rtn != HAL_OK || huart->gState == HAL_UART_STATE_ERROR)
    {
        HAL_UART_ErrorCallback(huart);
    }

    if (rxCallback_)
        rxCallback_(curr, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    status_t status = ERROR_FAIL;

    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
        status = ERROR_UART_RX_OVERRUN;
    else if (huart->ErrorCode & HAL_UART_ERROR_FE)
        status = ERROR_UART_FRAMING_ERR;
    else if (huart->ErrorCode & HAL_UART_ERROR_NE)
        status = ERROR_UART_RX_NOISE;
    else if (huart->ErrorCode & HAL_UART_ERROR_DMA)
        status = ERROR_UART_TX_DMA_ERR;
    else if (huart->gState == HAL_UART_STATE_BUSY)
        status = STATUS_BUSY;
    else if (huart->gState == HAL_UART_STATE_BUSY_RX)
        status = STATUS_BUSY;
    else if (huart->gState == HAL_UART_STATE_BUSY_TX)
        status = STATUS_BUSY;
    else if (huart->gState == HAL_UART_STATE_BUSY_TX_RX)
        status = STATUS_BUSY;
    else if (huart->gState == HAL_UART_STATE_ERROR)
        status = ERROR_FAIL;
    else if (huart->gState == HAL_UART_STATE_TIMEOUT)
        status = ERROR_TIMEOUT;

    if (errorCallback_)
        errorCallback_(status);
}
