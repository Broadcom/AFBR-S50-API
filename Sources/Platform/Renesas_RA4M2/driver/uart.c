/*************************************************************************//**
 * @file
 * @brief       This file is part of the RA4M2 platform layer.
 * @details     This file provides UART driver functionality.
 *
 * @copyright
 *
 * Copyright (c) 2022, Broadcom Inc
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


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "driver/uart.h"
#include "driver/irq.h"
#include "utility/time.h"
#include "board/board_config.h"

#include "hal_data.h"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
status_t print(const char  *fmt_s, ...);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! Flag to track wether the UART module has been initialized. */
static volatile bool isInitialized = false;

static uart_baud_rates_t setBaudrate = UART_INVALID_BPS;

/*! Data buffer for sending data. */
static uint8_t myBuffer[1024];

/*! Flag to determine if TX is ongoing. */
static volatile bool isTxOnGoing = false;

/*! User callback that is invoked when a transfer is finished. */
static uart_tx_callback_t myTxCallback = 0;

/*! Callback parameter/state passed to the myTxCallback callback when invoked. */
static void * myTxCallbackState = 0;

/*! User callback that is invoked for each incoming byte. */
static uart_rx_callback_t myRxCallback = 0;

/*! User callback that is invoked when a transmission error occurs. */
static uart_error_callback_t myErrorCallback = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t UART_Init(void)
{
    if (isInitialized) return STATUS_OK;

    /* Ensure the default baudrate is valid */
    status_t status = UART_CheckBaudRate(UART_BAUDRATE);
    if (status != STATUS_OK) return status;

    /* Open the UART peripheral */
    fsp_err_t retVal = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    if (retVal != FSP_SUCCESS) return ERROR_FAIL;

    status = UART_SetBaudRate(UART_BAUDRATE);
    if (status != STATUS_OK) return status;

    isInitialized = true;
    setBaudrate = UART_BAUDRATE;

    return STATUS_OK;
}

uart_baud_rates_t UART_GetBaudRate(void)
{
    return setBaudrate;
}

status_t UART_CheckBaudRate(uart_baud_rates_t baudRate)
{
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

    /* Check that we're not busy.*/
    IRQ_LOCK();
    if (isTxOnGoing)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }
    isTxOnGoing = true;
    IRQ_UNLOCK();

    baud_setting_t baudSetting;
    fsp_err_t retVal = R_SCI_UART_BaudCalculate(baudRate, false, 5000u, &baudSetting);
    if (retVal != FSP_SUCCESS)
    {
        isTxOnGoing = false;
        return ERROR_UART_BAUDRATE_NOT_SUPPORTED;
    }

    retVal = R_SCI_UART_BaudSet(&g_uart0_ctrl, &baudSetting);
    if (retVal != FSP_SUCCESS)
    {
        isTxOnGoing = false;
        return ERROR_FAIL;
    }

    setBaudrate = baudRate;
    isTxOnGoing = false;

    return STATUS_OK;
}

static status_t UART_AwaitIdle()
{
    const uint32_t timeout_ms = 500;
    ltc_t start;
    Time_GetNow(&start);

    /* Wait until no transfer is ongoing and claim the control. */
    for (;;)
    {
        while (isTxOnGoing)
        {
            if (Time_CheckTimeoutMSec(&start, timeout_ms))
            {
                return ERROR_TIMEOUT;
            }
        }

        /* make sure that no IRQ has happened meanwhile... */
        IRQ_LOCK();
        if (!isTxOnGoing)
        {
            isTxOnGoing = true;
            IRQ_UNLOCK();
            break;
        }
        IRQ_UNLOCK();
    }

    return STATUS_OK;
}

__attribute__((weak))  status_t print(const char * fmt_s, ...)
{
    /* The UART mutex logic is needed here in order to protect the printf buffer as well,
     * otherwise an overlapping call would corrupt it. This function also sets isTxOnGoing
     * when the UART becomes Idle. */
    status_t status = UART_AwaitIdle(true);
    if (status != STATUS_OK) return status;

    va_list ap;
    va_start(ap, fmt_s);
    int len = vsnprintf((char*)myBuffer, sizeof(myBuffer), fmt_s, ap);
    va_end(ap);

    if (len < 0)
    {
        return ERROR_FAIL;
    }

    if (R_SCI_UART_Write(&g_uart0_ctrl, myBuffer, (uint32_t)len) != FSP_SUCCESS)
    {
        isTxOnGoing = false;
        return ERROR_FAIL;
    }

    return STATUS_OK;
}

status_t UART_SendBuffer(uint8_t const * txBuff, size_t txSize, uart_tx_callback_t f, void * state)
{
    assert(isInitialized);
    if (!isInitialized) return ERROR_NOT_INITIALIZED;

    /* Verify arguments. */
    if (!txBuff || !txSize) return ERROR_INVALID_ARGUMENT;

    status_t status = UART_AwaitIdle();
    if (status != STATUS_OK) return status;

    myTxCallback = f;
    myTxCallbackState = state;

    if (FSP_SUCCESS != R_SCI_UART_Write(&g_uart0_ctrl, txBuff, txSize))
    {
        isTxOnGoing = false;
        return ERROR_FAIL;
    }

    return STATUS_OK;
}

bool UART_IsTxBusy(void)
{
    return isTxOnGoing;
}

void UART_SetRxCallback(uart_rx_callback_t f)
{
    assert(isInitialized);
    IRQ_LOCK();
    myRxCallback = f;
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
    IRQ_LOCK();
    myErrorCallback = f;
    IRQ_UNLOCK();
}

void UART_RemoveErrorCallback(void)
{
    assert(isInitialized);
    UART_SetErrorCallback(0);
}

void user_uart_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            if (myRxCallback)
            {
                const uint8_t data = (uint8_t)p_args->data;
                myRxCallback(&data, 1);
            }
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            isTxOnGoing = false;

            if (myTxCallback)
            {
                myTxCallback(STATUS_OK, myTxCallbackState);
            }
            break;
        }

        case UART_EVENT_TX_DATA_EMPTY:
        case UART_EVENT_RX_COMPLETE:
        case UART_EVENT_BREAK_DETECT:
            break;

        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_ERR_PARITY:
        {
            bool wasTxTransfer = false;
            if (isTxOnGoing)
            {
                wasTxTransfer = true;
                isTxOnGoing = false;
            }

            if (myErrorCallback)
            {
                myErrorCallback(ERROR_FAIL);
            }

            if (wasTxTransfer && myTxCallback)
            {
                myTxCallback(ERROR_FAIL, myTxCallbackState);
            }
            break;
        }
    }
}
