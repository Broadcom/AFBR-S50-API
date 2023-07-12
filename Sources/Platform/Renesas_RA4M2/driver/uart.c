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

/*! Data buffer for sending data. */
static uint8_t myBuffer[1024];

/*! Flag to determine if TX is ongoing. */
static volatile bool isTxOnGoing = false;

/*! User callback that is invoked for each incoming byte. */
static uart_rx_callback_t myRxCallback = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t UART_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    assert(err == FSP_SUCCESS);
    return (err == FSP_SUCCESS) ? STATUS_OK : ERROR_FAIL;
}

static status_t UART_AwaitIdle(void)
{
    const uint32_t timeout_ms = 500;
    ltc_t start;
    Time_GetNow(&start);

    /* Wait until no transfer is ongoing and claim the control. */
    for(;;)
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

static status_t UART_Write(uint8_t const * buffer, uint32_t length)
{
    assert(buffer != 0);
    assert(length > 0);

    if (FSP_SUCCESS != R_SCI_UART_Write(&g_uart0_ctrl, buffer, length))
    {
        isTxOnGoing = false;
        return ERROR_FAIL;
    }

    return STATUS_OK;
}

__attribute__((weak)) status_t print(const char * fmt_s, ...)
{
    status_t status = UART_AwaitIdle();
    if (status != STATUS_OK) return status;

    va_list ap;
    va_start(ap, fmt_s);
    int len = vsnprintf((char*)myBuffer, sizeof(myBuffer), fmt_s, ap);
    va_end(ap);

    if (len < 0)
    {
        isTxOnGoing = false;
        return ERROR_FAIL;
    }

    return UART_Write(myBuffer, (uint32_t)len);
}

bool UART_IsTxBusy(void)
{
    return isTxOnGoing;
}

void UART_SetRxCallback(uart_rx_callback_t f)
{
    IRQ_LOCK();
    myRxCallback = f;
    IRQ_UNLOCK();
}

void UART_RemoveRxCallback(void)
{
    UART_SetRxCallback(0);
}

void user_uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            if (myRxCallback)
            {
                const uint8_t data = (uint8_t)p_args->data;
                myRxCallback(&data, 1);
            }
            break;

        case UART_EVENT_TX_COMPLETE:
            isTxOnGoing = false;
            break;

        case UART_EVENT_TX_DATA_EMPTY:
        case UART_EVENT_RX_COMPLETE:
        case UART_EVENT_BREAK_DETECT:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_ERR_PARITY:
            break;

        default:
            assert(0);
    }
}
