
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


#include "uart_api.h"

#include "driver/uart.h"
#include "driver/irq.h"
#include "main.h"
#include "hal_data.h"

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>

/*! Indicates received UART data. */
static volatile uint8_t myUartRxData;

/*! Callback invoked from the UART interface for incoming data. */
static void uart_rx_callback(uint8_t const * data, uint32_t const size);

/*! UART Data characters definition*/
typedef enum argus_uart_t
{
    /*! Remote Frame ID for starting measurements */
    UART_START = 's',

    /*! Remote Frame ID for stopping measurements */
    UART_STOP = 'p',

} argus_uart_t;


void UART_API_Init(void)
{
    status_t status = UART_Init();
    if (status != STATUS_OK)
    {
        print("UART Init API failed with error code: %d\n", status);
        handle_error(status, "UART Init failed.");
    }

    UART_SetRxCallback(uart_rx_callback);
}

void UART_Send1D(argus_results_t const * res)
{
    print("%4d.%06d s;  Range: %5d mm;  Amplitude: %4d LSB;  Quality: %3d;  Status: %d\n",
          res->TimeStamp.sec,
          res->TimeStamp.usec,
          res->Bin.Range / (Q9_22_ONE / 1000),
          res->Bin.Amplitude / UQ12_4_ONE,
          res->Bin.SignalQuality,
          res->Status);
}

void UART_HandleCommand(void)
{
    IRQ_LOCK();
    uint8_t rxData = myUartRxData;
    /* Rx command handled: clear flags.. */
    myUartRxData = 0;
    IRQ_UNLOCK();

    switch (rxData)
    {
        case UART_START:
            start_measurements();
            break;

        case UART_STOP:
            stop_measurements();
            break;

        default:
            /* Nothing to do */
            break;
    }
}

static void uart_rx_callback(uint8_t const * data, uint32_t const size)
{
    (void)size;
    assert(size == 1);
    if (myUartRxData != 0)
    {
        // TODO: implemented message queue
        print("UART OVERRUN ERROR: Received new UART data byte but the "
              "previous byte was not yet handled.\n");
    }
    myUartRxData = *data;
}

