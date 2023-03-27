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

#ifndef UART_H
#define UART_H

/*!***************************************************************************
 * @defgroup    UART UART:Universal Asynchronous Receiver/Transmitter
 * @ingroup     driver
 * @brief       UART Hardware Layer Module
 * @details     Provides a layer for the UART driver functions.
 *              It uses UART interface with DMA
 *
 *              Example:
 * @code
 *                  UART_Print("Hello");
 * @endcode
 * @code
 *                  UART_SendBuffer(txBuff, sizeof(txBuff), 0, 0);
 *                  while (UART_IsTxBusy());
 * @endcode
 * @addtogroup  UART
 * @{
 *****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "utility/status.h"


/*! @brief Return status for the UART driver.
 *  @ingroup status */
enum StatusUART
{
    /*! Baud rate not supported by system. */
    ERROR_UART_BAUDRATE_NOT_SUPPORTED = -71,

    /*! Receiver buffer hasen't been read before receiving new data.
     *  Data loss! */
    ERROR_UART_RX_OVERRUN = -72,

    /*! Noise detected in the received character. */
    ERROR_UART_RX_NOISE = -73,

    /*! Framing error occurs when the receiver detects a logic 0 where a stop
     *  bit was expected. This suggests the receiver was not properly aligned
     *  to a character frame. */
    ERROR_UART_FRAMING_ERR = -74,

    /*! Transmitting error stemming from the DMA module. */
    ERROR_UART_TX_DMA_ERR = -75,

    /*! Receiving error stemming from the DMA module. */
    ERROR_UART_RX_DMA_ERR = -75,
};

typedef enum uart_baud_rates_t
{
    UART_INVALID_BPS = 0,
    UART_115200_BPS = 115200,
    UART_500000_BPS = 500000,
    UART_1000000_BPS = 1000000,
    UART_2000000_BPS = 2000000,
} uart_baud_rates_t;

/*!***************************************************************************
 * @brief   SCI physical layer received byte callback function type.
 * @details Callback that is invoked whenever data has been received via the
 *          physical layer.
 * @param   data The received data as byte (uint8_t) array.
 * @param   size The size of the received data.
 * @return  -
 *****************************************************************************/
typedef void (*uart_rx_callback_t)(uint8_t const * data, uint32_t const size);

/*!***************************************************************************
 * @brief   SCI physical layer transmit done callback function type.
 * @details Callback that is invoked whenever the physical layer has finished
 *          transmitting the current data buffer.
 * @param   status The \link #status_t status\endlink of the transmitter;
 *                   (#STATUS_OK on success).
 * @param   state A pointer to the state that was passed to the Tx function.
 * @return  -
 *****************************************************************************/
typedef void (*uart_tx_callback_t)(status_t status, void *state);

/*!***************************************************************************
 * @brief   SCI error callback function type.
 * @detail  Callback that is invoked whenever a error occurs.
 * @param   status The error \link #status_t status\endlink that invoked the
 *                 callback.
 * @return  -
 *****************************************************************************/
typedef void (*uart_error_callback_t)(status_t status);

/*!***************************************************************************
 * @brief   Initialize the Universal Asynchronous Receiver/Transmitter
 *          (UART or LPSCI) bus and DMA module
 * @param   -
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t UART_Init(void);

status_t UART_CheckBaudRate(uart_baud_rates_t baudRate);
status_t UART_SetBaudRate(uart_baud_rates_t baudRate);
uart_baud_rates_t UART_GetBaudRate(void);

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
status_t UART_SendBuffer(uint8_t const * txBuff, size_t txSize, uart_tx_callback_t f, void * state);

/*!***************************************************************************
 * @brief   Reads the transmittion status of the uart interface
 * @param   -
 * @return  Booleon value:
 *           - true: device is busy
 *           - false: device is idle
 *****************************************************************************/
bool UART_IsTxBusy(void);

/*!***************************************************************************
 * @brief   Installs an callback function for the byte received event.
 * @param   f The callback function pointer.
 *****************************************************************************/
void UART_SetRxCallback(uart_rx_callback_t f);

/*!***************************************************************************
 * @brief   Removes the callback function for the byte received event.
 *****************************************************************************/
void UART_RemoveRxCallback(void);

/*!***************************************************************************
 * @brief   Installs an callback function for the error occurred event.
 * @param   f The callback function pointer.
 *****************************************************************************/
void UART_SetErrorCallback(uart_error_callback_t f);

/*!***************************************************************************
 * @brief   Removes the callback function for the error occurred event.
 *****************************************************************************/
void UART_RemoveErrorCallback(void);

/*! @} */
#endif /* UART_H */
