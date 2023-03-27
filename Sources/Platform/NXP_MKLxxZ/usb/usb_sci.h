/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _USB_SCI_H_
#define _USB_SCI_H_


/*!***************************************************************************
 * Include files
 *****************************************************************************/
#include "usb/include/usb.h"
//#include "sci/sci_frame.h"
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Return status for the USB driver.
 *  @ingroup status */
enum StatusUSB
{
    /*! USB Error. */
    ERROR_USB = -30,

    /*! USB Busy Error. */
    ERROR_USB_BUSY = -31,

    /*! USB Timeout Error. */
    ERROR_USB_TIMEOUT = -32

};

/*!***************************************************************************
 * @brief   SCI physical layer received byte callback function type.
 * @details Callback that is invoked whenever data has been received via the
 *          physical layer.
 * @param   data The received data as byte (uint8_t) array.
 * @param   size The size of the received data.
 * @return  -
 *****************************************************************************/
typedef void (*usb_rx_callback_t)(uint8_t const * data, uint32_t const size);

/*!***************************************************************************
 * @brief   SCI physical layer transmit done callback function type.
 * @details Callback that is invoked whenever the physical layer has finished
 *          transmitting the current data buffer.
 * @param   status The \link #status_t status\endlink of the transmitter;
 *                   (#STATUS_OK on success).
 * @param   state A pointer to the state that was passed to the Tx function.
 * @return  -
 *****************************************************************************/
typedef void (*usb_tx_callback_t)(status_t status, void *state);

/*!***************************************************************************
 * @brief   SCI error callback function type.
 * @detail  Callback that is invoked whenever a error occurs.
 * @param   status The error \link #status_t status\endlink that invoked the
 *                 callback.
 * @return  -
 *****************************************************************************/
typedef void (*usb_error_callback_t)(status_t status);

/*******************************************************************************
 * API
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Initialize the USB Receiver/Transmitter
 * @details This API is used to initialize the class.
 * @param   -
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t USB_DeviceApplicationInit(void);

/*!***************************************************************************
 * @brief   Writes several bytes to the USB connection.
 * @details This API is used by the application to send data to the host system.
 * @param   txBuff Data array to write to the USB connection
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
status_t USB_SendBuffer(uint8_t * txBuff,
                        size_t txSize,
                        usb_tx_callback_t f,
                        void * state);


/*!***************************************************************************
 * @brief   Reads the transmittion status of the USB interface
 * @return  Booleon value:
 *           - true: device is busy
 *           - false: device is idle
 *****************************************************************************/
bool USB_IsTxBusy(void);

/*!***************************************************************************
 * @brief   Checks if USB Sending timeout is elapsed and cancels the request in
 *          case of elapsed timeout.
 *
 * @return  True if the transfer has been canceled due to timeout.
 *****************************************************************************/
bool USB_CancelIfTimeOutElapsed(void);

/*!***************************************************************************
 * @brief   Installs an callback function for the byte received event.
 * @param   f The callback function pointer.
 *****************************************************************************/
 void USB_SetRxCallback(usb_rx_callback_t f);

/*!***************************************************************************
 * @brief   Installs an callback function for the error occurred event.
 * @param   f The callback function pointer.
 *****************************************************************************/
void USB_SetErrorCallback(usb_error_callback_t f);

#endif /* _USB_SCI_H_ */
