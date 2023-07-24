/*************************************************************************//**

 * @brief       USB-SCI Layer: HAL layer abstracting USB for the SCI usage.
 * @details     This file provides an abstraction layer on top of the USB driver
 *              to be used by the SCI module.
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

/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "usb_sci.h"
#include "debug.h"
#include "hal_data.h"
#include "driver/irq.h"
#include "utility/time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! General timeout threshold used for USB operations. */
#define USB_TX_TIMEOUT (100U) // ms

/*! USB maximum packet size (both TX and RX). */
#define USB_PK_SIZE             (64u)

/*! Number of buffers used for USB RX operations.
 * Note: the number is quite high in order to ensure smooth operations at high speed */
#define USB_RX_BUFFER_COUNT     (10u)


/* USB driver specific parameters. */
#define USB_REQ_SIZE            (20u)
#define USB_SET_VENDOR_NO_DATA  (0x0000U)
#define USB_SET_VENDOR          (0x0100U)
#define USB_GET_VENDOR          (0x0200U)

#define USB_BREAK_ON_ERR(err)   if (err != FSP_SUCCESS) { BREAKPOINT(); }

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! Cancel the currently running USB TX operation. */
static status_t USB_CancelSend(void);

/*! Poll for and process USB events. This is called by the USB Polling timer. */
static void process_usb_event(void);

/*! Handles app level handshake when a USB connection is made. */
static fsp_err_t USB_configured_event_process(void);

/*! Handles device info data when a USB connection is made. */
static fsp_err_t USB_status_request(usb_event_info_t * pEvent);

/*! Callback for USB data packet received. */
static void USB_DataReceivedCallback(uint8_t const * data, uint32_t size);

/*! Callback for USB data packet sent. */
static void USB_DataSentCallback(status_t txStatus);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! Flag for the initialization state of the USB module. */
static bool isInitialized = false;

/*! Flag for tracking whether the board is connected to a host USB. */
static bool isAttached = false;

/*! Flag for tracking if the host requested a suspension of the interface. */
static bool isSuspended = false;

static usb_error_callback_t myErrorCallback = 0;
static usb_rx_callback_t myRxCallback = 0;
static usb_tx_callback_t myTxCallback = 0;
static void * myTxCallbackState = 0;

/*! Start time for timeout handling. */
static ltc_t myTxStartTimeStamp;

/*! Flag for avoiding overlapping transmissions. */
static volatile bool isTxOnGoing = false;

/*! Pipe ID associated with the Bulk IN endpoint (to be determined when the USB is connected). */
static uint8_t bulk_in_pipe = 0u;
/*! Pipe ID associated with the Bulk OUT endpoint (to be determined when the USB is connected). */
static uint8_t bulk_out_pipe = 0u;

/*! Array of buffers used for USB RX operations (support for multi-buffering). */
static uint8_t usb_rx_buf[USB_RX_BUFFER_COUNT][USB_PK_SIZE] = { 0u };
/*! ID of the next free RX buffer */
static uint8_t rx_buf_idx = 0u;

/*! Buffer for USB control requests. */
static uint8_t usb_request_buf[USB_REQ_SIZE] = { 0u };

/*******************************************************************************
 * Functions
 ******************************************************************************/

status_t USB_DeviceApplicationInit(void)
{
    if (!isInitialized)
    {
        fsp_err_t err = g_usb_on_usb.open(&g_basic0_ctrl, &g_basic0_cfg);
        if (err != FSP_SUCCESS) return ERROR_FAIL;

        isInitialized = true;
    }

    return STATUS_OK;
}

status_t USB_SendBuffer(uint8_t * txBuff, size_t txSize, usb_tx_callback_t f, void * state)
{
    /* Check that we're not busy.*/
    if (!isInitialized) return ERROR_NOT_INITIALIZED;

    /* Prevent sending data if the USB is not connected or in suspend mode */
    if ((!isAttached) || (isSuspended))
    {
        /* This case is not considered a failure, so the transmission will simply be skipped
         * and the callback is directly called here */
        f(STATUS_OK, state);
        return STATUS_OK;
    }

    IRQ_LOCK();
    if (isTxOnGoing)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }

    /* Verify arguments. */
    if (!txBuff || !txSize)
    {
        IRQ_UNLOCK();
        return ERROR_INVALID_ARGUMENT;
    }

    /* Ensure USB is attached (the pipe must be opened) */
    if (bulk_in_pipe == 0) return ERROR_USB;

    /* Set Tx Busy Status. */
    isTxOnGoing = true;
    IRQ_UNLOCK();

    Time_GetNow(&myTxStartTimeStamp);

    myTxCallback = f;
    myTxCallbackState = state;

    fsp_err_t ret = R_USB_PipeWrite(&g_basic0_ctrl, txBuff, txSize, bulk_in_pipe);

    if (ret == FSP_ERR_USB_BUSY)
    {
        return STATUS_BUSY;
    }
    else if (ret != FSP_SUCCESS)
    {
        myTxCallback = 0;
        myTxCallbackState = 0;
        myTxStartTimeStamp.sec = 0;
        myTxStartTimeStamp.usec = 0;
        isTxOnGoing = false;
        return ERROR_USB;
    }

    return STATUS_OK;
}

bool USB_IsTxBusy(void)
{
    return isTxOnGoing;
}

static status_t USB_CancelSend(void)
{
    /* Check that we're not busy. */
    if (!isInitialized) return ERROR_NOT_INITIALIZED;
    if (!isTxOnGoing) return STATUS_IDLE;

    fsp_err_t ret = R_USB_PipeStop(&g_basic0_ctrl, bulk_in_pipe);

    return ret == FSP_SUCCESS ? STATUS_OK : ERROR_USB;
}

bool USB_CancelIfTimeOutElapsed(void)
{
    IRQ_LOCK();
    if (!isTxOnGoing)
    {
        IRQ_UNLOCK();
        return false;
    }
    if (Time_CheckTimeoutMSec(&myTxStartTimeStamp, USB_TX_TIMEOUT))
    {
        USB_CancelSend();
        IRQ_UNLOCK();
        return true;
    }
    IRQ_UNLOCK();
    return false;
}

void USB_SetRxCallback(usb_rx_callback_t f)
{
    IRQ_LOCK();
    myRxCallback = f;
    IRQ_UNLOCK();
}

void USB_SetErrorCallback(usb_error_callback_t f)
{
    IRQ_LOCK();
    myErrorCallback = f;
    IRQ_UNLOCK();
}

static void USB_DataReceivedCallback(uint8_t const * data, uint32_t size)
{
    if (myRxCallback)
        myRxCallback(data, size);
}

static void USB_DataSentCallback(status_t txStatus)
{
    usb_tx_callback_t cb = myTxCallback;
    void * state = myTxCallbackState;

    myTxCallback = 0;
    myTxCallbackState = 0;
    myTxStartTimeStamp.sec = 0;
    myTxStartTimeStamp.usec = 0;
    isTxOnGoing = false;

    if (txStatus != STATUS_OK && txStatus != ERROR_ABORTED)
    {
        if (myErrorCallback)
            myErrorCallback(ERROR_USB);
    }
    else if (cb != 0)
    {
        cb(txStatus, state);
    }
}

static fsp_err_t USB_configured_event_process(void)
{
    uint16_t used_pipe = 0u;
    usb_pipe_t pipe_info = { 0 };

    /* Get USB Pipe Information */
    fsp_err_t err = R_USB_UsedPipesGet(&g_basic0_ctrl, &used_pipe, USB_CLASS_PVND);
    if (err != FSP_SUCCESS) return err;

    for (uint8_t pipe = USB_PIPE1; pipe <= USB_PIPE9; pipe++)
    {
        /* check for the used pipe */
        if ((used_pipe & (USB_PIPE1 << pipe)) != 0)
        {
            err = R_USB_PipeInfoGet(&g_basic0_ctrl, &pipe_info, (uint8_t)pipe);

            if (pipe_info.transfer_type == USB_TRANSFER_TYPE_BULK)
            {
                if ((pipe_info.endpoint & USB_EP_DIR_IN) != USB_EP_DIR_IN)
                {
                    /* Out Transfer */
                    bulk_out_pipe = pipe;
                }
                else
                {
                    /* In Transfer */
                    bulk_in_pipe = pipe;
                }
            }
        }
    }

    return err;
}

static fsp_err_t USB_status_request(usb_event_info_t * pEvent)
{
    fsp_err_t err = FSP_SUCCESS;
    uint16_t request_length = 0u;

    uint16_t request = (pEvent->setup.request_type & USB_BREQUEST);

    if (request == USB_SET_VENDOR_NO_DATA)
    {
        /* Set ACk to host */
        err = R_USB_PeriControlStatusSet(&g_basic0_ctrl, USB_SETUP_STATUS_ACK);
    }
    else if (request == USB_SET_VENDOR)
    {
        request_length = pEvent->setup.request_length;
        /* Get data length from host */
        err = R_USB_PeriControlDataGet(&g_basic0_ctrl, usb_request_buf, request_length);
    }
    else if (request == USB_GET_VENDOR)
    {
        /* Set data length in peripheral */
        err = R_USB_PeriControlDataSet(&g_basic0_ctrl, usb_request_buf, request_length);
    }

    return err;
}

/***************************************************************
 * Callbacks                                                   *
 ***************************************************************/

static void process_usb_event(void)
{
    fsp_err_t ret = FSP_SUCCESS;
    static usb_event_info_t usb_event;
    static usb_status_t usb_event_status;

    while (1)   // the loop is not infinite, but will use a forced return
    {
        ret = R_USB_EventGet(&usb_event, &usb_event_status);
        USB_BREAK_ON_ERR(ret);

        /* Early return if there is no event to process or if there is a failure in polling */
        if ((usb_event_status == USB_STATUS_NONE) || (ret != FSP_SUCCESS)) return;

        /* USB event received by R_USB_EventGet */
        switch (usb_event_status)
        {
            case USB_STATUS_CONFIGURED:
            {
                /* Process USB configured event */
                ret = USB_configured_event_process();
                USB_BREAK_ON_ERR(ret);

                if (ret == FSP_SUCCESS)
                {
                    /* Listen for incoming data */
                    ret = R_USB_PipeRead(&g_basic0_ctrl, usb_rx_buf[rx_buf_idx], USB_PK_SIZE, bulk_out_pipe);
                    if (ret == FSP_SUCCESS)
                    {
                        isAttached = true;
                        isSuspended = false;
                    }
                }
                break;
            }

            case USB_STATUS_READ_COMPLETE:
            {
                if ((usb_event.pipe == bulk_out_pipe) && (usb_event.status != FSP_ERR_USB_FAILED))
                {
                    uint8_t * pRxBuf = usb_rx_buf[rx_buf_idx];
                    uint32_t rxSize = usb_event.data_size;

                    /* Data received, setup new buffer for the next incoming data */
                    if ((++rx_buf_idx) >= USB_RX_BUFFER_COUNT)
                        rx_buf_idx = 0u;
                    ret = R_USB_PipeRead(&g_basic0_ctrl, usb_rx_buf[rx_buf_idx], USB_PK_SIZE, bulk_out_pipe);
                    USB_BREAK_ON_ERR(ret);

                    /* call the SCI layer callback */
                    USB_DataReceivedCallback(pRxBuf, rxSize);
                }
                break;
            }

            case USB_STATUS_WRITE_COMPLETE:
            {
                if (usb_event.pipe == bulk_in_pipe)
                {
                    /* Note: No need to send ZLP - it is a bulk endpoint and it should never stop */

                    //status_t status = (usb_event.status == FSP_SUCCESS) ? STATUS_OK : ERROR_FAIL;

                    /* WORKAROUND: The reconnect during streaming will trigger a CancelSend, which
                    sometimes leads to an error here. For now, we ignore any error here to avoid
                    breaking this scenario, so we always send FSP_SUCCESS. */
                    USB_DataSentCallback(FSP_SUCCESS);
                }
                break;
            }

            case USB_STATUS_REQUEST:
            {
                /* process USB status request event */
                ret = USB_status_request(&usb_event);
                USB_BREAK_ON_ERR(ret);
                break;
            }

            case USB_STATUS_REQUEST_COMPLETE:
            {
                if ((usb_event.setup.request_type & USB_BREQUEST) == USB_GET_VENDOR)
                {
                    /* Start reading data */
                    ret = R_USB_PipeRead(&g_basic0_ctrl, usb_rx_buf[rx_buf_idx], USB_PK_SIZE, bulk_out_pipe);
                    USB_BREAK_ON_ERR(ret);
                }
                break;
            }

            case USB_STATUS_DETACH:
            {
                isAttached = false;
                break;
            }

            case USB_STATUS_SUSPEND:
            {
                isSuspended = true;
                break;
            }

            case USB_STATUS_RESUME:
            {
                isSuspended = false;
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

void usb_poll_callback(timer_callback_args_t * p_args)
{
    (void)p_args;
    process_usb_event();
}
