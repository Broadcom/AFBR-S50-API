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

#include "usb_sci.h"
#include "usb_device_config.h"
#include "usb/include/usb_device.h"
#include "driver/irq.h"
#include "board/board_config.h"
#include "utility/time.h"
#include "debug.h"

#include "usb/include/usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief USB timeout */
#define USB_TX_TIMEOUT (100U) // ms

/*! @brief maximum USB receive buffer size */
#define MAX_RECEIVE_BUFFER_SIZE 64

/*! @brief the TX data structure */
typedef struct _usb_sci_tx_data_struct
{
    uint8_t epNumber;                                                         /*!< Endpoint number */
    uint32_t transferSize;
    uint8_t *sendData; /*!< Data to send */
} usb_sci_tx_data_struct_t;

/*! @brief the RX data structure */
typedef struct _usb_sci_rx_data_struct
{
    uint8_t epNumber;                            /*!< Endpoint number */
    uint16_t epMaxPacketSize;                    /*!< Endpoint max packet size */
    uint32_t transferCount;                      /*!< Size of transferred data */
    uint8_t *recvData; /*!< Data to receive */
} usb_sci_rx_data_struct_t;

/*! @brief USB SCI structure */
typedef struct _usb_sci_agent_struct
{
    usb_device_handle deviceHandle;               /*!< The device handle */
    uint8_t speed;                                /*!< Used to store the device speed */
    uint8_t attach;                               /*!< Used to store the attach event */
    uint8_t currentConfig;                        /*!< Current configuration */
    uint8_t currentInterfaceAlternateSetting[1U]; /*!< Current alternate setting of the interface*/

    usb_sci_rx_data_struct_t bulkOutData;     /*!< Receive data information */
    usb_sci_tx_data_struct_t bulkInData;      /*!< Send data information */
    uint8_t *recvDataBuffer;                   /*!< Receive data buffer */

} usb_sci_agent_struct_t;

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0)
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void USB_DeviceClockInit(void);
static void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

static void SCI_USB_DataReceivedCallback(uint8_t const * data, uint32_t size);

static usb_status_t USB_DeviceSCISetConfigure(usb_device_handle handle, uint8_t configure);

static usb_status_t USB_DeviceSCIBulkInCallback(usb_device_handle handle,
                                                        usb_device_endpoint_callback_message_struct_t *message,
                                                        void *callbackParam);
static usb_status_t USB_DeviceSCIBulkOutCallback(usb_device_handle handle,
                                                         usb_device_endpoint_callback_message_struct_t *message,
                                                         void *callbackParam);
static usb_status_t USB_SCIAgentRecvComplete(uint32_t handle, void *param);

static usb_status_t USB_SCIAgentSendData(uint32_t handle, uint8_t *appBuffer, uint32_t size);

static usb_status_t USB_SCIAgentCancelSendData(uint32_t handle);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool isInitialized = false;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint32_t s_RecvDataBuffer[MAX_RECEIVE_BUFFER_SIZE];

static usb_error_callback_t myErrorCallback = 0;
static usb_rx_callback_t myRxCallback = 0;
static usb_tx_callback_t myTxCallback = 0;
static void * myTxCallbackState = 0;

static volatile bool isTxOnGoing = false;

/*! Start time for timeout handling. */
static ltc_t myTxStartTimeStamp;

/*! @brief agent instance */
static usb_sci_agent_struct_t usb_sciAgent;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t USB_DeviceApplicationInit(void)
{
    if(!isInitialized)
    {
        USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
        SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

        usb_sciAgent.speed = USB_SPEED_FULL;
        usb_sciAgent.attach = 0U;
        usb_sciAgent.deviceHandle = NULL;
        usb_sciAgent.recvDataBuffer = (uint8_t *)(&s_RecvDataBuffer[0]);

        if (kStatus_USB_Success != USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &usb_sciAgent.deviceHandle))
        {
            usb_echo("USB device SCI usb demo init failed\r\n");
            return kStatus_USB_Error;
        }
        else
        {
            usb_echo("USB device SCI usb demo\r\n");

        }

        /* Install isr, set priority, and enable IRQ. */
        USB_DeviceIsrEnable();

        USB_DeviceRun(usb_sciAgent.deviceHandle);

        isInitialized = true;
    }

    return STATUS_OK;
}

status_t USB_SendBuffer(uint8_t * txBuff, size_t txSize, usb_tx_callback_t f, void * state)
{
    /* Check that we're not busy.*/
    if (!isInitialized) return ERROR_NOT_INITIALIZED;

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

    /* Set Tx Busy Status. */
    isTxOnGoing = true;
    IRQ_UNLOCK();

    Time_GetNow(&myTxStartTimeStamp);

    myTxCallback = f;
    myTxCallbackState = state;

    usb_status_t usb_status = USB_SCIAgentSendData((uint32_t) usb_sciAgent.deviceHandle, txBuff, txSize);
    if (usb_status == kStatus_USB_Busy)
    {
        return STATUS_BUSY;
    }
    else if (usb_status != kStatus_USB_Success)
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

status_t USB_CancelSend(void)
{
    /* Check that we're not busy. */
    if (!isInitialized) return ERROR_NOT_INITIALIZED;
    if (!isTxOnGoing) return STATUS_IDLE;

    usb_status_t usb_status = USB_SCIAgentCancelSendData((uint32_t) usb_sciAgent.deviceHandle);
    if (usb_status != kStatus_USB_Success)
    {
        return ERROR_USB;
    }

    return STATUS_OK;
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

void USB_SetErrorCallback(usb_error_callback_t f)
{
    myErrorCallback = f;
}

void USB_SetRxCallback(usb_rx_callback_t f)
{
    IRQ_LOCK();
    myRxCallback = f;
    IRQ_UNLOCK();
}

#if (defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U))
void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(usb_sciAgent.deviceHandle);
}
#endif

static void USB_DeviceClockInit(void)
{
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    /*
     * If the SOC has USB KHCI dedicated RAM, the RAM memory needs to be clear after
     * the KHCI clock is enabled. When the demo uses USB EHCI IP, the USB KHCI dedicated
     * RAM can not be used and the memory can't be accessed.
     */
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U))
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS) && (FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS > 0U))
    for (int i = 0; i < FSL_FEATURE_USB_KHCI_USB_RAM; i++)
    {
        ((uint8_t *)FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
    }
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS */
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM */
#endif
}

static void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    uint8_t usbDeviceKhciIrq[] = USB_IRQS;
    irqNumber = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];
#endif
    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type) irqNumber, IRQPRIO_USB);
    EnableIRQ((IRQn_Type) irqNumber);
}

#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    USB_DeviceKhciTaskFunction(deviceHandle);
#endif
}
#endif

static void SCI_USB_DataReceivedCallback(uint8_t const * data, uint32_t size)
{
    if (myRxCallback)
        myRxCallback(data, size);
}

static usb_status_t USB_DeviceSCIBulkInCallback(usb_device_handle handle,
                                                usb_device_endpoint_callback_message_struct_t * message,
                                                void *callbackParam)
{
    (void) handle;
    (void) message;
    (void) callbackParam;

    status_t status = STATUS_OK;
    if (NULL == message)
    {
        status = ERROR_USB;
    }
    else if (USB_UNINITIALIZED_VAL_32 == message->length)
    {
        status = ERROR_ABORTED;
    }
    else
    {
        status = STATUS_OK;
    }

    usb_tx_callback_t cb = myTxCallback;
    void * state = myTxCallbackState;

    myTxCallback = 0;
    myTxCallbackState = 0;
    myTxStartTimeStamp.sec = 0;
    myTxStartTimeStamp.usec = 0;
    isTxOnGoing = false;

    if (status != STATUS_OK && status != ERROR_ABORTED)
    {
        if (myErrorCallback)
        myErrorCallback(ERROR_USB);
    }
    else if (cb != 0)
    {
        cb(status, state);
    }

    return status < STATUS_OK ? kStatus_USB_Error : kStatus_USB_Success;
}

static usb_status_t USB_DeviceSCIBulkOutCallback(usb_device_handle handle,
                                                 usb_device_endpoint_callback_message_struct_t * message,
                                                 void *callbackParam)
{
    (void) callbackParam;
    return USB_SCIAgentRecvComplete((uint32_t) handle, message);
}

usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void * param)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t *temp8 = (uint8_t *) param;
    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            usb_sciAgent.attach = 0U;
            USB_DeviceControlPipeInit(handle);
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceGetStatus(handle, kUSB_DeviceStatusSpeed, &usb_sciAgent.speed))
            {
                USB_DeviceSetSpeed(usb_sciAgent.speed);
            }
#endif
            /* save endpoint info */
            if (USB_SPEED_HIGH == usb_sciAgent.speed)
            {
                usb_sciAgent.bulkOutData.epMaxPacketSize = HS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE;
            }
            else
            {
                usb_sciAgent.bulkOutData.epMaxPacketSize = FS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE;
            }
            usb_sciAgent.bulkOutData.epNumber = USB_SCI_BULK_ENDPOINT_OUT;
            usb_sciAgent.bulkOutData.transferCount = 0U;
            usb_sciAgent.bulkOutData.recvData = NULL;
            /* bulk in endpoint information */
            usb_sciAgent.bulkInData.epNumber = USB_SCI_BULK_ENDPOINT_IN;

//          isTxOnGoing = false;
        }
        break;

        case kUSB_DeviceEventSetConfiguration:
        {
            if (USB_SCI_CONFIGURE_INDEX == (*temp8))
            {
                USB_DeviceSCISetConfigure(handle, (*temp8));
            }
            else
            {
            }
            usb_sciAgent.attach = 1U;
            /* send the first NULL data to establish a connection between the device and host */
            USB_SCIAgentSendData((uint32_t) handle, NULL, 0U);
            /* prepare for the first receiving */
            USB_DeviceRecvRequest(handle, usb_sciAgent.bulkOutData.epNumber, usb_sciAgent.recvDataBuffer,
                                  usb_sciAgent.bulkOutData.epMaxPacketSize);
        }
        break;

        case kUSB_DeviceEventResume:
        {

        }
        break;

        default: break;
    }
    return error;
}

static usb_status_t USB_DeviceSCISetConfigure(usb_device_handle handle, uint8_t configure)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;

    if (USB_SCI_CONFIGURE_INDEX == configure)
    {
        /* BulkOUT ep */
        epCallback.callbackFn = USB_DeviceSCIBulkOutCallback;
        epCallback.callbackParam = handle;

        epInitStruct.zlt = 0U;
        epInitStruct.transferType = USB_ENDPOINT_BULK;
        epInitStruct.endpointAddress =
        USB_SCI_BULK_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        if (USB_SPEED_HIGH == usb_sciAgent.speed)
        {
            epInitStruct.maxPacketSize = HS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE;
        }
        else
        {
            epInitStruct.maxPacketSize = FS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE;
        }
        USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

        /* BulkIN ep */
        epCallback.callbackFn = USB_DeviceSCIBulkInCallback;
        epCallback.callbackParam = handle;

        epInitStruct.zlt = 0U;
        epInitStruct.transferType = USB_ENDPOINT_BULK;
        epInitStruct.endpointAddress =
        USB_SCI_BULK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        if (USB_SPEED_HIGH == usb_sciAgent.speed)
        {
            epInitStruct.maxPacketSize = HS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE;
        }
        else
        {
            epInitStruct.maxPacketSize = FS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE;
        }
        USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    (void) handle;
    static uint32_t phdcWeighscaleSetup[2U];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *) &phdcWeighscaleSetup;
    return kStatus_USB_Success;
}


usb_status_t USB_DeviceGetVendorReceiveBuffer(usb_device_handle handle,
                                              usb_setup_struct_t *setup,
                                              uint32_t *length,
                                              uint8_t **buffer)
{
    (void) handle;
    (void) setup;
    (void) length;
    (void) buffer;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceProcessVendorRequest(usb_device_handle handle,
                                            usb_setup_struct_t *setup,
                                            uint32_t *length,
                                            uint8_t **buffer)
{
    (void) handle;
    (void) setup;
    (void) length;
    (void) buffer;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    (void) handle;
    (void) enable;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    (void) handle;
    (void) ep;
    (void) status;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                             usb_setup_struct_t *setup,
                                             uint32_t *length,
                                             uint8_t **buffer)
{
    (void) handle;
    (void) setup;
    (void) length;
    (void) buffer;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    (void) handle;
    (void) setup;
    (void) length;
    (void) buffer;
    return kStatus_USB_InvalidRequest;
}

static usb_status_t USB_SCIAgentRecvComplete(uint32_t handle, void *param)
{
    usb_device_endpoint_callback_message_struct_t *message = (usb_device_endpoint_callback_message_struct_t *) param;

    /* Save the length of the received data block */
    usb_sciAgent.bulkOutData.transferCount = message->length;

    usb_sciAgent.bulkOutData.recvData = &usb_sciAgent.recvDataBuffer[0];
    /* Save the received data */
    memcpy(usb_sciAgent.bulkOutData.recvData, message->buffer, message->length);

    /* sci rx Callback function */
    SCI_USB_DataReceivedCallback(usb_sciAgent.bulkOutData.recvData,
                                 usb_sciAgent.bulkOutData.transferCount);

    usb_sciAgent.bulkOutData.transferCount = 0;
//  usb_sciAgent.bulkOutData.recvData.transferSize = 0;
    usb_sciAgent.bulkOutData.recvData = NULL;

    /* Prepare for the next receiving */
    USB_DeviceRecvRequest((void *) handle, usb_sciAgent.bulkOutData.epNumber, usb_sciAgent.recvDataBuffer,
                          usb_sciAgent.bulkOutData.epMaxPacketSize);

    return kStatus_USB_Success;
}

static usb_status_t USB_SCIAgentSendData(uint32_t handle, uint8_t *appBuffer, uint32_t size)
{
    /* initialize the data to send */
    usb_sci_tx_data_struct_t * dataToSend = &usb_sciAgent.bulkInData;

    dataToSend->transferSize = size;
    dataToSend->sendData = appBuffer;

    return USB_DeviceSendRequest((void *) handle, dataToSend->epNumber, appBuffer, size);
}

static usb_status_t USB_SCIAgentCancelSendData(uint32_t handle)
{
    /* cancels the data send */
    usb_sci_tx_data_struct_t * dataToSend = &usb_sciAgent.bulkInData;

    dataToSend->sendData = 0;
    dataToSend->transferSize = 0;

    return USB_DeviceCancel((void *) handle, dataToSend->epNumber | (USB_IN << 7U));
}

