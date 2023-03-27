/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 , 2018 NXP
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

#include "usb/usb_device_config.h"
#include "usb/include/usb.h"
#include "usb/include/usb_device.h"


#include "usb/usb_device_descriptor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t g_currentConfigure = 0U;
static uint8_t g_interface[USB_SCI_INTERFACE_COUNT];

/*!***************************************************************************
 * @brief   USB device descriptor
 *****************************************************************************/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceDescriptor[] =
{
    USB_DESCRIPTOR_LENGTH_DEVICE, /* size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE,   /* DEVICE descriptor type */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION),
    /* USB Specification Release Number in
       Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,            /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,         /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,         /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE, /* Maximum packet size for endpoint zero
                                    (only 8, 16, 32, or 64 are valid) */
    USB_SHORT_GET_LOW(USB_VENDOR_ID),               /* Vendor ID (assigned by the USB-IF) */
    USB_SHORT_GET_HIGH(USB_VENDOR_ID),
    USB_SHORT_GET_LOW(USB_PRODUCT_ID),              /* Product ID (assigned by the manufacturer) */
    USB_SHORT_GET_HIGH(USB_PRODUCT_ID),
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION), /* Device release number in binary-coded decimal */
    0x01U,                                           /* Index of string descriptor describing manufacturer */
    0x02U,                                           /* Index of string descriptor describing product */
    0x00U,                                           /* Index of string descriptor describing the
                                                        device's serial number */
    USB_DEVICE_CONFIGURATION_COUNT,                  /* Number of possible configurations */
};


/*!***************************************************************************
 * @brief   SCI configuration descriptor
 *****************************************************************************/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceConfigurationDescriptor[] =
{
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */

    /* Total length of data returned for this configuration. */
    USB_SHORT_GET_LOW(
        USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
        USB_DESCRIPTOR_LENGTH_ENDPOINT  + USB_DESCRIPTOR_LENGTH_ENDPOINT),
    USB_SHORT_GET_HIGH(
        USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
        USB_DESCRIPTOR_LENGTH_ENDPOINT  + USB_DESCRIPTOR_LENGTH_ENDPOINT),

    USB_SCI_INTERFACE_COUNT, /* Number of interfaces supported by this configuration */
    USB_SCI_CONFIGURE_INDEX, /* Value to use as an argument to the
                                SetConfiguration() request to select this configuration */
    0U,                      /* Index of string descriptor describing this configuration */

    /* Configuration characteristics
       D7: Reserved (set to one)
       D6: Self-powered
       D5: Remote Wakeup
       D4...0: Reserved (reset to zero) */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),

    /* Maximum power consumption of the USB device from the bus in this
     * specific configuration when the device is fully operational.
     * Expressed in 2 mA units (i.e., 50 = 100 mA). */
    USB_DEVICE_MAX_POWER,

    USB_DESCRIPTOR_LENGTH_INTERFACE,       /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_INTERFACE,         /* INTERFACE Descriptor Type */
    USB_SCI_INTERFACE_INDEX, /* Number of this interface. */
    0x00U,                                 /* Value used to select this alternate setting
                                              for the interface identified in the prior field */
    USB_SCI_ENDPOINT_COUNT,  /* Number of endpoints used by this
                                              interface (excluding endpoint zero). */
    USB_SCI_CLASS,                        /* Class code (assigned by the USB-IF). */

    USB_SCI_SUBCLASS,                                 /* Subclass code (assigned by the USB-IF). */
    USB_SCI_PROTOCOL,                                 /* Protocol code (assigned by the USB). */
    0x04U,                                            /* Index of string descriptor describing this interface */

    /****************************************************************************
     * Bulk In Endpoint
     ****************************************************************************/

    USB_DESCRIPTOR_LENGTH_ENDPOINT, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,   /* ENDPOINT Descriptor Type */

    /* The address of the endpoint on the USB device described by this descriptor. */
    USB_SCI_BULK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),

    USB_ENDPOINT_BULK, /* This field describes the endpoint's attributes */

    /* Maximum packet size this endpoint is capable of sending or receiving
     * when this configuration is selected. */
    USB_SHORT_GET_LOW(FS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE),

    0x00U, /* Interval for polling endpoint for data transfers. */
    /* bInterval is used to specify the polling interval of certain transfers.
       The units are expressed in frames, thus this equates to either 1ms for low/full
       speed devices and 125us for high speed devices.*/

    /****************************************************************************
     * Bulk Out Endpoint
     ****************************************************************************/

    USB_DESCRIPTOR_LENGTH_ENDPOINT,         /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,           /* ENDPOINT Descriptor Type */

    /* The address of the endpoint on the USB device described by this descriptor. */
    USB_SCI_BULK_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),

    USB_ENDPOINT_BULK, /* This field describes the endpoint's attributes */

    /* Maximum packet size this endpoint is capable of sending or receiving
     * when this configuration is selected. */
    USB_SHORT_GET_LOW(FS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE),

    0x00U, /* Interval for polling endpoint for data transfers. */
    /* bInterval is used to specify the polling interval of certain transfers.
       The units are expressed in frames, thus this equates to either 1ms for low/full
       speed devices and 125us for high speed devices.*/
};


/*!***************************************************************************
 * @brief   SCI string 0 descriptor
 *****************************************************************************/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString0[] = {2U + 2U, USB_DESCRIPTOR_TYPE_STRING, 0x09U, 0x04U};


/*!***************************************************************************
 * @brief   SCI string 1 descriptor
 *****************************************************************************/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString1[] = {2U + 2U * 8U, USB_DESCRIPTOR_TYPE_STRING,
                                'B',           0x00U,
                                'R',           0x00U,
                                'O',           0x00U,
                                'A',           0x00U,
                                'D',           0x00U,
                                'C',           0x00U,
                                'O',           0x00U,
                                'M',           0x00U,
};

/*!***************************************************************************
 * @brief   SCI string 2 descriptor
 *****************************************************************************/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
#if USB_PRODUCT_ID == 0x0310U
uint8_t g_UsbDeviceString2[] = {2U + 2U * 13U, USB_DESCRIPTOR_TYPE_STRING,
                                'A',           0x00U,
                                'F',           0x00U,
                                'B',           0x00U,
                                'R',           0x00U,
                                '-',           0x00U,
                                'S',           0x00U,
                                '5',           0x00U,
                                '0',           0x00U,
                                '-',           0x00U,
                                'H',           0x00U,
                                'T',           0x00U,
                                'O',           0x00U,
                                'L',           0x00U,
};
#else
uint8_t g_UsbDeviceString2[] = {2U + 2U * 11U, USB_DESCRIPTOR_TYPE_STRING,
                                'A',           0x00U,
                                'F',           0x00U,
                                'B',           0x00U,
                                'R',           0x00U,
                                '-',           0x00U,
                                'S',           0x00U,
                                '5',           0x00U,
                                '0',           0x00U,
                                '-',           0x00U,
                                'E',           0x00U,
                                'K',           0x00U,
};
#endif

/*!***************************************************************************
 * @brief   SCI string 3 descriptor
 *****************************************************************************/
uint8_t g_UsbDeviceStringN[] = {2U + 2U * 16U, USB_DESCRIPTOR_TYPE_STRING,
                                'B',           0x00U,
                                'A',           0x00U,
                                'D',           0x00U,
                                ' ',           0x00U,
                                'S',           0x00U,
                                'T',           0x00U,
                                'R',           0x00U,
                                'I',           0x00U,
                                'N',           0x00U,
                                'G',           0x00U,
                                ' ',           0x00U,
                                'I',           0x00U,
                                'N',           0x00U,
                                'D',           0x00U,
                                'E',           0x00U,
                                'X',           0x00U};

/*!***************************************************************************
 * @brief   SCI string descriptor length
 *****************************************************************************/
uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT + 1U] = {
    sizeof(g_UsbDeviceString0), sizeof(g_UsbDeviceString1), sizeof(g_UsbDeviceString2), sizeof(g_UsbDeviceStringN)};

/*!***************************************************************************
 * @brief   SCI string descriptor array
 *****************************************************************************/
uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT + 1U] = {g_UsbDeviceString0, g_UsbDeviceString1,
                                                                           g_UsbDeviceString2, g_UsbDeviceStringN};

/*!***************************************************************************
 * @brief   SCI USB language
 *****************************************************************************/
usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray, g_UsbDeviceStringDescriptorLength, (uint16_t)0x0409U,
}};

/*!***************************************************************************
 * @brief   SCI USB language list
 *****************************************************************************/
usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0, sizeof(g_UsbDeviceString0), g_UsbDeviceLanguage, USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!***************************************************************************
 * @brief Get the descriptor.
 *
 * The function is used to get the descriptor, including the device descriptor, configuration descriptor, and string
 * descriptor, etc.
 *
 * @param handle              The device handle.
 * @param setup               The setup packet buffer address.
 * @param length              It is an OUT parameter, return the data length need to be sent to host.
 * @param buffer              It is an OUT parameter, return the data buffer address.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceGetDescriptor(usb_device_handle handle,
                                     usb_setup_struct_t *setup,
                                     uint32_t *length,
                                     uint8_t **buffer)
{
    (void)handle;

    usb_status_t error = kStatus_USB_Success;
    uint8_t descriptorType = (uint8_t)((setup->wValue & 0xFF00U) >> 8U);
    uint8_t descriptorIndex = (uint8_t)((setup->wValue & 0x00FFU));
    if (USB_REQUEST_STANDARD_GET_DESCRIPTOR != setup->bRequest)
    {
        return kStatus_USB_InvalidRequest;
    }
    switch (descriptorType)
    {
        case USB_DESCRIPTOR_TYPE_STRING:
        {
            if (descriptorIndex == 0U)
            {
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
                *length = g_UsbDeviceLanguageList.stringLength;
            }
            else
            {
                uint8_t languageId = 0U;
                uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

                for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++)
                {
                    if (setup->wIndex == g_UsbDeviceLanguageList.languageList[languageId].languageId)
                    {
                        if (descriptorIndex < USB_DEVICE_STRING_COUNT)
                        {
                            languageIndex = descriptorIndex;
                        }
                        break;
                    }
                }

                if (USB_DEVICE_STRING_COUNT == languageIndex)
                {
                    error = kStatus_USB_InvalidRequest;
                }
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
                *length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
            }
        }
        break;
        case USB_DESCRIPTOR_TYPE_DEVICE:
        {
            *buffer = g_UsbDeviceDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_DEVICE;
        }
        break;
        case USB_DESCRIPTOR_TYPE_CONFIGURE:
        {
            *buffer = g_UsbDeviceConfigurationDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        }
        break;
        default:
            error = kStatus_USB_InvalidRequest;
            break;
    } /* End Switch */
    return error;
}

/*!***************************************************************************
 * @brief Set the device configuration.
 *
 * The function is used to set the device configuration.
 *
 * @param handle              The device handle.
 * @param configure           The configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceSetConfigure(usb_device_handle handle, uint8_t configure)
{
    if (!configure)
    {
        return kStatus_USB_Error;
    }
    g_currentConfigure = configure;
    return USB_DeviceCallback(handle, kUSB_DeviceEventSetConfiguration, &configure);
}

/*!***************************************************************************
 * @brief Get the device configuration.
 *
 * The function is used to get the device configuration.
 *
 * @param handle              The device handle.
 * @param configure           It is an OUT parameter, save the current configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceGetConfigure(usb_device_handle handle, uint8_t *configure)
{
    (void)handle;
    *configure = g_currentConfigure;
    return kStatus_USB_Success;
}

/*!***************************************************************************
 * @brief Set an interface alternate setting.
 *
 * The function is used to set an interface alternate setting.
 *
 * @param handle              The device handle.
 * @param interface           The interface index.
 * @param alternateSetting   The new alternate setting value.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting)
{
    g_interface[interface] = alternateSetting;
    return USB_DeviceCallback(handle, kUSB_DeviceEventSetInterface, &interface);
}

/*!***************************************************************************
 * @brief Get an interface alternate setting.
 *
 * The function is used to get an interface alternate setting.
 *
 * @param handle              The device handle.
 * @param interface           The interface index.
 * @param alternateSetting   It is an OUT parameter, save the new alternate setting value of the interface.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceGetInterface(usb_device_handle handle, uint8_t interface, uint8_t *alternateSetting)
{
    (void)handle;

    *alternateSetting = g_interface[interface];
    return kStatus_USB_Success;
}

/*!***************************************************************************
 * @brief USB device set speed function.
 *
 * This function sets the speed of the USB device.
 *
 * @param speed Speed type. USB_SPEED_HIGH/USB_SPEED_FULL/USB_SPEED_LOW.
 *
 * @return A USB error code or kStatus_USB_Success.
 *****************************************************************************/
usb_status_t USB_DeviceSetSpeed(uint8_t speed)
{
    usb_descriptor_union_t *pDescStart;
    usb_descriptor_union_t *pDescEnd;

    pDescStart = (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[0]);
    pDescEnd =
        (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1U]);

    while (pDescStart < pDescEnd)
    {
        if (pDescStart->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT)
        {
            if (USB_SPEED_HIGH == speed)
            {
                if (((pDescStart->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                          USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                         (USB_SCI_BULK_ENDPOINT_IN ==
                          (pDescStart->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE,
                                                       pDescStart->endpoint.wMaxPacketSize);
                }
                else if (((pDescStart->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                          USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                         (USB_SCI_BULK_ENDPOINT_OUT ==
                          (pDescStart->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE,
                                                       pDescStart->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
            else
            {
                if (((pDescStart->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                          USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                         (USB_SCI_BULK_ENDPOINT_IN ==
                          (pDescStart->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_USB_SCI_BULK_ENDPOINT_IN_PACKET_SIZE,
                                                       pDescStart->endpoint.wMaxPacketSize);
                }
                else if (((pDescStart->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                          USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                         (USB_SCI_BULK_ENDPOINT_OUT ==
                          (pDescStart->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_USB_SCI_BULK_ENDPOINT_OUT_PACKET_SIZE,
                                                       pDescStart->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
        }
        pDescStart = (usb_descriptor_union_t *)((uint8_t *)pDescStart + pDescStart->common.bLength);
    }
    return kStatus_USB_Success;
}
