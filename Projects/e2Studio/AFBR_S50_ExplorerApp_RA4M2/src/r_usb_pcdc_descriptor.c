/***********************************************************************************************************************
 * File Name    : r_usb_pcdc_descriptor.c
 * Description  : USB PCDC Descriptor definition.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

/******************************************************************************
 Includes   <System Includes> , "Project Includes"
 ******************************************************************************/
#include <r_usb_basic.h>
#include <r_usb_basic_api.h>
#include "r_usb_basic_cfg.h"

/******************************************************************************
 Macro definitions
 ******************************************************************************/

/* bcdUSB */
#define USB_BCDNUM               (0x0200U)

/* Release Number */
#define USB_RELEASE              (0x0101U)

/* Configuration number */
#define USB_CONFIGNUM            (1U)

/* vndror ID */
#define USB_VENDORID             (0x276EU)

/* Product ID */
#define USB_PRODUCTID            (0x0301U)

/* DCP max packet size */
#define USB_DCPMAXP              (64U)

/* Descriptor length */
#define USB_ID_LEN               (9U)  /* Interface Descriptor Length */
#define USB_EPD_LEN              (7U)  /* Endpoint Descriptor Length */
#define USB_DD_LEN               (18U) /* Device Desciptor Length */
#define USB_QD_LEN               (10U) /* Qualifier Descriptor Length */
#define USB_CD_LEN               (32U) /* Configuration Descriptor Length */
#define USB_STRING0_LEN          (4U)  /* String0 Descriptor Length */
#define USB_STRING1_LEN          (18U) /* String1 Descriptor Length (iManufacturer) */
#define USB_STRING2_LEN          (24U) /* String2 Descriptor Length (iProduct) */
#define USB_STRING3_LEN          (30U) /* String3 Descriptor Length (iInterface) */
#define USB_STRING4_LEN          (22U) /* String4 Descriptor Length (iConfiguration) */
#define USB_STRING5_LEN          (18U) /* String5 Descriptor Length (iConfiguration) */
#define USB_STRING6_LEN          (28U) /* String6 Descriptor Length (iSerialNumber) */
#define NUM_STRING_DESCRIPTOR    (7U)  /* Number of String Descriptor */

#define USB_VALUE_64             (64U)
#define USB_VALUE_100            (100U)
#define USB_VALUE_256            (256U)
#define USB_VALUE_FFH            (0xFFU)
#define USB_VENDOR_CODE          (USB_VALUE_FFH)
#define USB_NUM_EP               (2U)

#define USB_MXPS_BULK_FULL       (64U)
#define USB_MXPS_BULK_HI         (512U)
#define USB_MXPS_INT             (64U)


#define USB_DEVICE_CLASS 	USB_VENDOR_CODE
#define USB_DEVICE_SUBCLASS (0xF0U)
#define USB_DEVICE_PROTOCOL (0x00U)

/******************************************************************************
 * Exported global variables (to be accessed by other files)
 ******************************************************************************/
uint8_t g_apl_device[USB_DD_LEN] =
{
    USB_DD_LEN,                                                           /*  0:bLength */
    USB_DT_DEVICE,                                                        /*  1:bDescriptorType */
    (uint8_t) (USB_BCDNUM & (uint8_t) USB_VALUE_FFH),                     /*  2:bcdUSB_lo */
    (uint8_t) ((uint8_t) (USB_BCDNUM >> 8) & (uint8_t) USB_VALUE_FFH),    /*  3:bcdUSB_hi */
	USB_DEVICE_CLASS,          	                                          /*  4:bDeviceClass */
	USB_DEVICE_SUBCLASS,                                                  /*  5:bDeviceSubClass */
	USB_DEVICE_PROTOCOL,                                                  /*  6:bDeviceProtocol */
    (uint8_t) USB_DCPMAXP,                                                /*  7:bMAXPacketSize(for DCP) */
    (uint8_t) (USB_VENDORID & (uint8_t) USB_VALUE_FFH),                   /*  8:idvndror_lo */
    (uint8_t) ((uint8_t) (USB_VENDORID >> 8) & (uint8_t) USB_VALUE_FFH),  /*  9:idvndror_hi */
    (uint8_t) (USB_PRODUCTID & (uint8_t) USB_VALUE_FFH),                  /* 10:idProduct_lo */
    (uint8_t) ((uint8_t) (USB_PRODUCTID >> 8) & (uint8_t) USB_VALUE_FFH), /* 11:idProduct_hi */
    (uint8_t) (USB_RELEASE & (uint8_t) USB_VALUE_FFH),                    /* 12:bcdDevice_lo */
    (uint8_t) ((uint8_t) (USB_RELEASE >> 8) & (uint8_t) USB_VALUE_FFH),   /* 13:bcdDevice_hi */
    1,                                                                    /* 14:iManufacturer */
    2,                                                                    /* 15:iProduct */
    0,                                                                    /* 16:iSerialNumber */
    USB_CONFIGNUM                                                         /* 17:bNumConfigurations */
};

uint8_t g_apl_hs_configuration[USB_CD_LEN + (USB_CD_LEN % 2)] =
{
    9,                                            /*  0:bLength */
    USB_SOFT_CHANGE,                              /*  1:bDescriptorType */
    (uint8_t) (USB_CD_LEN % USB_VALUE_256),       /*  2:wTotalLength(L) */
    (uint8_t) (USB_CD_LEN / USB_VALUE_256),       /*  3:wTotalLength(H) */
    1,                                            /*  4:bNumInterfaces */
    1,                                            /*  5:bConfigurationValue */
    5,                                            /*  6:iConfiguration */
    (uint8_t) (USB_CF_RESERVED | USB_CF_SELFP),   /*  7:bmAttributes */
    (uint8_t) (USB_VALUE_100 / 2),                /*  8:bMaxPower (2mA unit) */

    /* Interface Descriptor */
    USB_ID_LEN,                                   /*  0:bLength */
    USB_DT_INTERFACE,                             /*  1:bDescriptor */
    0,                                            /*  2:bInterfaceNumber */
    0,                                            /*  3:bAlternateSetting */
    USB_NUM_EP,                                   /*  4:bNumEndpoints */
	USB_DEVICE_CLASS,                             /*  5:bInterfaceClass */
	USB_DEVICE_SUBCLASS,                          /*  6:bInterfaceSubClass */
	USB_DEVICE_PROTOCOL,                          /*  7:bInterfaceProtocol */
    0,                                            /*  8:iInterface */

    /* Endpoint Descriptor 0 */
    USB_EPD_LEN,                                  /*  0:bLength */
    USB_DT_ENDPOINT,                              /*  1:bDescriptorType */
    (uint8_t) (USB_EP_IN | USB_EP1),              /*  2:bEndpointAddress */
    USB_EP_BULK,                                  /*  3:bmAttribute */
    (uint8_t) (USB_MXPS_BULK_HI % USB_VALUE_256), /*  4:wMaxPacketSize_lo */
    (uint8_t) (USB_MXPS_BULK_HI / USB_VALUE_256), /*  5:wMaxPacketSize_hi */
    0,                                            /*  6:bInterval */

    /* Endpoint Descriptor 1 */
    USB_EPD_LEN,                                  /*  0:bLength */
    USB_DT_ENDPOINT,                              /*  1:bDescriptorType */
    (uint8_t) (USB_EP_OUT | USB_EP1),             /*  2:bEndpointAddress */
    USB_EP_BULK,                                  /*  3:bmAttribute */
    (uint8_t) (USB_MXPS_BULK_HI % USB_VALUE_256), /*  4:wMaxPacketSize_lo */
    (uint8_t) (USB_MXPS_BULK_HI / USB_VALUE_256), /*  5:wMaxPacketSize_hi */
    0,                                            /*  6:bInterval */
};

uint8_t g_apl_configuration[USB_CD_LEN + (USB_CD_LEN % 2)] =
{
    9,                                              /*  0:bLength */
    USB_SOFT_CHANGE,                                /*  1:bDescriptorType */
    (uint8_t) (USB_CD_LEN % USB_VALUE_256),         /*  2:wTotalLength(L) */
    (uint8_t) (USB_CD_LEN / USB_VALUE_256),         /*  3:wTotalLength(H) */
    1,                                              /*  4:bNumInterfaces */
    1,                                              /*  5:bConfigurationValue */
    0,                                              /*  6:iConfiguration */
    (uint8_t) (USB_CF_RESERVED | USB_CF_SELFP),     /*  7:bmAttributes */
    (uint8_t) (USB_VALUE_100 / 2),                  /*  8:bMaxPower (2mA unit) */

    /* Interface Descriptor */
    USB_ID_LEN,                                     /*  0:bLength */
    USB_DT_INTERFACE,                               /*  1:bDescriptor */
    0,                                              /*  2:bInterfaceNumber */
    0,                                              /*  3:bAlternateSetting */
	USB_NUM_EP,                                   /*  4:bNumEndpoints */
	USB_DEVICE_CLASS,                             /*  5:bInterfaceClass */
	USB_DEVICE_SUBCLASS,                          /*  6:bInterfaceSubClass */
	USB_DEVICE_PROTOCOL,                          /*  7:bInterfaceProtocol */
	0,                                            /*  8:iInterface */

    /* Endpoint Descriptor 0 */
    USB_EPD_LEN,                                    /*  0:bLength */
    USB_DT_ENDPOINT,                                /*  1:bDescriptorType */
    (uint8_t) (USB_EP_IN | USB_EP1),                /*  2:bEndpointAddress */
    USB_EP_BULK,                                    /*  3:bmAttribute */
    (uint8_t) (USB_MXPS_BULK_FULL % USB_VALUE_256), /*  4:wMaxPacketSize_lo */
    (uint8_t) (USB_MXPS_BULK_FULL / USB_VALUE_256), /*  5:wMaxPacketSize_hi */
    0,                                              /*  6:bInterval */

    /* Endpoint Descriptor 1 */
    USB_EPD_LEN,                                    /*  0:bLength */
    USB_DT_ENDPOINT,                                /*  1:bDescriptorType */
    (uint8_t) (USB_EP_OUT | USB_EP1),               /*  2:bEndpointAddress */
    USB_EP_BULK,                                    /*  3:bmAttribute */
    (uint8_t) (USB_MXPS_BULK_FULL % USB_VALUE_256), /*  4:wMaxPacketSize_lo */
    (uint8_t) (USB_MXPS_BULK_FULL / USB_VALUE_256), /*  5:wMaxPacketSize_hi */
    0,                                              /*  6:bInterval */
};

uint8_t g_apl_qualifier[USB_QD_LEN] =
{
    USB_QD_LEN,                                                        /*  0:bLength */
    USB_DT_DEVICE_QUALIFIER,                                           /*  1:bDescriptorType */
    (uint8_t) (USB_BCDNUM & (uint8_t) USB_VALUE_FFH),                  /*  2:bcdUSB_lo */
    (uint8_t) ((uint8_t) (USB_BCDNUM >> 8) & (uint8_t) USB_VALUE_FFH), /*  3:bcdUSB_hi */
    USB_VENDOR_CODE,                                                   /*  4:bDeviceClass */
    USB_VENDOR_CODE,                                                   /*  5:bDeviceSubClass */
    USB_VENDOR_CODE,                                                   /*  6:bDeviceProtocol */
    (uint8_t) USB_DCPMAXP,                                             /*  7:bMAXPacketSize(for DCP) */
    USB_CONFIGNUM,                                                     /*  8:bNumConfigurations */
    0                                                                  /*  9:bReserved */
};

/* UNICODE 0x0409 English (United States) */
uint8_t g_apl_string0[USB_STRING0_LEN] =
{
    USB_STRING0_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    0x09, 0x04                         /*  2:wLANGID[0] */
};

/* iManufacturer */
uint8_t g_apl_string1[USB_STRING1_LEN + (USB_STRING1_LEN % 2)] =
{
    USB_STRING1_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    'B',           0x00U,
    'R',           0x00U,
    'O',           0x00U,
    'A',           0x00U,
    'D',           0x00U,
    'C',           0x00U,
    'O',           0x00U,
    'M',           0x00U,
};

/* iProduct */
uint8_t g_apl_string2[USB_STRING2_LEN + (USB_STRING2_LEN % 2)] =
{
    USB_STRING2_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
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

/* iInterface */
uint8_t g_apl_string3[USB_STRING3_LEN + (USB_STRING3_LEN % 2)] =
{
    USB_STRING3_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    'V', 0x00,
    'e', 0x00,
    'n', 0x00,
    'd', 0x00,
    'o', 0x00,
    'r', 0x00,

    ' ', 0x00,
    'D', 0x00,
    'e', 0x00,
    'v', 0x00,
    'i', 0x00,
    'c', 0x00,
    'e', 0x00,
};

/* iConfiguration */
uint8_t g_apl_string4[USB_STRING4_LEN + (USB_STRING4_LEN % 2)] =
{
    USB_STRING4_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    'F', 0x00,                         /*  2:wLANGID[0] */
    'u', 0x00,
    'l', 0x00,
    'l', 0x00,
    '-', 0x00,
    'S', 0x00,
    'p', 0x00,
    'e', 0x00,
    'e', 0x00,
    'd', 0x00
};

/* iConfiguration */
uint8_t g_apl_string5[USB_STRING5_LEN + (USB_STRING5_LEN % 2)] =
{
    USB_STRING5_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    'H', 0x00,                         /*  2:wLANGID[0] */
    'i', 0x00,
    '-', 0x00,
    'S', 0x00,
    'p', 0x00,
    'e', 0x00,
    'e', 0x00,
    'd', 0x00
};

/* iSerialNumber */
uint8_t g_apl_string6[USB_STRING6_LEN + (USB_STRING6_LEN % 2)] =
{
    USB_STRING6_LEN,                   /*  0:bLength */
    USB_DT_STRING,                     /*  1:bDescriptorType */
    '0', 0x00,                         /*  2:wLANGID[0] */
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '1', 0x00,
};

uint8_t * g_apl_string_table[] =
{
    g_apl_string0,
    g_apl_string1,
    g_apl_string2,
    g_apl_string3,
    g_apl_string4,
    g_apl_string5,
    g_apl_string6
};

const usb_descriptor_t g_usb_descriptor =
{
    g_apl_device,                      /* Pointer to the device descriptor */
    g_apl_configuration,               /* Pointer to the configuration descriptor for Full-speed */
    g_apl_hs_configuration,            /* Pointer to the configuration descriptor for Hi-speed */
    g_apl_qualifier,                   /* Pointer to the qualifier descriptor */
    g_apl_string_table,                /* Pointer to the string descriptor table */
    NUM_STRING_DESCRIPTOR
};

/******************************************************************************
 End  Of File
 ******************************************************************************/
