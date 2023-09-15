/***********************************************************************************************************************
 * Copyright [2020-2023] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics America Inc. and may only be used with products
 * of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.  Renesas products are
 * sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for the selection and use
 * of Renesas products and Renesas assumes no liability.  No license, express or implied, to any intellectual property
 * right is granted by Renesas. This software is protected under all applicable laws, including copyright laws. Renesas
 * reserves the right to change or discontinue this software and/or this documentation. THE SOFTWARE AND DOCUMENTATION
 * IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND TO THE FULLEST EXTENT
 * PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY, INCLUDING WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE SOFTWARE OR
 * DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.  TO THE MAXIMUM
 * EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR DOCUMENTATION
 * (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER, INCLUDING,
 * WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY LOST PROFITS,
 * OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY
 * OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

/******************************************************************************
 * Includes   <System Includes> , "Project Includes"
 ******************************************************************************/
#include <r_usb_basic.h>
#include <r_usb_basic_api.h>

#include "inc/r_usb_typedef.h"
#include "inc/r_usb_extern.h"
#include "../hw/inc/r_usb_bitdefine.h"
#include "../hw/inc/r_usb_reg_access.h"

#if ((USB_CFG_MODE & USB_CFG_HOST) == USB_CFG_HOST)

/******************************************************************************
 * Renesas Abstracted Host Signal functions
 ******************************************************************************/

/******************************************************************************
 * Function Name   : usb_hstd_vbus_control
 * Description     : USB VBUS ON/OFF setting.
 * Arguments       : usb_utr_t    *ptr    : Pointer to usb_utr_t structure.
 *                 : uint16_t     command : ON / OFF.
 * Return value    : none
 ******************************************************************************/
void usb_hstd_vbus_control (usb_utr_t * ptr, uint16_t command)
{
    if (USB_VBON == command)
    {
        hw_usb_set_vbout(ptr);
 #if USB_CFG_BC == USB_CFG_ENABLE
  #if defined(USB_HIGH_SPEED_MODULE)
        if (USB_IP1 == ptr->ip)
        {
            usb_hstd_bc_func[g_usb_hstd_bc[ptr->ip].state][USB_BC_EVENT_VB](ptr);
        }
  #endif                               /* defined (USB_HIGH_SPEED_MODULE) */
 #endif                                /* USB_CFG_BC == USB_CFG_ENABLE */
    }
    else
    {
        hw_usb_clear_vbout(ptr);
    }
}

/******************************************************************************
 * End of function usb_hstd_vbus_control
 ******************************************************************************/

/******************************************************************************
 * Function Name   : usb_hstd_suspend_process
 * Description     : Set USB registers as required when USB Device status is moved
 *               : to "Suspend".
 * Arguments       : usb_utr_t    *ptr    : Pointer to usb_utr_t structure.
 * Return value    : none
 ******************************************************************************/
void usb_hstd_suspend_process (usb_utr_t * ptr)
{
    /* SUSPENDED check */
    if (USB_SUSPENDED == g_usb_hstd_remort_port[ptr->ip])
    {
        /* SOF OFF */
        hw_usb_hclear_uact(ptr);

        /* Wait */
        usb_cpu_delay_xms((uint16_t) 1);
        usb_hstd_chk_sof(ptr);

        /* RWUPE=1, UACT=0 */
        hw_usb_hset_rwupe(ptr);

        /* Enable port BCHG interrupt */
        usb_hstd_bchg_enable(ptr);

        /* Wait */
        usb_cpu_delay_xms((uint16_t) 5);
    }
    else
    {
        /* SOF OFF */
        hw_usb_hclear_uact(ptr);

        /* Wait */
        usb_cpu_delay_xms((uint16_t) 5);
    }
}

/******************************************************************************
 * End of function usb_hstd_suspend_process
 ******************************************************************************/

/******************************************************************************
 * Function Name   : usb_hstd_attach
 * Description     : Set USB registers as required when USB device is attached,
 *                 : and notify MGR (manager) task that attach event occurred.
 * Arguments       : usb_utr_t    *ptr    : Pointer to usb_utr_t structure.
 *                 : uint16_t     result  : Result.
 *                 : uint16_t     port    : Port number.
 * Return value    : none
 ******************************************************************************/
void usb_hstd_attach (usb_utr_t * ptr, uint16_t result)
{
    /* DTCH  interrupt enable */
    usb_hstd_dtch_enable(ptr);

    /* Interrupt Enable */
    usb_hstd_berne_enable(ptr);

    /* USB Mng API */
    usb_hstd_notif_ator_detach(ptr, result);
 #if USB_CFG_BC == USB_CFG_ENABLE
  #if defined(USB_HIGH_SPEED_MODULE)
    if (USB_IP1 == ptr->ip)
    {
        usb_hstd_bc_func[g_usb_hstd_bc[ptr->ip].state][USB_BC_EVENT_AT](ptr);
    }
  #endif                               /* defined (USB_HIGH_SPEED_MODULE) */
 #endif                                /* USB_CFG_BC == USB_CFG_ENABLE */
}

/******************************************************************************
 * End of function usb_hstd_attach
 ******************************************************************************/

/******************************************************************************
 * Function Name   : usb_hstd_detach
 * Description     : Set USB register as required when USB device is detached, and
 *                 : notify MGR (manager) task that detach occurred.
 * Arguments       : usb_utr_t    *ptr    : Pointer to usb_utr_t structure.
 *                 : uint16_t     port    : Port number.
 * Return value    : none
 ******************************************************************************/
void usb_hstd_detach (usb_utr_t * ptr)
{
 #if !defined(USB_CFG_OTG_USE)
  #if USB_CFG_BC == USB_CFG_ENABLE
   #if defined(USB_HIGH_SPEED_MODULE)
    if (USB_IP1 == ptr->ip)
    {
        usb_hstd_bc_func[g_usb_hstd_bc[ptr->ip].state][USB_BC_EVENT_DT](ptr);
    }
   #endif                              /* defined(USB_HIGH_SPEED_MODULE) */
  #endif                               /* USB_CFG_BC == USB_CFG_ENABLE */

    /* DVSTCTR clear */
    hw_usb_clear_dvstctr(ptr, (uint16_t) (USB_RWUPE | USB_USBRST | USB_RESUME | USB_UACT));
 #endif /* !defined (USB_CFG_OTG_USE) */
    /* ATTCH interrupt enable */
    usb_hstd_attch_enable(ptr);

    /* USB Mng API */
    usb_hstd_notif_ator_detach(ptr, (uint16_t) USB_DETACH);
}

/******************************************************************************
 * End of function usb_hstd_detach
 ******************************************************************************/
#endif                                 /* (USB_CFG_MODE & USB_CFG_HOST) == USB_CFG_HOST */

/******************************************************************************
 * End  Of File
 ******************************************************************************/
