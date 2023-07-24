/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Interface
 * @details     This file defines an CAN interface to communicate with the
 *              AFBR-S50 Time-Of-Flight sensor API.
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
 *
 *****************************************************************************/

/*!***************************************************************************
 * @addtogroup  can_intf
 * @{
 *****************************************************************************/

#include "can_api.h"

#include "main.h"
#include "debug.h"

#include "r_can.h"
#include "hal_data.h"
#include "driver/bsp.h"
#include "driver/irq.h"


/*! CAN transmission busy status. */
static volatile bool is_can_tx_busy = false;

/*! CAN remote frame if received but not handled. */
static volatile uint32_t can_rx_remote_id = 0;


/*! The used mail box number */
#define CAN_MAILBOX_ID_TRANSMIT CAN_MAILBOX_ID_0

/*! Data length for TX frames. */
#define CAN_FRAME_TRANSMIT_DATA_BYTES   (8U)


/*! CAN Data Frame ID definition. */
typedef enum argus_can_frame_id_t
{
    /*! Remote Frame ID for starting measurements */
    CAN_FRAME_ID_START = 8,

    /*! Remote Frame ID for stopping measurements */
    CAN_FRAME_ID_STOP = 9,

    /*! 1D Data Frame ID */
    CAN_FRAME_ID_1D = 28,

} argus_can_frame_id_t;

void CAN_Init(void)
{
    fsp_err_t err = R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
    if (FSP_SUCCESS != err)
    {
        print("CAN Open API failed with error code: %d\n", err);
        handle_error(ERROR_FAIL, "CAN Open failed.");
    }
}

void CAN_Deinit(void)
{
    fsp_err_t err = R_CAN_Close(&g_can0_ctrl);
    if (FSP_SUCCESS != err)
    {
        print("CAN Close API failed with error code: %d\n", err);
        handle_error(ERROR_FAIL, "CAN Close failed.");
    }
}

static void CAN_AwaitIdle(void)
{
    const uint32_t timeout_ms = 500;
    ltc_t start;
    Time_GetNow(&start);

    /* Wait until no transfer is ongoing and claim the control. */
    for (;;)
    {
        while (is_can_tx_busy)
        {
            if (Time_CheckTimeoutMSec(&start, timeout_ms))
            {
                handle_error(ERROR_TIMEOUT, "Can Write has yielded a timeout!");
            }
        }

        /* make sure that no IRQ has happened meanwhile... */
        IRQ_LOCK();
        if (!is_can_tx_busy)
        {
            is_can_tx_busy = true;
            IRQ_UNLOCK();
            break;
        }
        IRQ_UNLOCK();
    }
}

static void CAN_Write(can_frame_t * const tx_frame)
{
    assert(tx_frame != 0);
    LED_CAN_TX_ON();

    /* Send frame via CAN */
    fsp_err_t err = R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_ID_TRANSMIT, tx_frame);
    if (FSP_SUCCESS != err)
    {
        LED_CAN_TX_OFF();
        is_can_tx_busy = false;
        print("CAN Write API failed with error code: %d\n", err);
        CAN_Deinit();
        handle_error(ERROR_FAIL, "Can Write has failed!");
    }
}

void CAN_Transmit1D(argus_results_t const * res)
{
    assert(res != NULL);

    CAN_AwaitIdle();

    int32_t tx_range = res->Bin.Range / (Q9_22_ONE / 1000);
    if (tx_range < 0) tx_range = 0;
    else if (tx_range > 0xFFFFFF) tx_range = 0xFFFFFF;

    uint16_t tx_amplitude = res->Bin.Amplitude / (UQ12_4_ONE);
    uint8_t tx_signal = res->Bin.SignalQuality;
    status_t tx_status = res->Status;

    /* configure sending CAN Communication */
    static can_frame_t tx_frame = { .id = CAN_FRAME_ID_1D,
                                    .type = CAN_FRAME_TYPE_DATA,
                                    .data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES
                                  };

    /* Filling CAN-Tx Frame with TOF-data */
    tx_frame.data[0] = (uint8_t)((tx_range >> 16) & 0xFF);
    tx_frame.data[1] = (uint8_t)((tx_range >> 8) & 0xFF);
    tx_frame.data[2] = (uint8_t)((tx_range) & 0xFF);
    tx_frame.data[3] = (uint8_t)((tx_amplitude >> 8) & 0xFF);
    tx_frame.data[4] = (uint8_t)((tx_amplitude) & 0xFF);
    tx_frame.data[5] = (uint8_t)tx_signal;
    tx_frame.data[6] = (uint8_t)((tx_status >> 8) & 0xFF);
    tx_frame.data[7] = (uint8_t)((tx_status) & 0xFF);

    CAN_Write(&tx_frame);
}

void CAN_HandleCommand(void)
{
    IRQ_LOCK();
    uint32_t can_rx = can_rx_remote_id;
    /* Rx command handled: clear flags.. */
    can_rx_remote_id = 0;
    IRQ_UNLOCK();

    switch (can_rx)
    {
        case CAN_FRAME_ID_START:
            start_measurements();

            break;

        case CAN_FRAME_ID_STOP:
            stop_measurements();

            break;

        default:
            /* Nothing to do */
            break;
    }
}

/*!***************************************************************************
 * @brief   CAN callback as defined in the "hal_data" module generated by the
 *          Renesas FSP Configuration.
 * @param   p_args The callback arguments provided by HAL.
 *****************************************************************************/
void can_callback(can_callback_args_t * p_args)
{
    assert(p_args != NULL);
    if (CAN_EVENT_TX_COMPLETE & p_args->event)
    {
        LED_CAN_TX_OFF();
        is_can_tx_busy = false;
    }

    if (CAN_EVENT_RX_COMPLETE & p_args->event)
    {
        // TODO: where to switch on?
        LED_CAN_RX_OFF();

        /* Save remote frame ID to be handled from main thread. */
        if (p_args->frame.type == CAN_FRAME_TYPE_REMOTE)
        {
            if (can_rx_remote_id != 0)
            {
                // TODO: implemented message queue
                print("CAN OVERRUN ERROR: Received new CAN frame but the "
                      "previous frame was not yet handled.\n");
            }
            can_rx_remote_id = p_args->frame.id;
        }
    }

    if (CAN_EVENT_ERR_WARNING & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR WARNING)", p_args->event);
    }
    if (CAN_EVENT_ERR_PASSIVE & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR PASSIVE)", p_args->event);
    }
    if (CAN_EVENT_ERR_BUS_OFF & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR BUS OFF)", p_args->event);
    }
    if (CAN_EVENT_BUS_RECOVERY & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT BUS RECOVERY)", p_args->event);
    }
    if (CAN_EVENT_MAILBOX_MESSAGE_LOST & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT MAILBOX MESSAGE LOST)", p_args->event);
    }
    if (CAN_EVENT_ERR_BUS_LOCK & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR BUS LOCK)", p_args->event);
    }
    if (CAN_EVENT_ERR_CHANNEL & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR CAHNNEL)", p_args->event);
    }
    if (CAN_EVENT_ERR_GLOBAL & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR GLOBAL)", p_args->event);
    }
    if (CAN_EVENT_TX_ABORTED & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT TX ABORTED)", p_args->event);
    }
    if (CAN_EVENT_TX_FIFO_EMPTY & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT TX FIFO EMPTY)", p_args->event);
    }
}

/*! @} */
