/*************************************************************************//**
 * @file
 * @brief       This file is part of the RA4M2 platform layer.
 * @details     This file provides an interface for IO Ports.
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
 * Include Files
 ******************************************************************************/

#include "bsp_api.h"
#include "r_ioport_api.h"
#include "common_data.h"

const ioport_pin_cfg_t g_bsp_pin_cfg_data2[] =
        {
          {
            .pin = BSP_IO_PORT_01_PIN_10,
            .pin_cfg = ((uint32_t)IOPORT_CFG_PORT_DIRECTION_INPUT) },
          {
            .pin = BSP_IO_PORT_01_PIN_09,
            .pin_cfg = ((uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT
                        | (uint32_t)IOPORT_CFG_PORT_OUTPUT_HIGH) },
          {
            .pin = BSP_IO_PORT_01_PIN_11,
            .pin_cfg = ((uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT) },
        };

const ioport_cfg_t g_bsp_pin_cfg2 =
        {
         .number_of_pins = sizeof(g_bsp_pin_cfg_data2) / sizeof(ioport_pin_cfg_t),
          .p_pin_cfg_data = &g_bsp_pin_cfg_data2[0]
        };


const ioport_pin_cfg_t g_bsp_pin_cfg_data3[] =
        {
          {
            .pin = BSP_IO_PORT_01_PIN_10,
            .pin_cfg =
                    ((uint32_t)IOPORT_CFG_DRIVE_HIGH
                     | (uint32_t)IOPORT_CFG_PERIPHERAL_PIN
                     | (uint32_t)IOPORT_PERIPHERAL_SPI) },
          {
            .pin = BSP_IO_PORT_01_PIN_09,
            .pin_cfg =
                    ((uint32_t)IOPORT_CFG_DRIVE_HIGH
                     | (uint32_t)IOPORT_CFG_PERIPHERAL_PIN
                     | (uint32_t)IOPORT_PERIPHERAL_SPI) },
          {
            .pin = BSP_IO_PORT_01_PIN_11,
            .pin_cfg =
                    ((uint32_t)IOPORT_CFG_DRIVE_HIGH
                     | (uint32_t)IOPORT_CFG_PERIPHERAL_PIN
                     | (uint32_t)IOPORT_PERIPHERAL_SPI) },
        };

const ioport_cfg_t g_bsp_pin_cfg3 =
        {
          .number_of_pins = sizeof(g_bsp_pin_cfg_data3) / sizeof(ioport_pin_cfg_t),
          .p_pin_cfg_data = &g_bsp_pin_cfg_data3[0]
        };



