/*************************************************************************//**
 * @file
 * @brief   Provides functions with debug information printed on a cli.
 *
 * @copyright
 *
 * Copyright (c) 2024, Broadcom, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/*!***************************************************************************
 * @addtogroup  argus_debug_cli
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus_debug_cli.h"
#include "platform/argus_print.h"  // declaration of print()

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Just a shortcut for better code readability */
#define SEP ARGUS_DEBUG_COLUMN_SEPARATOR

/*******************************************************************************
 * Code
 ******************************************************************************/

void Print_DebugHeader(void)
{
    /* Print the debugging measurement results:
     *
     * Note: Sending data via UART creates a large delay which might prevent
     *       the API from reaching the full frame rate. This example sends
     *       approximately 750 characters
     *       -> 115200 bps limits the max. frame rate to 15 fps
     *       -> 2000000 bps limits the max. frame rate to 266 fps
     *       115200 bps / 10 [bauds-per-byte] / 750 [bytes-per-frame] = 15 fps */

    /*Creating data header */
    /*1.*/
    print("Frame_No" SEP "Timestamp[sec]" SEP "R_bin[mm]" SEP "A_bin[LSB]" SEP "Status" SEP);

    /* Next print requires calling function Argus_EvaluateDataDebug! */
    /*------------------------------------------------------------------------*/
    print("DCAAmpl[LSB]" SEP);
    /*------------------------------------------------------------------------*/

    /*2.*/
    for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
    {
        for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
        {
            print("R(%d/%d)[mm]" SEP "A(%d/%d)[LSB]" SEP "Status(%d/%d)" SEP, x, y, x, y, x, y);

            /* Next prints require calling function Argus_EvaluateDataDebug! */
            /*------------------------------------------------------------------------*/
            for (uint_fast8_t p = 0; p < ARGUS_PHASECOUNT; ++p)
            {
                print("S%d(%d/%d)[LSB]" SEP, p, x, y);
            }
            /*------------------------------------------------------------------------*/
        }
    }
    /* Reference pixel information */
    print("R_ref[mm]" SEP "A_ref[LSB]" SEP "St_ref" SEP);

    /* Next prints require calling function Argus_EvaluateDataDebug! */
    /*------------------------------------------------------------------------*/
    /* Crosstalk Values */
    for (uint_fast8_t y = 0; y < (ARGUS_PIXELS_Y >> 1); ++y)
    {
        print("xtkPred[%d]dS[LSB]" SEP, y);
        print("xtkPred[%d]dC[LSB]" SEP, y);
    }
    for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
    {
        print("xtkMon[%d]dS[LSB]" SEP, y);
        print("xtkMon[%d]dC[LSB]" SEP, y);
    }
    /*------------------------------------------------------------------------*/

    print("Sat_px_no" SEP);
    /*3.*/
    print("AInt" SEP "DInt" SEP "ModCurrent[mA]" SEP "PixelGain[LSB]" SEP);
    /*4.*/
    print("AUX_IAPD[LSB]" SEP "AUX_SNA[LSB]" SEP "AUX_Temp[°C]" SEP "AUX_VDD[LSB]" SEP "AUX_VDDL[LSB]" SEP "AUX_VSUB[LSB]\n");
}

void Print_DebugResults(uint32_t frame_cnt, argus_results_t const * res)
{
    /* Print the debugging measurement results:
     *
     * Note: Sending data via UART creates a large delay which might prevent
     *       the API from reaching the full frame rate. This example sends
     *       approximately 380 characters per frame
     *       -> 115200 bps limits the max. frame rate to 30 fps
     *       -> 2000000 bps limits the max. frame rate to 500 fps
     *       115200 bps / 10 [bauds-per-byte] / 380 [bytes-per-frame] = 30 fps */

    assert(res->Debug != 0); // requires the debug data structure!
    if (res->Debug == 0) return;
    const argus_results_debug_t * res_dbg = res->Debug;

    print("%lu" SEP, frame_cnt);

    /*1.*/
    print("%lu.%06lu" SEP "%ld" SEP "%d" SEP "%ld" SEP,
          res->TimeStamp.sec,
          res->TimeStamp.usec,
          res->Bin.Range / (Q9_22_ONE / 1000),
          res->Bin.Amplitude / UQ12_4_ONE,
          res->Status);

    /* Next print requires calling function Argus_EvaluateDataDebug! */
    /*------------------------------------------------------------------------*/
    print("%d" SEP, res_dbg->DCAAmplitude / UQ12_4_ONE);
    /*------------------------------------------------------------------------*/

    /*2.*/
    uint8_t sat_px_cnt = 0;
    for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
    {
        for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
        {
            print("%ld" SEP "%d" SEP "0x%02X" SEP,
                  res->Pixel[x][y].Range / (Q9_22_ONE / 1000),
                  res->Pixel[x][y].Amplitude / UQ12_4_ONE,
                  res->Pixel[x][y].Status);
            sat_px_cnt += (res->Pixel[x][y].Status & PIXEL_SAT) * 0.5;

            /* Next prints require calling function Argus_EvaluateDataDebug! */
            /*------------------------------------------------------------------------*/
            for (uint_fast8_t p = 0; p < ARGUS_PHASECOUNT; ++p)
            {
                uint32_t sample = (res_dbg->Data[p + (y + (x * ARGUS_PIXELS_Y)) * ARGUS_PHASECOUNT] & 0x3FFFFFU);
                print("%lu" SEP, sample / res->Frame.DigitalIntegrationDepth);
            }
            /*------------------------------------------------------------------------*/
        }
    }
    /* Reference pixel information */
    print("%ld" SEP "%d" SEP "0x%02X" SEP,
          res->PixelRef.Range / (Q9_22_ONE / 1000),
          res->PixelRef.Amplitude / UQ12_4_ONE,
          res->PixelRef.Status);

    /* Next prints require calling function Argus_EvaluateDataDebug! */
    /*------------------------------------------------------------------------*/
    /* Crosstalk Values */
    for (uint_fast8_t y = 0; y < (ARGUS_PIXELS_Y >> 1); ++y)
    {
        print("%4.2f" SEP, res_dbg->XtalkPredictor[y].dS / (double)(Q11_4_ONE));
        print("%4.2f" SEP, res_dbg->XtalkPredictor[y].dC / (double)(Q11_4_ONE));
    }
    for (uint_fast8_t y = 0; y < ARGUS_PIXELS_Y; ++y)
    {
        print("%4.2f" SEP, res_dbg->XtalkMonitor[y].dS / (double)(Q11_4_ONE));
        print("%4.2f" SEP, res_dbg->XtalkMonitor[y].dC / (double)(Q11_4_ONE));
    }
    /*------------------------------------------------------------------------*/

    print("%d" SEP, sat_px_cnt);

    /*3.*/
    print("%d" SEP "%d" SEP "%d" SEP "%d" SEP,
          res->Frame.AnalogIntegrationDepth / (UQ10_6_ONE),
          res->Frame.DigitalIntegrationDepth,
          res->Frame.OutputPower / (UQ12_4_ONE),
          res->Frame.PixelGain);

    /*4.*/
    print("%d" SEP "%d" SEP "%d" SEP "%d" SEP "%d" SEP "%d\n",
          res->Auxiliary.IAPD / (UQ12_4_ONE),
          res->Auxiliary.SNA / (UQ12_4_ONE),
          res->Auxiliary.TEMP / (Q11_4_ONE),
          res->Auxiliary.VDD / (UQ12_4_ONE),
          res->Auxiliary.VDDL / (UQ12_4_ONE),
          res->Auxiliary.VSUB / (UQ12_4_ONE));
}

/*! @} */
