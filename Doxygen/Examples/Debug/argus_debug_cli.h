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

#ifndef ARGUS_DEBUG_CLI_H
#define ARGUS_DEBUG_CLI_H

/*!***************************************************************************
 * @defgroup    argus_debug_cli Debugging CLI
 * @ingroup     argus
 *
 * @brief       A CLI module to print debugging information on frame basis
 *
 * @details     Just replace your data handling function by this print function
 *              to stream debugging measurement data via UART to a terminal.
 *
 * @addtogroup  argus_debug_cli
 * @{
 *****************************************************************************/

#include "argus.h"

/*!***************************************************************************
 * @brief   Version number of the debug information via CLI.
 *
 * @details Changes:
 *          * v1.0:
 *              - Initial release.
 *
 *****************************************************************************/
#define ARGUS_DEBUG_CLI_VERSION "v1.0"

/*!***************************************************************************
 * @brief   The column separator for the debug print functionality.
 *          Examples: ";", ",", " ", "\t"
 *****************************************************************************/
#define ARGUS_DEBUG_COLUMN_SEPARATOR ";"

/*!***************************************************************************
 * @brief   Prints debugging measurement header via UART.
 *
 * @note    It is suggested to increase the UART baud rate to a sufficient large
 *          value (e.g. 2 Mbps) to avoid slow down of measurement rate due to a
 *          UART bottleneck.
 *
 * @details Prints an formatted header for the extended measurement data via UART.
 *          The measurement data is printed via \ref Print_DebugResults function.
 *          Calling this function before printing any error creates the
 *          according (and optional) header.
 *
 *          The data stream consists of four parts:
 *          1. Frame & binned data
 *          2. Single pixel data (incl. sat pixel count)
 *          3. Integration parameters
 *          4. Auxiliary data
 *
 * @sa      Print_DebugResults
 *****************************************************************************/
void Print_DebugHeader(void);

/*!***************************************************************************
 * @brief   Prints debugging measurement results via UART.
 *
 * @note    It is suggested to increase the UART baud rate to a sufficient large
 *          value (e.g. 2 Mbps) to avoid slow down of measurement rate due to a
 *          UART bottleneck.
 *
 * @details Prints a formatted string of measurement and debug data in table
 *          format via UART. Each call to this function will send a line of
 *          separated data via UART. The default separator is `;` but can be
 *          changed via the \ref ARGUS_DEBUG_COLUMN_SEPARATOR definition.
 *
 *          Note that this uses the debug data structure! So it requires the
 *          `res->Debug` pointer to be set which is done by using the
 *          #Argus_EvaluateDataDebug method instead of the #Argus_EvaluateData
 *          method!
 *
 *          The data stream consists of four parts:
 *          1. Frame & binned data
 *          2. Single pixel data (incl. sat pixel count)
 *          3. Integration data
 *          4. Auxiliary data
 *
 *          In detail:
 *          1. Frame number; timestamp; binned range in mm; binned amplitude in LSB; frame status;
 *          2. Range pixel(0/0); Amplitude pixel(0/0); Status pixel(0/0); ...
 *             ...Range pixel(7/3); Amplitude pixel(7/3); Status pixel(7/3); saturated pixel count of frame;
 *          3. Analog integration in LSB; Digital integration; LaserCurrent in mA; pixel gain in LSB;
 *          4. Auxiliary task: APD current; Shot noise amplitude, ASIC temperature; VDD, VDDL, Substrate voltage
 *
 *          Use the \ref Print_DebugHeader function to output an proper
 *          header for the debug data table.
 *
 * @sa      Print_DebugHeader
 *
 * @param   frame_cnt An integer to print the frame number; this should be
 *                    incremented by one each time the function is called.
 * @param   res The pointer to the measurement results structure to be printed.
 *              Note that the res->Debug pointer must be set for this to work
 *              properly!
 *****************************************************************************/
void Print_DebugResults(uint32_t frame_cnt, argus_results_t const * res);

/*! @} */
#endif /* ARGUS_DEBUG_CLI_H */
