/*************************************************************************//**
 * @file
 * @brief   Tests for the AFBR-S50 API hardware abstraction layer.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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

#ifndef ARGUS_EXAMPLE_H
#define ARGUS_EXAMPLE_H

/*!***************************************************************************
 * @defgroup    example_apps AFBR-S50 API Examples
 * @ingroup     demo_apps
 *
 * @brief       Example and Demo Projects for the AFBR-S50 API.
 *
 * @details     A series of example and demo projects that utilize the AFBR-S50
 *              API in several ways.
 *
 *              The following example projects are available:
 *
 *              - 01_simple_example.c: Uses the API in the most straightforward
 *                way by invoking single measurements in a blocking manner.
 *
 *              - 02_advanced_example.c: A more sophisticated example that
 *                utilizes a timer to trigger periodic measurements asynchronously
 *                from an interrupt service routine.
 *
 *              - 03_high_speed_example.c: Demonstrates the high speed
 *                configuration of the device that allows measurement rates
 *                up to 3000 frames per second (depending on hardware!!).
 *
 *              - 04_multi_device_example.c: Demonstrates the usage of multiple
 *                devices on a single MCU.
 *
 *              - 05_simple_example_debug.c: Demonstrates how to obtain debug
 *                information.
 *
 * @warning     The example code provides is not intended to be used as a for
 *              production systems. It is intended to demonstrate the usage of
 *              the API and to provide a starting point for custom applications.
 *
 * @addtogroup  example_apps
 * @{
 *****************************************************************************/

/*! Selector for example:
 *  - 1: 01_simple_example.c: Runs measurements in simplest blocking manner.
 *  - 2: 02_advanced_example.c: Starts measurements automatically from timer interrupt.
 *  - 3: 03_high_speed_example.c: Runs measurements with up to 3000 frames per second.
 *  - 4: 04_multi_device_example.c: Demonstrates the usage of multiple devices per MCU.
 *  - 5: 05_simple_example_debug.c: Demonstrates how to obtain debug information. */
#ifndef API_EXAMPLE
#define API_EXAMPLE 1
#endif

/*! Selector for HAL test demo:
 *  - 0: no HAL tests are executed.
 *  - 1: HAL tests are executed before any API code is executed. */
#ifndef RUN_HAL_TESTS
#define RUN_HAL_TESTS 1
#endif

/*! Selector for XTALK calibration demo:
 *  - 0: no XTALK calibration is executed.
 *  - 1: XTALK calibration is executed before any API code is executed. */
#ifndef RUN_XTALK_CALIBRATION
#define RUN_XTALK_CALIBRATION 0
#endif

/*! Define the SPI slave for device. */
#ifndef SPI_SLAVE
#define SPI_SLAVE 1
#endif

/*! Test example configuration! */
#if API_EXAMPLE < 1 || API_EXAMPLE > 5
#error The selected API_EXAMPLE is not available!
#endif

#include "board/board.h"
#include "platform/argus_print.h" // declaration of print()

/*!***************************************************************************
 * @brief   Application entry point for the specified example.
 *
 * @details The main function of the specified example, called after startup code
 *          and hardware initialization.
 *
 *          This function will never be exited!
 *****************************************************************************/
void ExampleMain(void);

/*!***************************************************************************
 * @brief   A callback function from the example code whenever an error occurs.
 *
 * @details The example code calls this function whenever an unexpected error
 *          occurs, for example, if an API function returns an error code.
 *
 *          This implementation of the function will print the error message.
 *          If specified, the program execution will be stopped with an
 *          infinite loop. Otherwise, the program will continue to run and the
 *          error is printed and ignored.
 *
 * @warning This is only a simple example implementation that does not handle
 *          errors in a production system. It is intended to demonstrate the
 *          usage of the API and to provide a starting point for custom
 *          applications.
 *
 *          This function needs to be replaced with a more sophisticated
 *          implementation to handle errors in a production system.
 *          For example, it could reset the device or try to recover from
 *          the error by re-initializing the device.
 *
 * @param   status The specified status to be checked for errors.
 * @param   stop Whether to stop the program (e.g. in case of a critical error).
 * @param   msg The associated error message to be printed in case of errors.
 *****************************************************************************/
void HandleError(status_t status, bool stop, char const * msg);

/*! @} */
#endif /* ARGUS_EXAMPLE_H */
