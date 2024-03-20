/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 SDK example application.
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

#include "argus.h"
 // additional #includes ...

#include "driver/irq.h" // declaration of IRQ_LOCK/UNLOCK()

/*!***************************************************************************
 *  Global measurement data ready event counter.
 *
 *  Determines the number of measurement data ready events happened and thus
 *  the number of timer the #Argus_EvaluateData function must be called to
 *  free API internal date structures that buffer the raw sensor readout data.
 *
 *  The #Argus_EvaluateData function must be called outside of the interrupt
 *  callback scope (i.e. from the main thread/task) to avoid huge delays due
 *  to the heavy data evaluation.
 *
 *  Note that the #Argus_EvaluateData function must be called once for each
 *  callback event since it clears the internal state of the raw data buffer.
 *  If not called, the API gets stuck waiting for the raw data buffer to be
 *  freed and ready to be filled with new measurement data.
 *
 *  In automatic measurement mode, i.e. if the measurements are automatically
 *  triggered on a time based schedule from the periodic interrupt timer (PIT),
 *  the callback may occur faster than the #Argus_EvaluateData function gets
 *  called from the main thread/task. This usually happens at high frame rates
 *  or too much CPU load on the main thread/task. In that case, the API delays
 *  new measurements until the previous buffers are cleared. Since the API
 *  contains two distinct raw data buffers, this counter raises up to 2 in the
 *  worst case scenario.
 *****************************************************************************/
static volatile uint8_t myDataReadyEvents = 0;

/*!***************************************************************************
 * @brief   Creates and initializes a new device instance.
 *
 * @param   slave The SPI slave identifier number that is passed to the S2PI
 *                layers by the API whenever it calls a function.
 *
 * @param   mode The device measurement mode to be initialized.
 *
 * @return  The pointer to the handle of the created device instance. Used to
 *          identify the calling instance in case of multiple devices.
 *****************************************************************************/
static argus_hnd_t* InitializeDevice(s2pi_slave_t slave, argus_mode_t mode)
{
    /* The API module handle that contains all data definitions that is
     * required within the API module for the corresponding hardware device.
     * Every call to an API function requires the passing of a pointer to this
     * data structure. */
    argus_hnd_t * device = Argus_CreateHandle();
    HandleError(device ? STATUS_OK : ERROR_FAIL, true, "Argus_CreateHandle failed!");

    /* Initialize the API with a specified measurement mode.
     * This implicitly calls the initialization functions
     * of the underlying API modules.
     *
     * The second parameter is stored and passed to all function calls
     * to the S2PI module. This piece of information can be utilized in
     * order to determine the addressed SPI slave and enabled the usage
     * of multiple devices on a single SPI peripheral.
     *
     * The third parameter chooses the measurement mode: see the #argus_mode_t
     * for more information on available measurement modes.
     *
     * Also note the #Argus_Init alternative that uses the dedicated default
     * measurement mode instead of the dedicated one here. But since this
     * example demonstrates the high speed modes (which are not default for
     * any device), the corresponding measurement mode parameter is passed here. */
    status_t status = Argus_InitMode(device, slave, mode);
    HandleError(status, true, "Argus_Init failed!");

    /* Setup API for High Speed mode, i.e. 1000 fps measurement rate.
     * The following changes are made:
     * - Disable the dual-frequency mode.
     * - Disable the smart-power-save mode.
     * - Set frame time to 1000 us.
     *
     * Note: The maximum frame rate is limited by the amount of data sent via UART.
     *       See #PrintResults function for more information.
     *
     * Note: To achieve even higher frame rates (up to 3 kHz), the corresponding
     *       high speed modes (see #argus_mode_t) must be chosen upon device
     *       initialization (see #Argus_Init). These modes reduce the pixel count
     *       in favor of higher measurement speed. */
    status = Argus_SetConfigurationDFMMode(device, DFM_MODE_OFF);
    HandleError(status, true, "Argus_SetConfigurationDFMMode failed!");

    status = Argus_SetConfigurationSmartPowerSaveEnabled(device, false);
    HandleError(status, true, "Argus_SetConfigurationSmartPowerSaveEnabled failed!");

    status = Argus_SetConfigurationFrameTime(device, 1000); // 0.001 second = 1000 Hz
    HandleError(status, true, "Argus_SetConfigurationFrameTime failed!");

    return device;
}

/*!***************************************************************************
 * @brief   Measurement data ready callback function.
 *
 * @param   status The measurement/device status from the last measurement cycle.
 *
 * @param   device The pointer to the handle of the calling API instance. Used to
 *                 identify the calling instance in case of multiple devices.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t MeasurementReadyCallback(status_t status, argus_hnd_t * device)
{
    (void)device; // unused in this example...

    HandleError(status, false, "Measurement Ready Callback received error!");

    /* Count the data ready events, i.e. the number of times the
     * Argus_EvaluateData function must be called from main thread.
     *
     * Note: Since only a single device is used, the device parameter
     *       can be ignored. In case of multiple device, the device
     *       parameter determines the calling device instance.
     *
     * Note: Do not call the Argus_EvaluateMeasurement method
     *       from within this callback since it is invoked in
     *       a interrupt service routine and should return as
     *       soon as possible. */
    myDataReadyEvents++;

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Prints measurement results via UART.
 *
 * @details Prints some measurement data via UART in the following format:
 *
 *          ```
 *          333; 123456
 *          ```
 *
 *          Where the values determine the following:
 *          - Time delta in microseconds to show the actual frame time.
 *          - Range in mm (converting the Q9.22 value to mm).
 *
 * @param   res A pointer to the latest measurement results structure.
 *****************************************************************************/
static void PrintResults(argus_results_t const * res)
{
    /* Static variable to store previous time stamp. */
    static ltc_t t_prev = { 0 };

    /* Print the recent measurement results:
     * 1. Time delta in microseconds to show the actual frame time.
     * 2. Range in mm (converting the Q9.22 value to mm).
     *
     * Note: To achieve 1000 fps, max. 11 bytes can be sent per frame at 115200 bps!!!
     *       115200 bps / 10 [bauds-per-byte] / 11 [bytes-per-frame] = 1047 fps
     *       Therefore not units are printed to the output! The first value is in
     *       microseconds, the second one is in millimeter.
     *
     * Note: The print formatting is expensive too. It must be reduced to a minimum by
     *       setting the #PRINTF_ADVANCED_ENABLE in the debug_console.h file to 0. */
    print("%4d;%5d\n", Time_DiffUSec(&t_prev, &res->TimeStamp), res->Bin.Range / (Q9_22_ONE / 1000));

    t_prev = res->TimeStamp;
}

/*!***************************************************************************
 * @brief   Prints information about the initialized devices.
 *
 * @param   device The pointer to the device handler.
 *****************************************************************************/
static void PrintDeviceInfo(argus_hnd_t * device)
{
    /* Print some information about current API and connected device. */
    const uint32_t value = Argus_GetAPIVersion();
    const uint8_t a = (uint8_t)((value >> 24) & 0xFFU);
    const uint8_t b = (uint8_t)((value >> 16) & 0xFFU);
    const uint8_t c = (uint8_t)(value & 0xFFFFU);
    const uint32_t id = Argus_GetChipID(device);
    const char * m = Argus_GetModuleName(device);

    print("\n##### AFBR-S50 API - High Speed Example #######################\n"
          "  API Version: v%d.%d.%d\n"
          "  Chip ID:     %d\n"
          "  Module:      %s\n"
          "###############################################################\n\n",
          a, b, c, id, m);
}

/*!***************************************************************************
 * @brief   Application entry point for the high-speed example.
 *
 * @details The main function of the advanced example, called after startup code
 *          and hardware initialization.
 *
 *          This function will never be exited!
 *****************************************************************************/
void main(void)
{
    HardwareInit(); // defined elsewhere

    status_t status = STATUS_OK;

    /* Instantiate and initialize the device handlers. */
    argus_hnd_t * device = InitializeDevice(SPI_SLAVE, ARGUS_MODE_HIGH_SPEED_SHORT_RANGE);

    /* Print a device information message. */
    PrintDeviceInfo(device);

    /* Start the measurement timers within the API module.
     * The callback is invoked every time a measurement has been finished.
     * The callback is used to schedule the data evaluation routine to the
     * main thread by the user.
     * Note that the timer based measurement is not implemented for multiple
     * instance yet! */
    status = Argus_StartMeasurementTimer(device, &MeasurementReadyCallback);
    HandleError(status, true, "Argus_StartMeasurementTimer failed!");

    /* The program loop ... */
    for (;;)
    {
        /* Check if new measurement data is ready. */
        if (myDataReadyEvents)
        {
            IRQ_LOCK();
            myDataReadyEvents--;
            IRQ_UNLOCK();

            /* The measurement data structure. */
            argus_results_t res;

            /* Evaluate the raw measurement results. */
            status = Argus_EvaluateData(device, &res);
            HandleError(status, false, "Argus_EvaluateData failed!");

            /* Use the obtain results, e.g. print via UART. */
            PrintResults(&res);
        }
    }
}