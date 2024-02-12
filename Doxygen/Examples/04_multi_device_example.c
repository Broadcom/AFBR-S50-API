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
#include "board/board_config.h"    // declaration of S2PI slaves

#include "driver/irq.h" // declaration of IRQ_LOCK/UNLOCK()

/*! The number of devices connected/used in the example. Range: 1 to 4. */
#ifndef DEVICE_COUNT
#define DEVICE_COUNT 4
#endif

/*! Define the SPI slave for device. */
#ifndef SPI_SLAVE1
#define SPI_SLAVE1 (SPI_SLAVE)
#endif
/*! Define the SPI slave for device. */
#ifndef SPI_SLAVE2
#define SPI_SLAVE2 (SPI_SLAVE1 + 1)
#endif
/*! Define the SPI slave for device. */
#ifndef SPI_SLAVE3
#define SPI_SLAVE3 (SPI_SLAVE2 + 1)
#endif
/*! Define the SPI slave for device. */
#ifndef SPI_SLAVE4
#define SPI_SLAVE4 (SPI_SLAVE3 + 1)
#endif

/*! Size of the date ready event queue. Max. 2 values per device. */
#define FIFO_SIZE 8

/*! FIFO (First-In-First-Out) Data Buffer Structure. */
typedef struct fifo_t
{
    /*! Data buffer. */
    argus_hnd_t * Buffer[FIFO_SIZE];

    /*! Data write pointer */
    argus_hnd_t ** volatile WrPtr;

    /*! Data read pointer */
    argus_hnd_t ** volatile  RdPtr;

    /*! Current number of data in buffer. */
    size_t volatile Load;

} fifo_t;

/*! The date ready event queue for storing data ready events from the API. */
static fifo_t DataReadyEventQueue = { 0 };

/*!***************************************************************************
 * @brief   Creates and initializes a new device instance.
 *
 * @param   slave The SPI slave identifier number that is passed to the S2PI
 *                layers by the API whenever it calls a function.
 *
 * @return  The pointer to the handle of the created device instance. Used to
 *          identify the calling instance in case of multiple devices.
 *****************************************************************************/
static argus_hnd_t* InitializeDevice(s2pi_slave_t slave)
{
    /* The API module handle that contains all data definitions that is
     * required within the API module for the corresponding hardware device.
     * Every call to an API function requires the passing of a pointer to this
     * data structure. */
    argus_hnd_t * device = Argus_CreateHandle();
    HandleError(device ? STATUS_OK : ERROR_FAIL, true, "Argus_CreateHandle failed!");

    /* Initialize the API with the dedicated default measurement mode.
     * This implicitly calls the initialization functions
     * of the underlying API modules.
     *
     * The second parameter is stored and passed to all function calls
     * to the S2PI module. This piece of information can be utilized in
     * order to determine the addressed SPI slave and enabled the usage
     * of multiple devices on a single SPI peripheral.
     *
     * Also note the #Argus_InitMode alternative that uses a third
     * parameter to choose the measurement mode: see the #argus_mode_t
     * enumeration for more information on available measurement modes. */
    status_t status = Argus_Init(device, slave);
    HandleError(status, true, "Argus_Init failed!");

    /* Adjust additional configuration parameters by invoking the dedicated API methods.
     * Note: The maximum frame rate is limited by the amount of data sent via UART.
     *       See #PrintResults function for more information. */
    status = Argus_SetConfigurationFrameTime(device, 100000); // 0.1 second = 10 Hz
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

    /* Queue the data ready events, i.e. the device handle pointer,
     * to inform the main thread to call the Argus_EvaluateData function.
     *
     * Note: Since multiple devices are used, the device parameter
     *       can NOT be ignored. The device parameter determines
     *       the calling device instance and will be stored in a
     *       event queue in the order of measurement ready events.
     *
     * Note: Do not call the Argus_EvaluateMeasurement method
     *       from within this callback since it is invoked in
     *       a interrupt service routine and should return as
     *       soon as possible. */

    assert(DataReadyEventQueue.Load < FIFO_SIZE);
    *DataReadyEventQueue.WrPtr = device;
    DataReadyEventQueue.WrPtr++;
    if (DataReadyEventQueue.WrPtr >= DataReadyEventQueue.Buffer + FIFO_SIZE)
        DataReadyEventQueue.WrPtr = DataReadyEventQueue.Buffer;
    DataReadyEventQueue.Load++;

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Prints measurement results via UART.
 *
 * @details Prints some measurement data via UART in the following format:
 *
 *          ```
 *          #1: 123.456789 s; Range: 123456 mm;  Amplitude: 1234 LSB; Quality: 100;  Status: 0
 *          ```
 *
 * @param   res A pointer to the latest measurement results structure.
 * @param   id The identifier number to prepend to the print statement.
 *****************************************************************************/
static void PrintResults(argus_results_t const * res, uint32_t id)
{
    /* Print the recent measurement results:
     * 0. Device ID
     * 1. Time stamp in seconds since the last MCU reset.
     * 2. Range in mm (converting the Q9.22 value to mm).
     * 3. Amplitude in LSB (converting the UQ12.4 value to LSB).
     * 4. Signal Quality in % (100% = good signal).
     * 5. Status (0: OK, <0: Error, >0: Warning.
     *
     * Note: Sending data via UART creates a large delay which might prevent
     *       the API from reaching the full frame rate. This example sends
     *       approximately 80 characters per frame at 115200 bps which limits
     *       the max. frame rate of 144 fps:
     *       115200 bps / 10 [bauds-per-byte] / 80 [bytes-per-frame] = 144 fps */
    print("#%06d:  %4d.%06d s; Range: %5d mm;  Amplitude: %4d LSB;  Quality: %3d;  Status: %d\n",
          id,
          res->TimeStamp.sec,
          res->TimeStamp.usec,
          res->Bin.Range / (Q9_22_ONE / 1000),
          res->Bin.Amplitude / UQ12_4_ONE,
          res->Bin.SignalQuality,
          res->Status);
}

/*!***************************************************************************
 * @brief   Prints information about the initialized devices.
 *
 * @details Prints information about the initialized devices.
 *
 * @param   device_count The number of connected devices.
 * @param   devices The array to the device handlers.
 *****************************************************************************/
static void PrintDeviceInfo(const uint8_t device_count, argus_hnd_t * devices[static device_count])
{
    /* Print some information about current API and connected device. */
    const uint32_t value = Argus_GetAPIVersion();
    const uint8_t a = (uint8_t)((value >> 24) & 0xFFU);
    const uint8_t b = (uint8_t)((value >> 16) & 0xFFU);
    const uint8_t c = (uint8_t)(value & 0xFFFFU);

    print("\n##### AFBR-S50 API - Multi-Device Example #####################\n"
          "  API Version: v%d.%d.%d\n", a, b, c);
    for (uint8_t d = 0; d < device_count; d++)
    {
        const uint32_t id = Argus_GetChipID(devices[d]);
        const char * m = Argus_GetModuleName(devices[d]);
        print("  Chip ID #%d:  %d\n"
              "  Module  #%d:  %s\n",
              d + 1, id, d + 1, m);
    }
    print("###############################################################\n\n");
}

/*!***************************************************************************
 * @brief   Application entry point for the multi-device example.
 *
 * @details The main function of the simple example, called after startup code
 *          and hardware initialization.
 *
 *          This function will never be exited!
 *****************************************************************************/
void main(void)
{
    HardwareInit(); // defined elsewhere

    status_t status = STATUS_OK;
    const s2pi_slave_t slaves[] = { SPI_SLAVE1, SPI_SLAVE2, SPI_SLAVE3, SPI_SLAVE4 };

    /* Initialize event queue */
    DataReadyEventQueue.WrPtr = DataReadyEventQueue.Buffer;
    DataReadyEventQueue.RdPtr = DataReadyEventQueue.Buffer;
    DataReadyEventQueue.Load = 0;

    /* Instantiate and initialize the device handlers. */
    argus_hnd_t * devices[DEVICE_COUNT] = { 0 };

    for (uint8_t d = 0; d < DEVICE_COUNT; d++)
    {
        devices[d] = InitializeDevice(slaves[d]);
    }

    /* Print a device information message. */
    PrintDeviceInfo(sizeof(devices) / sizeof(devices[0]), devices);

    /* Start the measurement timers within the API module.
     * The callback is invoked every time a measurement has been finished.
     * The callback is used to schedule the data evaluation routine to the
     * main thread by the user.
     * Note that the timer based measurement is not implemented for multiple
     * instance yet! */
    for (uint8_t d = 0; d < DEVICE_COUNT; d++)
    {
        status = Argus_StartMeasurementTimer(devices[d], &MeasurementReadyCallback);
        HandleError(status, true, "Argus_StartMeasurementTimer failed!");
    }

    /* The program loop ... */
    for (;;)
    {
        /* Check if new measurement data is ready. */
        if (DataReadyEventQueue.Load > 0)
        {
            IRQ_LOCK();
            argus_hnd_t * device = *DataReadyEventQueue.RdPtr;
            DataReadyEventQueue.Load--;
            DataReadyEventQueue.RdPtr++;
            if (DataReadyEventQueue.RdPtr >= DataReadyEventQueue.Buffer + FIFO_SIZE)
                DataReadyEventQueue.RdPtr = DataReadyEventQueue.Buffer;
            IRQ_UNLOCK();

            /* The measurement data structure. */
            argus_results_t res;

            /* Evaluate the raw measurement results. */
            status = Argus_EvaluateData(device, &res);
            HandleError(status, false, "Argus_EvaluateData failed!");

            /* Use the obtain results, e.g. print via UART. */
            uint32_t id = Argus_GetChipID(device);
            PrintResults(&res, id);
        }
    }
}