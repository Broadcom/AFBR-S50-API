/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Demo Application
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


/*!***************************************************************************
 * @addtogroup  can_app
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "main.h"
#include "uart_api.h"
#include "can_api.h"
#include "can_app_version.h"

#include "board/board.h"
#include "driver/irq.h"
#include "driver/bsp.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 1

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 6000000


/*******************************************************************************
 * Variables
 ******************************************************************************/


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


/*! The AFBR-S50 Data Handle. */
argus_hnd_t * hnd = 0;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*!***************************************************************************
 * @brief   Prints a help string to the serial interface.
 *****************************************************************************/
static void print_help(void);

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
status_t measurement_ready_callback(status_t status, argus_hnd_t * device);


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Application entry point.
 *
 * @details The main function of the program, called after startup code
 *          This function should never be exited.
 *
 * @return  Function never returns...
 *****************************************************************************/
int main(void)
{
    status_t status = STATUS_OK;

    /* Initialize the platform hardware including the required peripherals
     * for the API. */
    Board_Init();

    /* Initialize the CAN hardware. */
    CAN_Init();

    /* Initialize the UART API. */
    UART_API_Init();

    /* The API module handle that contains all data definitions that is
     * required within the API module for the corresponding hardware device.
     * Every call to an API function requires the passing of a pointer to this
     * data structure. */
    hnd = Argus_CreateHandle();
    handle_error(hnd ? STATUS_OK : ERROR_FAIL, "Argus_CreateHandle failed!");


    /* Initialize the API with default values.
     * This implicitly calls the initialization functions
     * of the underlying API modules.
     *
     * The second parameter is stored and passed to all function calls
     * to the S2PI module. This piece of information can be utilized in
     * order to determine the addressed SPI slave and enabled the usage
     * of multiple devices on a single SPI peripheral. */
    status = Argus_Init(hnd, SPI_SLAVE);
    handle_error(status, "Argus_Init failed!");


    /* Print some information about current API and connected device. */
    uint32_t value = Argus_GetAPIVersion();
    uint8_t a = (uint8_t)((value >> 24) & 0xFFU);
    uint8_t b = (uint8_t)((value >> 16) & 0xFFU);
    uint16_t c = (uint16_t)(value & 0xFFFFU);
    uint32_t id = Argus_GetChipID(hnd);
    const char * module = Argus_GetModuleName(hnd);

    print("\n##### AFBR-S50 API - CAN Application #############\n"
          "  APP Version: v%d.%d.%d\n"
          "  API Version: v%d.%d.%d\n"
          "  Chip ID:     %d\n"
          "  Module:      %s\n"
          "##################################################\n",
          CAN_APP_VERSION_MAJOR, CAN_APP_VERSION_MINOR, CAN_APP_VERSION_BUGFIX,
          a, b, c, id, module);

    print_help();


    /* Adjust some configuration parameters by invoking the dedicated API methods.
     * Note: The maximum frame rate is limited by the amount of data sent via UART.
     *       See #print_results function for more information. */
    status = Argus_SetConfigurationFrameTime( hnd, 100000 ); // 0.1 second = 10 Hz
    handle_error(status, "Argus_SetConfigurationFrameTime failed!");


    /* The program loop ... */
    for (;;)
    {
        /* Check for incoming CAN commands and handle them. */
        CAN_HandleCommand();
        /* Check for incoming UART commands and handle the m. */
        UART_HandleCommand();

        /* Check if new measurement data is ready. */
        if (myDataReadyEvents)
        {
            IRQ_LOCK();
            myDataReadyEvents--;
            IRQ_UNLOCK();

            /* The measurement data structure. */
            argus_results_t res;

            /* Evaluate the raw measurement results. */
            status = Argus_EvaluateData(hnd, &res);
            handle_error(status, "Argus_EvaluateData failed!");

            /* Sending measurement data via serial (UART) interface. */
            UART_Send1D(&res);

            /* Sending measurement data via CAN */
            CAN_Transmit1D(&res);
        }
    }
}


void start_measurements(void)
{
    /* Start the measurement timers within the API module.
     * The callback is invoked every time a measurement has been finished.
     * The callback is used to schedule the data evaluation routine to the
     * main thread by the user.
     * Note that the timer based measurement is not implemented for multiple
     * instance yet! */
    status_t status = Argus_StartMeasurementTimer(hnd, measurement_ready_callback);
    handle_error(status, "Argus_StartMeasurementTimer failed!");
    print("Successfully started measurements\n");
}

void stop_measurements(void)
{
    status_t status = Argus_StopMeasurementTimer(hnd);
    handle_error(status, "Argus_StopMeasurementTimer failed!");
    print("Successfully stopped measurements\n");
    print_help();
}

void print_help(void)
{
    print("\n"
          "# AFBR-S50 CAN usage:\n"
          "# > Start measurements by sending a CAN Remote Frame with ID: 8\n"
          "# > Stop measurements by sending a CAN Remote Frame with ID: 9\n"
          "# > Data is sent via CAN data Frame with ID 28\n"
          "\n"
          "# AFBR-S50 UART usage:\n"
          "# > Start measurements by typing 's'\n"
          "# > Stop measurements by typing 'p'\n\n");
}

void handle_error(status_t status, char const * msg)
{
    /* Check for status < 0 and print message and halt the program execution. */
    if (status < STATUS_OK)
    {
        LED_STATUS_ON();
        print("ERROR: %s\nError Code: %d\n", msg, status);
        while (1) __asm("nop"); // stop!
    }
}


status_t measurement_ready_callback(status_t status, argus_hnd_t * device)
{
    (void) device; // currently unused...

    handle_error(status, "Measurement Ready Callback received error!");

    /* Count the events... */
    myDataReadyEvents++;

    return STATUS_OK;
}

/*! @} */
