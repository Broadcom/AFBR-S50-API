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
#include "main.h"

#if RUN_XTALK_CALIBRATION
#include "argus_xtalk_cal_cli.h"
#endif

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
    status = Argus_SetConfigurationFrameTime(device, 10000); // 0.01 second = 100 Hz
    HandleError(status, true, "Argus_SetConfigurationFrameTime failed!");

    /* Turn-off DFM to allow the DCA to reduce the number of frames required to adapt its settings.
     * Make sure that your application allows the reduced unambiguous range!*/
    status = Argus_SetConfigurationDFMMode(device, DFM_MODE_OFF);
    HandleError(status, true, "Argus_SetConfigurationDFMMode failed!");

    return device;
}

/*!***************************************************************************
 * @brief   Triggers a measurement cycle in blocking manner.
 *
 * @param   device The pointer to the handle of the calling API instance. Used to
 *                identify the calling instance in case of multiple devices.
 *
 * @param   res The pointer to the results data structure where the final
 *              measurement results are stored.
 *****************************************************************************/
static void TriggerMeasurementBlocking(argus_hnd_t * device, argus_results_t * res)
{
    status_t status = STATUS_OK;

    /* Triggers a single measurement.
     *
     * Note that due to the laser safety algorithms, the method might refuse
     * to restart a measurement when the appropriate time has not been elapsed
     * right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
     * the function must be called again later. Use the frame time configuration
     * in order to adjust the timing between two measurement frames.
     *
     * The callback can be null for the trigger function if the #Argus_GetStatus
     * function is used to await the measurement cycle to finish. Otherwise, the
     * callback should be set to receive the measurement ready event. See the
     * advanced example on how to use the callback. */
    do
    {
        status = Argus_TriggerMeasurement(device, 0);
    } while (status == STATUS_ARGUS_POWERLIMIT);
    HandleError(status, false, "Argus_TriggerMeasurement failed!");

    /* Wait until measurement data is ready by polling the #Argus_GetStatus
     * function until the status is not #STATUS_BUSY any more. Note that
     * the actual measurement is performed asynchronously in the background
     * (i.e. on the device, in DMA transfers and in interrupt service routines).
     * Thus, one could do more useful stuff while waiting here... */
    do
    {
        status = Argus_GetStatus(device);
    }
    while (status == STATUS_BUSY);
    HandleError(status, false, "Waiting for measurement data ready (Argus_GetStatus) failed!");

    /* Evaluate the raw measurement results by calling the #Argus_EvaluateData function. */
    status = Argus_EvaluateData(device, res);
    HandleError(status, false, "Argus_EvaluateData failed!");
}

/*!***************************************************************************
 * @brief   Prints measurement results via UART.
 *
 * @details Prints some measurement data via UART in the following format:
 *
 *          ```
 *          123.456789 s; Range: 123456 mm;  Amplitude: 1234 LSB; Quality: 100;  Status: 0
 *          ```
 *
 * @param   res A pointer to the latest measurement results structure.
 *****************************************************************************/
static void PrintResults(argus_results_t const * res, uint8_t num)
{
    /* Print the recent measurement results:
     * 1. Time stamp in seconds since the last MCU reset.
     * 2. Range in mm (converting the Q9.22 value to mm).
     * 3. Amplitude in LSB (converting the UQ12.4 value to LSB).
     * 4. Status (0: OK, <0: Error, >0: Warning.
     *
     *  - DCA - Integration energy parameters -
     * 5. Output Power stage
     * 6. Analog Integration
     * 7. Digital Integration
     * 8. Pixel gain
     *
     * Note1: The digital integration (Ddepth) in crucial when it comes to power consumption. Higher values
     *        are in direct relation to a higher current consumption of the sensor. Ddepth
     *        can be controlled by the power saving ratio (PSR) parameter to the costs of precision.
     *
     * Note2: Sending data via UART creates a large delay which might prevent
     *       the API from reaching the full frame rate. This example sends
     *       approximately 80 characters per frame at 115200 bps which limits
     *       the max. frame rate of 144 fps:
     *       115200 bps / 10 [bauds-per-byte] / 80 [bytes-per-frame] = 144 fps */
    print("%d; %4d.%3d s; Range: %5d mm;  Amplitude: %4d LSB; Status: %d; Pow: %d mA;  Adepth: %d;  Ddepth: %d;  Gain: %d LSB\n",
          num,
    	  res->TimeStamp.sec,
          res->TimeStamp.usec,
          res->Bin.Range / (Q9_22_ONE / 1000),
          res->Bin.Amplitude / UQ12_4_ONE,
          res->Status,
		  res->Frame.OutputPower / UQ12_4_ONE,
		  res->Frame.AnalogIntegrationDepth / UQ10_6_ONE,
		  res->Frame.DigitalIntegrationDepth,
		  res->Frame.PixelGain);
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

    print("\n##### AFBR-S50 API - Simple Example ###########################\n"
          "  API Version: v%d.%d.%d\n"
          "  Chip ID:     %d\n"
          "  Module:      %s\n"
          "###############################################################\n\n",
          a, b, c, id, m);
}

/*!***************************************************************************
 * @brief   Invokes a wake-up sequence
 *
 * @details A Wake-up is basically obtained by switching-on VDD via a connected GPIO
 * 			which is controlled by a HAL function.
 *
 *			This function is optional and requires additional hardware to realize
 *			a VDD switch!
 *
 * @param   device The pointer to the device handler.
 *****************************************************************************/
static void WakeUp(argus_hnd_t * device)
{
  	status_t status;
  	/* Invoke wake-up
  	 * STM32 HAL specific function - For other vendors, please check the respective MCU manual or library
  	 * Make sure your implementation requires a low state for a wake-up, if not change to set!
  	 */
  	HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_RESET);
  	/* Wait for Power-on-reset in the ToF ASIC
  	 * STM32 HAL specific function - For other vendors, please check the respective MCU manual or library */
  	HAL_Delay(1);

  	do{
  	  	/*Revert GPIO pin control for SPI mode */
  		S2PI_ReleaseGpioControl(SPI_SLAVE);
  	  	/*This function invalidates and writes preserved register settings used before the last sleep*/
  		status = Argus_RestoreDeviceState(device);
  	  	HandleError(status, false, "Restoring device state failed!");
  	}while(status < ERROR_ARGUS_STALLED);
}

/*!***************************************************************************
 * @brief   Invokes a device sleep
 *
 * @details Sleep is basically obtained by switching-off VDD via a connected GPIO
 * 			which is controlled by a HAL function.
 *
 *			This function is optional and requires additional hardware to realize
 *			a VDD switch!
 *
 * @param   device The pointer to the device handler.
 *****************************************************************************/
static void Sleep(argus_hnd_t * device)
{
  	/*Set SPI GPIO pins to low before going to sleep*/
  	/*Set S2PI pins to GPIO mode*/
	S2PI_CaptureGpioControl(SPI_SLAVE);

	/*Pull all SPI signals to low
	 *STM32 HAL specific function - For other vendors, please check the respective MCU manual or library*/
	S2PI_WriteGpioPin(SPI_SLAVE, S2PI_MOSI, 0);
	S2PI_WriteGpioPin(SPI_SLAVE, S2PI_MISO, 0);
	S2PI_WriteGpioPin(SPI_SLAVE, S2PI_CLK, 0);
	S2PI_WriteGpioPin(SPI_SLAVE, S2PI_CS, 0);
	S2PI_WriteGpioPin(SPI_SLAVE, S2PI_IRQ, 0);

  	/* Invoke sleep
  	 * STM32 HAL specific function - For other vendors, please check the respective MCU manual or library
  	 * Make sure your implementation requires a high state for a sleep, if not change to reset!
  	 */
  	HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_SET);
}

/*!***************************************************************************
 * @brief   Application entry point for the simple example.
 *
 * @details The main function of the simple example, called after startup code
 *          and hardware initialization.
 *
 *          This function will never be exited!
 *****************************************************************************/
void main(void)
{
    HardwareInit(); // defined elsewhere

  /* Instantiate and initialize the device handlers. */
  argus_hnd_t * device = InitializeDevice(SPI_SLAVE);

  /* Print a device information message. */
  PrintDeviceInfo(device);

#if RUN_XTALK_CALIBRATION
  /* Enter the CLI to perform a xtalk calibration interactively.
   * It guides through all steps needed to compensate electrical
   * as well as optical xtalk caused by an application design. */
  Argus_XtalkCalibration_CLI(device);
#endif // RUN_XTALK_CALIBRATION

  /* The program loop ... */
  for (;;)
  {
	/* Change the number of measurements to see the direct impact on current consumption.
	 * When ambient conditions may change it is suggested to allow at least 3 x measurements (:= frames).*/
	uint8_t no_of_measurements = 3;
    argus_results_t res_arr[no_of_measurements];

  	for (int n = 0; n < no_of_measurements; n++)
  	{
          /* The measurement data structure. */
          argus_results_t res;

          /* Trigger a measurement for the current device. */
          TriggerMeasurementBlocking(device, &res);

          /* Use the obtain results, e.g. print via UART. */
//          PrintResults(&res);
          res_arr[n] = res;
  	}

  	Sleep(device);

  	//post-processing .... e.g. print results on Terminal
  	for (int m=0;m<no_of_measurements;m++)
  	{
  		/*do something..e.g.*/
  		PrintResults(&res_arr[m],m+1);
  	}

  	/* delay in ms -> This defines duty cycle and average current consumption
  	 * STM32 HAL specific function - For other vendors, please check the respective MCU manual or library*/
  	HAL_Delay(500);

  	WakeUp(device);
  }
}