/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 SDK example application.
 *
 * @copyright
 * 
 * Copyright (c) 2021, Broadcom Inc
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
#include "argus.h"

#include "board/clock_config.h"
#include "driver/irq.h"
#include "driver/cop.h"
#include "driver/s2pi.h"
#include "driver/uart.h"
#include "driver/timer.h"

#include "test/argus_hal_test.h"

#if defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL17Z256VFM4)
#elif defined(STM32F401xE)
#include "main.h"
#else
#error No target specified!
#endif


/*******************************************************************************
 * Defines
 ******************************************************************************/

/*! Selector for simple/advanced demo:
 *  - 0: measurements are triggered in synchronously from the main thread.
 *  - 1: measurements are triggered asynchronously from the background. */
#ifndef ADVANCED_DEMO
#define ADVANCED_DEMO 1
#endif

/*! Selector for default/high-speed demo:
 *  - 0: defaul demo at lower speed including dual-frequency mode enabled.
 *  - 1: high-speed demo with 1000 fps and disabled dual-frequency mode.
 *  Note that ADVANCE_MODE must be enabled to achieve full measurement speed.
 *  Note that 12Mbps SPI Baud Rate must be set for KL46z; KL17z does not achieve full speed! */
#ifndef HIGH_SPEED_DEMO
#define HIGH_SPEED_DEMO 0
#endif

/*! Selector for HAL test demo:
 *  - 0: no HAL tests are executed.
 *  - 1: HAL tests are executed before any API code is executed. */
#ifndef RUN_HAL_TESTS
#define RUN_HAL_TESTS 1
#endif

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 1

/*! Define the SPI baud rate (to be used in the SPI module). */
#if HIGH_SPEED_DEMO
#define SPI_BAUD_RATE 12000000
#else
#define SPI_BAUD_RATE 6000000
#endif

#if HIGH_SPEED_DEMO && !ADVANCED_DEMO
#error HIGH_SPEED_DEMO can only be enabled w/ ADVANCED_DEMO enabled too!
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if ADVANCED_DEMO
/*!***************************************************************************
 *  Global raw data pointer variables for ADVANCED_MODE.
 *
 *  Stores the raw data pointer obtained from the API measurement data ready
 *  callback for passing it to the #Argus_EvaluateData function outside of the
 *  interrupt callback scope (i.e. from the main thread/task). Note that the
 *  #Argus_EvaluateData function must be called once for each callback event
 *  since it clears the internal state of the raw data buffer. If not called,
 *  the API gets stuck waiting for the raw data buffer to be freed and ready
 *  to be filled with new measurement data.
 *
 *  In automatic measurement mode (#ADVANCED_DEMO = 1), i.e. if the measurements
 *  are automatically triggered on a time based schedule from the periodic
 *  interrupt timer (PIT), the callback may occur faster than the
 *  #Argus_EvaluateData function gets called from the main thread/task. This
 *  usually happens at high frame rates or too much CPU load on the main
 *  thread/task. In that case, the API delays new measurements until the
 *  previous buffers are cleared. Since the API contains two distinct raw
 *  data buffer, the user code must store two pointers in worst case scenario.
 *****************************************************************************/
static volatile void * myRawDataPtr[2] = { 0 };
#else

/*!***************************************************************************
 *  Global raw data pointer variable for SIMPLE_MODE.
 *
 *  Stores the raw data pointer obtained from the API measurement data ready
 *  callback for passing it to the #Argus_EvaluateData function outside of the
 *  interrupt callback scope (i.e. from the main thread/task). Note that the
 *  #Argus_EvaluateData function must be called once for each callback event
 *  since it clears the internal state of the raw data buffer. If not called,
 *  the API gets stuck waiting for the raw data buffer to be freed and ready
 *  to be filled with new measurement data.
 *
 *  In simple measurement mode (#ADVANCED_DEMO = 0), i.e. if the measurements
 *  are manually triggered via calls to #Argus_TriggerMeasurement from the
 *  main thread/task, the callback occurs exactly once for each successful
 *  call to the trigger function. Thus the #Argus_EvaluateData function needs
 *  to be called once for each trigger command accordingly.
 *****************************************************************************/
static volatile void * myRawDataPtr = 0;
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief	printf-like function to send print messages via UART.
 *
 * @details Defined in "driver/uart.c" source file.
 *
 * 			Open an UART connection with 115200 bps, 8N1, no handshake to
 * 			receive the data on a computer.
 *
 * @param	fmt_s The usual printf parameters.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
extern status_t print(const char  *fmt_s, ...);

/*!***************************************************************************
 * @brief	Prints measurement results via UART.
 *
 * @details Prints some measurement data via UART in the following format:
 *
 * 			Range: 123456 mm;  Amplitude: 1234 LSB  Status: 0
 *
 * @param	res A pointer to the latest measurement results structure.
 *****************************************************************************/
static void print_results(argus_results_t const * res);

/*!***************************************************************************
 * @brief	Initialization routine for board hardware and peripherals.
 *****************************************************************************/
static void hardware_init(void);

/*!***************************************************************************
 * @brief	Measurement data ready callback function.
 *
 * @param	status The measurement/device status from the last measurement cycle.
 * @param	data A pointer to the raw measurement data results that need to be
 * 				 passed to the #Argus_EvaluateData function.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t measurement_ready_callback(status_t status, void * data);

/*!***************************************************************************
 * @brief	A very brief example for error handling.
 *
 * @details	Checks the specified status for errors (i.e. negative values) and
 * 			prints a specified error message if any. An endless loop is entered
 * 			to halt program execution.
 *
 * @param	status The specified status to be checked for errors.
 * @param	msg The associated error message to be printed in case of errors.
 *****************************************************************************/
static void handle_error(status_t status, char const * msg);


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!***************************************************************************
 * @brief	Application entry point.
 *
 * @details	The main function of the program, called after startup code
 * 			This function should never be exited.
 *
 * @return	Never returns anything...
 *****************************************************************************/
int main(void)
{
	status_t status = STATUS_OK;

	/* Initialize the platform hardware including the required peripherals
	 * for the API. */
	hardware_init();


#if RUN_HAL_TESTS
	/* Running a sequence of test in order to verify the HAL implementation. */
	status = Argus_VerifyHALImplementation(SPI_SLAVE);
	handle_error(status, "HAL Implementation verification failed!");
#endif


	/* The API module handle that contains all data definitions that is
	 * required within the API module for the corresponding hardware device.
	 * Every call to an API function requires the passing of a pointer to this
	 * data structure. */
	argus_hnd_t * hnd = Argus_CreateHandle();
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
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	uint32_t id = Argus_GetChipID(hnd);
	argus_module_version_t mv = Argus_GetModuleVersion(hnd);

#if ADVANCED_DEMO && HIGH_SPEED_DEMO
	print("\n##### AFBR-S50 API - High Speed Example ##########\n"
#elif ADVANCED_DEMO
	print("\n##### AFBR-S50 API - Advanced Example ############\n"
#else
	print("\n##### AFBR-S50 API - Simple Example ##############\n"
#endif
		  "  API Version: v%d.%d.%d\n"
		  "  Chip ID:     %d\n"
		  "  Module:      %s\n"
		  "##################################################\n",
		  a, b, c, id,
		  mv == AFBR_S50MV85G_V1 ? "AFBR-S50MV85G (v1)" :
		  mv == AFBR_S50MV85G_V2 ? "AFBR-S50MV85G (v2)" :
		  mv == AFBR_S50MV85G_V3 ? "AFBR-S50MV85G (v3)" :
		  mv == AFBR_S50LV85D_V1 ? "AFBR-S50LV85D (v1)" :
		  mv == AFBR_S50MV68B_V1 ? "AFBR-S50MV68B (v1)" :
		  mv == AFBR_S50MV85I_V1 ? "AFBR-S50MV85I (v1)" :
		  mv == AFBR_S50SV85K_V1 ? "AFBR-S50SV85K (v1)" :
		  "unknown");


	/* Choose the measurement mode via API functions:
	 * - ARGUS_MODE_A: Long Range Measurement Mode
	 * - ARGUS_MODE_B: Short Range Measurement Mode */
	argus_mode_t mode = ARGUS_MODE_B;
	status = Argus_SetConfigurationMeasurementMode(hnd, mode);
	handle_error(status, "Argus_SetConfigurationMeasurementMode failed!");

#if HIGH_SPEED_DEMO

	/* Setup API for High Speed mode, i.e. 1000 fps measurement rate.
	 * The following changes are made:
	 * - Disable the dual-frequency mode.
	 * - Disable the smart-power-save mode.
	 * - Set frame time to 1000 us.
	 *
	 * Note: The maximum frame rate is limited by the amount of data sent via UART.
	 *       See #print_results function for more information. */

	status = Argus_SetConfigurationDFMMode(hnd, mode, DFM_MODE_OFF);
	handle_error(status, "Argus_SetConfigurationDFMMode failed!");

	status = Argus_SetConfigurationSmartPowerSaveEnabled(hnd, mode, false);
	handle_error(status, "Argus_SetConfigurationSmartPowerSaveEnabled failed!");

	status = Argus_SetConfigurationFrameTime(hnd, 1000); // 0.001 second = 1000 Hz
	handle_error(status, "Argus_SetConfigurationFrameTime failed!");

#else

	/* Adjust additional configuration parameters by invoking the dedicated API methods.
	 * Note: The maximum frame rate is limited by the amount of data sent via UART.
	 *       See #print_results function for more information. */
	status = Argus_SetConfigurationFrameTime(hnd, 100000); // 0.1 second = 10 Hz
	handle_error(status, "Argus_SetConfigurationFrameTime failed!");

#endif



#if ADVANCED_DEMO

	/* Start the measurement timers within the API module.
	 * The callback is invoked every time a measurement has been finished.
	 * The callback is used to schedule the data evaluation routine to the
	 * main thread by the user.
	 * Note that the timer based measurement is not implemented for multiple
	 * instance yet! */
	status = Argus_StartMeasurementTimer(hnd, measurement_ready_callback);
	handle_error(status, "Argus_StartMeasurementTimer failed!");


	/* The program loop ... */
	for(;;)
	{
		/* Disable IRQs to avoid update of the myRawDataPtr array between distinct read commands. */
		IRQ_LOCK();
		void * dataPtr = (void*) myRawDataPtr[0];
		myRawDataPtr[0] = myRawDataPtr[1];
		myRawDataPtr[1] = 0;
		IRQ_UNLOCK();

		/* Check if new measurement data is ready. */
		if (dataPtr != 0)
		{
			/* The measurement data structure. */
			argus_results_t res;

			/* Evaluate the raw measurement results. */
		    status = Argus_EvaluateData(hnd, &res, dataPtr);
		    handle_error(status, "Argus_EvaluateData failed!");

			/* Use the obtain results, e.g. print via UART. */
			print_results(&res);
		}
	}

#else

	/* The program loop ... */
	for (;;)
	{
		/* Triggers a single measurement.
		 * Note that due to the laser safety algorithms, the method might refuse
		 * to restart a measurement when the appropriate time has not been elapsed
		 * right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
		 * the function must be called again later. Use the frame time configuration
		 * in order to adjust the timing between two measurement frames. */
		status = Argus_TriggerMeasurement(hnd, measurement_ready_callback);
		handle_error(status, "Argus_StartMeasurementTimer failed!");

		if (status == STATUS_ARGUS_POWERLIMIT)
		{
			/* Not ready (due to laser safety) to restart the measurement yet.
			 * Come back later. */
			continue;
		}
		else
		{
			/* Wait until measurement data is ready. */
			do
			{
				status = Argus_GetStatus(hnd);
			}
			while (status == STATUS_BUSY);
			handle_error(status, "Waiting for measurement data ready (Argus_GetStatus) failed!");

			/* The measurement data structure. */
			argus_results_t res;

			/* Evaluate the raw measurement results. */
			status = Argus_EvaluateData(hnd, &res, (void*) myRawDataPtr);
			handle_error(status, "Argus_EvaluateData failed!");

			myRawDataPtr = 0;

			/* Use the obtain results, e.g. print via UART. */
			print_results(&res);
		}
	}

#endif
}


static void print_results(argus_results_t const * res)
{
#if !HIGH_SPEED_DEMO

	/* Print the recent measurement results:
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
	print("%4d.%06d s; Range: %5d mm;  Amplitude: %4d LSB;  Quality: %3d;  Status: %d\n",
		  res->TimeStamp.sec,
		  res->TimeStamp.usec,
		  res->Bin.Range / (Q9_22_ONE / 1000),
		  res->Bin.Amplitude / UQ12_4_ONE,
		  res->Bin.SignalQuality,
		  res->Status);

#else
	/* Static variable to store previous time stamp. */
	static ltc_t t_prev = { 0 };

	/* Print the recent measurement results:
	 * 1. Time delta in microsconds to show the actual frame time.
	 * 2. Range in mm (converting the Q9.22 value to mm).
	 *
	 * Note: To achieve 1000 fps, max. 11 bytes can be sent per frame at 115200 bps!!!
	 *       115200 bps / 10 [bauds-per-byte] / 11 [bytes-per-frame] = 1047 fps
	 *       Therefore not units are printed to the output! The first value is in
	 *       microseconds, the second one is in millimeter.
	 *
	 * Note: The print formatting is expensive too. It must be reduced to a minimum by
	 * 		 setting the #PRINTF_ADVANCED_ENABLE in the debug_console.h file to 0. */
	print("%4d;%5d\n", Time_DiffUSec(&t_prev, &res->TimeStamp), res->Bin.Range / (Q9_22_ONE / 1000));

	t_prev = res->TimeStamp;

#endif
}


static void handle_error(status_t status, char const * msg)
{
	/* Check for status < 0 and print message and halt the program execution. */
	if (status < STATUS_OK)
	{
		print("ERROR: %s\nError Code: %d", msg, status);
		while (1) __asm("nop"); // stop!
	}
}


static void hardware_init(void)
{
	/* Initialize the board with clocks. */
	BOARD_ClockInit();

	/* Disable the watchdog timer. */
	COP_Disable();

	/* Initialize timer required by the API. */
	Timer_Init();

	/* Initialize UART for print functionality. */
	UART_Init();

	/* Initialize the S2PI hardware required by the API. */
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);
}


status_t measurement_ready_callback(status_t status, void * data)
{
	handle_error(status, "Measurement Ready Callback received error!");

	/* Store the data pointer for the main thread/task.
	 *
	 * Note: Do not call the evaluate measurement method
	 *       from within this callback since it is invoked in
	 *       a interrupt service routine and should return as
	 *       soon as possible. */

#if ADVANCED_DEMO
	assert(myRawDataPtr[0] == 0 || myRawDataPtr[1] == 0);
	if (myRawDataPtr[0] == 0)
		myRawDataPtr[0] = data;
	else
		myRawDataPtr[1] = data;
#else
    myRawDataPtr = data;
#endif

	return status;
}
