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
/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 1

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 6000000

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! Global raw data variable. */
static volatile void * myData = 0;


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

	/* Running a sequence of test in order to verify the HAL implementation. */
	status = Argus_VerifyHALImplementation(SPI_SLAVE);
	handle_error(status, "HAL Implementation verification failed!");


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
	print("\n##### AFBR-S50 API - Advanced Example ############\n"
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


	/* Adjust some configuration parameters by invoking the dedicated API methods. */
	status = Argus_SetConfigurationFrameTime( hnd, 100000 ); // 0.1 second = 10 Hz
	handle_error(status, "Argus_SetConfigurationFrameTime failed!");

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
		/* Check if new measurement data is ready. */
		if (myData != 0)
		{
			/* Release for next measurement data. */
			void * data = (void *) myData;
			myData = 0;

			/* The measurement data structure. */
			argus_results_t res;

			/* Evaluate the raw measurement results. */
		    status = Argus_EvaluateData(hnd, &res, data);
		    handle_error(status, "Argus_EvaluateData failed!");

			/* Use the obtain results, e.g. print via UART. */
			print_results(&res);
		}
		else
		{
			/* User code here... */
			__asm("nop");
		}
	}
}

static void print_results(argus_results_t const * res)
{
	/* Print the recent measurement results:
	 * 1. Range in mm (converting the Q9.22 value to mm)
	 * 2. Amplitude in LSB (converting the UQ12.4 value to LSB)
	 * 3. Status (0: OK, <0: Error, >0: Warning */
	print("Range: %5d mm;  Amplitude: %4d LSB;  Quality: %3d;  Status: %d\n",
		  res->Bin.Range / (Q9_22_ONE / 1000),
		  res->Bin.Amplitude / UQ12_4_ONE,
		  res->Bin.SignalQuality,
		  res->Status);
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

	/* Inform the main task about new data ready.
	 * Note: do not call the evaluate measurement method
	 * from within this callback since it is invoked in
	 * a interrupt service routine and should return as
	 * soon as possible. */
	assert(myData == 0);
	myData = data;
	return status;
}
