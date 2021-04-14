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
#include "driver/gpio.h"
#include "driver/s2pi.h"
#include "driver/uart.h"
#include "driver/timer.h"

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
#define ADVANCED_DEMO 0
#endif

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
 * @brief	Initialization routine for board hardware and peripherals.
 *****************************************************************************/
static void hardware_init(void);

/*!***************************************************************************
 * @brief	Measurement data ready callback function.
 *
 * @details
 *
 * @param	status *
 * @param	data *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t measurement_ready_callback(status_t status, void * data);


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!***************************************************************************
 * @brief	Application entry point.
 *
 * @details	The main function of the program, called after startup code
 * 			This function should never be exited.
 *****************************************************************************/
int main(void)
{
	/* The API module handle that contains all data definitions that is
	 * required within the API module for the corresponding hardware device.
	 * Every call to an API function requires the passing of a pointer to this
	 * data structure. */
	argus_hnd_t * hnd = Argus_CreateHandle();

	if (hnd == 0)
	{
		/* Error Handling ...*/
	}

	/* Initialize the platform hardware including the required peripherals
	 * for the API. */
	hardware_init();

	/* Initialize the API with default values.
	 * This implicitly calls the initialization functions
	 * of the underlying API modules.
	 *
	 * The second parameter is stored and passed to all function calls
	 * to the S2PI module. This piece of information can be utilized in
	 * order to determine the addressed SPI slave and enabled the usage
	 * of multiple devices on a single SPI peripheral. */
	status_t status = Argus_Init(hnd, SPI_SLAVE);

	if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}

	/* Print some information about current API and connected device. */
	uint32_t value = Argus_GetAPIVersion();
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	uint32_t id = Argus_GetChipID(hnd);
	argus_module_version_t mv = Argus_GetModuleVersion(hnd);

#if ADVANCED_DEMO
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

	/* Adjust some configuration parameters by invoking the dedicated API methods. */
	Argus_SetConfigurationFrameTime( hnd, 100000 ); // 0.1 second = 10 Hz

#if ADVANCED_DEMO

	/* Start the measurement timers within the API module.
	 * The callback is invoked every time a measurement has been finished.
	 * The callback is used to schedule the data evaluation routine to the
	 * main thread by the user.
	 * Note that the timer based measurement is not implemented for multiple
	 * instance yet! */
	status = Argus_StartMeasurementTimer(hnd, measurement_ready_callback);

	if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}

	for(;;)
	{
		/* Check if new measurement data is ready. */
		if(myData != 0)
		{
		/* Release for next measurement data. */
			void * data = (void *) myData;
			myData = 0;

			/* The measurement data structure. */
			argus_results_t res;

			/* Evaluate the raw measurement results. */
		    status = Argus_EvaluateData( hnd, &res, data );

			if (status != STATUS_OK)
			{
				/* Error Handling ...*/
			}

			else
			{
				/* Use the recent measurement results
				 * (converting the Q9.22 value to float and print or display it). */
				print("Range: %d mm\n", res.Bin.Range / (Q9_22_ONE / 1000));
			}
		}
		else
		{
			/* User code here... */
			__asm("nop");
		}
	}


#else
	/* The program loop ... */
	for(;;)
	{
		myData = 0;

		/* Triggers a single measurement.
		 * Note that due to the laser safety algorithms, the method might refuse
		 * to restart a measurement when the appropriate time has not been elapsed
		 * right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
		 * the function must be called again later. Use the frame time configuration
		 * in order to adjust the timing between two measurement frames. */
		status = Argus_TriggerMeasurement(hnd, measurement_ready_callback);
		if (status == STATUS_ARGUS_POWERLIMIT)
		{
			/* Not ready (due to laser safety) to restart the measurement yet.
			 * Come back later. */
			__asm("nop");
		}
		else if (status != STATUS_OK)
		{
			/* Error Handling ...*/
		}
		else
		{
			/* Wait until measurement data is ready. */
			do
			{
				status = Argus_GetStatus(hnd);
				__asm("nop");
			}
			while(status == STATUS_BUSY);

			if (status != STATUS_OK)
			{
				/* Error Handling ...*/
			}

			else
			{
				/* The measurement data structure. */
				argus_results_t res;

				/* Evaluate the raw measurement results. */
				status = Argus_EvaluateData(hnd, &res, (void*)myData);

				if (status != STATUS_OK)
				{
					/* Error Handling ...*/
				}

				else
				{
					/* Use the recent measurement results
					 * (converting the Q9.22 value to float and print or display it). */
					print("Range: %d mm\n", res.Bin.Range / (Q9_22_ONE / 1000));
				}
			}
		}
	}
#endif
}

static void hardware_init(void)
{
	/* Initialize the board with clocks. */
	BOARD_ClockInit();

	/* Disable the watchdog timer. */
	COP_Disable();

	/* Init GPIO ports. */
	GPIO_Init();

	/* Initialize timer required by the API. */
	Timer_Init();

	/* Initialize UART for print functionality. */
	UART_Init();

	/* Initialize the S2PI hardware required by the API. */
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);
}

status_t measurement_ready_callback(status_t status, void * data)
{
	if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}
	else
	{
		/* Inform the main task about new data ready.
		 * Note: do not call the evaluate measurement method
		 * from within this callback since it is invoked in
		 * a interrupt service routine and should return as
		 * soon as possible. */
		assert(myData == 0);

		myData = data;
	}
	return status;
}
