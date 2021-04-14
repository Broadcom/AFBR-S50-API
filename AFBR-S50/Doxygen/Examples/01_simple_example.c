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
 * @param	fmt_s : The usual printf parameters.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
extern status_t print(const char  *fmt_s, ...);

/*!***************************************************************************
 * @brief	Initialization routine for board hardware and peripherals.
 *
 * @return	-
 *****************************************************************************/
static void hardware_init(void);

/*!***************************************************************************
 * @brief	Measurement data ready callback function.
 *
 * @details
 *
 * @param	status :
 *
 * @param	data :
 *
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
 *
 * @return	-
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

	/* Adjust some configuration parameters by invoking the dedicated API methods. */
	Argus_SetConfigurationFrameTime(hnd, 100000); // 0.1 second = 10 Hz

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
		myData = data;
	}
	return status;
}