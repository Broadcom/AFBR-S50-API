/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		
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

#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "driver/irq.h"
#include "driver/s2pi.h"



/*! An additional delay to be added after each GPIO access in order to decrease
 *  the baud rate of the software EEPROM protocol. Increase the delay if timing
 *  issues occur while reading the EERPOM.
 *  e.g. Delay = 10 µsec => Baud Rate < 100 kHz */
#ifndef S2PI_GPIO_DELAY_US
#define S2PI_GPIO_DELAY_US 10
#endif


#if (S2PI_GPIO_DELAY_US == 0)
#define S2PI_GPIO_DELAY() ((void)0)
#else
#include "utility/time.h"
#define S2PI_GPIO_DELAY() Time_DelayUSec(S2PI_GPIO_DELAY_US)
#endif


/*! A structure that holds the mapping to port and pin for all SPI modules. */
typedef struct
{
	/*! The GPIO port */
	GPIO_TypeDef * Port;

	/*! The GPIO pin */
	uint32_t Pin;
}
s2pi_gpio_mapping_t;

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct
{
	/*! Determines the current driver status. */
	volatile status_t Status;

	/*! Determines the current S2PI slave. */
	volatile s2pi_slave_t Slave;

	/*! A callback function to be called after transfer/run mode is completed. */
	s2pi_callback_t Callback;

	/*! A parameter to be passed to the callback function. */
	void * CallbackData;

	/*! A callback function to be called after external interrupt is triggered. */
	s2pi_irq_callback_t IrqCallback;

	/*! A parameter to be passed to the interrupt callback function. */
	void * IrqCallbackData;

	/*! The alternate function for this SPI port. */
	const uint32_t SpiAlternate;

	/*! The mapping of the GPIO blocks and pins for this device. */
	const s2pi_gpio_mapping_t GPIOs[ S2PI_IRQ+1 ];
}
s2pi_handle_t;

/*! The instance holding all internal data required by the S2PI module. */
static s2pi_handle_t s2pi_ = { .SpiAlternate = GPIO_AF5_SPI1,
							   .GPIOs = { [ S2PI_CLK  ] = { GPIOA, GPIO_PIN_5 },
									   	  [ S2PI_CS   ] = { GPIOB, GPIO_PIN_6 },
										  [ S2PI_MOSI ] = { GPIOA, GPIO_PIN_7 },
										  [ S2PI_MISO ] = { GPIOA, GPIO_PIN_6 },
										  [ S2PI_IRQ  ] = { GPIOC, GPIO_PIN_7 } } };

/*!***************************************************************************
 * @brief	Sets the SPI baud rate in bps.
 * @param	baudRate_Bps The default SPI baud rate in bauds-per-second.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 * 			- #STATUS_OK on success
 * 			- #ERROR_S2PI_INVALID_BAUDRATE on invalid baud rate value.
 *****************************************************************************/
status_t S2PI_SetBaudRate(uint32_t baudRate_Bps)
{
	uint32_t prescaler = 0;
	/* Determine the maximum value of the prescaler */
	for (; prescaler < 8; ++prescaler)
		if (SystemCoreClock >> (prescaler + 1) <= baudRate_Bps)
			break;
	MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR, prescaler << SPI_CR1_BR_Pos);
	return STATUS_OK;
}


/*!***************************************************************************
 * @brief	Gets the current SPI baud rate in bps.
 * @return 	Returns the current baud rate.
 *****************************************************************************/
uint32_t S2PI_GetBaudRate(void)
{
	uint32_t prescaler = (hspi1.Instance->CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
	return SystemCoreClock >> (prescaler + 1);
}



static void S2PI_SetGPIOMode(bool gpio_mode);


/*!***************************************************************************
 * @brief	Initialize the S2PI module.
 * @details	Setup the board as a S2PI master, this also sets up up the S2PI
 * 			pins.
 * 			The SPI interface is initialized with the corresponding default
 * 			SPI slave (i.e. CS and IRQ lines) and the default baud rate.
 *
 * @param	defaultSlave The default SPI slave to be addressed right after
 * 							  module initialization.
 * @param	baudRate_Bps The default SPI baud rate in bauds-per-second.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Init(s2pi_slave_t defaultSlave, uint32_t baudRate_Bps)
{
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_GPIO_Init();

	S2PI_SetGPIOMode(true);
	S2PI_SetGPIOMode(false);

	if (defaultSlave < 0)
		defaultSlave = S2PI_S1;

	if (defaultSlave != S2PI_S1)
		return ERROR_S2PI_INVALID_SLAVE;

	return S2PI_SetBaudRate(baudRate_Bps);
}

status_t S2PI_GetStatus(void)
{
	return s2pi_.Status;
}

/*!***************************************************************************
 * @brief	Sets the mode in which the S2PI pins operate.
 * @details This is a helper function to switch the modes between SPI and GPIO.
 * @param	gpio_mode Enables the GPIO mode: true for GPIO, false for SPI.
 *****************************************************************************/
static void S2PI_SetGPIOMode(bool gpio_mode)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = s2pi_.SpiAlternate;

	/* *** OUTPUT pins *** */
    GPIO_InitStruct.Mode      = gpio_mode ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_AF_PP;

	/* SPI CLK */
	GPIO_InitStruct.Pin       = s2pi_.GPIOs[S2PI_CLK].Pin;
	HAL_GPIO_Init(s2pi_.GPIOs[S2PI_CLK].Port, &GPIO_InitStruct);

	/* SPI MOSI */
	GPIO_InitStruct.Pin      = s2pi_.GPIOs[S2PI_MOSI].Pin;
	HAL_GPIO_Init(s2pi_.GPIOs[S2PI_MOSI].Port, &GPIO_InitStruct);

	/* *** INPUT pins *** */
	GPIO_InitStruct.Mode      = gpio_mode ? GPIO_MODE_INPUT : GPIO_MODE_AF_PP;

	/* SPI MISO */
	GPIO_InitStruct.Pin       = s2pi_.GPIOs[S2PI_MISO].Pin;
	HAL_GPIO_Init(s2pi_.GPIOs[S2PI_MISO].Port, &GPIO_InitStruct);
}


/*!*****************************************************************************
 * @brief	Captures the S2PI pins for GPIO usage.
 * @details	The SPI is disabled (module status: #STATUS_S2PI_GPIO_MODE) and the
 * 			pins are configured for GPIO operation. The GPIO control must be
 * 			release with the #S2PI_ReleaseGpioControl function in order to
 * 			switch back to ordinary SPI functionality.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_CaptureGpioControl(void)
{
	/* Check if something is ongoing. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;
	if (status != STATUS_IDLE)
	{
		IRQ_UNLOCK();
		return status;
	}
	s2pi_.Status = STATUS_S2PI_GPIO_MODE;
	IRQ_UNLOCK();

	/* Note: Clock must be HI after capturing */
	HAL_GPIO_WritePin(s2pi_.GPIOs[S2PI_CLK].Port, s2pi_.GPIOs[S2PI_CLK].Pin, GPIO_PIN_SET);

	S2PI_SetGPIOMode(true);

	return STATUS_OK;
}


/*!*****************************************************************************
 * @brief	Releases the S2PI pins from GPIO usage and switches back to SPI mode.
 * @details	The GPIO pins are configured for SPI operation and the GPIO mode is
 * 			left. Must be called if the pins are captured for GPIO operation via
 * 			the #S2PI_CaptureGpioControl function.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReleaseGpioControl(void)
{
	/* Check if something is ongoing. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;
	if (status != STATUS_S2PI_GPIO_MODE)
	{
		IRQ_UNLOCK();
		return status;
	}
	s2pi_.Status = STATUS_IDLE;
	IRQ_UNLOCK();

	S2PI_SetGPIOMode(false);

	return STATUS_OK;
}


/*!*****************************************************************************
 * @brief	Writes the output for a specified SPI pin in GPIO mode.
 * @details This function writes the value of an SPI pin if the SPI pins are
 * 			captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 * @param	slave The specified S2PI slave.
 * @param	pin The specified S2PI pin.
 * @param	value The GPIO pin state to write (0 = low, 1 = high).
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
	/* Check if pin is valid. */
	if (pin > S2PI_IRQ || value > 1)
		return ERROR_INVALID_ARGUMENT;

	/* Check if in GPIO mode. */
	if(s2pi_.Status != STATUS_S2PI_GPIO_MODE)
		return ERROR_S2PI_INVALID_STATE;

	HAL_GPIO_WritePin(s2pi_.GPIOs[pin].Port, s2pi_.GPIOs[pin].Pin, value);

	S2PI_GPIO_DELAY();

	return STATUS_OK;
}


/*!*****************************************************************************
 * @brief	Reads the input from a specified SPI pin in GPIO mode.
 * @details This function reads the value of an SPI pin if the SPI pins are
 * 			captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 * @param	slave The specified S2PI slave.
 * @param	pin The specified S2PI pin.
 * @param	value The GPIO pin state to read (0 = low, GND level, 1 = high, VCC level).
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t * value)
{
	/* Check if pin is valid. */
	if (pin > S2PI_IRQ || !value)
		return ERROR_INVALID_ARGUMENT;

	/* Check if in GPIO mode. */
	if(s2pi_.Status != STATUS_S2PI_GPIO_MODE)
		return ERROR_S2PI_INVALID_STATE;

	*value = HAL_GPIO_ReadPin(s2pi_.GPIOs[pin].Port, s2pi_.GPIOs[pin].Pin);

	S2PI_GPIO_DELAY();

	return STATUS_OK;
}


/*!***************************************************************************
 * @brief	Cycles the chip select line.
 * @details In order to cancel the integration on the ASIC, a fast toggling
 * 			of the chip select pin of the corresponding SPI slave is required.
 * 			Therefore, this function toggles the CS from high to low and back.
 * 			The SPI instance for the specified S2PI slave must be idle,
 * 			otherwise the status #STATUS_BUSY is returned.
 * @param	slave The specified S2PI slave.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
	/* Check the driver status. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;
	if ( status != STATUS_IDLE )
	{
		IRQ_UNLOCK();
		return status;
	}
	s2pi_.Status = STATUS_BUSY;
	IRQ_UNLOCK();

	HAL_GPIO_WritePin(s2pi_.GPIOs[S2PI_CS].Port, s2pi_.GPIOs[S2PI_CS].Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s2pi_.GPIOs[S2PI_CS].Port, s2pi_.GPIOs[S2PI_CS].Pin, GPIO_PIN_SET);

	s2pi_.Status = STATUS_IDLE;

	return STATUS_OK;
}


/*!***************************************************************************
 * @brief 	Transfers a single SPI frame asynchronously.
 * @details Transfers a single SPI frame in asynchronous manner. The Tx data
 * 			buffer is written to the device via the MOSI line.
 * 			Optionally the data on the MISO line is written to the provided
 * 			Rx data buffer. If null, the read data is dismissed.
 * 			The transfer of a single frame requires to not toggle the chip
 * 			select line to high in between the data frame.
 * 			An optional callback is invoked when the asynchronous transfer
 * 			is finished. Note that the provided buffer must not change while
 * 			the transfer is ongoing. Use the slave parameter to determine
 * 			the corresponding slave via the given chip select line.
 *
 * @param	slave The specified S2PI slave.
 * @param	txData The 8-bit values to write to the SPI bus MOSI line.
 * @param	rxData The 8-bit values received from the SPI bus MISO line
 *                   (pass a null pointer if the data don't need to be read).
 * @param	frameSize The number of 8-bit values to be sent/received.
 * @param	callback A callback function to be invoked when the transfer is
 * 					   finished. Pass a null pointer if no callback is required.
 * @param	callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			 - #STATUS_OK: Successfully invoked the transfer.
 *         	 - #ERROR_INVALID_ARGUMENT: An invalid parameter has been passed.
 *         	 - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *         	 - #STATUS_BUSY: An SPI transfer is already in progress. The
 *         	                 transfer was not started.
 *         	 - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode. The transfer
 *         	                           was not started.
 *****************************************************************************/
status_t S2PI_TransferFrame(s2pi_slave_t spi_slave,
							uint8_t const * txData,
							uint8_t * rxData,
							size_t frameSize,
							s2pi_callback_t callback,
							void * callbackData)
{
	/* Verify arguments. */
	if (!txData || frameSize == 0 || frameSize >= 0x10000)
		return ERROR_INVALID_ARGUMENT;

	/* Check the spi slave.*/
	if (spi_slave != S2PI_S1)
		return ERROR_S2PI_INVALID_SLAVE;

	/* Check the driver status, lock if idle. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;
	if (status != STATUS_IDLE)
	{
		IRQ_UNLOCK();
		return status;
	}
	s2pi_.Status = STATUS_BUSY;
	IRQ_UNLOCK();

	/* Set the callback information */
	s2pi_.Callback = callback;
	s2pi_.CallbackData = callbackData;

	/* Manually set the chip select (active low) */
	HAL_GPIO_WritePin(s2pi_.GPIOs[S2PI_CS].Port, s2pi_.GPIOs[S2PI_CS].Pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef hal_error;

	/* Lock interrupts to prevent completion interrupt before setup is complete */
	IRQ_LOCK();
	if (rxData)
		hal_error = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) txData, rxData, (uint16_t) frameSize);
	else
		hal_error = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *) txData, (uint16_t) frameSize);
	IRQ_UNLOCK();

	if (hal_error != HAL_OK)
		//return ERROR_FAIL;
		return -1000-hal_error;

	return STATUS_OK;
}


/*!***************************************************************************
 * @brief	Triggers the callback function with the provided status.
 * @details It first checks if a callback function is present,
 *          otherwise it returns immediately.
 *          The callback function is reset to 0, and must be set up again
 *          for the next transfer, if required.
 * @param	status The status to be provided to the callback funcition.
 * @return 	Returns the status received from the callback function
 ****************************************************************************/
static inline status_t S2PI_CompleteTransfer(status_t status)
{
	s2pi_.Status = STATUS_IDLE;

	/* Deactivate CS (set high), as we use GPIO pin */
	HAL_GPIO_WritePin(s2pi_.GPIOs[S2PI_CS].Port, s2pi_.GPIOs[S2PI_CS].Pin, GPIO_PIN_SET);

	/* Invoke callback if there is one */
	if (s2pi_.Callback != 0)
	{
		s2pi_callback_t callback = s2pi_.Callback;
		s2pi_.Callback = 0;
		status = callback(status, s2pi_.CallbackData);
	}
	return status;
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *         the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	S2PI_CompleteTransfer(STATUS_OK);
}


/**
  * @brief  DMA SPI transmit receive process complete callback for delayed transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
void SPI_DMATransmitReceiveCpltDelayed(DMA_HandleTypeDef *hdma)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
	HAL_SPI_TxCpltCallback(hspi);
}


/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* The problem we have here is that the next SPI transfer is set up in the interrupt.
	 * The default implementation always connects this callback with the RX DMA complete interrupt.
	 * However, the order of TX and RX interrupts is not specified.
	 * If we perform the callback before both interrupts are done, the next SPI transfer will fail. */
	if ( hspi->hdmatx->Lock == HAL_UNLOCKED ) /* TX Interrupt already received */
		HAL_SPI_TxCpltCallback(hspi);
	else /* There is still the TX DMA Interrupt we have to wait for */
		hspi->hdmatx->XferCpltCallback = SPI_DMATransmitReceiveCpltDelayed;
}


/*!***************************************************************************
 * @brief	Terminates a currently ongoing asynchronous SPI transfer.
 * @details	When a callback is set for the current ongoing activity, it is
 * 			invoked with the #ERROR_ABORTED error byte.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Abort(void)
{
	status_t status = s2pi_.Status;

	/* Check if something is ongoing. */
	if(status == STATUS_IDLE)
	{
		return STATUS_OK;
	}

	/* Abort SPI transfer. */
	if(status == STATUS_BUSY)
	{
		HAL_SPI_Abort(&hspi1);
	}

	return STATUS_OK;
}


/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	S2PI_CompleteTransfer(ERROR_ABORTED);
}


/**
  * @brief  SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	S2PI_CompleteTransfer(ERROR_FAIL);
}


/*!***************************************************************************
 * @brief 	Set a callback for the GPIO IRQ for a specified S2PI slave.
 *
 * @param	slave The specified S2PI slave.
 * @param	callback A callback function to be invoked when the specified
 * 					   S2PI slave IRQ occurs. Pass a null pointer to disable
 * 					   the callback.
 * @param	callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			 - #STATUS_OK: Successfully installation of the callback.
 *         	 - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *****************************************************************************/
status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
							 s2pi_irq_callback_t callback,
							 void * callbackData)
{
	s2pi_.IrqCallback = callback;
	s2pi_.IrqCallbackData = callbackData;

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Reads the current status of the IRQ pin.
 * @details In order to keep a low priority for GPIO IRQs, the state of the
 * 			IRQ pin must be read in order to reliable check for chip timeouts.
 *
 * 			The execution of the interrupt service routine for the data-ready
 * 			interrupt from the corresponding GPIO pin might be delayed due to
 * 			priority issues. The delayed execution might disable the timeout
 * 			for the eye-safety checker too late causing false error messages.
 * 			In order to overcome the issue, the state of the IRQ GPIO input
 * 			pin is read before raising a timeout error in order to check if
 * 			the device has already finished but the IRQ is still pending to be
 * 			executed!

 * @param	slave The specified S2PI slave.
 * @return 	Returns 1U if the IRQ pin is high (IRQ not pending) and 0U if the
 * 			devices pulls the pin to low state (IRQ pending).
 *****************************************************************************/
uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
	return HAL_GPIO_ReadPin(s2pi_.GPIOs[S2PI_IRQ].Port, s2pi_.GPIOs[S2PI_IRQ].Pin);
}


/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == s2pi_.GPIOs[S2PI_IRQ].Pin && s2pi_.IrqCallback)
	{
		s2pi_.IrqCallback(s2pi_.IrqCallbackData);
	}
}

/*!***************************************************************************
 * @brief	[Deprecated] Shuts the driver down, i.e. clears all pins to low state.
 * @details	Shuts down and disables the driver and its output pins. Ongoing
 * 			activity is aborted and no callback is invoked. The output pins
 * 			will go to a low level. A call to another S2PI function will
 * 			automatically start the driver before execution.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Disable(void)
{
	HAL_SPI_DeInit(&hspi1);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

	return STATUS_OK;
}


status_t S2PI_PinsOff(void)
{
	HAL_SPI_DeInit(&hspi1);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

	return STATUS_OK;
}

status_t S2PI_PinsLow(void)
{
	HAL_SPI_DeInit(&hspi1);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

	return STATUS_OK;
}
