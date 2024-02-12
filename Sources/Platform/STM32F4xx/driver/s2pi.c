/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides driver functionality for the S2PI interface.
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
 *
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "s2pi.h"
#include "irq.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "board/board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/




/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! A structure that holds the mapping to port and pin for all SPI modules. */
typedef struct s2pi_gpio_mapping_t
{
    /*! The GPIO port */
    GPIO_TypeDef * Port;

    /*! The GPIO pin */
    uint32_t Pin;
} s2pi_gpio_mapping_t;

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct s2pi_hnd_t
{
    /*! Determines the current driver status. */
    volatile status_t Status;

    /*! Determines the current S2PI slave. */
    volatile s2pi_slave_t Slave;

    /*! A callback function to be called after transfer/run mode is completed. */
    s2pi_callback_t Callback;

    /*! A parameter to be passed to the callback function. */
    void * CallbackData;

    /*! A callback function to be called after external interrupt is triggered.
     * Note: slave index starts with 1, so the array size needs an extra element */
    s2pi_irq_callback_t IrqCallback[S2PI_SLAVE_COUNT+1];

    /*! A parameter to be passed to the interrupt callback function.
     * Note: slave index starts with 1, so the array size needs an extra element */
    void * IrqCallbackData[S2PI_SLAVE_COUNT+1];

    /*! The alternate function for this SPI port. */
    const uint32_t SpiAlternate;

    /*! Mapping slave ID to its corresponding pin number (used for IRQ handling)
     * Note: slave index starts with 1, so the array size needs an extra element */
    uint32_t SlaveIrqMapping[S2PI_SLAVE_COUNT+1];

    /*! The mapping of the GPIO blocks and pins for this device. */
    s2pi_gpio_mapping_t GPIOs[ S2PI_IRQ+1 ];

    /*! A mutex used for queue operations. */
    volatile bool SpiMutexBlocked;

} s2pi_hnd_t;


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


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static inline status_t S2PI_SetSlaveInternal(s2pi_slave_t slave);

static inline void S2PI_SetGPIOMode(bool gpio_mode);

/*! Initializes the required pins. */
static inline void S2PI_InitPins();


/******************************************************************************
 * Variables
 ******************************************************************************/

/*! The instance holding all internal data required by the S2PI module. */
static s2pi_hnd_t myS2PIHnd = { .SpiAlternate = S2PI_FUNCTION,
                                .Slave = SPI_DEFAULT_SLAVE,
                                .GPIOs = { [ S2PI_CLK  ] = { S2PI_CLK_GPIO, S2PI_CLK_GPIO_PIN },
                                           [ S2PI_CS   ] = { S2PI_CS1_GPIO, S2PI_CS1_GPIO_PIN },
                                           [ S2PI_MOSI ] = { S2PI_MOSI_GPIO, S2PI_MOSI_GPIO_PIN },
                                           [ S2PI_MISO ] = { S2PI_MISO_GPIO, S2PI_MISO_GPIO_PIN },
                                           [ S2PI_IRQ  ] = { S2PI_IRQ1_GPIO, S2PI_IRQ1_GPIO_PIN } } };

/*******************************************************************************
 * Code
 ******************************************************************************/


status_t S2PI_Init(s2pi_slave_t defaultSlave,
                   uint32_t baudRate_Bps)
{
    MX_DMA_Init();
    MX_SPI1_Init();

    S2PI_InitPins();

    if (defaultSlave < 0)
        defaultSlave = S2PI_SLAVE1;

    if (defaultSlave > S2PI_SLAVE_COUNT)
        return ERROR_S2PI_INVALID_SLAVE;

    return S2PI_SetBaudRate(defaultSlave, baudRate_Bps);
}
static inline void S2PI_InitPins()
{
    /* Initializes ports and pins: PWRx, CSx, IRQx */
    MX_GPIO_Init();

    /* Initializes Pins: MOSI/MISO/CLK */
    S2PI_SetGPIOMode(true);
    S2PI_SetGPIOMode(false);

    myS2PIHnd.SlaveIrqMapping[S2PI_SLAVE1] = S2PI_IRQ1_GPIO_PIN;

#if S2PI_SLAVE_COUNT >= 2
    myS2PIHnd.SlaveIrqMapping[S2PI_SLAVE2] = S2PI_IRQ2_GPIO_PIN;
#endif

#if S2PI_SLAVE_COUNT >= 3
    myS2PIHnd.SlaveIrqMapping[S2PI_SLAVE3] = S2PI_IRQ3_GPIO_PIN;
#endif

#if S2PI_SLAVE_COUNT >= 4
    myS2PIHnd.SlaveIrqMapping[S2PI_SLAVE4] = S2PI_IRQ4_GPIO_PIN;
#endif

    /* The 4X board equipped with power switches require some delay
     * until the devices have finished the power-on-reset (POR).
     * Must be >= 2ms (?) */
    Time_DelayMSec(3);
}

static inline status_t S2PI_SetSlaveInternal(s2pi_slave_t slave)
{
    switch (slave)
    {
        case S2PI_SLAVE1:
            myS2PIHnd.GPIOs[S2PI_CS].Port = S2PI_CS1_GPIO;
            myS2PIHnd.GPIOs[S2PI_CS].Pin = S2PI_CS1_GPIO_PIN;
            myS2PIHnd.GPIOs[S2PI_IRQ].Port = S2PI_IRQ1_GPIO;
            myS2PIHnd.GPIOs[S2PI_IRQ].Pin = S2PI_IRQ1_GPIO_PIN;
            break;

#if S2PI_SLAVE_COUNT >= 2
        case S2PI_SLAVE2:
            myS2PIHnd.GPIOs[S2PI_CS].Port = S2PI_CS2_GPIO;
            myS2PIHnd.GPIOs[S2PI_CS].Pin = S2PI_CS2_GPIO_PIN;
            myS2PIHnd.GPIOs[S2PI_IRQ].Port = S2PI_IRQ2_GPIO;
            myS2PIHnd.GPIOs[S2PI_IRQ].Pin = S2PI_IRQ2_GPIO_PIN;
            break;
#endif

#if S2PI_SLAVE_COUNT >= 3
        case S2PI_SLAVE3:
            myS2PIHnd.GPIOs[S2PI_CS].Port = S2PI_CS3_GPIO;
            myS2PIHnd.GPIOs[S2PI_CS].Pin = S2PI_CS3_GPIO_PIN;
            myS2PIHnd.GPIOs[S2PI_IRQ].Port = S2PI_IRQ3_GPIO;
            myS2PIHnd.GPIOs[S2PI_IRQ].Pin = S2PI_IRQ3_GPIO_PIN;
            break;
#endif

#if S2PI_SLAVE_COUNT >= 4
        case S2PI_SLAVE4:
            myS2PIHnd.GPIOs[S2PI_CS].Port = S2PI_CS4_GPIO;
            myS2PIHnd.GPIOs[S2PI_CS].Pin = S2PI_CS4_GPIO_PIN;
            myS2PIHnd.GPIOs[S2PI_IRQ].Port = S2PI_IRQ4_GPIO;
            myS2PIHnd.GPIOs[S2PI_IRQ].Pin = S2PI_IRQ4_GPIO_PIN;
            break;
#endif

        default:
            return ERROR_S2PI_INVALID_SLAVE;
    }

    myS2PIHnd.Slave = slave;

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Sets the mode in which the S2PI pins operate.
 * @details This is a helper function to switch the modes between SPI and GPIO.
 * @param   gpio_mode Enables the GPIO mode: true for GPIO, false for SPI.
 *****************************************************************************/
static inline void S2PI_SetGPIOMode(bool gpio_mode)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = myS2PIHnd.SpiAlternate;

    /* *** OUTPUT pins *** */
    GPIO_InitStruct.Mode      = gpio_mode ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_AF_PP;

    /* SPI CLK */
    GPIO_InitStruct.Pin       = myS2PIHnd.GPIOs[S2PI_CLK].Pin;
    HAL_GPIO_Init(myS2PIHnd.GPIOs[S2PI_CLK].Port, &GPIO_InitStruct);

    /* SPI MOSI */
    GPIO_InitStruct.Pin      = myS2PIHnd.GPIOs[S2PI_MOSI].Pin;
    HAL_GPIO_Init(myS2PIHnd.GPIOs[S2PI_MOSI].Port, &GPIO_InitStruct);

    /* *** INPUT pins *** */
    GPIO_InitStruct.Mode      = gpio_mode ? GPIO_MODE_INPUT : GPIO_MODE_AF_PP;

    /* SPI MISO */
    GPIO_InitStruct.Pin       = myS2PIHnd.GPIOs[S2PI_MISO].Pin;
    HAL_GPIO_Init(myS2PIHnd.GPIOs[S2PI_MISO].Port, &GPIO_InitStruct);
}

status_t S2PI_SetBaudRate(s2pi_slave_t slave, uint32_t baudRate_Bps)
{
    (void) slave; // not used in this implementation
    uint32_t prescaler = 0;
    /* Determine the maximum value of the prescaler */
    for (; prescaler < 8; ++prescaler)
        if (SystemCoreClock >> (prescaler + 1) <= baudRate_Bps)
            break;
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR, prescaler << SPI_CR1_BR_Pos);
    return STATUS_OK;
}

uint32_t S2PI_GetBaudRate(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation
    uint32_t prescaler = (hspi1.Instance->CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos;
    return SystemCoreClock >> (prescaler + 1);
}

status_t S2PI_GetStatus(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation
    return myS2PIHnd.Status;
}

status_t S2PI_CaptureGpioControl(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation

    /* Check if something is ongoing. */
    IRQ_LOCK();
    status_t status = myS2PIHnd.Status;
    if (status != STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return status;
    }
    myS2PIHnd.Status = STATUS_S2PI_GPIO_MODE;
    IRQ_UNLOCK();

    /* Note: Clock must be HI after capturing */
    HAL_GPIO_WritePin(myS2PIHnd.GPIOs[S2PI_CLK].Port, myS2PIHnd.GPIOs[S2PI_CLK].Pin, GPIO_PIN_SET);

    S2PI_SetGPIOMode(true);

    return STATUS_OK;
}

status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation

    /* Check if something is ongoing. */
    IRQ_LOCK();
    status_t status = myS2PIHnd.Status;
    if (status != STATUS_S2PI_GPIO_MODE)
    {
        IRQ_UNLOCK();
        return status;
    }
    myS2PIHnd.Status = STATUS_IDLE;
    IRQ_UNLOCK();

    S2PI_SetGPIOMode(false);

    return STATUS_OK;
}

status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
    /* Check if pin is valid. */
    if (pin > S2PI_IRQ || value > 1)
        return ERROR_INVALID_ARGUMENT;

    /* Check if in GPIO mode. */
    if(myS2PIHnd.Status != STATUS_S2PI_GPIO_MODE)
        return ERROR_S2PI_INVALID_STATE;

    if (pin == S2PI_CS)
    {
        GPIO_PinState pinState = value ? GPIO_PIN_SET : GPIO_PIN_RESET;
        switch (slave)
        {
            case S2PI_SLAVE1:
                HAL_GPIO_WritePin(S2PI_CS1_GPIO, S2PI_CS1_GPIO_PIN, pinState);
                break;

#if S2PI_SLAVE_COUNT >= 2
            case S2PI_SLAVE2:
                HAL_GPIO_WritePin(S2PI_CS2_GPIO, S2PI_CS2_GPIO_PIN, pinState);
                break;
#endif

#if S2PI_SLAVE_COUNT >= 3
            case S2PI_SLAVE3:
                HAL_GPIO_WritePin(S2PI_CS3_GPIO, S2PI_CS3_GPIO_PIN, pinState);
                break;
#endif

#if S2PI_SLAVE_COUNT >= 4
            case S2PI_SLAVE4:
                HAL_GPIO_WritePin(S2PI_CS4_GPIO, S2PI_CS4_GPIO_PIN, pinState);
                break;
#endif

            default:
                return ERROR_S2PI_INVALID_SLAVE;
        }
    }
    else
    {
        HAL_GPIO_WritePin(myS2PIHnd.GPIOs[pin].Port, myS2PIHnd.GPIOs[pin].Pin, value);
    }

    S2PI_GPIO_DELAY();

    return STATUS_OK;
}

status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t * value)
{
    /* Check if pin is valid. */
    if (pin > S2PI_IRQ || !value)
        return ERROR_INVALID_ARGUMENT;

    /* Check if in GPIO mode. */
    if(myS2PIHnd.Status != STATUS_S2PI_GPIO_MODE)
        return ERROR_S2PI_INVALID_STATE;

    if (pin == S2PI_CS)
    {
        switch (slave)
        {
            case S2PI_SLAVE1:
                *value = HAL_GPIO_ReadPin(S2PI_CS1_GPIO, S2PI_CS1_GPIO_PIN);
                break;

#if S2PI_SLAVE_COUNT >= 2
            case S2PI_SLAVE2:
                *value = HAL_GPIO_ReadPin(S2PI_CS2_GPIO, S2PI_CS2_GPIO_PIN);
                break;
#endif

#if S2PI_SLAVE_COUNT >= 3
            case S2PI_SLAVE3:
                *value = HAL_GPIO_ReadPin(S2PI_CS3_GPIO, S2PI_CS3_GPIO_PIN);
                break;
#endif

#if S2PI_SLAVE_COUNT >= 4
            case S2PI_SLAVE4:
                *value = HAL_GPIO_ReadPin(S2PI_CS4_GPIO, S2PI_CS4_GPIO_PIN);
                break;
#endif

            default:
                return ERROR_S2PI_INVALID_SLAVE;
        }
    }
    else
    {
        *value = HAL_GPIO_ReadPin(myS2PIHnd.GPIOs[pin].Port, myS2PIHnd.GPIOs[pin].Pin);
    }

    S2PI_GPIO_DELAY();

    return STATUS_OK;
}

status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
    /* Check the driver status. */
    IRQ_LOCK();
    status_t status = myS2PIHnd.Status;
    if ( status != STATUS_IDLE )
    {
        IRQ_UNLOCK();
        return status;
    }
    myS2PIHnd.Status = STATUS_BUSY;
    IRQ_UNLOCK();

    status = STATUS_OK;
    switch (slave)
    {
        case S2PI_SLAVE1:
            HAL_GPIO_WritePin(S2PI_CS1_GPIO, S2PI_CS1_GPIO_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S2PI_CS1_GPIO, S2PI_CS1_GPIO_PIN, GPIO_PIN_SET);
            break;

#if S2PI_SLAVE_COUNT >= 2
        case S2PI_SLAVE2:
            HAL_GPIO_WritePin(S2PI_CS2_GPIO, S2PI_CS2_GPIO_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S2PI_CS2_GPIO, S2PI_CS2_GPIO_PIN, GPIO_PIN_SET);
            break;
#endif

#if S2PI_SLAVE_COUNT >= 3
        case S2PI_SLAVE3:
            HAL_GPIO_WritePin(S2PI_CS3_GPIO, S2PI_CS3_GPIO_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S2PI_CS3_GPIO, S2PI_CS3_GPIO_PIN, GPIO_PIN_SET);
            break;
#endif

#if S2PI_SLAVE_COUNT >= 4
        case S2PI_SLAVE4:
            HAL_GPIO_WritePin(S2PI_CS4_GPIO, S2PI_CS4_GPIO_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S2PI_CS4_GPIO, S2PI_CS4_GPIO_PIN, GPIO_PIN_SET);
            break;
#endif

        default:
            status = ERROR_S2PI_INVALID_SLAVE;
    }

    myS2PIHnd.Status = STATUS_IDLE;

    return status;
}

status_t S2PI_TransferFrame(s2pi_slave_t slave,
                            uint8_t const * txData,
                            uint8_t * rxData,
                            size_t frameSize,
                            s2pi_callback_t callback,
                            void * callbackData)
{
    /* Verify arguments. */
    if (!txData || frameSize == 0 || frameSize > UINT16_MAX)
        return ERROR_INVALID_ARGUMENT;

    /* Check the driver status, lock if idle. */
    IRQ_LOCK();
    status_t status = myS2PIHnd.Status;
    if (status != STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return status;
    }
    myS2PIHnd.Status = STATUS_BUSY;
    IRQ_UNLOCK();

    /* Set the callback information */
    myS2PIHnd.Callback = callback;
    myS2PIHnd.CallbackData = callbackData;

    /* Manually set the chip select (active low) */
    status = S2PI_SetSlaveInternal(slave);
    if (status == STATUS_OK)
    {
        HAL_GPIO_WritePin(myS2PIHnd.GPIOs[S2PI_CS].Port, myS2PIHnd.GPIOs[S2PI_CS].Pin, GPIO_PIN_RESET);
    }

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

    return status;
}


/*!***************************************************************************
 * @brief   Triggers the callback function with the provided status.
 * @details It first checks if a callback function is present,
 *          otherwise it returns immediately.
 *          The callback function is reset to 0, and must be set up again
 *          for the next transfer, if required.
 * @param   status The status to be provided to the callback funcition.
 * @return  Returns the status received from the callback function
 ****************************************************************************/
static inline status_t S2PI_CompleteTransfer(status_t status)
{
    myS2PIHnd.Status = STATUS_IDLE;

    /* Deactivate CS (set high), as we use GPIO pin */
    HAL_GPIO_WritePin(myS2PIHnd.GPIOs[S2PI_CS].Port, myS2PIHnd.GPIOs[S2PI_CS].Pin, GPIO_PIN_SET);

    /* Invoke callback if there is one */
    if (myS2PIHnd.Callback != 0)
    {
        s2pi_callback_t callback = myS2PIHnd.Callback;
        myS2PIHnd.Callback = 0;
        status = callback(status, myS2PIHnd.CallbackData);
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

status_t S2PI_TryGetMutex(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation

    status_t retVal;

    IRQ_LOCK();
    if (!myS2PIHnd.SpiMutexBlocked)
    {
        myS2PIHnd.SpiMutexBlocked = true;
        retVal = STATUS_OK;
    }
    else
    {
        retVal = STATUS_BUSY;
    }
    IRQ_UNLOCK();

    return retVal;
}

void S2PI_ReleaseMutex(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation

    myS2PIHnd.SpiMutexBlocked = false;
}

status_t S2PI_Abort(s2pi_slave_t slave)
{
    (void) slave; // not used in this implementation

    status_t status = myS2PIHnd.Status;

    /* Check if something is ongoing. */
    if(status == STATUS_IDLE)
    {
        return STATUS_OK;
    }

    /* Abort SPI transfer. */
    if(status == STATUS_BUSY)
    {
        HAL_SPI_Abort(&hspi1);
        myS2PIHnd.Status = STATUS_IDLE;
    }

    status = S2PI_CompleteTransfer(ERROR_ABORTED);
    if(status == ERROR_ABORTED) status = STATUS_OK;

    return STATUS_OK;
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

status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
                             s2pi_irq_callback_t callback,
                             void * callbackData)
{
    if (slave > S2PI_SLAVE_COUNT)
        return ERROR_S2PI_INVALID_SLAVE;

    myS2PIHnd.IrqCallback[slave] = callback;
    myS2PIHnd.IrqCallbackData[slave] = callbackData;

    return STATUS_OK;
}

uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
    /* NOTE: this must return 0 if an interrupt is pending, i.e. if
     * the interrupt is pending AND the pin is in low state. */
    switch (slave)
    {
        case S2PI_SLAVE1:
            return !(HAL_NVIC_GetPendingIRQ(S2PI_IRQ1_EXTI)
                     && !HAL_GPIO_ReadPin(S2PI_IRQ1_GPIO, S2PI_IRQ1_GPIO_PIN));

#if S2PI_SLAVE_COUNT >= 2
        case S2PI_SLAVE2:
            return !(HAL_NVIC_GetPendingIRQ(S2PI_IRQ2_EXTI)
                     && !HAL_GPIO_ReadPin(S2PI_IRQ2_GPIO, S2PI_IRQ2_GPIO_PIN));
#endif

#if S2PI_SLAVE_COUNT >= 3
        case S2PI_SLAVE3:
            return !(HAL_NVIC_GetPendingIRQ(S2PI_IRQ3_EXTI)
                     && !HAL_GPIO_ReadPin(S2PI_IRQ3_GPIO, S2PI_IRQ3_GPIO_PIN));
#endif

#if S2PI_SLAVE_COUNT >= 4
        case S2PI_SLAVE4:
            return !(HAL_NVIC_GetPendingIRQ(S2PI_IRQ4_EXTI)
                     && !HAL_GPIO_ReadPin(S2PI_IRQ4_GPIO, S2PI_IRQ4_GPIO_PIN));
#endif

        default:
            return 1U;
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (uint_fast8_t slaveID = 1; slaveID <= S2PI_SLAVE_COUNT; slaveID++)
    {
        if (GPIO_Pin == myS2PIHnd.SlaveIrqMapping[slaveID])
        {
            /* Interrupt for slaveID has been fired */
            s2pi_irq_callback_t cb = myS2PIHnd.IrqCallback[slaveID];
            void * cbParam = myS2PIHnd.IrqCallbackData[slaveID];

            if (cb)
            {
                /* Invoke corresponding callback */
                cb(cbParam);
            }
        }
    }
}
