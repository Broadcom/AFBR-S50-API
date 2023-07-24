/*************************************************************************//**
 * @file
 * @brief       This file is part of the RA4M2 platform layer.
 * @details     This file provides driver functionality for the S2PI interface.
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
 *
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "driver/s2pi.h"
#include "driver/irq.h"
#include "bsp_api.h"
#include "hal_data.h"
#include "io.h"
#include "debug.h"
#include "board/board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     BSP_IO_PORT_01_PIN_11
#define SPIx_MISO_PIN                    BSP_IO_PORT_01_PIN_10
#define SPIx_MOSI_PIN                    BSP_IO_PORT_01_PIN_09
#define SPIx_CS_PIN                      BSP_IO_PORT_01_PIN_12
#define SPIx_IRQ_PIN                     BSP_IO_PORT_01_PIN_04

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

/* Event flags for master and slave */
static volatile spi_event_t g_master_event_flag;    // Master Transfer Event completion flag

/*******************************************************************************
 * Types
 ******************************************************************************/

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct s2pi_handle_t
{
    /*! Determines the current driver status. */
    volatile status_t Status;

    /*! Determines the current S2PI slave. */
    volatile s2pi_slave_t Slave;

    /*! A callback function to be called after transfer/run mode is completed. */
    s2pi_callback_t Callback;

    /*! A parameter to be passed to the callback function. */
    void * CallbackParam;

    s2pi_irq_callback_t IrqCallback;

    void * IrqCallbackParam;

    /*! The actual SPI baud rate in bps. */
    uint32_t BaudRate;

} s2pi_handle_t;

typedef struct
{
    bsp_io_port_pin_t CS;
    bsp_io_port_pin_t IRQ;
} s2pi_map_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! Completes the current series of SPI transfers. */
static status_t S2PI_CompleteTransfer(status_t status);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*! The S2PI data handle. */
static s2pi_handle_t spiHnd_ = { 0 };

/*! The S2PI driver handle. */
static spi_instance_ctrl_t * const pSpiDrvHandle = &g_spi0_ctrl;
/*! The S2PI driver config handle. */
static spi_cfg_t const * const pSpiDrvConfig = &g_spi0_cfg;

static const bsp_io_port_pin_t s2pi_gpios_[] = {
    [S2PI_CLK ] = SPIx_SCK_PIN,
    [S2PI_CS ] = SPIx_CS_PIN,
    [S2PI_MOSI] = SPIx_MOSI_PIN,
    [S2PI_MISO] = SPIx_MISO_PIN,
    [S2PI_IRQ ] = SPIx_IRQ_PIN
};

static const s2pi_map_t s2pi_map[] =
{
    [S2PI_SLAVE1] = { .CS = SPIx_CS_PIN, .IRQ = SPIx_IRQ_PIN }
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Initialize the S2PI module.
 * @details Setup the board as a S2PI master, this also sets up up the S2PI
 *          pins.
 *          The SPI interface is initialized with the corresponding default
 *          SPI slave (i.e. CS and IRQ lines) and the default baud rate.
 *
 * @param   defaultSlave The default SPI slave to be addressed right after
 *                            module initialization.
 * @param   baudRate_Bps The default SPI baud rate in bauds-per-second.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Init(s2pi_slave_t defaultSlave, uint32_t baudRate_Bps)
{
    (void)(defaultSlave);
    (void)(baudRate_Bps);

    static bool isInitialized = false;

    if (isInitialized) return STATUS_OK;

    spiHnd_.Status = STATUS_IDLE;
    spiHnd_.BaudRate = 0;
    spiHnd_.Callback = 0;
    spiHnd_.CallbackParam = 0;
    spiHnd_.Slave = SPI_DEFAULT_SLAVE;

    /* Configure CS pin */
    R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) SPIx_CS_PIN, BSP_IO_LEVEL_HIGH);

    /* Configure the IRQ pin */
    R_ICU_ExternalIrqOpen(&g_external_irq0_ctrl, &g_external_irq0_cfg);
    R_ICU_ExternalIrqEnable(&g_external_irq0_ctrl);

    /* Calculate register set values for the requested baudrate */
    rspck_div_setting_t spiSettings;
    fsp_err_t ret = R_SPI_CalculateBitrate(baudRate_Bps, &spiSettings);
    bool invalidSettings = (spiSettings.brdv == 0) && (spiSettings.spbr == 0);
    if ((ret != FSP_SUCCESS) || invalidSettings) return ERROR_FAIL;

    /* Set the requested baudrate */
    spi_extended_cfg_t * pExtCfg = (spi_extended_cfg_t*)pSpiDrvConfig->p_extend;
    pExtCfg->spck_div = spiSettings;

    /* Open the S2PI peripheral */
    if (FSP_SUCCESS != R_SPI_Open(pSpiDrvHandle, pSpiDrvConfig))
        return ERROR_FAIL;

    isInitialized = true;
    spiHnd_.BaudRate = baudRate_Bps;

    return STATUS_OK;
}

status_t S2PI_GetStatus(s2pi_slave_t slave)
{
    (void)slave;
    return spiHnd_.Status;
}

status_t S2PI_TryGetMutex(s2pi_slave_t slave)
{
    (void)slave;
    // Note: Function is currently not used (as the driver only supports single-device mode)
    return STATUS_OK;
}

void S2PI_ReleaseMutex(s2pi_slave_t slave)
{
    (void)slave;
    // Note: Function is currently not used (as the driver only supports single-device mode)
}

/*!***************************************************************************
 * @brief   Transfers a single SPI frame asynchronously.
 * @details Transfers a single SPI frame in asynchronous manner. The Tx data
 *          buffer is written to the device via the MOSI line.
 *          Optionally the data on the MISO line is written to the provided
 *          Rx data buffer. If null, the read data is dismissed.
 *          The transfer of a single frame requires to not toggle the chip
 *          select line to high in between the data frame.
 *          An optional callback is invoked when the asynchronous transfer
 *          is finished. Note that the provided buffer must not change while
 *          the transfer is ongoing. Use the slave parameter to determine
 *          the corresponding slave via the given chip select line.
 *
 * @param   slave The specified S2PI slave.
 * @param   txData The 8-bit values to write to the SPI bus MOSI line.
 * @param   rxData The 8-bit values received from the SPI bus MISO line
 *                   (pass a null pointer if the data don't need to be read).
 * @param   frameSize The number of 8-bit values to be sent/received.
 * @param   callback A callback function to be invoked when the transfer is
 *                     finished. Pass a null pointer if no callback is required.
 * @param   callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK: Successfully invoked the transfer.
 *           - #ERROR_INVALID_ARGUMENT: An invalid parameter has been passed.
 *           - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *           - #STATUS_BUSY: An SPI transfer is already in progress. The
 *                           transfer was not started.
 *           - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode. The transfer
 *                                     was not started.
 *****************************************************************************/
status_t S2PI_TransferFrame(s2pi_slave_t spi_slave,
                            uint8_t const * txData,
                            uint8_t * rxData,
                            size_t frameSize,
                            s2pi_callback_t callback,
                            void * callbackData)
{
    (void)(spi_slave);

    /* Verify arguments. */
    if (!txData || frameSize == 0 || frameSize >= 0x10000) return ERROR_INVALID_ARGUMENT;

    /* Check the driver status and set spi slave.*/
    IRQ_LOCK();
    if (spiHnd_.Status != STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return spiHnd_.Status;
    }
    spiHnd_.Status = STATUS_BUSY;
    IRQ_UNLOCK();

    spiHnd_.Callback = callback;
    spiHnd_.CallbackParam = callbackData;

    R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) SPIx_CS_PIN, BSP_IO_LEVEL_LOW);

    IRQ_LOCK();

    /* Master send data to Slave */
    fsp_err_t err = rxData ? R_SPI_WriteRead(pSpiDrvHandle, txData, rxData, frameSize, SPI_BIT_WIDTH_8_BITS)
                                             : R_SPI_Write(pSpiDrvHandle, txData, frameSize, SPI_BIT_WIDTH_8_BITS);

    IRQ_UNLOCK();

    if (err != FSP_SUCCESS)
        return ERROR_FAIL;
    else
        return STATUS_OK;
}

/*!*****************************************************************************
 * @brief   Captures the S2PI pins for GPIO usage.
 * @details The SPI is disabled (module status: #STATUS_S2PI_GPIO_MODE) and the
 *          pins are configured for GPIO operation. The GPIO control must be
 *          release with the #S2PI_ReleaseGpioControl function in order to
 *          switch back to ordinary SPI functionality.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
// Note: Clock must be HI after capturing
status_t S2PI_CaptureGpioControl(s2pi_slave_t slave)
{
    (void)slave;

    /* Check if something is ongoing. */
    IRQ_LOCK();
    if (spiHnd_.Status != STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return spiHnd_.Status;
    }
    spiHnd_.Status = STATUS_S2PI_GPIO_MODE;
    IRQ_UNLOCK();

    R_IOPORT_PinWrite(&g_ioport_ctrl, s2pi_gpios_[S2PI_CLK], BSP_IO_LEVEL_HIGH);

    R_IOPORT_PinsCfg(&g_ioport_ctrl, &g_bsp_pin_cfg2);

    return STATUS_OK;
}

/*!*****************************************************************************
 * @brief   Releases the S2PI pins from GPIO usage and switches back to SPI mode.
 * @details The GPIO pins are configured for SPI operation and the GPIO mode is
 *          left. Must be called if the pins are captured for GPIO operation via
 *          the #S2PI_CaptureGpioControl function.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave)
{
    (void)slave;

    /* Check if something is ongoing. */
    IRQ_LOCK();
    if (spiHnd_.Status != STATUS_S2PI_GPIO_MODE)
    {
        IRQ_UNLOCK();
        return spiHnd_.Status;
    }
    spiHnd_.Status = STATUS_IDLE;
    IRQ_UNLOCK();

    R_IOPORT_PinsCfg(&g_ioport_ctrl, &g_bsp_pin_cfg3);

    return STATUS_OK;
}

/*!*****************************************************************************
 * @brief   Writes the output for a specified SPI pin in GPIO mode.
 * @details This function writes the value of an SPI pin if the SPI pins are
 *          captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 * @param   slave The specified S2PI slave.
 * @param   pin The specified S2PI pin.
 * @param   value The GPIO pin state to write (0 = low, 1 = high).
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
    (void)(slave);

    if (pin > S2PI_IRQ) return ERROR_INVALID_ARGUMENT;

    /* Check if in GPIO mode. */
    if (spiHnd_.Status != STATUS_S2PI_GPIO_MODE)
        return ERROR_S2PI_INVALID_STATE;

    R_IOPORT_PinWrite(&g_ioport_ctrl, s2pi_gpios_[pin], value);

    S2PI_GPIO_DELAY();

    return STATUS_OK;
}

/*!*****************************************************************************
 * @brief   Reads the input from a specified SPI pin in GPIO mode.
 * @details This function reads the value of an SPI pin if the SPI pins are
 *          captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 * @param   slave The specified S2PI slave.
 * @param   pin The specified S2PI pin.
 * @param   value The GPIO pin state to read (0 = low, 1 = high).
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t * value)
{
    (void)(slave);

    if (pin > S2PI_IRQ) return ERROR_INVALID_ARGUMENT;

    /* Check if in GPIO mode. */
    if (spiHnd_.Status != STATUS_S2PI_GPIO_MODE)
        return ERROR_S2PI_INVALID_STATE;

    *value = R_BSP_PinRead(s2pi_gpios_[pin]);

    S2PI_GPIO_DELAY();

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Set a callback for the GPIO IRQ for a specified S2PI slave.
 *
 * @param   slave The specified S2PI slave.
 * @param   callback A callback function to be invoked when the specified
 *                     S2PI slave IRQ occurs. Pass a null pointer to disable
 *                     the callback.
 * @param   callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK: Successfully installation of the callback.
 *           - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *****************************************************************************/
status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
                             s2pi_irq_callback_t callback,
                             void * callbackData)
{
    (void)(slave);
    spiHnd_.IrqCallback = callback;
    spiHnd_.IrqCallbackParam = callbackData;

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Reads the current status of the IRQ pin.
 * @details In order to keep a low priority for GPIO IRQs, the state of the
 *          IRQ pin must be read in order to reliable check for chip timeouts.
 *
 *          The execution of the interrupt service routine for the data-ready
 *          interrupt from the corresponding GPIO pin might be delayed due to
 *          priority issues. The delayed execution might disable the timeout
 *          for the eye-safety checker too late causing false error messages.
 *          In order to overcome the issue, the state of the IRQ GPIO input
 *          pin is read before raising a timeout error in order to check if
 *          the device has already finished but the IRQ is still pending to be
 *          executed!

 * @param   slave The specified S2PI slave.
 * @return  Returns 1U if the IRQ pin is high (IRQ not pending) and 0U if the
 *          devices pulls the pin to low state (IRQ pending).
 *****************************************************************************/
uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
    (void)(slave);
    return (R_ICU->IELSR_b[g_external_irq0_ctrl.irq].IR) ? 0u : 1u;
}

/*!***************************************************************************
 * @brief   Cycles the chip select line.
 * @details In order to cancel the integration on the ASIC, a fast toggling
 *          of the chip select pin of the corresponding SPI slave is required.
 *          Therefore, this function toggles the CS from high to low and back.
 *          The SPI instance for the specified S2PI slave must be idle,
 *          otherwise the status #STATUS_BUSY is returned.
 * @param   slave The specified S2PI slave.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
    /* Check the driver status. */
    IRQ_LOCK();
    if (spiHnd_.Status != STATUS_IDLE)
    {
        IRQ_UNLOCK();
        return STATUS_BUSY;
    }
    spiHnd_.Status = STATUS_BUSY;
    IRQ_UNLOCK();

    R_IOPORT_PinWrite(&g_ioport_ctrl, s2pi_map[slave].CS, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, s2pi_map[slave].CS, BSP_IO_LEVEL_HIGH);

    spiHnd_.Status = STATUS_IDLE;

    return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Terminates a currently ongoing asynchronous SPI transfer.
 * @details When a callback is set for the current ongoing activity, it is
 *          invoked with the #ERROR_ABORTED error byte.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Abort(s2pi_slave_t slave)
{
    (void)slave;

    IRQ_LOCK();

    /* Check if something is ongoing. */
    if (spiHnd_.Status == STATUS_BUSY)
    {
        /* Abort SPI transfer. */
        R_SPI_Close(pSpiDrvHandle);
        R_SPI_Open(pSpiDrvHandle, pSpiDrvConfig);
    }

    IRQ_UNLOCK();

    status_t status = S2PI_CompleteTransfer(ERROR_ABORTED);
    if (status == ERROR_ABORTED) status = STATUS_OK;

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
static status_t S2PI_CompleteTransfer(status_t status)
{
    spiHnd_.Status = STATUS_IDLE;

    /* Deactivate CS (set high), as we use GPIO pin */
    /* Note: hardcoded default SPI slave because there is no 'source' param in the callback */
    R_IOPORT_PinWrite(&g_ioport_ctrl, s2pi_map[SPI_DEFAULT_SLAVE].CS, BSP_IO_LEVEL_HIGH);

    /* Invoke callback if there is one */
    if (spiHnd_.Callback != 0)
    {
        s2pi_callback_t callback = spiHnd_.Callback;
        spiHnd_.Callback = 0;
        status = callback(status, spiHnd_.CallbackParam);
    }

    /* If the callback ran, then this is the updated status. Otherwise, the received status is
     * returned (no processing was done) */
    return status;
}

uint32_t S2PI_GetBaudRate(s2pi_slave_t slave)
{
    (void)slave;
    return spiHnd_.BaudRate;
}

status_t S2PI_SetBaudRate(s2pi_slave_t slave, uint32_t baudRate_Bps)
{
    (void)slave;

    rspck_div_setting_t spiSettings;
    fsp_err_t ret = R_SPI_CalculateBitrate(baudRate_Bps, &spiSettings);

    bool invalidSettings = (spiSettings.brdv == 0) && (spiSettings.spbr == 0);
    if ((ret != FSP_SUCCESS) || invalidSettings) return ERROR_S2PI_INVALID_BAUDRATE;

    ret = R_SPI_Close(pSpiDrvHandle);
    spi_extended_cfg_t * pExtCfg = (spi_extended_cfg_t*)pSpiDrvConfig->p_extend;
    pExtCfg->spck_div = spiSettings;
    ret += R_SPI_Open(pSpiDrvHandle, pSpiDrvConfig);

    if (ret != FSP_SUCCESS)
        return ERROR_FAIL;
    else
    {
        spiHnd_.BaudRate = baudRate_Bps;
        return STATUS_OK;
    }
}

/*******************************************************************************************************************//**
 * @brief Master SPI callback function.
 * @param[in]  p_args
 * @retval     None
 **********************************************************************************************************************/
void user_spi_callback(spi_callback_args_t * p_args)
{
    if (p_args->event == SPI_EVENT_TRANSFER_COMPLETE)
    {
        // Note: the p_args->pContext can be used to convey slave ID if needed
        g_master_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
        S2PI_CompleteTransfer(STATUS_OK);
    }
    else
    {
        g_master_event_flag = SPI_EVENT_TRANSFER_ABORTED;
    }
}

/*******************************************************************************************************************//**
 * @brief      User defined external irq callback.
 * @param[IN]  p_args
 * @retval     None
 **********************************************************************************************************************/
void user_irq_callback(external_irq_callback_args_t * p_args)
{
    /* Make sure it's the right interrupt*/
    if (p_args->channel == 0x01)
    {
        if (spiHnd_.IrqCallback != 0)
        {
            spiHnd_.IrqCallback(spiHnd_.IrqCallbackParam);
        }
    }
}
