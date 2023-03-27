/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides driver functionality for GPIO.
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

#ifndef GPIO_H
#define GPIO_H

/*!***************************************************************************
 * @defgroup    GPIO GPIO: General Purpose Input/Output
 * @ingroup     driver
 * @brief       GPIO Hardware Module
 * @details     Provides functionality for GPIO operation.
 * @addtogroup  GPIO
 * @{
 *****************************************************************************/

#include <stdint.h>

/*!***************************************************************************
 * @brief   GPIO layer interrupt service routine function type.
 * @param   param A void pointer that is passed to the function.
 *****************************************************************************/
typedef void (* gpio_isr_t)(void * param);

/*! GPIO layer pin type. */
typedef struct gpio_pin_definition_t gpio_pin_definition_t;
typedef gpio_pin_definition_t * gpio_pin_t;

/*! GPIO direction definition*/
typedef enum gpio_pin_direction_t
{
    /*! Set current pin as input. */
    Pin_Input  = 0U,

    /*! Set current pin as output. */
    Pin_Output = 1U,

} gpio_pin_direction_t;

#if defined(CPU_MKL17Z256VFM4)

//extern gpio_pin_t Pin_PTA01;          /*!< Pin: Port A, Pin 1 */
//extern gpio_pin_t Pin_PTA02;          /*!< Pin: Port A, Pin 2 */

extern gpio_pin_t Pin_PTE16;            /*!< Pin: Port E, Pin 16 */
extern gpio_pin_t Pin_PTE17;            /*!< Pin: Port E, Pin 17 */
extern gpio_pin_t Pin_PTE18;            /*!< Pin: Port E, Pin 18 */
extern gpio_pin_t Pin_PTE19;            /*!< Pin: Port E, Pin 19 */

/* S2PI Interface Pins */
extern gpio_pin_t Pin_SPI1_MISO;        /*!< Pin: SPI1: MISO, master-in-slave-out */
extern gpio_pin_t Pin_SPI1_MOSI;        /*!< Pin: SPI1: MOSI, master-out-slave-in */
extern gpio_pin_t Pin_SPI1_CLK;         /*!< Pin: SPI1: CLK, clock */
extern gpio_pin_t Pin_S2PI_IRQ1;        /*!< Pin: S2PI: IRQ1, slave 1 interrupt */
extern gpio_pin_t Pin_S2PI_CS1;         /*!< Pin: S2PI: CS1, slave 1 chip select */



#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

/* Board Specific Pins */
//extern gpio_pin_t Pin_LED_GREEN;        /*!< Pin: Green LED */
extern gpio_pin_t Pin_LED_RED;          /*!< Pin: Red LED   */
//extern gpio_pin_t Pin_SW1;              /*!< Pin: Switch 1  */
//extern gpio_pin_t Pin_SW3;              /*!< Pin: Switch 2  */

extern gpio_pin_t Pin_PTB0;             /*!< Pin: PortB Pin0 - Debug Pin */
extern gpio_pin_t Pin_PTB1;             /*!< Pin: PortB Pin1 - Debug Pin */
extern gpio_pin_t Pin_PTB2;             /*!< Pin: PortB Pin2 - Debug Pin */
extern gpio_pin_t Pin_PTB3;             /*!< Pin: PortB Pin3 - Debug Pin */
extern gpio_pin_t Pin_PTC1;             /*!< Pin: PortC Pin1 - Debug Pin */
extern gpio_pin_t Pin_PTC2;             /*!< Pin: PortC Pin2 - Debug Pin */

/* S2PI Interface Pins */
extern gpio_pin_t Pin_SPI0_MISO;        /*!< Pin: SPI0: MISO, master-in-slave-out */
extern gpio_pin_t Pin_SPI0_MOSI;        /*!< Pin: SPI0: MOSI, master-out-slave-in */
extern gpio_pin_t Pin_SPI0_CLK;         /*!< Pin: SPI0: CLK, clock */

extern gpio_pin_t Pin_SPI1_MISO;        /*!< Pin: SPI1: MISO, master-in-slave-out */
extern gpio_pin_t Pin_SPI1_MOSI;        /*!< Pin: SPI1: MOSI, master-out-slave-in */
extern gpio_pin_t Pin_SPI1_CLK;         /*!< Pin: SPI1: CLK, clock */

extern gpio_pin_t Pin_S2PI_IRQ1;        /*!< Pin: S2PI: IRQ1, slave 1 interrupt */
extern gpio_pin_t Pin_S2PI_IRQ2;        /*!< Pin: S2PI: IRQ2, slave 2 interrupt */
extern gpio_pin_t Pin_S2PI_IRQ3;        /*!< Pin: S2PI: IRQ3, slave 3 interrupt */
extern gpio_pin_t Pin_S2PI_IRQ4;        /*!< Pin: S2PI: IRQ4, slave 4 interrupt */
extern gpio_pin_t Pin_S2PI_IRQ5;        /*!< Pin: S2PI: IRQ5, slave 5 interrupt */
extern gpio_pin_t Pin_S2PI_IRQ6;        /*!< Pin: S2PI: IRQ6, slave 6 interrupt */

extern gpio_pin_t Pin_S2PI_CS1;         /*!< Pin: S2PI: CS1, slave 1 chip select */
extern gpio_pin_t Pin_S2PI_CS2;         /*!< Pin: S2PI: CS2, slave 2 chip select */
extern gpio_pin_t Pin_S2PI_CS3;         /*!< Pin: S2PI: CS3, slave 3 chip select */
extern gpio_pin_t Pin_S2PI_CS4;         /*!< Pin: S2PI: CS4, slave 4 chip select */
extern gpio_pin_t Pin_S2PI_CS5;         /*!< Pin: S2PI: CS5, slave 5 chip select */
extern gpio_pin_t Pin_S2PI_CS6;         /*!< Pin: S2PI: CS6, slave 6 chip select */

#endif

/*!***************************************************************************
 * @brief   Initializes the GPIO driver and does pin muxing.
 * @details The first call to this function does the initialization. Every
 *          further call does not have an effect and just returns.
 *          The clock to all ports are ungated and the interrupts are enabled.
 *          The pins are categorized into interrupt-enabled pins and common
 *          input output pins. They are are initialized with the following setup:
 *              - Low drive strength.
 *              - Slow slew rate.
 *              - Passive filter disabled.
 *              - Pull disable (common) or pull up (interrupt-enable).
 *              - Pin muxing as GPIO.
 *              - Default state for output pins is low (0).
 *              - IRQ for interrupt-enable pins disabled (use #GPIO_SetISR to
 *                enable and intall a callback).
 *              .
 *****************************************************************************/
void GPIO_Init(void);

/*!***************************************************************************
 * @brief   Set the output of an GPIO pin to high level.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinOutput(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Clears the output of an GPIO pin to low level.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_ClearPinOutput(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Toggles the output of an GPIO pin to the inverse level.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_TogglePinOutput(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Writes the output of an GPIO pin corresponding to the parameter.
 * @param   pin The address of the GPIO pin.
 * @param   value==0: clear pin; !=0: set pin.
 *****************************************************************************/
void GPIO_WritePinOutput(gpio_pin_t pin, uint32_t value);

/*!***************************************************************************
 * @brief   Read the input of an GPIO pin.
 * @param   pin The address of the GPIO pin.
 * @return  0 for low level, 1 for high level input signal.
 *****************************************************************************/
uint32_t GPIO_ReadPinInput(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Read the interrupt status of an GPIO pin.
 * @param   pin The address of the GPIO pin.
 * @return  Returns 1 if a interrupt is currently pending.
 *****************************************************************************/
uint32_t GPIO_GetInterruptStatus(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Sets the direction (input/output) of an GPIO pin.
 * @param   pin The address of the GPIO pin.
 * @param   dir The direction.
 *****************************************************************************/
void GPIO_SetPinDir(gpio_pin_t pin, gpio_pin_direction_t dir);

/*!***************************************************************************
 * @brief   Sets an interrupt callback for an specified pin.
 * @param   pin The address of the GPIO pin.
 * @param   f The callback function pointer.
 * @param   p A void pointer to be passed to the callback function.
 *****************************************************************************/
void GPIO_SetISR(gpio_pin_t pin, gpio_isr_t f, void * p);

/*!***************************************************************************
 * @brief   Gets the current interrupt callback for an specified pin.
 * @param   pin The address of the GPIO pin.
 * @param   f The callback function pointer.
 * @param   p The callback parameter.
 *****************************************************************************/
void GPIO_GetISR(gpio_pin_t pin, gpio_isr_t * f, void ** p);

/*!***************************************************************************
 * @brief   Removes the interrupt callback.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_RemoveISR(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Sets the pin muxing for a specified pin.
 * @param   pin The address of the GPIO pin.
 * @param   mux The pin muxing slot selection.
 *                  - 0: Pin disabled or work in analog function.
 *                  - 1: Set as GPIO.
 *                  - 2-7: chip-specific.
 *****************************************************************************/
void GPIO_SetPinMux(gpio_pin_t pin, uint32_t mux);

/*!***************************************************************************
 * @brief   Selects the internal pin pull-up for a specified pin.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullUp(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Selects the internal pin pull-down for a specified pin.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullDown(gpio_pin_t pin);

/*!***************************************************************************
 * @brief   Selects the internal pin pull-disable for a specified pin.
 * @param   pin The address of the GPIO pin.
 *****************************************************************************/
void GPIO_SetPinPullDisable(gpio_pin_t pin);

/*! @} */
#endif /* GPIO_H */
