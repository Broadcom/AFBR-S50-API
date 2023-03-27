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

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "gpio.h"

#include <stdint.h>

#include "board/board_config.h"
#include "driver/fsl_clock.h"
#include "driver/fsl_port.h"
#include "driver/irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! GPIO pin structure. */
typedef struct gpio_pin_definition_t
{
    GPIO_Type * base;   /*!< Base address of GPIO port. */
    PORT_Type * port;   /*!< GPIO Port. */
    uint32_t pin;       /*!< GPIO Pin Number. */
    uint32_t mask;      /*!< GPIO Pin Mask. */
    gpio_isr_t isr;     /*!< IRQ callback function. */
    void * isrp;        /*!< IRQ callback parameter pointer. */
} gpio_pin_definition_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void PORTA_IRQHandler(void);        /*!< PORTA pin detect */
void PORTC_PORTD_IRQHandler(void);  /*!< Single interrupt vector for PORTC and PORTD pin detect */

static inline void GPIO_InitPin(gpio_pin_t const pin);
static inline void GPIO_InitIrqPin(gpio_pin_t const pin);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Board Specific Pins */


/*! Macro to create a pin structure.
 *  It first creates a static instance of the pin of type:
 *  static gpio_pin_definition_t myPin_<name> = {...};
 *  Then it initializes the global pointer to that structure:
 *  gpio_pin_t Pin_<name> = &myPin_<name>; */
#define GPIO_MAKE_PIN(name) \
    static gpio_pin_definition_t myPin_##name = { \
        .base = name##_GPIO, \
        .port = name##_PORT, \
        .pin = name##_GPIO_PIN, \
        .mask = 1u << name##_GPIO_PIN, \
        .isr = 0 \
    }; \
    gpio_pin_t Pin_##name = &myPin_##name;

#if defined(CPU_MKL17Z256VFM4)

//GPIO_MAKE_PIN(PTA01);
//GPIO_MAKE_PIN(PTA02);
GPIO_MAKE_PIN(PTE16);
GPIO_MAKE_PIN(PTE17);
GPIO_MAKE_PIN(PTE18);
GPIO_MAKE_PIN(PTE19);

/* S2PI Pins */
GPIO_MAKE_PIN(SPI1_MISO);
GPIO_MAKE_PIN(SPI1_MOSI);
GPIO_MAKE_PIN(SPI1_CLK);
GPIO_MAKE_PIN(S2PI_IRQ1);
GPIO_MAKE_PIN(S2PI_CS1);

#elif defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

//GPIO_MAKE_PIN(BOARD_LED_GREEN); // Pin conflict with Green LED!!
GPIO_MAKE_PIN(LED_RED);
//GPIO_MAKE_PIN(BOARD_SW1);
//GPIO_MAKE_PIN(BOARD_SW3);

GPIO_MAKE_PIN(PTB0);
GPIO_MAKE_PIN(PTB1);
GPIO_MAKE_PIN(PTB2);
GPIO_MAKE_PIN(PTB3);
GPIO_MAKE_PIN(PTC1);
GPIO_MAKE_PIN(PTC2);

/* S2PI Pins */
GPIO_MAKE_PIN(SPI0_MISO);
GPIO_MAKE_PIN(SPI0_MOSI);
GPIO_MAKE_PIN(SPI0_CLK);

GPIO_MAKE_PIN(SPI1_MISO);
GPIO_MAKE_PIN(SPI1_MOSI);
GPIO_MAKE_PIN(SPI1_CLK);

GPIO_MAKE_PIN(S2PI_IRQ1);
GPIO_MAKE_PIN(S2PI_IRQ2);
GPIO_MAKE_PIN(S2PI_IRQ3);
GPIO_MAKE_PIN(S2PI_IRQ4);
GPIO_MAKE_PIN(S2PI_IRQ5);
GPIO_MAKE_PIN(S2PI_IRQ6);

GPIO_MAKE_PIN(S2PI_CS1);
GPIO_MAKE_PIN(S2PI_CS2);
GPIO_MAKE_PIN(S2PI_CS3);
GPIO_MAKE_PIN(S2PI_CS4);
GPIO_MAKE_PIN(S2PI_CS5);
GPIO_MAKE_PIN(S2PI_CS6);

#endif

/*!@brief Collection of all interrupt enabled input pins. */
gpio_pin_t myIrqPins[] = {
#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        /* S2PI Pins */
        &myPin_S2PI_IRQ1,
        &myPin_S2PI_IRQ2,
        &myPin_S2PI_IRQ3,
        &myPin_S2PI_IRQ4,
        &myPin_S2PI_IRQ5,
        &myPin_S2PI_IRQ6,

//        /* Other Pins */
//        &myPin_SW1,
//        &myPin_SW3,
#elif defined(CPU_MKL17Z256VFM4)
        /* S2PI Pins */
        &myPin_S2PI_IRQ1,
#endif
        /* EOL */
        0
};

/*!@brief Collection of all pins (w/o interrupt). */
gpio_pin_t myPins[] = {
#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        /* S2PI Pins */
        &myPin_SPI0_CLK,
        &myPin_SPI0_MOSI,
        &myPin_SPI0_MISO,
        &myPin_SPI1_CLK,
        &myPin_SPI1_MOSI,
        &myPin_SPI1_MISO,

        &myPin_S2PI_CS1,
        &myPin_S2PI_CS2,
        &myPin_S2PI_CS3,
        &myPin_S2PI_CS4,
        &myPin_S2PI_CS5,
        &myPin_S2PI_CS6,

        /* Led pins. */
//      &myPin_LED_GREEN,
        &myPin_LED_RED,

        /* Debug pins */
        &myPin_PTB0,
        &myPin_PTB1,
        &myPin_PTB2,
        &myPin_PTB3,
        &myPin_PTC1,
        &myPin_PTC2,

#elif defined(CPU_MKL17Z256VFM4)
        /* S2PI Pins */
        &myPin_SPI1_CLK,
        &myPin_SPI1_MOSI,
        &myPin_SPI1_MISO,
        &myPin_S2PI_CS1,

         /* Other Pins */
         &myPin_PTE16,
         &myPin_PTE17,
         &myPin_PTE18,
         &myPin_PTE19,
#endif
        /* EOL */
        0
};


/*******************************************************************************
 * Code
 ******************************************************************************/
static inline void GPIO_InitPin(gpio_pin_t const pin)
{
    port_pin_config_t pinConfig = {
            .driveStrength = kPORT_LowDriveStrength,
            .slewRate = kPORT_SlowSlewRate,
            .passiveFilterEnable = kPORT_PassiveFilterDisable,
            .pullSelect = kPORT_PullDisable,
            .mux = kPORT_MuxAsGpio };

    PORT_SetPinConfig(pin->port, pin->pin, &pinConfig);

#if defined(CPU_MKL17Z256VFM4)
    GPIO_SetPinOutput(pin);
#else
    GPIO_ClearPinOutput(pin);
#endif
    GPIO_SetPinDir(pin, Pin_Output);
}

static inline void GPIO_InitIrqPin(gpio_pin_t const pin)
{
    port_pin_config_t pinConfig = {
            .driveStrength = kPORT_LowDriveStrength,
            .slewRate = kPORT_SlowSlewRate,
            .passiveFilterEnable = kPORT_PassiveFilterDisable,
            .pullSelect = kPORT_PullUp,
            .mux = kPORT_MuxAsGpio };

    PORT_SetPinConfig(pin->port, pin->pin, &pinConfig);
    GPIO_SetPinDir(pin, Pin_Input);
    GPIO_RemoveISR(pin);
}

void GPIO_Init(void)
{
    static bool isInitialized = false;
    if(!isInitialized)
    {
        /* Un-gate port clock*/
#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

        CLOCK_EnableClock(kCLOCK_PortA);
        CLOCK_EnableClock(kCLOCK_PortB);
        CLOCK_EnableClock(kCLOCK_PortC);
        CLOCK_EnableClock(kCLOCK_PortD);
        CLOCK_EnableClock(kCLOCK_PortE);

#elif defined(CPU_MKL17Z256VFM4)

        CLOCK_EnableClock(kCLOCK_PortA);
        CLOCK_EnableClock(kCLOCK_PortD);
        CLOCK_EnableClock(kCLOCK_PortE);

#endif

        unsigned i = 0;

        /* Initialize input pins. */
        while(myIrqPins[i])
        {
            GPIO_InitIrqPin(myIrqPins[i]);
            ++i;
        }

        i = 0;

        /* Initialize output pins. */
        while(myPins[i])
        {
            GPIO_InitPin(myPins[i]);
            ++i;
        }

        /* Enable GPIO interrupt.*/
        NVIC_SetPriority(PORTA_IRQn, IRQPRIO_GPIOA);
        NVIC_SetPriority(PORTC_PORTD_IRQn, IRQPRIO_GPIOCD);
        EnableIRQ(PORTA_IRQn);
        EnableIRQ(PORTC_PORTD_IRQn);

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
        /* Intitialize pins */
        GPIO_SetPinOutput(Pin_LED_RED);
//      GPIO_SetPinOutput(Pin_LED_GREEN);
#endif

        isInitialized = true;
    }
}

void GPIO_SetISR(gpio_pin_t pin, gpio_isr_t f, void * p)
{
    pin->isr = f;
    pin->isrp = p;

    /* Set input pin interrupts. */
    PORT_SetPinInterruptConfig(pin->port, pin->pin, kPORT_InterruptFallingEdge);
}
void GPIO_GetISR(gpio_pin_t pin, gpio_isr_t * f, void ** p)
{
    *f = pin->isr;
    *p = pin->isrp;
}

void GPIO_RemoveISR(gpio_pin_t pin)
{
    pin->isr = 0;
    pin->isrp = 0;

    /* Disable input pin interrupts. */
    PORT_SetPinInterruptConfig(pin->port, pin->pin, kPORT_InterruptOrDMADisabled);
}

/*******************************************************************************
 * Code for inline functions
 ******************************************************************************/
void GPIO_SetPinMux(gpio_pin_t pin, uint32_t mux)
{
    PORT_SetPinMux(pin->port, pin->pin, mux);
}

void GPIO_SetPinOutput(gpio_pin_t pin)
{
    (pin->base)->PSOR = pin->mask;
}

void GPIO_ClearPinOutput(gpio_pin_t pin)
{
    (pin->base)->PCOR = pin->mask;
}

void GPIO_TogglePinOutput(gpio_pin_t pin)
{
    (pin->base)->PTOR = pin->mask;
}

void GPIO_WritePinOutput(gpio_pin_t pin, uint32_t value)
{
    if (value == 0U)
    {
        (pin->base)->PCOR = pin->mask;
    }
    else
    {
        (pin->base)->PSOR = pin->mask;
    }
}

uint32_t GPIO_ReadPinInput(gpio_pin_t pin)
{
    return ((pin->base)->PDIR >> pin->pin) & 1U;
}

uint32_t GPIO_GetInterruptStatus(gpio_pin_t pin)
{
    return (PORT_GetPinsInterruptFlags(pin->port) >> pin->pin) & 1U;
}

void GPIO_SetPinDir(gpio_pin_t pin, gpio_pin_direction_t dir )
{
    if (dir == Pin_Input)
    {
        (pin->base)->PDDR &= ~(1U << pin->pin);
    }
    else
    {
        (pin->base)->PDDR |= (1U << pin->pin);
    }
}

void GPIO_SetPinPullUp(gpio_pin_t pin)
{
    uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    uint32_t val = PORT_PCR_PE(1) | PORT_PCR_PS(1); // Pull enable + select high
    pin->port->PCR[pin->pin] = (pin->port->PCR[pin->pin] & ~msk) | val;
}

void GPIO_SetPinPullDown(gpio_pin_t pin)
{
    uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    uint32_t val = PORT_PCR_PE(1) | PORT_PCR_PS(0); // Pull enable + select down
    pin->port->PCR[pin->pin] = (pin->port->PCR[pin->pin] & ~msk) | val;
}

void GPIO_SetPinPullDisable(gpio_pin_t pin)
{
    uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    uint32_t val = PORT_PCR_PE(0); // Pull disable
    pin->port->PCR[pin->pin] = (pin->port->PCR[pin->pin] & ~msk) | val;
}

/*!
 * @brief Interrupt service function of PORTC and PORTD.
 *
 * This function toggles the IRQ callback, if installed
 */
static void GPIO_IRQHandler(void)
{
    /* Check IRQS on PORTC. */
    gpio_pin_t * pin = myIrqPins;
    do
    {
        /* Workaround for missing GPIO interrupt issue:
         * Sometimes, the GPIO interrupt is missed which causes the device to get stuck.
         * This seems to be a timing issue if interrupts happen at the same time (probably
         * between reading and resetting the ISRF register). The interrupt still gets called
         * but the ISRF register reads 0 (or only a single flag is set), which is obviously
         * wrong. Thus, we double check the IRQ pin interrupt state by reading the IRQ pin
         * and invoke the callback for each pin that has low state.
         * NOTE: if the IRQ pin is not cleared in the generating device, the callback might
         * get called again in the next IRQ event that is generated from a different pin. */
        if (!GPIO_ReadPinInput(*pin))
        {
            if ((*pin)->isr)
            {
                (*pin)->isr((*pin)->isrp);
            }
        }
    } while (*(++pin));
}

/*!
 * @brief Interrupt service function of PORTA.
 *
 * This function calls the IRQ callback, if installed
 */
void PORTA_IRQHandler(void)
{
    /* Get external interrupt flags and copy locally. */
    const uint32_t isfr = PORTA->ISFR;

    /* Clear external interrupt flags. */
    PORTA->ISFR = isfr;

    GPIO_IRQHandler();
}

/*!
 * @brief Interrupt service function of PORTC and PORTD.
 *
 * This function toggles the IRQ callback, if installed
 */
void PORTC_PORTD_IRQHandler(void)
{
    /* Get external interrupt flags. */
    uint32_t isfr_portc = PORTC->ISFR;
    uint32_t isfr_portd = PORTD->ISFR;
    // assert(isfr_portc != 0 || isfr_portd != 0);

    /* Clear external interrupt flags. */
    PORTC->ISFR = isfr_portc;
    PORTD->ISFR = isfr_portd;

    GPIO_IRQHandler();
}

