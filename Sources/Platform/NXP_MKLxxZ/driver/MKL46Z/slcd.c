/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides drivers for SLCD display
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
#include "slcd.h"
#include "devices/MKL46Z/MKL46Z4.h"
#include "driver/fsl_port.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
//Create Macros for each pin
#define BACKPLANE0          40u                     /*!< SLCD COM1 --- LCD_P40. */
#define BACKPLANE1          52u                     /*!< SLCD COM2 --- LCD_P52. */
#define BACKPLANE2          19u                     /*!< SLCD COM3 --- LCD_P19. */
#define BACKPLANE3          18u                     /*!< SLCD COM4 --- LCD_P18. */
#define FRONTPLANE0         37u                     /*!< SLCD P05  --- LCD_P37. */
#define FRONTPLANE1         17u                     /*!< SLCD P06  --- LCD_P17. */
#define FRONTPLANE2          7u                     /*!< SLCD P07  --- LCD_P07. */
#define FRONTPLANE3          8u                     /*!< SLCD P08  --- LCD_P08. */
#define FRONTPLANE4         53u                     /*!< SLCD P09  --- LCD_P53. */
#define FRONTPLANE5         38u                     /*!< SLCD P10  --- LCD_P38. */
#define FRONTPLANE6         10u                     /*!< SLCD P11  --- LCD_P10. */
#define FRONTPLANE7         11u                     /*!< SLCD P12  --- LCD_P11. */

//Define Number of Front and Back plane pins
#define NUM_FRONTPLANE_PINS 8u                      /*!< Number of frontplane pins. */
#define NUM_BACKPLANE_PINS  4u                      /*!< Number of backplane pins. */
#define NUM_DIGITS          4u                      /*!< Digit count. */

#define PHASE_OFF           0x00U                   /*!< LCD waveform no phase activates. */
#define PHASE_A             0x01U                   /*!< LCD waveform phase A activates. */
#define PHASE_B             0x02U                   /*!< LCD waveform phase B activates. */
#define PHASE_C             0x04U                   /*!< LCD waveform phase C activates. */
#define PHASE_D             0x08U                   /*!< LCD waveform phase D activates. */
#define PHASE_E             0x10U                   /*!< LCD waveform phase E activates. */
#define PHASE_F             0x20U                   /*!< LCD waveform phase F activates. */
#define PHASE_G             0x40U                   /*!< LCD waveform phase G activates. */
#define PHASE_H             0x80U                   /*!< LCD waveform phase H activates. */


#define PHASE_0_A (PHASE_A | PHASE_B | PHASE_D)                 /*!< Digit 0: Phase A */
#define PHASE_0_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit 0: Phase B */
#define PHASE_1_A (PHASE_OFF)                                   /*!< Digit 1: Phase A */
#define PHASE_1_B (PHASE_B | PHASE_C)                           /*!< Digit 1: Phase B */
#define PHASE_2_A (PHASE_A | PHASE_B | PHASE_C)                 /*!< Digit 2: Phase A */
#define PHASE_2_B (PHASE_C | PHASE_D)                           /*!< Digit 2: Phase B */
#define PHASE_3_A (PHASE_A | PHASE_C)                           /*!< Digit 3: Phase A */
#define PHASE_3_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit 3: Phase B */
#define PHASE_4_A (PHASE_C | PHASE_D)                           /*!< Digit 4: Phase A */
#define PHASE_4_B (PHASE_B | PHASE_C)                           /*!< Digit 4 Phase B */
#define PHASE_5_A (PHASE_A | PHASE_C | PHASE_D)                 /*!< Digit 5: Phase A */
#define PHASE_5_B (PHASE_B | PHASE_D)                           /*!< Digit 5: Phase B */
#define PHASE_6_A (PHASE_A | PHASE_B | PHASE_C | PHASE_D)       /*!< Digit 6: Phase A */
#define PHASE_6_B (PHASE_B | PHASE_D)                           /*!< Digit 6: Phase B */
#define PHASE_7_A (PHASE_OFF)                                   /*!< Digit 7: Phase A */
#define PHASE_7_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit 7: Phase B */
#define PHASE_8_A (PHASE_A | PHASE_B | PHASE_C | PHASE_D)       /*!< Digit 8: Phase A */
#define PHASE_8_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit 8: Phase B */
#define PHASE_9_A (PHASE_A | PHASE_C | PHASE_D)                 /*!< Digit 9: Phase A */
#define PHASE_9_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit 9: Phase B */
#define PHASE_A_A (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit A: Phase A */
#define PHASE_A_B (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit A: Phase B */
#define PHASE_B_A (PHASE_A | PHASE_B | PHASE_C | PHASE_D)       /*!< Digit B: Phase A */
#define PHASE_B_B (PHASE_B)                                     /*!< Digit B: Phase B */
#define PHASE_C_A (PHASE_A | PHASE_B | PHASE_D)                 /*!< Digit C: Phase A */
#define PHASE_C_B (PHASE_D)                                     /*!< Digit C: Phase B */
#define PHASE_D_A (PHASE_A | PHASE_B | PHASE_C)                 /*!< Digit D: Phase A */
#define PHASE_D_B (PHASE_B | PHASE_C)                           /*!< Digit D: Phase B */
#define PHASE_E_A (PHASE_A | PHASE_B | PHASE_C | PHASE_D)       /*!< Digit E: Phase A */
#define PHASE_E_B (PHASE_D)                                     /*!< Digit E: Phase B */
#define PHASE_F_A (PHASE_B | PHASE_C | PHASE_D)                 /*!< Digit F: Phase A */
#define PHASE_F_B (PHASE_D)                                     /*!< Digit F: Phase B */

#define PHASE_MINUS_A (PHASE_C)                                 /*!< Digit -: Phase A */
#define PHASE_MINUS_B (PHASE_OFF)                               /*!< Digit -: Phase B */
#define PHASE_ERROR_A (PHASE_A | PHASE_B | PHASE_C | PHASE_D)   /*!< Digit error: Phase A */
#define PHASE_ERROR_B (PHASE_D)                                 /*!< Digit error: Phase B */
#define PHASE_CLEAR   (PHASE_OFF)                               /*!< Clear */
#define PHASE_ALL     (PHASE_A | PHASE_B | PHASE_C | PHASE_D)   /*!< All */

#define SLCD_WAVEFORM_CONFIG_NUM 16                             /*!< Waveform configuration count. */

static const uint8_t FrontplaneA[NUM_DIGITS] = {FRONTPLANE0, FRONTPLANE2, FRONTPLANE4, FRONTPLANE6};
static const uint8_t FrontplaneB[NUM_DIGITS] = {FRONTPLANE1, FRONTPLANE3, FRONTPLANE5, FRONTPLANE7};
static const uint8_t PhaseA[0x10] = {PHASE_0_A, PHASE_1_A, PHASE_2_A, PHASE_3_A, PHASE_4_A, PHASE_5_A, PHASE_6_A, PHASE_7_A, PHASE_8_A, PHASE_9_A, PHASE_A_A, PHASE_B_A, PHASE_C_A, PHASE_D_A, PHASE_E_A, PHASE_F_A};
static const uint8_t PhaseB[0x10] = {PHASE_0_B, PHASE_1_B, PHASE_2_B, PHASE_3_B, PHASE_4_B, PHASE_5_B, PHASE_6_B, PHASE_7_B, PHASE_8_B, PHASE_9_B, PHASE_A_B, PHASE_B_B, PHASE_C_B, PHASE_D_B, PHASE_E_B, PHASE_F_B};


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Sets a value from 0-F to a specified digit.
 * @param   value Value to be set (0...F)
 * @param   digit 1 is the leftmost, 4 the rightmost digit.
 *****************************************************************************/
static void SLCD_Set(uint8_t value, uint8_t digit);

/*!***************************************************************************
 * @brief   Initializes the SLCD output pins.
*****************************************************************************/
static void SLCD_InitPins(void);

/*!***************************************************************************
 * @brief   Sets the SLCD back plane pin phase.
 * @param   pinIndx SLCD back plane pin index. Range from 0 to 63.
 * @param   phase   The phase activates for the back plane pin.
 *****************************************************************************/
static inline void SLCD_SetBackPlanePhase(uint32_t pinIndx, uint8_t phase);

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef DEBUG
static volatile bool isInitialized = false;
#endif


void SLCD_Init(void)
{
    assert(!isInitialized);

    /* Ungate the clock */
    CLOCK_EnableClock(kCLOCK_Slcd0);

    /* Un-gate the SLCD clock. */
    CLOCK_EnableClock(kCLOCK_Slcd0);

    /* Set Pin Muxing */
    SLCD_InitPins();

    /* SLCD Initialize. */
    uint32_t gcrReg = 0;

    /* Configure general setting: power supply. */
    gcrReg = LCD_GCR_CPSEL(1U) |        /* VLL3 connected to VDD internally, charge pump is used to generate VLL1 and VLL2. */
             LCD_GCR_LADJ(3U);      /* Adjust in high load or selects slowest clock. */

    /* Configure general setting: clock source. */
    gcrReg |= LCD_GCR_SOURCE(1U) |      /* Select alternate clock source 1 : MCGIRCLK. */
              LCD_GCR_LCLK(1U) |        /* Prescaler 1. */
              LCD_GCR_ALTDIV(0U);       /* No divide for alternate clock. */

    /* Configure the duty and set the work for low power wait and stop mode. */
    gcrReg |= LCD_GCR_DUTY(3U) |        /*  LCD use 4 BP 1/4 duty cycle. */
              LCD_GCR_LCDSTP(0);        /* SLCD does not work in stop mode. */

    gcrReg |= LCD_GCR_LCDDOZE(0);       /* SLCD does not work in wait mode. */

    /* Select the alternate clock for alternate clock source. */
    gcrReg |= LCD_GCR_ALTSOURCE(1U);    /* Select alternate clock source 1 : MCGIRCLK. */

    /* Configure the for fast frame rate. */
    gcrReg |= LCD_GCR_FFR( 0U);         /* Fast frame rate enable flag. */

    gcrReg |= LCD_GCR_RVTRIM(8U);       /* Work in low power mode. Increase the voltage to 0.92 V. */

    LCD->GCR = gcrReg;

    /* Set display mode to normal. */
    LCD->AR = 0;

    /* Configure the front plane and back plane pin setting. */
    LCD->BPEN[0] = 0x000c0000U;         /* Backplane Low Pin: LCD_P19/P18 --> b19/b18. */
    LCD->BPEN[1] = 0x00100100U;         /* Backplane High Pin: LCD_P52/P40 --> b20/b8. */
    LCD->PEN[0] =  0x000e0d80U;         /* Backplane Low Pin Enabled: LCD_P19/P18/P17/P11/P10/P8/P7. */
    LCD->PEN[1] =  0x00300160U;         /* Backplane High Pin Enabled: LCD_P53/P52/P40/P38/P37. */

    /* Disable the fault frame detection. */
    LCD->FDCR = 0;

    /* Initialize the Waveform. */
    for (int regNum = 0; regNum < SLCD_WAVEFORM_CONFIG_NUM; regNum++)
    {
        LCD->WF[regNum] = 0;
    }
    /* Set SLCD back plane phase. */
    SLCD_SetBackPlanePhase(BACKPLANE0, PHASE_A); /* SLCD COM1 --- LCD_P40. */
    SLCD_SetBackPlanePhase(BACKPLANE1, PHASE_B); /* SLCD COM2 --- LCD_P52. */
    SLCD_SetBackPlanePhase(BACKPLANE2, PHASE_C); /* SLCD COM3 --- LCD_P19. */
    SLCD_SetBackPlanePhase(BACKPLANE3, PHASE_D); /* SLCD COM4 --- LCD_P18. */


    /* Starts SLCD display. */
    LCD->GCR |= LCD_GCR_LCDEN_MASK;

#ifdef DEBUG
    isInitialized = true;
#endif

    SLCD_ClearDisplay();
}


static inline void SLCD_SetBackPlanePhase(uint32_t pinIndx, uint8_t phase)
{
    LCD->WF8B[pinIndx] = phase; /* Set Back Plane Phase. */
}


void SLCD_DisplayDecimalUnsigned(uint16_t value)
{
    assert(isInitialized);

    if (value > 9999)
    {
        SLCD_DisplayError(0x00); //Display "Err" if value is greater than 4 digits
    }
    else
    {
        SLCD_ClearDisplay();
        if (value > 999U) SLCD_Set((uint8_t)(((value / 1000U))), 0U);
        if (value > 99U) SLCD_Set((uint8_t)((value - (value / 1000U) * 1000U) / 100U), 1U);
        if (value > 9U) SLCD_Set((uint8_t)((value - (value / 100U) * 100U) / 10U), 2U);
        SLCD_Set((uint8_t)((value - (value / 10U) * 10U)), 3U);
    }
}

void SLCD_DisplayDecimalSigned(int16_t value)
{
    assert(isInitialized);

    if (value < 0)
    {
        if (value < -999)
        { // out of range error
            SLCD_DisplayError(0x00); // Display "Err" if value is greater than 4 digits
        }
        else
        {
            SLCD_ClearDisplay();
            if (value < -99)
            {
                SLCD_SetBackPlanePhase(FrontplaneA[0], PHASE_MINUS_A);
                SLCD_SetBackPlanePhase(FrontplaneB[0], PHASE_MINUS_B);
                SLCD_Set((uint8_t)(-value / 100), 1U);
                SLCD_Set((uint8_t)(((value / 100) * 100 - value) / 10), 2U);
                SLCD_Set((uint8_t)(((value / 10) * 10 - value)), 3U);
            }
            else if (value < -9)
            {
                SLCD_SetBackPlanePhase(FrontplaneA[1], PHASE_MINUS_A);
                SLCD_SetBackPlanePhase(FrontplaneB[1], PHASE_MINUS_B);
                SLCD_Set((uint8_t)(-value / 10), 2U);
                SLCD_Set((uint8_t)(((value / 10) * 10 - value)), 3U);
            }
            else
            {
                SLCD_SetBackPlanePhase(FrontplaneA[2], PHASE_MINUS_A);
                SLCD_SetBackPlanePhase(FrontplaneB[2], PHASE_MINUS_B);
                SLCD_Set((uint8_t)(-value), 3U);
            }
        }
    }
    else
    {
        SLCD_DisplayDecimalUnsigned((uint16_t)value);
    }
}

void SLCD_DisplayError(uint8_t error)
{
    assert(isInitialized);

    SLCD_SetBackPlanePhase(FrontplaneA[0], PHASE_ERROR_A);
    SLCD_SetBackPlanePhase(FrontplaneB[0], PHASE_ERROR_B);
    SLCD_SetBackPlanePhase(FrontplaneA[1], PHASE_MINUS_A);
    SLCD_SetBackPlanePhase(FrontplaneB[1], PHASE_MINUS_B);
    SLCD_Set((error & 0xf0) >> 4, 2); // Display error in digit 3 and 4
    SLCD_Set((error & 0x0f) >> 0, 3);
}

void SLCD_SetDecimalPointPosition(uint8_t pos)
{
    assert(isInitialized);

    for (unsigned i = 0; i < NUM_DIGITS; ++i)
    {
        if (pos == i + 1)
        {
            LCD->WF8B[FrontplaneB[i]] |= PHASE_A; /* Set Phase A */
        }
        else
        {
            LCD->WF8B[FrontplaneB[i]] &= (uint8_t)(~PHASE_A); /* Unset Phase A */
        }
    }
}


void SLCD_DisplayBar(void)
{
    assert(isInitialized);

    for (unsigned i = 0; i < NUM_DIGITS; ++i)
    {
        SLCD_SetBackPlanePhase(FrontplaneA[i], PHASE_MINUS_A);
        SLCD_SetBackPlanePhase(FrontplaneB[i], PHASE_MINUS_B);
    }
}

void SLCD_ClearDisplay(void)
{
    assert(isInitialized);

    for (unsigned i = 0; i < NUM_DIGITS; ++i)
    {
        SLCD_SetBackPlanePhase(FrontplaneA[i], PHASE_CLEAR);
        SLCD_SetBackPlanePhase(FrontplaneB[i], PHASE_CLEAR);
    }
}

static void SLCD_Set(uint8_t Value, uint8_t Digit)
{
    if (Digit < NUM_DIGITS)
    {
        SLCD_SetBackPlanePhase(FrontplaneA[Digit], PhaseA[Value & 0x0f]);
        SLCD_SetBackPlanePhase(FrontplaneB[Digit], PhaseB[Value & 0x0f]);
    }
    else
    {
        // Display "Err" if trying to access a digit that does not exist
        SLCD_DisplayError(0x00);
    }
}//End SegLCD_Set

static void SLCD_InitPins(void)
{
    /* Set current pin as gpio.*/
    /* From Reference manual, set pins to MUX 0 for normal LCD display operation, only use MUX 7 if using LCD fault detection */
    CLOCK_EnableClock(kCLOCK_PortB); /* Ungates the port clock */
    PORT_SetPinMux(PORTB, 7u, kPORT_PinDisabledOrAnalog); /* LCD_P07 */
    PORT_SetPinMux(PORTB, 8u, kPORT_PinDisabledOrAnalog); /* LCD_P08 */
    PORT_SetPinMux(PORTB, 10u, kPORT_PinDisabledOrAnalog); /* LCD_P10 */
    PORT_SetPinMux(PORTB, 11u, kPORT_PinDisabledOrAnalog); /* LCD_P11 */
    PORT_SetPinMux(PORTB, 21u, kPORT_PinDisabledOrAnalog); /* LCD_P17 */
    PORT_SetPinMux(PORTB, 22u, kPORT_PinDisabledOrAnalog); /* LCD_P18 */
    PORT_SetPinMux(PORTB, 23u, kPORT_PinDisabledOrAnalog); /* LCD_P19 */

    CLOCK_EnableClock(kCLOCK_PortC); /* Ungates the port clock */
    PORT_SetPinMux(PORTC, 17u, kPORT_PinDisabledOrAnalog); /* LCD_P37 */
    PORT_SetPinMux(PORTC, 18u, kPORT_PinDisabledOrAnalog); /* LCD_P38 */

    CLOCK_EnableClock(kCLOCK_PortD); /* Ungates the port clock */
    PORT_SetPinMux(PORTD, 0u, kPORT_PinDisabledOrAnalog); /* LCD_P40 */

    CLOCK_EnableClock(kCLOCK_PortE); /* Ungates the port clock */
    PORT_SetPinMux(PORTE, 4u, kPORT_PinDisabledOrAnalog); /* LCD_P52 */
    PORT_SetPinMux(PORTE, 5u, kPORT_PinDisabledOrAnalog); /* LCD_P53 */
}
