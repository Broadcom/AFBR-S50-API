/*************************************************************************//**
 * @file
 * @brief   Provides a CLI to run crosstalk calibration with the AFBR-S50 API.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/*!***************************************************************************
 * @addtogroup  argus_xtk_cli
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus_xtalk_cal_cli.h"

#include "platform/argus_print.h"
#include "driver/uart.h"
#include "utility/fp_mul.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Macro to proceed to next step, see #User_Query.
 *
 * @details Supportive function for the CLI.
 *****************************************************************************/
#define READY() (rxdataS[0]=='y')

/*!***************************************************************************
 * @brief   Macro to abort crosstalk calibration, see #User_Query.
 *
 * @details Supportive function for the CLI.
 *****************************************************************************/
#define ABORT() (rxdataS[0]=='n')

/*!***************************************************************************
 * @brief   Macro to get user input, see #Argus_XtalkCalibration_CLI.
 *
 * @details Supportive function for the CLI.
 *****************************************************************************/
#define PROMPT_INPUT() (rxdataS[0])

/*!***************************************************************************
 * @brief   Macro to clear user input.
 *
 * @details Supportive function for the CLI.
 *****************************************************************************/
#define CLEAR_INPUT() (rxdataS[0] = 0, rxdataS[1] = 0, flag = 0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//Check type functions
static void User_Query(void);
static void Device_Query(argus_hnd_t * hnd);
static void Set_DCA_to_MaxState(argus_hnd_t * hnd);
//Measuring type functions
static void Exec_SingleMeasurement(argus_hnd_t * hnd, uint8_t const cnt, bool print_all);
static void Exec_XtalkMeasurement(argus_hnd_t * hnd, argus_cal_xtalk_table_t * xtk, uint8_t step);
//print-out type functions
static void Print_BinnedResults(argus_results_t const * res);
static void Print_PixelAmplResults(argus_results_t const * res);
static void Print_PixelSatResults(argus_results_t const * res);
static void Print_PixelMapCoordinates(void);
static void Print_XtalkMap(argus_cal_xtalk_table_t * extXtalk, uint8_t step);
static void Print_XtalkVectorTable(char * mode, argus_cal_xtalk_table_t * extXtalk);
static void Print_FP_XtalkVectorTable(char * mode, argus_cal_xtalk_table_t * extXtalk);
static void Print_TotalXtalkMap(argus_hnd_t * hnd);
static void Print_IntegrationEnergyInfo(argus_results_t const * res);
//Misc functions
static void Get_GoldenPixel(argus_hnd_t * hnd);
static void Interpolate_ActivePixels(argus_hnd_t * hnd);
static void Set_Xtalk_AmplitudeThreshold(argus_hnd_t * hnd, uint8_t step_n, uint8_t trim);
static void Handle_Error(status_t status, char const * msg);
static void Get_UARTRxdata(void);
static void UART_Rx_Callback(uint8_t const * data, uint32_t const size);

/******************************************************************************
 * Variables
 ******************************************************************************/


/*!***************************************************************************
 *  Function pointer for the result data structure
 *
 *  The #argus_results_t data structure is a rich set of function which cannot
 *  be displayed all in once. This function pointer is used to call different
 *  print functions from within the same measurement function.
 *****************************************************************************/
static void (*set_print_fct)(argus_results_t const*);

/******************************************************************************
 * Miscellaneous Global Variables
 ******************************************************************************/
static uint8_t loop_var = 0;
static volatile uint8_t rxdata = 0;
static uint8_t rxdataS[1280] = { 0 }; // 20 max number of char, but for set vector table in one array needed more: 64 *20 = 1280
static bool flag = 0; // flag for completely received command
static bool quit = false;
static uint16_t max_ampl;
static uint8_t sat_pixels_cnt;
static uint32_t frame_time;
static uint8_t gp_x = 0;
static uint8_t gp_y = 0;

//custom typedefs
static status_t status = STATUS_OK;
static argus_mode_t myMode;
static argus_dfm_mode_t myDFM;
static argus_cfg_dca_t myDCA;

//xtalk variables
static argus_cal_xtalk_table_t myXtalk;
static argus_cal_xtalk_table_t totalXtalk;
static argus_cal_xtalk_table_t eXtalkArr;
static argus_cal_xtalk_table_t oeXtalkNoCoverArr;
static argus_cal_xtalk_table_t oeXtalkCoverArr;
static bool cancel_xtalk_cal_flag = false;

//timer variables
static uint32_t const my_meas_time_ms = 1000;
static uint32_t const device_query_timer_ms = 6000;
static uint8_t const cnt_trig_meas = 20;

/*******************************************************************************
 * Code
 ******************************************************************************/

void Argus_XtalkCalibration_CLI(argus_hnd_t * hnd)
{
    UART_SetRxCallback(UART_Rx_Callback);

    print("\n#####################################################################################");
    print("\n##### AFBR-S50 API - Interactive Xtalk Calibration Guide - " ARGUS_XTALK_CAL_CLI_VERSION "\n");
    print("\n#####################################################################################\n");
    print("\nThis interactive procedure guides you through the steps needed \n"
          "to compensate the application specific xtalk. A dominant source of\n"
          "xtalk could be the usage of cover glasses. Please read application\n"
          "note AFBR-S50-XTK-Crosstalk-Guide to get more information.\n"
          "https://docs.broadcom.com/docs/AFBR-S50-XTK-Crosstalk-Guide\n"
          "\n#####################################################################################\n");

    while (!quit)
    {
        if (loop_var == 0)
        {
            print("\n-------------------------------------------------------------------------------------"
                  "\nEnter 's' to start the step-by-step xtalk calibration guide        - or -"
                  "\nEnter 'c' to run a continuous measurement showing binned results   - or -"
                  "\nEnter 'x' to read the xtalk calibration table from memory          - or -"
                  "\nEnter 'o' to get an overview about other commands.\n\n");
            loop_var++;
        }
        Get_UARTRxdata();

        if (flag)
        {
            print("Received: %c\n\n", rxdataS[0]);

            switch (PROMPT_INPUT())

            {
                case 'a':
                    /*start single trigger measurements to get amplitude map*/
                    print("\nMeasure pixel amplitudes w/ maximum DCA settings:\n");

                    /* Set maximum DCA settings for xtalk calibration */
                    Set_DCA_to_MaxState(hnd); //--> Already implemented since v1.3.5

                    /* Assign print function to display results at prompt */
                    set_print_fct = Print_PixelAmplResults;
                    Exec_SingleMeasurement(hnd, 5, false); // Use only the x(int)-th measurement for the print function

                    Device_Query(hnd);

                    set_print_fct = Print_PixelSatResults;
                    Exec_SingleMeasurement(hnd, 5, false);

                    print("\nCurrent module type: %s\n\n", Argus_GetModuleName(hnd));

                    if ((Argus_GetModuleVersion(hnd) == AFBR_S50MV68B_V1) ||
                        (Argus_GetModuleVersion(hnd) == AFBR_S50LV85D_V1) ||
                        (Argus_GetModuleVersion(hnd) == AFBR_S50LX85D_V1))
                    {
                        print("Tip 2:\n"
                              "-----------------------------------------------\n"
                              "For this module type it is recommended to achieve a maximum amplitude\n"
                              "of not more than 100 LSB. Higher amplitudes may still work depending on the cover\n"
                              "glass characteristics and application accuracy requirements.\n"
                              "To further reduce the amplitude either increase the distance to target and/or\n"
                              "use a lower reflective target.\n");
                    }
                    else
                    {
                        print("Tip 2:\n"
                              "-----------------------------------------------\n"
                              "For this module type it is recommended to achieve a maximum amplitude\n"
                              "of about 10 LSB. Higher amplitudes may still work depending on the cover\n"
                              "glass characteristics and application accuracy requirements.\n"
                              "To further reduce the amplitude either increase the distance to target and/or\n"
                              "use a lower reflective target.\n");
                    }

                    /* Reset DCA settings */
                    status = Argus_SetConfigurationDynamicAdaption(hnd, &myDCA);
                    Handle_Error(status, "Reset DCA settings failed!");

                    break;

                case 'c':
                    /* Example of a continuous measurement over x seconds with the typical binned results:
                     * Range in mm - Amplitude in LSB - Quality 1-100 - status etc.
                     * Get more information on the StartMeasurement function in the 'argus_api.h' header file.
                     * Get more information on the status in the 'argus_status.h' header file.
                     * */

                    /*Check if DCA is still on MAX settings*/
                    if (myDCA.DepthMin == 1U << 10U)
                    {
                        print("\n Re-setting DCA settings...\n");
                        status = Argus_SetConfigurationDynamicAdaption(hnd, &myDCA);
                        Handle_Error(status, "Reset DCA settings failed!");
                        Device_Query(hnd);
                    }
                    /*Set print function*/
                    set_print_fct = Print_BinnedResults;

                    print("\n-- Performing %d x single triggered measurements --\n", cnt_trig_meas);
                    Exec_SingleMeasurement(hnd, cnt_trig_meas, true);

                    status = Argus_GetConfigurationFrameTime(hnd, &frame_time);
                    print("\n\nNote(s):\n"
                          "-----------------------------------------------\n"
                          "- The Argus_TriggerMeasurement() function is called periodically to start a series of\n"
                          "  asynchronous measurement cycles at a certain frame rate. The current frame rate is\n"
                          "  %.1f Hz. The first two frames may appear with status -110 because the dual frequency mode\n"
                          "  (DFM) needs three frames for setting the integration energy to the ambient conditions.\n"
                          "- If the number of frames does not match the expected number within %.1f second(s)\n"
                          "  you may need to either increase the UART speed, reduce the amount of streamed characters\n"
                          "  or reduce the frame rate.\n\n",
                          1000000.0 / (frame_time), (float)my_meas_time_ms / 1000);
                    print("Tip(s):\n"
                          "-----------------------------------------------\n"
                          "- Press 'i' to get more information on your sensor and the currently used configuration.\n\n");
                    break;

                case 'i':
                    /* Shows the following device information and configuration parameters:
                     * - module type & chipID
                     * - golden pixel position
                     * - currently used frame rate
                     * - measurement mode
                     * - DCA configuration parameters*/

                    Get_GoldenPixel(hnd);

                    print("Device Info:\n"
                          "-----------------------------------------------\n"
                          "  Module:       %s\n"
                          "  ChipID:       %d\n", Argus_GetModuleName(hnd), Argus_GetChipID(hnd));

                    int mod = Argus_GetModuleVersion(hnd);
                    Get_GoldenPixel(hnd);
                    if (!((mod == AFBR_S50MV85I_V1) || (mod == AFBR_S50MX85I_V1)))
                    {
                        print("  Golden Pixel: %d/%d\n\n", gp_x, gp_y);
                    }else{
                        print("  Golden Pixel: n/a\n\n");
                    }

                    /* API information. */
                    const uint32_t value = Argus_GetAPIVersion();
                    const uint8_t a = (uint8_t)((value >> 24) & 0xFFU);
                    const uint8_t b = (uint8_t)((value >> 16) & 0xFFU);
                    const uint8_t c = (uint8_t)(value & 0xFFFFU);

                    /* Get frame-rate */
                    status = Argus_GetConfigurationFrameTime(hnd, &frame_time);
                    Handle_Error(status, "Retrieving frame rate failed!");

                    /* Get current measurement mode */
                    status = Argus_GetMeasurementMode(hnd, &myMode);
                    Handle_Error(status, "Argus_GetMeasurementMode failed!");

                    char meas_mode[23];
                    switch (myMode)
                    {
                        case ARGUS_MODE_SHORT_RANGE:
                            strcpy(meas_mode, "Short Range");
                            break;
                        case ARGUS_MODE_LONG_RANGE:
                            strcpy(meas_mode, "Long Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_SHORT_RANGE:
                            strcpy(meas_mode, "High Speed Short Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_LONG_RANGE:
                            strcpy(meas_mode, "High Speed Long Range");
                            break;
                        default:
                            strcpy(meas_mode, "undefined");
                            break;
                    }

                    /* Get current dfm mode */
                    status = Argus_GetConfigurationDFMMode(hnd, &myDFM);
                    Handle_Error(status, "Argus_GetConfigurationDFMMode failed!");

                    char dfm_mode[13];
                    switch (myDFM)
                    {
                        case DFM_MODE_OFF:
                            strcpy(dfm_mode, "DFM off");
                            break;
                        case DFM_MODE_4X:
                            strcpy(dfm_mode, "DFM 4x");
                            break;
                        case DFM_MODE_8X:
                            strcpy(dfm_mode, "DFM 8x");
                            break;
                        default:
                            strcpy(meas_mode, "undefined");
                            break;
                    }

                    print("Configuration Info:\n"
                          "-----------------------------------------------\n"
                          "  API Version:  v%d.%d.%d\n"
                          "  Mode:         %s\n"
                          "  DFM mode:     %s\n"
                          "  Frame rate:   %d Hz\n",
                          a, b, c, meas_mode, dfm_mode, 1000000 / frame_time);

                    print("\nNote(s):\n"
                          "--------\n"
                          "- Changing the measurement and/or dfm modes affects the xtalk data.\n"
                          "  If you aim to dynamically change modes in your application\n"
                          "  you will need to store xtalk vector tables for each mode separately!\n"
                          "  See application note AFBR-S50-XTK-Crosstalk-Guide for more information.\n\n"
                          "- To see the effect on the xtalk values with other configurations\n"
                          "  go back to the calling example's C-code, modify the settings (e.g. long instead\n"
                          "  of short range or vice versa) and perform the xtalk calibration again.\n\n");
                    break;

                case 'o':
                    /* Command help screen
                     * tbd*/
                    print("\n#####################################################################################\n");
                    print("Overview of commands to verify and execute xtalk calibrations with an AFBR-S50 ToF sensor:\n\n"
                          "Enter 'a' - Measure pixel amplitudes & saturation statuses w/ max DCA settings\n"
                          "Enter 'c' - Start continuous measurement showing binned results\n"
                          "Enter 'i' - Show device info\n"
                          "Enter 'o' - Command overview\n"
                          "Enter 'r' - Re-initialize sensor\n"
                          "Enter 'R' - Reset xtalk calibration table\n"
                          "Enter 's' - Start xtalk calibration step-by-step guide\n"
                          "Enter 'x' - Read xtalk calibration table from memory\n"
                          "Enter 'X' - Set total xtalk vector table from last calibration\n"
                          "Enter 'y' - Get xtalk calibration table with FP instructions for callback\n"
                          "Enter 'q' - Exit the CLI and return to the calling application code\n\n"
                          "Note: Commands are case sensitive!");
                    print("\n#####################################################################################\n\n");
                    break;

                case 'r':
                    /* Initiate a re-init of the sensor module
                     * Purpose is to see the initial behavior of the sensor after initialization. */
                    status = Argus_GetMeasurementMode(hnd, &myMode);
                    Handle_Error(status, "Argus_GetConfigurationMeasurementMode failed!");
                    status = Argus_Reinit(hnd);
                    print("Performed a re-init of the AFBR-S50 ToF sensor.\n\n"
                          "Tip(s):\n"
                          "-----------------------------------------------\n"
                          "- Always query the status of the device even when performing a re-init.\n",
                          status);
                    break;

                case 'R':
                    /* Initiates a reset of the xtalk vector table. */
                    Argus_ResetCalibrationCrosstalkVectorTable(hnd);
                    print("Performed a reset of the xtalk vector table.\n\n"
                          "Tip(s):\n"
                          "-----------------------------------------------\n"
                          "Start xtalk calibration 's' to generate a new table.\n",
                          status);
                    break;

                case 's':
                    /* Start xtalk calibration routine */
                    print("\n-------------------------------------------------------------------------------------\n"
                          "\n#####################################################################################\n"
                          "Start of the xtalk calibration routine in - 3 steps -:"
                          "\n#####################################################################################\n\n"
                          "1. Electrical xtalk measurement - Blinding the sensor with a laser blocking material\n"
                          "2. Optical/Electrical xtalk measurement w/o cover glass against infinity.\n"
                          "3. Optical/Electrical xtalk measurement  with cover glass against infinity.");
                    print("\n-------------------------------------------------------------------------------------\n"
                          " Test Setup:\n\n"
                          "             #|        |                  |#\n"
                          "             #|        |                  |#\n"
                          " Application #|-----+  |                  |#\n"
                          "         PCB #| RX  |  |                  |# Low Reflective\n"
                          "           + #|----++  | Cover            |# Calibration\n"
                          "      Sensor #| TX |   | Glass            |# Target (== infinity)\n"
                          "             #|----+   |                  |#\n"
                          "             #|        |                  |#\n"
                          "\n-------------------------------------------------------------------------------------"
                          "\n-------------------------------------------------------------------------------------");

                    /* Get measurement mode */
                    status = Argus_GetMeasurementMode(hnd, &myMode);
                    Handle_Error(status, "Argus_GetConfigurationMeasurementMode failed!");

                    /* Interactive user interface*/
                    print("\nNote: If you only want to measure the electrical xtalk then simply abort \n"
                          "      after the 1.step!\n\n");
                    print("\n#####################################################################################\n"
                          "---1. STEP---"
                          "\n#####################################################################################\n"
                          "Cover the sensor to perform a measurement of the electrical xtalk."
                          "\n#####################################################################################\n");

                    /*Wait for user input*/
                    User_Query();

                    /*Start measurement of xtalk data and save it to respective def type*/
                    Exec_XtalkMeasurement(hnd, &eXtalkArr, 1);

                    if (!cancel_xtalk_cal_flag)
                    {
                        print("\n#####################################################################################\n"
                              "---2. STEP---"
                              "\n#####################################################################################\n"
                              "Point the sensor to a low reflective target w/o cover glass in place."
                              "\n#####################################################################################\n");

                        /*Wait for user input*/
                        User_Query();

                        /*Start measurement of xtalk data and save it to respective def type*/
                        Exec_XtalkMeasurement(hnd, &oeXtalkNoCoverArr, 2);

                        if (!cancel_xtalk_cal_flag)
                        {
                            print("\n#####################################################################################\n"
                                  "---3. STEP---"
                                  "\n#####################################################################################\n"
                                  "Point the sensor to a low reflective target with the target cover glass in place."
                                  "\n#####################################################################################\n");

                            print("\nThe transmission factor of %.2f will be used for the xtalk calculation.",
                            T_GLASS / (float)100);

                            /*Wait for user input*/
                            User_Query();

                            /*Start measurement of xtalk data and save it to respective def type*/
                            Exec_XtalkMeasurement(hnd, &oeXtalkCoverArr, 3);

                            if (!ABORT())
                            {
                                /*Calculate total xtalk*/
                                Print_TotalXtalkMap(hnd);

                                /*Apply interpolation to active pixels. This is only performed on selected types. */
                                Interpolate_ActivePixels(hnd);

                                /*Write total xtalk vector table to module*/
                                Argus_SetCalibrationCrosstalkVectorTable(hnd, &totalXtalk);

                                /*Wait for idle*/
                                Device_Query(hnd);

                                /* Perform a Re-init to reset DCA
                                 status = Argus_GetMeasurementMode(hnd, &myMode);
                                 Handle_Error(status, "Argus_GetConfigurationMeasurementMode failed!");
                                 status = Argus_Reinit(hnd);
                                 Handle_Error(status, "Argus_Reinit failed!"); */

                                print("Xtalk calibration procedure finished!...\n"
                                      "Module re-initialized after calibration!...\n"
                                      "Initiate test measurements by pressing 'c'.\n\n");
                            }
                        }
                        /*Calibration abort after electrical xtalk */
                        totalXtalk = eXtalkArr;
                    }
                    cancel_xtalk_cal_flag = false;
                    break;

                case 'x':
                    print("Print xtalk vector table stored in flash memory:\n");

                    /*Retrieve External xtalk vector table and display values at prompt
                     *Values are shown in fixed point arithmetic.*/
                    status = Argus_GetCalibrationCrosstalkVectorTable(hnd, &myXtalk);
                    Handle_Error(status, "Read xtalk vector table failed!");

                    /* Get current measurement mode */
                    status = Argus_GetMeasurementMode(hnd, &myMode);
                    Handle_Error(status, "Argus_GetMeasurementMode failed!");

                    char m_mode[23];
                    switch (myMode)
                    {
                        case ARGUS_MODE_SHORT_RANGE:
                            strcpy(m_mode, "Short Range");
                            break;
                        case ARGUS_MODE_LONG_RANGE:
                            strcpy(m_mode, "Long Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_SHORT_RANGE:
                            strcpy(m_mode, "High Speed Short Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_LONG_RANGE:
                            strcpy(m_mode, "High Speed Long Range");
                            break;
                        default:
                            strcpy(m_mode, "undefined");
                            break;
                    }

                    Print_XtalkVectorTable(m_mode, &myXtalk);

                    print("\n\nNote(s):\n"
                          "-----------------------------------------------\n"
                          "- The table shows the recently calibrated xtalk values which are\n"
                          "  still existing in the RAM memory of the MCU. You can use this view\n"
                          "  to either cross-check your xtalk values or to further process the\n"
                          "  values for using the crosstalk callback function. Please read the\n"
                          "  application note AFBR-S50-XTK-Crosstalk-Guide section 6.2 for more information.\n"
                          "- The xtalk table is empty in case you didn't perform an xtalk\n"
                          "  calibration in this session. Press 's' to do so.");
                    print("\n\nTip(s):\n"
                          "-----------------------------------------------\n"
                          "- Press 'i' to get more information on your sensor settings.\n\n");

                    break;

                case 'X':
                    print("Load total xtalk vector table from last calibration to module:\n");

                    /* Check if golden pixel is empty */
                    if (totalXtalk.Table[0][5][1].dC != 0)
                    {
                        status = Argus_SetCalibrationCrosstalkVectorTable(hnd, &totalXtalk);
                        (status < 0) ? Handle_Error(status, "Write xtalk vector table failed!") :
                                print("Xtalk vector table successfully updated.\n");
                    }
                    else
                    {
                        print("Warning -> Total xtalk vector table is empty.\n\n"
                              "Please perform xtalk calibration first!\n");
                    }

                    break;

                case 'y':
                    print("Print xtalk FP vector table stored in flash memory:\n");

                    /*Retrieve External xtalk vector table and display values at prompt
                     *Values are shown in fixed point arithmetic.*/
                    status = Argus_GetCalibrationCrosstalkVectorTable(hnd, &myXtalk);
                    Handle_Error(status, "Read xtalk vector table failed!");

                    /* Get current measurement mode */
                    status = Argus_GetMeasurementMode(hnd, &myMode);
                    Handle_Error(status, "Argus_GetMeasurementMode failed!");

                    char m_modeFP[23];
                    switch (myMode)
                    {
                        case ARGUS_MODE_SHORT_RANGE:
                            strcpy(m_modeFP, "Short Range");
                            break;
                        case ARGUS_MODE_LONG_RANGE:
                            strcpy(m_modeFP, "Long Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_SHORT_RANGE:
                            strcpy(m_modeFP, "High Speed Short Range");
                            break;
                        case ARGUS_MODE_HIGH_SPEED_LONG_RANGE:
                            strcpy(m_modeFP, "High Speed Long Range");
                            break;
                        default:
                            strcpy(m_modeFP, "undefined");
                            break;
                    }

                    Print_FP_XtalkVectorTable(m_modeFP, &myXtalk);

                    print("\n\nNote(s):\n"
                          "-----------------------------------------------\n"
                          "- The table shows the last calibrated xtalk values which are still\n"
                          "  present in the RAM memory of the MCU. The string instructions can be\n"
                          "  directly copied & pasted to the callback function for xtalk hard-coding.\n"
                          "  Read the application note 'https://docs.broadcom.com/docs/AFBR-S50-XTK-Crosstalk-Guide'\n"
                          "  section 6.2 for more information.\n"
                          "- The xtalk table is empty in case you didn't perform an xtalk\n"
                          "  calibration in this session. Press 's' to do so.");
                    print("\n\nTip(s):\n"
                          "-----------------------------------------------\n"
                          "- Press 'i' to get more information on your sensor settings.\n\n");

                    break;

                case 'q':
                    /*Quit xtalk calibration guide and return to calling example*/
                    quit = true;
                    print("Exiting interactive guide and returning to calling example ...\n\n\n");

                    break;

                default:
                    print("Command not known.\n\n");
            }
            CLEAR_INPUT();
            loop_var = 0;
        }
    }
}

/* Print-out type functions */

/*!***************************************************************************
 * @brief   Prints the contend of binned parameters and the status of the frame.
 *
 * @details Function can be called from within a measurement function type
 *          or/and get assigned to a function pointer.
 *
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 *****************************************************************************/
static void Print_BinnedResults(argus_results_t const * res)
{
    /* Print the recent measurement results:
     * 1. Range in mm (converting the Q9.22 value to mm)
     * 2. Amplitude in LSB (converting the UQ12.4 value to LSB)
     * 3. Status (0: OK, <0: Error, >0: Warning */
    print("Range: %5d mm;  Amplitude: %4d LSB;  Quality: %3d;  Status: %4d;  ",
          res->Bin.Range / (Q9_22_ONE / 1000),
          res->Bin.Amplitude / UQ12_4_ONE,
          res->Bin.SignalQuality,
          res->Status);
}

/*!***************************************************************************
 * @brief   Prints the amplitudes of all 32 pixels in an 8 x 4 matrix.
 *
 * @details Additionally, the maximum amplitude is shown. Supportive
 *          function to the xtalk calibration sequence.
 *
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 *****************************************************************************/
static void Print_PixelAmplResults(argus_results_t const * res)
{
    uint16_t temp_ampl;
    max_ampl = 0;

    print("\nPixel amplitudes in LSB:\n"
          "********************************************\n");
    for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
    {
        for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
        {
            /*Print single pixel amplitude in LSB (converting the UQ12.4 value to LSB)*/
            print(" %8.2f", (float)res->Pixel[x][y].Amplitude / (UQ12_4_ONE));
            temp_ampl = res->Pixel[x][y].Amplitude / (UQ12_4_ONE);
            if (max_ampl < temp_ampl) max_ampl = temp_ampl;
        }
        print("\n");
    }
    print("\nMaximum Amplitude: %d LSB\n", max_ampl);
}

/*!***************************************************************************
 * @brief   Prints the saturation status of all 32 pixels in an 8 x 4 matrix
 *
 * @details Additionally, the total number of saturated pixels is shown. It serves as a
 *          supportive function to the xtalk calibration sequence.
 *
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 *****************************************************************************/
static void Print_PixelSatResults(argus_results_t const * res)
{
    sat_pixels_cnt = 0;

    print("\nPixel saturation flags (1:=saturated):\n"
          "********************************************\n");
    for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
    {
        for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
        {
            /*Print single pixel amplitude in LSB (converting the UQ12.4 value to LSB)*/
            print(" %8d", (res->Pixel[x][y].Status & PIXEL_SAT) ? 1 : 0);
            sat_pixels_cnt += (res->Pixel[x][y].Status & PIXEL_SAT) * 0.5;
        }
        print("\n");
    }
    print("\nNumber of Saturated Pixels: %d \n\n"
          "Tip 1:\n"
          "----------------\n"
          "A cover glass calibration should not be performed with any saturated pixels!\n"
          "Try to either increase the distance to the low reflective target or/and change\n"
          "the material of the target.\n\n",
          sat_pixels_cnt);
}

/*!***************************************************************************
 * @brief   Prints the coordinates of all 32 pixels in an 8 x 4 matrix.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *****************************************************************************/
static void Print_PixelMapCoordinates(void)
{
    print("\n\nAFBR-S50 pixel Map in x/y-coordinates:\n"
          "********************************************\n");
    for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
    {
        for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
        {
            /*Print single pixel coordinates*/
            print("     %d/%d", x, y);
        }
        print("\n");
    }
    print("\n");
}

/*!***************************************************************************
 * @brief   Prints the crosstalk values of each pixel in an 8x4 matrix.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *
 * @param   extXtalk A pointer to the argus_cal_xtalk_table_t structure that will
 *                  be populated with measured crosstalk calibration data.
 * @param   step Integer indicating the calling calibration step (1-3)
 *****************************************************************************/
static void Print_XtalkMap(argus_cal_xtalk_table_t * extXtalk, uint8_t step)
{
    print("\n%d.step - <measurement mode> - <frame rate>\n"
          "AFBR-S50 single pixel xtalk (dS/dC) in LSB:\n"
          "********************************************\n",
          step);
    for (uint8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
    {
        print("%s-Frame\n", f == 0 ? "A" : "B");
        for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
        {
            for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
            {
                /*Print single pixel statuses (excluding 1D binned flag)*/
                print(" %6.2f/%4.2f", extXtalk->Table[f][x][y].dS / (float)(Q11_4_ONE),
                      extXtalk->Table[f][x][y].dC / (float)(Q11_4_ONE));
            }
            print("\n");
        }
    }
    print("\n");
}

/*!***************************************************************************
 * @brief   Prints the crosstalk fixed-point values of each pixel in a table.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *
 * @param   extXtalk A pointer to the argus_cal_xtalk_table_t structure that will
 *                   be populated with measured crosstalk calibration data.
 * @param   mode     A pointer to the char variable containing the current
 *                   measurement mode.
 *****************************************************************************/
static void Print_FP_XtalkVectorTable(char * mode, argus_cal_xtalk_table_t * extXtalk)
{
    print("\n-------------------------------------------------------------------------\n"
          "\n-------------------------------------------------------------------------\n");
    print("This is the xtalk table for the currently set '%s' mode:\n"
          "\n-------------------------------------------------------------------------\n",
          mode);

    for (uint8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
    {
        for (uint8_t x = 0; x < ARGUS_PIXELS_X; x++)
        {
            for (uint8_t y = 0; y < ARGUS_PIXELS_Y; y++)
            {
                print("xtalk->Frame%s[%d][%d].dS=%d\n",
                      (f == 0) ? "A" : "B", x, y,
                      extXtalk->Table[f][x][y].dS);
            }
        }
    }
    for (uint8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
    {
        for (uint8_t x = 0; x < ARGUS_PIXELS_X; x++)
        {
            for (uint8_t y = 0; y < ARGUS_PIXELS_Y; y++)
            {
                print("xtalk->Frame%s[%d][%d].dC=%d\n",
                      (f == 0) ? "A" : "B", x, y,
                      extXtalk->Table[f][x][y].dC);
            }
        }
    }
    print("\n-------------------------------------------------------------------------\n");
}


/*!***************************************************************************
 * @brief   Prints the crosstalk values of each pixel in a table.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *
 * @param   extXtalk A pointer to the argus_cal_xtalk_table_t structure that will
 *                   be populated with measured crosstalk calibration data.
 * @param   mode     A pointer to the char variable containing the current
 *                   measurement mode.
 *****************************************************************************/
static void Print_XtalkVectorTable(char * mode, argus_cal_xtalk_table_t * extXtalk)
{
    print("\n-------------------------------------------------------------------------\n"
          "\n-------------------------------------------------------------------------\n");
    print("This is the xtalk table for the currently set '%s' mode:\n"
          "\n-------------------------------------------------------------------------\n"
          "Frame   X   Y  xtalk-dS  xtalk-dC"
          "\n-------------------------------------------------------------------------\n",
          mode);

    for (uint8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
    {
        for (uint8_t x = 0; x < ARGUS_PIXELS_X; x++)
        {
            for (uint8_t y = 0; y < ARGUS_PIXELS_Y; y++)
            {
                print("%5s%4d%4d%10.2f%10.2f\n",
                      (f == 0) ? "A" : "B", x, y,
                      extXtalk->Table[f][x][y].dS / (float)(Q11_4_ONE),
                      extXtalk->Table[f][x][y].dC / (float)(Q11_4_ONE));
            }
        }
    }
    print("\n-------------------------------------------------------------------------\n");
}

/*!***************************************************************************
 * @brief   Calculates and prints the total crosstalk values of each pixel in an
 *          8x4 matrix.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *****************************************************************************/
static void Print_TotalXtalkMap(argus_hnd_t * hnd)
{
    print("\n Total xtalk (dS/dC) in LSB for sensorID: %d\n"
          "********************************************\n",
          Argus_GetChipID(hnd));
    for (uint8_t f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
    {
        print("%s-Frame\n", f == 0 ? "A" : "B");
        /*        if (prompt)
         (f == 0) ? print("A-Frame\n") : print("B-Frame\n");*/
        for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
        {
            for (unsigned int x = 0; x < ARGUS_PIXELS_X; x++)
            {
                /* Calculate total xtalk according to xtk_tot = xtk_eo_cover-T_glass*(xtk_eo_noCover+xtk_e) */
                totalXtalk.Table[f][x][y].dS = oeXtalkCoverArr.Table[f][x][y].dS
                        - (fp_muls(T_GLASS, (oeXtalkNoCoverArr.Table[f][x][y].dS - eXtalkArr.Table[f][x][y].dS), 8));
                totalXtalk.Table[f][x][y].dC = oeXtalkCoverArr.Table[f][x][y].dC
                        - (fp_muls(T_GLASS, (oeXtalkNoCoverArr.Table[f][x][y].dC - eXtalkArr.Table[f][x][y].dC), 8));

                /*Print single pixel amplitude in LSB (converting the UQ12.4 value to LSB)*/
                print(" %6.2f/%4.2f", totalXtalk.Table[f][x][y].dS / (float)(Q11_4_ONE),
                      totalXtalk.Table[f][x][y].dC / (float)(Q11_4_ONE));
            }
            print("\n");
        }
    }
    print("\n");
}

/*!***************************************************************************
 * @brief   Prints the values of integration parameters of a frame.
 *
 * @details Function can be called from within a measurement function type
 *          or/and get assigned to a function pointer.
 *
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 *****************************************************************************/
static void Print_IntegrationEnergyInfo(argus_results_t const * res)
{
    /* Print the recent frame settings:
     * 1. AnaInt = analoge integration
     * 2. DigInt = digital integration
     * 3. Mod Curr = Laser Modulation Current per sample in mA.
     * 4. Gain = Pixel Gain on the receiver side */
    print("AnaInt: %2d LSB;  DigInt: %3d LSB;  Mod Curr: %2d mA;  Gain: %2d LSB",
          res->Frame.AnalogIntegrationDepth / (UQ10_6_ONE),
          res->Frame.DigitalIntegrationDepth,
          res->Frame.OutputPower / (UQ12_4_ONE),
          res->Frame.PixelGain);
}

/* Measurement functions */

/*!***************************************************************************
 * @brief   Measurement sequence calling the #Argus_TriggerMeasurement function
 *          a certain number of times. Results are going be printed
 *          according to the assigned print function (via #set_print_fct) at the end.
 *
 * @details Supportive function to the xtalk calibration sequence. In the crosstalk
 *          calibration sequence it is used to measure the pixel amplitudes
 *          while performing the crosstalk calibration. It will be called x-times
 *          to accommodate deviations in the first frames when DFM is used (default).
 *          See function #Exec_XtalkMeasurement for more details.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   cnt A counter as integer value to determine which frame is printed at the CLI.
 * @param	print_all Boolean to indicate if all or just last single measurement is printed
 *****************************************************************************/
static void Exec_SingleMeasurement(argus_hnd_t * hnd, uint8_t const cnt, bool print_all)
{
    argus_results_t res;
    uint8_t t_cnt = 1;
    do
    {
        status = Argus_TriggerMeasurement(hnd, 0);
        Handle_Error(status, "Argus_StartMeasurementTimer failed!");

        /* If status is #STATUS_ARGUS_POWERLIMIT, the device is not ready (due to laser
         * safety) to restart the measurement yet. -> Come back and try again later. */
        if (status != STATUS_ARGUS_POWERLIMIT)
        {
            /* Wait until measurement data is ready. */
            Device_Query(hnd);
            Handle_Error(status, "Waiting for measurement data ready (Argus_GetStatus) failed!");

            /* Evaluate the raw measurement results. */
            status = Argus_EvaluateData(hnd, &res);
            Handle_Error(status, "Argus_EvaluateData failed!");
            /* Wait until evaluated data is ready. */
            Device_Query(hnd);

            if (print_all)
            {
                print("#%02d  ", t_cnt);
                set_print_fct(&res);
                Print_IntegrationEnergyInfo(&res);
                print("\n");
            }
            t_cnt++;
        }
    }
    while (t_cnt <= cnt);

    if (!print_all) set_print_fct(&res);
}

/*!***************************************************************************
 * @brief   Measurement sequence to acquire crosstalk values.
 *
 * @details The calibration measurement is carried out by the following steps:
 *          1. The integration parameters are set to maximum (see #Set_DCA_to_MaxState).
 *             Intention is to display maximum pixel amplitudes to make user
 *             aware of insufficient blocking or too high reflections.
 *          2. Measure and print pixel coordinates (see #Print_PixelMapCoordinates)
 *             and amplitudes (see #Print_PixelAmplResults) to visualize the
 *             amplitude distribution over the pixel matrix.
 *          3. Set crosstalk amplitude threshold (see #Set_Xtalk_AmplitudeThreshold).
 *             This threshold determines at which amplitude the error -112 is invoked.
 *             (see #Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold, #status)
 *          4. Crosstalk measurement by calling the #Argus_ExecuteXtalkCalibrationSequence
 *          5. Save measured crosstalk table by calling #Argus_GetCalibrationCrosstalkVectorTable
 *          6. Printing single xtalk values in a 8x4 matrix (see #Print_XtalkMap).
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   xtk A pointer to the argus_cal_xtalk_table_t structure that will
 *                  be populated with measured crosstalk calibration data.
 * @param   step Integer indicating the calling calibration step (1-3)
 *****************************************************************************/
static void Exec_XtalkMeasurement(argus_hnd_t * hnd, argus_cal_xtalk_table_t * xtk, uint8_t step)
{
    if (READY())
    {
        do
        {
            /* Set maximum DCA settings for xtalk calibration.
             * This is necessary to measure the maximum amplitudes at an optical sink. */
            Set_DCA_to_MaxState(hnd);

            /*clear prompt input after xtalk measurement has been executed*/
            CLEAR_INPUT();

            /* First, a pixel map shows the pixel coordinates within the matrix.
             * Secondly, a measurement of the actual pixel amplitudes at maximum
             * DCA settings is executed. All amplitudes are supposed to be lower
             * than ELEC_XTK_AMPL_LMT.
             * Values are shown in LSB!. */
            Print_PixelMapCoordinates();

            /* Assign print function to display results at prompt */
            set_print_fct = Print_PixelAmplResults;

            /* Start single trigger measurement over 3 frames and obtain max_ampl. */
            Exec_SingleMeasurement(hnd, 6, false);

            /* Set xtalk amplitude threshold according to maximum pixel
             * amplitude max_ampl and set threshold based on the formula
             * <trim * max_ampl> */
            Set_Xtalk_AmplitudeThreshold(hnd, step, 2.5);

            /* Perform an xtalk measurement */
            print("Run xtalk calibration measurements ...\n");
            status = Argus_ExecuteXtalkCalibrationSequence(hnd);
            Handle_Error(status, "Xtalk calibration failed!");

            if (status == STATUS_OK)
            {
                /*Save External xtalk vector table and display values at prompt
                 *Values are shown in fixed point arithmetic.*/
                Argus_GetCalibrationCrosstalkVectorTable(hnd, xtk);
                Print_XtalkMap(xtk, step);
            }

            if (status != STATUS_OK) User_Query();

        }
        while ((status != STATUS_OK) && !ABORT());

        if (ABORT())
        {
            print("Xtalk calibration aborted \n");
            cancel_xtalk_cal_flag = true;
        }
    }
    else if (ABORT())
    {
        print("Xtalk calibration aborted \n");
        cancel_xtalk_cal_flag = true;
    }
    else
    {
        cancel_xtalk_cal_flag = true;
    }
}

/* xtalk supportive functions */

/*!***************************************************************************
 * @brief   Gets the golden pixel coordinates stored in the EEPROM.
 *
 * @details Gets the golden pixel coordinates and stores it to gp_x and gp_y
 *          (see #Argus_GetCalibrationGoldenPixel)
 *
 * @param   hnd The API handle; contains all internal states and data.
 *****************************************************************************/
static void Get_GoldenPixel(argus_hnd_t * hnd)
{
    status = Argus_GetCalibrationGoldenPixel(hnd, &gp_x, &gp_y);
    Handle_Error(status, "Argus_GetCalibrationGoldenPixel failed!");
}

/*!***************************************************************************
 * @brief   Sets the integration parameters to maximum.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *
 *          Note: Maximizing the integration energy is automatically done when
 *          calling #Argus_ExecuteXtalkCalibrationSequence, however, it does not
 *          return any pixel which amplitude is above the threshold!
 *
 * @param   hnd The API handle; contains all internal states and data.
 *****************************************************************************/
static void Set_DCA_to_MaxState(argus_hnd_t * hnd)
{
    /* Only read and save current DCA settings when empty
     *  -> After first call of function!*/
    if (myDCA.DepthMin == 0 && myDCA.DepthMax == 0)
    {
        Argus_GetConfigurationDynamicAdaption(hnd, &myDCA);
        Device_Query(hnd);
    }

    argus_cfg_dca_t tempDCA = myDCA;

    tempDCA.DepthMin = myDCA.DepthMax;
    tempDCA.DepthNom = myDCA.DepthMax;
    tempDCA.DepthMax = myDCA.DepthMax;
    tempDCA.Power = myDCA.Power = DCA_POWER_HIGH;
    tempDCA.GainMax = myDCA.GainMax;
    tempDCA.GainMin = myDCA.GainMax;
    tempDCA.GainNom = myDCA.GainMax;

    Argus_SetConfigurationDynamicAdaption(hnd, &tempDCA);
    Device_Query(hnd);
}

/*!***************************************************************************
 * @brief   Sets the crosstalk amplitude threshold based on input parameters.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *          Dynamically sets the crosstalk amplitude threshold (see
 *          #Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold) based on
 *          the maximum measured pixel amplitude (see #Print_PixelAmplResults).
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   step Integer indicating the calling calibration step (1-3)
 * @param   trim Scaling factor between measured pixel amplitude and crosstalk values
 *****************************************************************************/
static void Set_Xtalk_AmplitudeThreshold(argus_hnd_t * hnd, uint8_t step, uint8_t trim)
{
    uq12_4_t myXtkAmpThr;
    /* Adapt Maximum xtalk threshold only for 2nd & 3rd calibration steps */
    if (step == 1)
    {
        status = Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(hnd, ELEC_XTK_AMPL_LMT * UQ12_4_ONE);
    }
    else
    {
        /* Check the amplitude threshold based on a measured maximum
         * amplitude and an arbitrary trim value */
        if ((max_ampl * trim) > (OPT_ELEC_XTK_AMPL_LMT))
            print("\nXtalk amplitude threshold exceeds nominal limit of %d LSB!\n",
                  OPT_ELEC_XTK_AMPL_LMT);

        status = Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(hnd, max_ampl * trim * UQ12_4_ONE);

    }
    Handle_Error(status, "Setting xtalk amplitude threshold failed!");

    /* Wait until new threshold is written. */
    Device_Query(hnd);

    /* Read xtalk amplitude threshold from register. */
    status = Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold(hnd, &myXtkAmpThr);
    Handle_Error(status, "An error occurred while reading calibration xtalk amplitude threshold!");
    Device_Query(hnd);

    print("Xtalk amplitude threshold set to: %d\n\n",
          (step > 1) ? (myXtkAmpThr / UQ12_4_ONE) : ELEC_XTK_AMPL_LMT);
}

/*!***************************************************************************
 * @brief   Interpolates crosstalk values in the total crosstalk vector table.
 *
 * @details Supportive function to the xtalk calibration sequence.
 *          This function only applies to the higher focused sensor types like
 *          AFBR-S50LV85D, AFBR-S50LX85D and AFBR-S50MV68B. Due to the higher
 *          collimated laser beam it may appear that the reflex from the optical
 *          sink (:=low reflective target) yields higher crosstalk amplitudes in
 *          the active pixels which appear as peaks within the total crosstalk
 *          distribution. This effect can be mitigated by interpolating the
 *          crosstalk values of passive pixels (not receiving the reflex of the
 *          cover glass).
 *
 * @param   hnd The API handle; contains all internal states and data.
 *****************************************************************************/
static void Interpolate_ActivePixels(argus_hnd_t * hnd)
{
    int mod = Argus_GetModuleVersion(hnd);

    if ((mod == AFBR_S50LV85D_V1) ||
        (mod == AFBR_S50LX85D_V1) ||
        (mod == AFBR_S50MV68B_V1))
    {
        for (unsigned int f = 0; f < ARGUS_DFM_FRAME_COUNT; f++)
        {
            for (unsigned int y = 0; y < ARGUS_PIXELS_Y; y++)
            {
                for (unsigned int x = 4; x < 7; x++)
                {
                    totalXtalk.Table[f][x][y].dS =
                            (float)(totalXtalk.Table[f][2][y].dS +
                                    totalXtalk.Table[f][3][y].dS +
                                    totalXtalk.Table[f][7][y].dS) / 3.f;
                    totalXtalk.Table[f][x][y].dC =
                            (float)(totalXtalk.Table[f][2][y].dC +
                                    totalXtalk.Table[f][3][y].dC +
                                    totalXtalk.Table[f][7][y].dC) / 3.f;
                }
            }
        }
        print("Active pixel peaks were interpolated.\n");
    }
}

/* Misc functions */

/*!***************************************************************************
 * @brief   CLI function while performing crosstalk calibration steps.
 *
 * @details Queries user to proceed to next step by confirming with 'ready' or
 *          'abort'.
 *****************************************************************************/
static void User_Query(void)
{
    /*clear previous input data*/
    CLEAR_INPUT();

    /*Prompt for input*/
    print("\nReady / Yes (type 'y') or Abort / No (type 'n')?\n");

    /*WAIT for input*/
    do
    {
        Get_UARTRxdata();
        if (rxdataS[0] != 0 && (!READY() && !ABORT()))
        {
            /*Prompt for input*/
            print("No valid entry. Please use either\n 'y' to continue or\n 'n' to abort operation! ");
            rxdataS[0] = 0;
        }
    }
    while (!READY() && !ABORT());
}

/*!***************************************************************************
 * @brief   Queries device status and invokes timeout.
 *
 * @details Checks #Argus_GetStatus function and triggers a timeout based
 *          on global variable #device_query_timer_ms.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *****************************************************************************/
static void Device_Query(argus_hnd_t * hnd)
{
    ltc_t start;

    /*Init timer */
    Time_GetNow(&start);
    do
    {
        status = Argus_GetStatus(hnd);
    }
    while ((status == STATUS_BUSY) && (!Time_CheckTimeoutMSec(&start, device_query_timer_ms)));
    Handle_Error(status, "Querying Argus status failed!");
}

/*!***************************************************************************
 * @brief   A callback function from the example code whenever an error occurs.
 *
 * @details The example code calls this function whenever an unexpected error
 *          occurs, for example, if an API function returns an error code.
 *
 *          This implementation of the function will print the error message
 *          and then enter an infinite loop. The infinite loop is intended to
 *          prevent the program from continuing execution in case of errors.
 *
 * @warning This is only a simple example implementation that does not handle
 *          errors in a production system. It is intended to demonstrate the
 *          usage of the API and to provide a starting point for custom
 *          applications.
 *
 *          This function needs to be replaced with a more sophisticated
 *          implementation to handle errors in a production system.
 *          For example, it could reset the device or try to recover from
 *          the error by re-initializing the device.
 *
 * @param   status The specified status to be checked for errors.
 * @param   msg The associated error message to be printed in case of errors.
 *****************************************************************************/
static void Handle_Error(status_t status, char const * msg)
{
    /* Check for status < 0 and print message and halt the program execution. */
    if (status < STATUS_OK)
    {
        print("ERROR: %s\nError Code: %d", msg, status);
        /*      while (1) __asm("nop"); // stop!*/
    }
}

/*!***************************************************************************
 * @brief   Received UART data ready callback implementation.
 *
 * @details Supportive function for the CLI.
 *          Receives data from the UART interrupt. Note that only the first
 *          character is evaluated per call.
 *
 * @param   data Byte pointer to the data array.
 * @param   size Size of the data array.
 *****************************************************************************/
static void UART_Rx_Callback(uint8_t const * data, uint32_t const size)
{
    assert(data != NULL);
    (void)size;

    rxdata = data[0]; // ignores other than first characters!
}

/*!***************************************************************************
 * @brief   Generic function to retrieve and assign UART data.
 *
 * @details Supportive function for the CLI.
 *****************************************************************************/
static void Get_UARTRxdata(void)
{
    /* Waiting for UART RX data */
    if (rxdata != 0)
    {
        /*Assign input data to receive data array*/
        rxdataS[0] = rxdata;

        /* set flag when command is completely received and restart counter and
         * buffer data. */
        flag = 1;

        /* clear variable for next data */
        rxdata = 0;
    }
}

/*! @} */
