/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 Explorer Demo Application.
 * @details     This file contains the hardware API for the Explorer Application.
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
 *****************************************************************************/

#ifndef EXPLORER_API_TYPES_H
#define EXPLORER_API_TYPES_H

/*!***************************************************************************
 * @defgroup    explorer_types Data Types for the AFBR-S50 Explorer Demo Application
 * @ingroup     explorer_app
 * @brief       AFBR-S50 Explorer Application - Data Types
 * @details     Definitions for AFBR-S50 Explorer Application specific data types.
 * @addtogroup  explorer_types
 * @{
 *****************************************************************************/

#include "argus.h"
#include "sci/sci.h"

/*! Command byte definitions. */
enum ExplorerApp_SerialCommandCodes
{
    /*! Gets the information about current software and device
     *  (e.g. version, device id, device family, ...) */
    CMD_SOFTWARE_INFO = 0x05,
    /*! Gets the current software version number. */
    CMD_SOFTWARE_VERSION = 0x0C,
    /*! Gets the current module information. */
    CMD_MODULE_TYPE = 0x0E,
    /*! Gets the current module identification number. */
    CMD_MODULE_UID = 0x0F,

    /* Generic measurement commands. */
    /*! Invokes the single measurement shot command. */
    CMD_MEASUREMENT_SINGLE_SHOT = 0x10,
    /*! Starts the time-scheduled measurements with given frame rate. */
    CMD_MEASUREMENT_START = 0x11,
    /*! Stops the time-scheduled measurements (after the current frame finishes). */
    CMD_MEASUREMENT_STOP = 0x12,
    /*! Aborts all device activity. */
    CMD_DEVICE_ABORT = 0x13,
    /*! Reinitializes the device with default configuration. */
    CMD_DEVICE_REINIT = 0x15,
    /*! Executes the calibration measurement sequence. */
    CMD_MEASUREMENT_CALIBRATION = 0x18,

    /*! Executed a flash read/write/clear command. */
    CMD_FLASH = 0x19,

//  /*! Gets a raw measurement data set containing the raw device readout samples. */
//  CMD_MEASUREMENT_DATA_RAW = 0x30,
    /*! Gets a full measurement data set containing all available data. */
    CMD_MEASUREMENT_DATA_FULL_DEBUG = 0x31,
    /*! Gets a full measurement data set containing all essential data. */
    CMD_MEASUREMENT_DATA_FULL = 0x32,
    /*! Gets a 3D measurement data set including distance, amplitude, raw and debug data per pixel. */
    CMD_MEASUREMENT_DATA_3D_DEBUG = 0x33,
    /*! Gets a 3D measurement data set including distance and amplitude data per pixel. */
    CMD_MEASUREMENT_DATA_3D = 0x34,
    /*! Gets a 1D measurement data set including a single distance and amplitude value plus debug information. */
    CMD_MEASUREMENT_DATA_1D_DEBUG = 0x35,
    /*! Gets a 1D measurement data set including a single distance and amplitude value. */
    CMD_MEASUREMENT_DATA_1D = 0x36,

    /*! Gets or sets the configuration of the measurement data output mode   */
    CMD_CONFIGURATION_DATA_OUTPUT_MODE = 0x41,
    /*! Gets or sets the configuration of the measurement data evaluation method */
    CMD_CONFIGURATION_MEASUREMENT_MODE = 0x42,
    /*! Gets or sets the measurement frame time. */
    CMD_CONFIGURATION_FRAME_TIME = 0x43,
    /*! Gets or sets the Dual Frequency Mode. */
    CMD_CONFIGURATION_DUAL_FREQUENCY_MODE = 0x44,
    /*! Gets or sets the Smart Power Save Mode Enabled flag. */
    CMD_CONFIGURATION_SMART_POWER_SAVE = 0x45,
    /*! Gets or sets the Shot Noise Monitor mode. */
    CMD_CONFIGURATION_SHOT_NOISE_MONITOR_MODE = 0x46,
    /*! Gets or sets the Crosstalk Monitor mode. */
    CMD_CONFIGURATION_XTALK_MONITOR_MODE = 0x47,

    /*! Gets or sets a full DCA (Dynamic Configuration Adaption) configuration set. */
    CMD_CONFIGURATION_DCA = 0x52,
    /*! Gets or sets a full PBA (Pixel Binning Algorithm) configuration set. */
    CMD_CONFIGURATION_PBA = 0x54,
    /*! Gets or sets a full SPI configuration set. */
    CMD_CONFIGURATION_SPI = 0x58,
    /*!< Gets or sets the UART configuration, i.e. the baud rate. */
    CMD_CONFIGURATION_UART = 0x59,
    /*! Gets or sets the global range offset. */
    CMD_CALIBRATION_GLOBAL_RANGE_OFFSET = 0x61,
    /*! Gets or sets the custom pixel range offsets. */
    CMD_CALIBRATION_PIXEL_RANGE_OFFSETS = 0x67,
    /*! Resets the custom pixel range offsets. */
    CMD_CALIBRATION_PIXEL_RANGE_OFFSETS_RESET = 0x68,
    /*! Gets or sets the range offset calibration sequence sample time. */
    CMD_CALIBRATION_RANGE_OFFSET_SAMPLE_TIME = 0x69,

    /*! Gets or sets the custom crosstalk vector table. */
    CMD_CALIBRATION_XTALK_VECTOR_TABLE = 0x62,
    /*! Resets the custom crosstalk vector table. */
    CMD_CALIBRATION_XTALK_RESET_VECTOR_TABLE = 0x63,
    /*! Gets or sets the crosstalk calibration sequence sample time. */
    CMD_CALIBRATION_XTALK_SAMPLE_TIME = 0x64,
    /*! Gets or sets the crosstalk calibration sequence maximum amplitude threshold. */
    CMD_CALIBRATION_XTALK_MAX_AMPLITUDE = 0x65,
    /*! Gets or sets the pixel-to-pixel crosstalk compensation parameters. */
    CMD_CALIBRATION_XTALK_PIXEL_2_PIXEL = 0x66,

};

/*! Flash sub-commands. */
typedef enum explorer_flash_cmd_t
{

    /*! Clears all (API) user (i.e. actual) values from the non-volatile flash memory. */
    CMD_FLASH_CLEAR_USER_CALIBRATION = 0x44,

    /*! Clears all data from the non-volatile flash memory. */
    CMD_FLASH_CLEAR_ALL = 0xFF,

} explorer_flash_cmd_t;

/*! Calibration sequences for SCI interface. */
typedef enum explorer_cal_sequence_t
{

    /*! Crosstalk calibration sequence. */
    CALIBRATION_SEQUENCE_XTALK = 2,

    /*! Range offsets calibration sequence. */
    CALIBRATION_SEQUENCE_OFFSETS = 5,

} explorer_cal_sequence_t;


/*! Available data output modes for the SCI interface. */
typedef enum data_output_mode_t
{
    /*! Streaming data output of 1D and 3D measurement data, raw samples and auxiliary and debug information. */
    DATA_OUTPUT_STREAMING_FULL_DEBUG = 2,

    /*! Streaming data output of 1D and 3D measurement data and auxiliary information. */
    DATA_OUTPUT_STREAMING_FULL = 3,

    /*! Streaming data output of 3D measurement data, auxiliary and debug information. */
    DATA_OUTPUT_STREAMING_3D_DEBUG = 4,

    /*! Streaming data output of 3D measurement data and auxiliary information. */
    DATA_OUTPUT_STREAMING_3D = 5,

    /*! Streaming data output of 1D measurement data, auxiliary and debug information. */
    DATA_OUTPUT_STREAMING_1D_DEBUG = 6,

    /*! Streaming data output of 1D measurement data only. */
    DATA_OUTPUT_STREAMING_1D = 7

} data_output_mode_t;

/*! AFBR-S50 Explorer Application configuration data. */
typedef struct explorer_cfg_t
{
    /*! Determines the SPI baud rate in bps. */
    uint32_t SPIBaudRate;

    /*! Determines the software is in debug mode. */
    volatile uint8_t DebugMode;

    /*! Determines the current data streaming mode. */
    volatile data_output_mode_t DataOutputMode;

} explorer_cfg_t;

/*! AFBR-S50 Explorer Application control block for a AFBR-S50 TOF device instance. */
typedef struct explorer_t
{
    /*! Determines the SPI slave. */
    sci_device_t DeviceID;

    /*! The device specific Explorer configuration data. */
    explorer_cfg_t Configuration;

    /*! A pointer to the AFBR-S50 API handle that represent a physical device. */
    argus_hnd_t * Argus;

} explorer_t;


/*! @} */
#endif /* EXPLORER_API_TYPES_H */
