/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 Explorer Demo Application.
 * @details		This file contains the hardware API for the Explorer Application.
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

#ifndef EXPLORER_API_H
#define EXPLORER_API_H

/*!***************************************************************************
 * @defgroup	explorerapi Serial Commands for the AFBR-S50 Explorer Demo Application.
 * @ingroup		explorerapp
 * @brief		AFBR-S50 Explorer Application - Serial Commands
 * @details		Definitions for AFBR-S50 Explorer Application specific serial commands.
 * @addtogroup 	explorerapi
 * @{
 *****************************************************************************/

#include "sci/sci.h"
#include "argus.h"

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
	/*! Executes the calibration measurement sequence. */
	CMD_MEASUREMENT_CALIBRATION = 0x18,
    /*! Reinitializes the device with default configuration. */
    CMD_DEVICE_REINIT = 0x15,

	/*! Gets or sets the software debug mode flag. */
	CMD_DEBUG_MODE = 0x16,

	/*! Direct SPI Write command for debugging. */
	CMD_SPI_WRITE = 0x1A,
	/*! Direct SPI Read (+ writing 0) command for debugging. */
	CMD_SPI_READ = 0x1C,
	/*! Direct SPI Read + Write command for debugging. */
	CMD_SPI_TRANSFER = 0x1D,
	/*! Response from previous read command. */
	CMD_SPI_RESPONSE = 0x1E,
	/*! Disables (i.e. pulls all pins to low) the SPI interface. */
	CMD_SPI_DISABLE = 0x1F,

	/*! Gets a raw measurement data set containing the raw device readout samples. */
	CMD_MEASUREMENT_DATA_RAW = 0x30,
	/*! Gets a full measurement data set containing all available data. */
	CMD_MEASUREMENT_DATA_DEBUG = 0x31,
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

	/*! Gets or sets a full DCA (Dynamic Configuration Adaption) configuration set. */
	CMD_CONFIGURATION_DCA = 0x52,
	/*! Gets or sets a full PBA (Pixel Binning Algorithm) configuration set. */
	CMD_CONFIGURATION_PBA = 0x54,
	/*! Gets or sets a fill SPI configuration set. */
	CMD_CONFIGURATION_SPI = 0x58,
	/*! Gets or sets the global range offset. */
	CMD_CALIBRATION_GLOBAL_RANGE_OFFSET = 0x61,
	/*! Gets or sets the custom pixel range offsets. */
	CMD_CALIBRATION_PIXEL_RANGE_OFFSETS = 0x67,
	/*! Resets the custom pixel range offsets. */
	CMD_CALIBRATION_PIXEL_RANGE_OFFSETS_RESET = 0x68,
	/*! Gets or sets the range offset calibration sequence sample count. */
	CMD_CALIBRATION_RANGE_OFFSET_SAMPLE_COUNT = 0x69,

	/*! Gets or sets the custom crosstalk vector table. */
	CMD_CALIBRATION_XTALK_VECTOR_TABLE = 0x62,
	/*! Resets the custom crosstalk vector table. */
	CMD_CALIBRATION_XTALK_RESET_VECTOR_TABLE = 0x63,
	/*! Gets or sets the crosstalk calibration sequence sample count. */
	CMD_CALIBRATION_XTALK_SAMPLE_COUNT = 0x64,
	/*! Gets or sets the crosstalk calibration sequence maximum amplitude threshold. */
	CMD_CALIBRATION_XTALK_MAX_AMPLITUDE = 0x65,
	/*! Gets or sets the pixel-to-pixel crosstalk compensation parameters. */
	CMD_CALIBRATION_XTALK_PIXEL_2_PIXEL = 0x66,


};

/*! Calibration sequences for SCI interface. */
typedef enum
{
	/*! Substrate voltage calibration sequence. */
	CALIBRATION_SEQUENCE_VSUB = 1,

	/*! Crosstalk calibration sequence for measurement mode A. */
	CALIBRATION_SEQUENCE_XTALK_MODE_A = 2,

	/*! Crosstalk calibration sequence for measurement mode B. */
	CALIBRATION_SEQUENCE_XTALK_MODE_B = 3,

	/*! Pll calibration sequence. */
	CALIBRATION_SEQUENCE_PLL = 4,

	/*! Range offsets calibration sequence for measurement mode A. */
	CALIBRATION_SEQUENCE_OFFSETS_MODE_A = 5,

	/*! Range offsets calibration sequence for measurement mode B. */
	CALIBRATION_SEQUENCE_OFFSETS_MODE_B = 6,

} cal_sequence_t;

/*! Available data output modes for the SCI interface. */
typedef enum
{
	/*! Data output on request. */
	DATA_OUTPUT_ON_REQUEST = 0,

	/*! Streaming data output of raw measurement data (i.e. correlation sampling values). */
	DATA_OUTPUT_STREAMING_RAW = 1,

	/*! Streaming data output of 1D and 3D measurement data with all the information. */
	DATA_OUTPUT_STREAMING_FULL = 2,

	/*! Streaming data output of 1D and 3D measurement data with the essential information for a faster data transfer. */
	DATA_OUTPUT_STREAMING_FAST = 3,

	/*! Streaming data output of 3D measurement data with all the information. */
	DATA_OUTPUT_STREAMING_3D_FULL = 4,

	/*! Streaming data output of 3D measurement data with the essential information for a faster data transfer. */
	DATA_OUTPUT_STREAMING_3D_FAST = 5,

	/*! Streaming data output of 1D measurement data with all the information. */
	DATA_OUTPUT_STREAMING_1D_FULL = 6,

	/*! Streaming data output of 1D measurement data with the essential information for a faster data transfer. */
	DATA_OUTPUT_STREAMING_1D_FAST = 7,

} data_output_mode_t;

/*!***************************************************************************
 * @brief	Initialize the serial commands module.
 * @param	argus The API handle for API specific command execution.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t ExplorerApp_InitCommands(argus_hnd_t * argus);

/*!***************************************************************************
 * @brief	Sends measurement results from the device.
 * @param	type The measurement data type to be sent.
 * @param	res Structure with latest measurement results to transmit.
 * @return	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t SendMeasurementData(sci_cmd_t type, const argus_results_t * res);

/*! @} */
#endif /* EXPLORER_API_H */
