# Command Details {#explorer_app_cmd_details}

The following section contains the current command details as implemented in the
@ref explorer_app. The implementation can be found in the
`Sources\ExplorerApp\api` folder of the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API).

See @ref explorer_app_cmd_overview for an Overview of all available commands.

## Generic Commands {#explorer_app_cmd_generic}

### Invalid Command {#cmd_invalid}

Reserved! Invalid Command;

| Caption / Name | Type  | Size | Unit | Comment |
| -------------- | ----- | ---- | ---- | ------- |
| Command        | UINT8 | 1    |      | 0x00    |

### Acknowledge {#cmd_ack}

Slave does acknowledge the successful reception of the last command.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x0A (basic); 0x8A (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Acknowledged Command         | UINT8 | 1    |      |                                                           |

@note The acknowledge is always sent with the same address as the received
command.

### Not Acknowledge {#cmd_nak}

Slave does not-acknowledge the successful reception of the last command.

| Caption / Name                 | Type   | Size | Unit | Comment                                                   |
| ------------------------------ | ------ | ---- | ---- | --------------------------------------------------------- |
| Command                        | UINT8  | 1    |      | 0x0B (basic); 0x8B (extended)                             |
| Address (extended mode only)   | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Not Acknowledged Command       | UINT8  | 1    |      |                                                           |
| Serial Status / Reason for NAK | UINT16 | 2    |      |                                                           |

@note The not-acknowledge is always sent with the same address as the received
command.

### Ping Command {#cmd_ping}

A ping message the is sent from the master and reflected by the slave. Used to
establish or test the UART connection.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x01 (basic); 0x81 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

@note The address byte is ignored and has not effect for the ping command.

### Log Message {#cmd_log}

An event/debug log message sent from the slave to inform the user.

| Caption / Name               | Type               | Size | Unit  | Comment                                                   |
| ---------------------------- | ------------------ | ---- | ----- | --------------------------------------------------------- |
| Command                      | UINT8              | 1    |       | 0x06 (basic); 0x86 (extended)                             |
| Address (extended mode only) | UINT8              | 1    |       | Extended frame address byte. Skipped in basic frame mode. |
| Time Stamp [sec]             | UINT32             | 4    | sec   |                                                           |
| Time Stamp [µsec]            | UINT16             | 2    | ms/16 |                                                           |
| Message                      | CHAR[] (= UINT8[]) | n/a  |       |                                                           |

@note The log message is usually sent with device address 0 even if it is sent
from a device with a different address.

### Test Message {#cmd_test}

Sending a test message to the slave that will be echoed in order to test the
interface. The slave will echo the exact message including the CRC values from
the original message.

| Caption / Name               | Type    | Size | Unit | Comment                                                             |
| ---------------------------- | ------- | ---- | ---- | ------------------------------------------------------------------- |
| Command                      | UINT8   | 1    |      | 0x04 (basic); 0x84 (extended)                                       |
| Address (extended mode only) | UINT8   | 1    |      | Extended frame address byte. Skipped in basic frame mode.           |
| Test message                 | UINT8[] | n/a  |      | The returned message does also contain the CRC of the sent message. |

@note The test message is always sent with the same address as the received test
message.

### MCU/Software Reset {#cmd_reset}

Invokes the software reset.

| Caption / Name               | Type   | Size | Unit | Comment                                                   |
| ---------------------------- | ------ | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x08 (basic); 0x88 (extended)                             |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Safety Code                  | UINT32 | 4    |      | 0xDEADC0DE                                                |

@note The reset happens after the device has completely sent its acknowledge!

@note The address is ignored in that case. The reset is always executed system
wide, i.e. it also affects devices not addressed directly.

### Software Version {#cmd_sw}

Gets the current software version number.

| Caption / Name               | Type                   | Size | Unit | Comment                                                                              |
| ---------------------------- | ---------------------- | ---- | ---- | ------------------------------------------------------------------------------------ |
| Command                      | UINT8                  | 1    |      | 0x0C (basic); 0x8C (extended)                                                        |
| Address (extended mode only) | UINT8                  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                            |
| Version Software             | UINT32                 | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                          |
| Software Build Number String | CHAR[14] (= UINT8[14]) | n/a  |      | (Added in v1.2.0) Software Build Number as String (14 Digits; e.g. "20200101123456") |

@note This command can be used to determine the current software version of the
connected device. However, the command has changed as the build number was added
to the command for `v1.2.0` and newer versions. In order to determine the
version, it is sufficient to encode only the version number. If the version is
at least `v1.2.0`, the build number can also be extracted from the command data
array.

\see

-   #Argus_GetAPIVersion
-   #Argus_GetBuildNumber

### Module Type / Version {#cmd_module}

Gets the module information, incl. module type with version number, chip version
and laser type.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x0E (basic); 0x8E (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Module Type/Version          | UINT8 | 1    |      | Module type number. See #argus_module_version_t           |
| Chip Type/Version            | UINT8 | 1    |      | Module type number. See #argus_chip_version_t             |
| Laser Type                   | UINT8 | 1    |      | Module type number. See #argus_laser_type_t               |

\see

-   #Argus_GetModuleVersion
-   #Argus_GetChipVersion
-   #Argus_GetLaserType

### Module UID {#cmd_uid}

Gets the chip/module unique identification number.

| Caption / Name               | Type   | Size | Unit | Comment                                                   |
| ---------------------------- | ------ | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x0F (basic); 0x8F (extended)                             |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Module UID                   | UINT24 | 3    |      | Unique Identification Number                              |

\see

-   #Argus_GetChipID

### Software Information / Identification {#cmd_info}

Software Information / Identification Gets the information about current
software and device (e.g. version, device id, device family, ...)

@note This command can be used to determine the current software version of the
connected device. However, the command has changed over time, e.g. the build
number was added to the command for `v1.2.0` and newer versions. In order to
determine the version, it is sufficient to encode only the first version number.
If the version is not current, the remaining command may differ. In case of
deprecated version, please refer to the corresponding API documentation to find
the correct command formats.

@note In `v1.4.4` the address byte has been introduced. If the request comes
with a specified address, the response will contain the exact information for
the dedicated device. If the request comes with address 0, the response will
contain the information for all devices.

#### Single Device Version (Basic Mode ore Address == 0)

| Caption / Name                  | Type               | Size | Unit | Comment                                                                                                                                              |
| ------------------------------- | ------------------ | ---- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                         | UINT8              | 1    |      | 0x05 (basic); 0x85 (extended)                                                                                                                        |
| Address (extended mode only)    | UINT8              | 1    |      | Extended frame address byte. Skipped in basic frame mode. Must be 0 for this type of command!                                                        |
| Version Software (Explorer App) | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Version AFBR-S50 API            | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Module Type/Version             | UINT8              | 1    |      | Module type number. See #argus_module_version_t                                                                                                      |
| Chip Type/Version               | UINT8              | 1    |      | Module type number. See #argus_chip_version_t                                                                                                        |
| Laser Type                      | UINT8              | 1    |      | Module type number. See #argus_laser_type_t                                                                                                          |
| Module UID                      | UINT24             | 3    |      | Unique Identification Number.                                                                                                                        |
| Software ID String              | CHAR[] (= UINT8[]) | n/a  |      | Software ID String incl. Build Number (Format: "AFBR-S50 Explorer App - [BuildNumber]"; w/ build number containing 14 digits, e.g. "20200101123456") |

#### Multi Device Version (Address != 0)

| Caption / Name                  | Type               | Size | Unit | Comment                                                                                                                                              |
| ------------------------------- | ------------------ | ---- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                         | UINT8              | 1    |      | 0x85 (extended only)                                                                                                                                 |
| Address (extended mode only)    | UINT8              | 1    |      | Address byte (must be != 0).                                                                                                                         |
| Version Software (Explorer App) | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Version AFBR-S50 API            | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Device Count (N)                | UINT8              | 1    |      | Number of devices (N) connected to the board.                                                                                                        |
| Device Address (Device 1)       | UINT8              | 1    |      | Device Address of Device 1.                                                                                                                          |
| Module Type/Version (Device 1)  | UINT8              | 1    |      | Module type number of Device 1. See #argus_module_version_t                                                                                          |
| Chip Type/Version (Device 1)    | UINT8              | 1    |      | Module type number of Device 1. See #argus_chip_version_t                                                                                            |
| Laser Type (Device 1)           | UINT8              | 1    |      | Module type number of Device 1. See #argus_laser_type_t                                                                                              |
| Module UID (Device 1)           | UINT24             | 3    |      | Unique Identification Number of Device 1                                                                                                             |
| Device Address (Device 2)       | UINT8              | 1    |      | Device Address of Device 2.                                                                                                                          |
| Module Type/Version (Device 2)  | UINT8              | 1    |      | Module type number of Device 2. See #argus_module_version_t                                                                                          |
| Chip Type/Version (Device 2)    | UINT8              | 1    |      | Module type number of Device 2. See #argus_chip_version_t                                                                                            |
| Laser Type (Device 2)           | UINT8              | 1    |      | Module type number of Device 2. See #argus_laser_type_t                                                                                              |
| Module UID (Device 2)           | UINT24             | 3    |      | Unique Identification Number of Device 2.                                                                                                            |
| ...                             | ...                | ...  |      | ...                                                                                                                                                  |
| Device Address (Device N)       | UINT8              | 1    |      | Device Address of Device N.                                                                                                                          |
| Module Type/Version (Device N)  | UINT8              | 1    |      | Module type number of Device N. See #argus_module_version_t                                                                                          |
| Chip Type/Version (Device N)    | UINT8              | 1    |      | Module type number of Device N. See #argus_chip_version_t                                                                                            |
| Laser Type (Device N)           | UINT8              | 1    |      | Module type number of Device N. See #argus_laser_type_t                                                                                              |
| Module UID (Device N)           | UINT24             | 3    |      | Unique Identification Number of Device N.                                                                                                            |
| Software ID String              | CHAR[] (= UINT8[]) | n/a  |      | Software ID String incl. Build Number (Format: "AFBR-S50 Explorer App - [BuildNumber]"; w/ build number containing 14 digits, e.g. "20200101123456") |

\see

-   #Argus_GetAPIVersion
-   #Argus_GetBuildNumber
-   #Argus_GetModuleVersion
-   #Argus_GetLaserType
-   #Argus_GetChipVersion
-   #Argus_GetChipID

## Device Control Commands {#explorer_app_cmd_ctrl}

### Measurement: Trigger Single Shot {#cmd_ctrl_trigger}

Executes a single shot measurement.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x10 (basic); 0x90 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_TriggerMeasurement

### Measurement: Start Auto {#cmd_ctrl_start}

Starts the automatic, time-scheduled measurements with given frame rate.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x11 (basic); 0x91 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_StartMeasurementTimer

### Measurement: Stop {#cmd_ctrl_stop}

Stops the time-scheduled measurements (after the current frame finishes).

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x12 (basic); 0x92 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_StopMeasurementTimer

### Measurement: Abort {#cmd_ctrl_abort}

Aborts the current measurements immediately.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x13 (basic); 0x93 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_Abort

### Run Calibration {#cmd_ctrl_cal}

Command triggers a specified calibration sequence.

| Caption / Name               | Type  | Size | Unit | Comment                                                                        |
| ---------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------ |
| Command                      | UINT8 | 1    |      | 0x18 (basic); 0x98 (extended)                                                  |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode.                      |
| Calibration Sequence ID      | ENUM8 | 1    | n/a  | The selected calibration measurement sequence. See table below.                |
| Optional Parameters          | n/a   | >=0  | n/a  | Optional parameters for the individual calibration sequences. See table below. |

\see

-   #Argus_ExecuteXtalkCalibrationSequence
-   #Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence
-   #Argus_ExecuteRelativeRangeOffsetCalibrationSequence

#### Calibration Sequence Enumerator

| Value | Name                           | Parameters                                                   | Description                         |
| ----- | ------------------------------ | ------------------------------------------------------------ | ----------------------------------- |
| 2     | Crosstalk Calibration          | n/a                                                          | Crosstalk calibration sequence.     |
| 5     | Pixel Range Offset Calibration | optional: Calibration Target Distance (Type: Q9.22; Unit: m) | Range offsets calibration sequence. |

\see

-   #explorer_cal_sequence_t

### Device Reinitialize {#cmd_ctrl_reinit}

Invokes the device (re-)initialization command. Resets and reinitialize the
API + ASIC with given config. (e.g. after unintended power cycle).

@note Does not perform a MCU reset! Use the [MCU/Software Reset](@ref cmd_reset)
if a complete MCU reset is desired.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x19 (basic); 0x99 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_Reinit

## Measurement Data Commands {#explorer_app_cmd_data}

Which type of measurement data is streamed depends on the current streaming
mode.

\see @ref cmd_cfg_output_mode

### Measurement Data Set (1D + 3D) - Debug {#cmd_data_full_dbg}

Gets a measurement data set containing all the available data.

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                                                                                                                |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Command                       | UINT8     | 1    |             | 0xB1 (extended mode only)                                                                                                                                                                                                                                                                                                                              |
| Address                       | UINT8     | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                                                                                                                                                                                                                                                          |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                                                                                                                       |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                                                                                                                   |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                                                                                                            |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                                                                                                            |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods).                                                                                          |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                                                                                                           |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                                                                                                            |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices.                                                                                                |
| Enabled ADC Channel Mask      | HEX32     | 4    | n/a         | The ADC channel enabled mask determines the misc. ADC channels that are enabled and converted by the ADC muxing. The mask is channel based (starting from channel 32), i.e. the n-th bit represents the n-th ADC channel. See the channel mapping for the corresponding channel purpuse.                                                               |
| Phase Count                   | UINT8     | 1    | #           | Number of phase steps.                                                                                                                                                                                                                                                                                                                                 |
| ADC Samples                   | UINT24[]  | 396  | LSB         | Raw sampling data from the pixel field. The 22 LSBs determine the raw 22-bit ADC readouts and the two MSBs determine the saturation flags. The values are ordered in increasing channel number where different phase steps are gathered (i.e. \f$idx = 4 n + p\f$). Disabled values (see Enabled Pixel Mask and Enabled ADC Channel Mask) are skipped. |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                      |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                              |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                          |
| Phase (x, y)                  | UQ1.15[,] | 64   | ?           | Phase values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                              |
| Status (ref)                  | HEX8      | 1    | m           | Status flags (see #argus_px_status_t) for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                     |
| Range (ref)                   | Q9.14     | 3    | m           | Range for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                                     |
| Amplitude (ref)               | UQ12.4    | 2    |             | Amplitude for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                                 |
| Phase (ref)                   | UQ1.15    | 2    | ?           | Phase for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                                     |
| 1D Range (binned)             | Q9.14     | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                                                                                                                       |
| 1D Amplitude (binned)         | UQ12.4    | 2    |             | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                                                                                                                   |
| Signal Quality                | UINT8     | 1    | %           | The signal quality indicator in % (0-100). 0%: invalid or not available; 1%: very bad signal, ...., 100%: perfect signal                                                                                                                                                                                                                               |
| VDD                           | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDD                                                                                                                                                                                                                                                                                                                  |
| VDDL                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDDL                                                                                                                                                                                                                                                                                                                 |
| VSUB                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VSUB                                                                                                                                                                                                                                                                                                                 |
| IAPD                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for IAPD                                                                                                                                                                                                                                                                                                                 |
| TEMP                          | Q11.4     | 2    | °C          | Auxiliary measurement results for TEMP                                                                                                                                                                                                                                                                                                                 |
| BGL                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the background light value.                                                                                                                                                                                                                                                                                                    |
| SNA                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the shot noise amplitude value.                                                                                                                                                                                                                                                                                                |
| Integration Time              | UINT32    | 4    | µsec        | The actual device integration time in microseconds (used for debugging only).                                                                                                                                                                                                                                                                          |
| Bias Current                  | UINT8     | 1    | LSB         | The BIAS laser current in LSB (used for debugging only).                                                                                                                                                                                                                                                                                               |
| PLL Offset                    | UINT8     | 1    | LSB         | The internal register offset value of the PLL_INT_PRD register (used for debugging only).                                                                                                                                                                                                                                                              |
| PLL Control Current           | UINT8     | 1    | LSB         | The internal register value of the PLL_CTRL_CUR register (used for debugging only).                                                                                                                                                                                                                                                                    |
| DCA Amplitude                 | UQ12.4    | 2    | LSB         | The internal amplitude used by the DCA algorithm to set up integration energy (used for debugging only).                                                                                                                                                                                                                                               |
| Crosstalk Predictor Vectors   | Q11.4[,]  | 8    | LSB         | The internal crosstalk predictor vector (used for debugging only). The first dS component is first, followed by its dC component. There are two vectors for upper/lower pixel rows.                                                                                                                                                                    |
| Crosstalk Monitor Vectors     | Q11.4[,]  | 16   | LSB         | The internal crosstalk monitor vectors (used for debugging only). The first dS component is first, followed by its dC component. There are 4 vectors for each pixel row.                                                                                                                                                                               |

### Measurement Data Set (1D + 3D) {#cmd_data_full}

Gets a measurement data set containing the essential data.

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0xB2 (extended mode only)                                                                                                                                                                                                                                     |
| Address                       | UINT8     | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                                                                                                                                                                 |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices.       |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                             |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                     |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                 |
| 1D Range (binned)             | Q9.14     | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                              |
| 1D Amplitude (binned)         | UQ12.4    | 2    |             | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                          |
| Signal Quality                | UINT8     | 1    | %           | The signal quality indicator in % (0-100). 0%: invalid or not available; 1%: very bad signal, ...., 100%: perfect signal                                                                                                                                      |
| VDD                           | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDD                                                                                                                                                                                                                         |
| VDDL                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDDL                                                                                                                                                                                                                        |
| VSUB                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VSUB                                                                                                                                                                                                                        |
| IAPD                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for IAPD                                                                                                                                                                                                                        |
| TEMP                          | Q11.4     | 2    | °C          | Auxiliary measurement results for TEMP                                                                                                                                                                                                                        |
| BGL                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the background light value.                                                                                                                                                                                                           |
| SNA                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the shot noise amplitude value.                                                                                                                                                                                                       |
| Integration Time              | UINT32    | 4    | µsec        | The actual device integration time in microseconds (used for debugging only).                                                                                                                                                                                 |
| DCA Amplitude                 | UQ12.4    | 2    | LSB         | The internal amplitude used by the DCA algorithm to set up integration energy (used for debugging only).                                                                                                                                                      |
| PLL Control Current           | UINT8     | 1    | LSB         | The internal register value of the PLL_CTRL_CUR register (used for debugging only).                                                                                                                                                                           |

### 3D Measurement Data Set - Debug {#cmd_data_3d_dbg}

Gets a 3D measurement data set containing all the available data per pixel.

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0xB3 (extended mode only)                                                                                                                                                                                                                                     |
| Address                       | UINT8     | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                                                                                                                                                                 |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices.       |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                             |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                     |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                 |
| Phase (x, y)                  | UQ1.15[,] | 64   | ?           | Phase values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                     |
| Integration Time              | UINT32    | 4    | µsec        | The actual device integration time in microseconds (used for debugging only).                                                                                                                                                                                 |
| Bias Current                  | UINT8     | 1    | LSB         | The BIAS laser current in LSB (used for debugging only).                                                                                                                                                                                                      |
| PLL Offset                    | UINT8     | 1    | LSB         | The internal register offset value of the PLL_INT_PRD register (used for debugging only).                                                                                                                                                                     |
| PLL Control Current           | UINT8     | 1    | LSB         | The internal register value of the PLL_CTRL_CUR register (used for debugging only).                                                                                                                                                                           |
| DCA Amplitude                 | UQ12.4    | 2    | LSB         | The internal amplitude used by the DCA algorithm to set up integration energy (used for debugging only).                                                                                                                                                      |
| Crosstalk Predictor Vectors   | Q11.4[,]  | 8    | LSB         | The internal crosstalk predictor vector (used for debugging only). The first dS component is first, followed by its dC component. There are two vectors for upper/lower pixel rows.                                                                           |
| Crosstalk Monitor Vectors     | Q11.4[,]  | 16   | LSB         | The internal crosstalk monitor vectors (used for debugging only). The first dS component is first, followed by its dC component. There are 4 vectors for each pixel row.                                                                                      |

### 3D Measurement Data Set {#cmd_data_3d}

Gets a 3D measurement data set containing the essential data per pixel.

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0xB4 (extended mode only)                                                                                                                                                                                                                                     |
| Address                       | UINT8     | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                                                                                                                                                                 |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices.       |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                             |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                     |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                 |

### 1D Measurement Data Set - Debug {#cmd_data_1d_dbg}

Gets a 1D measurement data set containing all the available distance measurement
data.

| Caption / Name                | Type     | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | -------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8    | 1    |             | 0xB5 (extended mode only)                                                                                                                                                                                                                                     |
| Address                       | UINT8    | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                                                                                                                                                                 |
| Status                        | HEX16    | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48   | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16    | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16   | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6   | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4   | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8    | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32    | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices.       |
| 1D Pixel Count                | UINT8    | 1    | #           | The number of pixels considered for the 1D range value.                                                                                                                                                                                                       |
| Saturated Pixel Count         | UINT8    | 1    | #           | The number of pixels with saturation flags set.                                                                                                                                                                                                               |
| 1D Range (binned)             | Q9.14    | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                              |
| 1D Amplitude (binned)         | UQ12.4   | 2    | LSB         | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                          |
| 1D Phase (binned)             | UQ1.15   | 2    | ?           | 1D phase as determined by the binning algorithm.                                                                                                                                                                                                              |
| Signal Quality                | UINT8    | 1    | %           | The signal quality indicator in % (0-100). 0%: invalid or not available; 1%: very bad signal, ...., 100%: perfect signal                                                                                                                                      |
| Integration Time              | UINT32   | 4    | µsec        | The actual device integration time in microseconds (used for debugging only).                                                                                                                                                                                 |
| Bias Current                  | UINT8    | 1    | LSB         | The BIAS laser current in LSB (used for debugging only).                                                                                                                                                                                                      |
| PLL Offset                    | UINT8    | 1    | LSB         | The internal register offset value of the PLL_INT_PRD register (used for debugging only).                                                                                                                                                                     |
| PLL Control Current           | UINT8    | 1    | LSB         | The internal register value of the PLL_CTRL_CUR register (used for debugging only).                                                                                                                                                                           |
| DCA Amplitude                 | UQ12.4   | 2    | LSB         | The internal amplitude used by the DCA algorithm to set up integration energy (used for debugging only).                                                                                                                                                      |
| Crosstalk Predictor Vectors   | Q11.4[,] | 8    | LSB         | The internal crosstalk predictor vector (used for debugging only). The first dS component is first, followed by its dC component. There are two vectors for upper/lower pixel rows.                                                                           |
| Crosstalk Monitor Vectors     | Q11.4[,] | 16   | LSB         | The internal crosstalk monitor vectors (used for debugging only). The first dS component is first, followed by its dC component. There are 4 vectors for each pixel row.                                                                                      |

### 1D Measurement Data Set {#cmd_data_1d}

Gets a 1D measurement data set containing the essential distance measurement
data.

| Caption / Name                | Type   | Size | Unit        | Comment                                                                                                                  |
| ----------------------------- | ------ | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------ |
| Command                       | UINT8  | 1    |             | 0xB6 (extended mode only)                                                                                                |
| Address                       | UINT8  | 1    |             | Extended frame address byte. Measurement Data is always explicitly sent from a single device.                            |
| Status                        | HEX16  | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                         |
| Timestamp                     | UINT48 | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                     |
| Measurement Frame State Flags | HEX16  | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                              |
| 1D Range (binned)             | Q9.14  | 3    | m           | 1D range as determined by the binning algorithm.                                                                         |
| 1D Amplitude (binned)         | UQ12.4 | 2    |             | 1D amplitude as determined by the binning algorithm.                                                                     |
| Signal Quality                | UINT8  | 1    | %           | The signal quality indicator in % (0-100). 0%: invalid or not available; 1%: very bad signal, ...., 100%: perfect signal |

## Configuration Commands {#explorer_app_cmd_cfg}

### Data Output Mode {#cmd_cfg_output_mode}

The measurement data output mode. (Hardware API only)

| Caption / Name               | Type  | Size | Unit | Comment                                                            |
| ---------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------ |
| Command                      | UINT8 | 1    |      | 0x41 (basic); 0xC1 (extended)                                      |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode.          |
| Measurement Data Output Mode | ENUM8 | 1    | n/a  | The measurement data output mode. See the table below for details. |

#### Measurement Data Output Mode Enumerator

| Value | Name                                            | Description                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----- | ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2     | [Streaming Debug Data](@ref cmd_data_full_dbg)  | When in 'Debug Data Streaming Mode', the software is streaming all available measurement data from the 1D and 3D measurements, i.e. raw correlation sampling data as well as the range, phase and amplitude values per pixel (3D) and from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided. If enabled, also the reference pixel results are available. |
| 3     | [Streaming Full Data](@ref cmd_data_full)       | When in 'Full Data Streaming Mode', the software is streaming all essential measurement data from the 1D and 3D measurements, i.e. range and amplitude values per pixel (3D) as well as from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided.                                                                                                           |
| 4     | [Streaming 3D Debug Data](@ref cmd_data_3d_dbg) | When in '3D Debug Data Streaming Mode', the software is streaming all available measurement data from the 3D measurements, i.e. the range, phase and amplitude values per pixel (3D). Additional information about the measurement frame is also provided.                                                                                                                                                    |
| 5     | [Streaming 3D Data](@ref cmd_data_3d)           | When in '3D Data Streaming Mode', the software is streaming all essential measurement data from the 3D measurements, i.e. range and amplitude values per pixel (3D). Additional information about the measurement frame is also provided.                                                                                                                                                                     |
| 6     | [Streaming 1D Debug Data](@ref cmd_data_1d_dbg) | When in '1D Debug Data Streaming Mode', the software is streaming all available measurement data from the 1D measurements, i.e. the range, phase and amplitude values from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided.                                                                                                                             |
| 7     | [Streaming 1D Data](@ref cmd_data_1d)           | When in '1D Data Streaming Mode', the software is streaming all essential measurement data from the 1D measurements, i.e. range and amplitude values from the pixel binning algorithm (1D).                                                                                                                                                                                                                   |

### Measurement Mode {#cmd_cfg_mode}

Gets or sets the measurement mode.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x42 (basic); 0xC2 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Measurement Mode             | ENUM8 | 1    | n/a  | The measurement mode. See #argus_mode_t for details.      |

\see

-   #Argus_GetMeasurementMode
-   #Argus_SetMeasurementMode
-   #Argus_ResetMeasurementMode

### Frame Time {#cmd_cfg_frame_time}

Gets or sets the measurement frame time or the inverse measurement frame rate.

| Caption / Name                  | Type   | Size | Unit | Comment                                                                                            |
| ------------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------- |
| Command                         | UINT8  | 1    |      | 0x43 (basic); 0xC3 (extended)                                                                      |
| Address (extended mode only)    | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                          |
| Frame Time (inverse Frame Rate) | UINT32 | 4    | µsec | The measurement frame time (1 / "frame rate") in usec. The interval to trigger measurement frames. |

\see

-   #Argus_GetConfigurationFrameTime
-   #Argus_SetConfigurationFrameTime

### Dual Frequency Mode {#cmd_cfg_dfm}

Gets or sets the Dual Frequency Mode configuration.

| Caption / Name               | Type  | Size | Unit | Comment                                                                                          |
| ---------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------------------------ |
| Command                      | UINT8 | 1    |      | 0x44 (basic); 0xC4 (extended)                                                                    |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                        |
| Dual Frequency Mode          | ENUM8 | 1    | n/a  | Selects the Dual-Frequency Mode (DFM). See See #argus_dfm_mode_t or the table below for details. |

\see

-   #Argus_GetConfigurationDFMMode
-   #Argus_SetConfigurationDFMMode

#### Dual Frequency Mode Enumerator

| Value | Name    | Description                                                                       |
| ----- | ------- | --------------------------------------------------------------------------------- |
| 0     | 1x Mode | Single Frequency Measurement Mode (w/ 1x Unambiguous Range). The DFM is disabled. |
| 1     | 4x Mode | 4X Dual Frequency Measurement Mode (w/ 4x Unambiguous Range).                     |
| 2     | 8x Mode | 8X Dual Frequency Measurement Mode (w/ 8x Unambiguous Range).                     |

### Smart Power Save Mode {#cmd_cfg_sps}

Gets or sets the smart power saving feature enabled flag.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x45 (basic); 0xC5 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Smart Power Save Enabled     | BOOL8 | 1    | n/a  | Enables the Smart Power Save Feature.                     |

\see

-   #Argus_GetConfigurationSmartPowerSaveEnabled
-   #Argus_SetConfigurationSmartPowerSaveEnabled

### Shot Noise Monitor Mode {#cmd_cfg_snm}

Gets or sets the shot noise monitor mode.

| Caption / Name               | Type  | Size | Unit | Comment                                                                                          |
| ---------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------------------------ |
| Command                      | UINT8 | 1    |      | 0x46 (basic); 0xC6 (extended)                                                                    |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                        |
| Shot Noise Monitor Mode      | ENUM8 | 1    | n/a  | Selects the Shot-Noise-Monitor (SNM) mode. See #argus_snm_mode_t or the table below for details. |

\see

-   #Argus_GetConfigurationShotNoiseMonitorMode
-   #Argus_SetConfigurationShotNoiseMonitorMode

#### Shot Noise Monitor Mode Enumerator

| Value | Name             | Description                                                            |
| ----- | ---------------- | ---------------------------------------------------------------------- |
| 0     | Static (Indoor)  | Static Shot Noise Monitoring Mode, optimized for indoor applications.  |
| 1     | Static (Outdoor) | Static Shot Noise Monitoring Mode, optimized for outdoor applications. |
| 2     | Dynamic          | Dynamic Shot Noise Monitoring Mode.                                    |

\see

-   #argus_snm_mode_t

### Crosstalk Monitor Mode {#cmd_cfg_xtm}

Gets or sets the crosstalk monitor mode.

| Caption / Name                 | Type  | Size | Unit | Comment                                                   |
| ------------------------------ | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                        | UINT8 | 1    |      | 0x47 (basic); 0xC7 (extended)                             |
| Address (extended mode only)   | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Crosstalk Monitor Mode Enabled | BOOL8 | 1    | n/a  | Enables the Crosstalk-Monitor (XTM) mode.                 |

\see

-   #Argus_GetConfigurationCrosstalkMonitorMode
-   #Argus_SetConfigurationCrosstalkMonitorMode

### Dynamic Configuration Adaption {#cmd_cfg_dca}

Gets or sets the setting parameters of the Dynamic Configuration Adaption (DCA)
algorithm.

| Caption / Name                                     | Type   | Size | Unit | Comment                                                                                                                                                                                                        |
| -------------------------------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                                            | UINT8  | 1    |      | 0x52 (basic); 0xD2 (extended)                                                                                                                                                                                  |
| Address (extended mode only)                       | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                                                                                                                                      |
| DCA Enabled Flags                                  | ENUM8  | 1    | n/a  | DCA feature enable flags. See #argus_dca_enable_t for details.                                                                                                                                                 |
| Saturated Pixel Threshold for Linear Response      | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an linear decrease of integration energy.                                                                                                       |
| Saturated Pixel Threshold for Exponential Response | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an exponential decrease of integration energy.                                                                                                  |
| Saturated Pixel Threshold for Reset Response       | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an integration energy reset. Use 32 to disable.                                                                                                 |
| Target Amplitude                                   | UQ12.4 | 2    | LSB  | The target amplitude. The DCA will try to linearity approach the target value.                                                                                                                                 |
| Low Amplitude Threshold                            | UQ12.4 | 2    | LSB  | The minimum allowed amplitude before the integration energy will be increased.                                                                                                                                 |
| High Amplitude Threshold                           | UQ12.4 | 2    | LSB  | The maximum allowed amplitude before the integration energy will be decreased.                                                                                                                                 |
| Amplitude Mode                                     | ENUM8  | 1    | n/a  | The amplitude evaluation mode for the DCA algorithm. Either maximum or average amplitude can be used. See #argus_dca_amplitude_mode_t for details.                                                             |
| Nominal Integration Depth                          | UQ10.6 | 2    | #    | The nominal analog integration depth. This determines the start value or the static value depending on the Enabled setting.                                                                                    |
| Min. Integration Depth                             | UQ10.6 | 2    | #    | The minimum analog integration depth, i.e. the minimum pattern count per sample.                                                                                                                               |
| Max. Integration Depth                             | UQ10.6 | 2    | #    | The maximum analog integration depth, i.e. the maximum pattern count per sample.                                                                                                                               |
| Optical Power                                      | ENUM8  | 1    | n/a  | The optical output power stages. See #argus_dca_power_t for details.                                                                                                                                           |
| Nominal Pixel Gain                                 | ENUM8  | 1    | n/a  | The nominal pixel gain value for the default DCA gain stage. This determines the start value or the static value depending on the Enabled setting. See #argus_dca_gain_t for details.                          |
| Low Pixel Gain                                     | ENUM8  | 1    | n/a  | The low pixel gain value for the DCA low gain stage. See #argus_dca_gain_t for details.                                                                                                                        |
| High Pixel Gain                                    | ENUM8  | 1    | n/a  | The high pixel gain value for the DCA high gain stage. See #argus_dca_gain_t for details.                                                                                                                      |
| Power Saving Ratio                                 | UQ0.8  | 1    | n/a  | Determines the percentage of the full available frame time that is not exploited for digital integration. Thus the device is idle within the specified portion of the frame time and does consume less energy. |

\see

-   #Argus_GetConfigurationDynamicAdaption
-   #Argus_SetConfigurationDynamicAdaption
-   #argus_cfg_dca_t

### Pixel Binning {#cmd_cfg_pba}

Gets or sets the setting parameters of the Pixel Binning (PBA) algorithm.

| Caption / Name                  | Type   | Size | Unit | Comment                                                                                                                                                                                                                                 |
| ------------------------------- | ------ | ---- | ---- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                         | UINT8  | 1    |      | 0x54 (basic); 0xD4 (extended)                                                                                                                                                                                                           |
| Address (extended mode only)    | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                                                                                                                                                               |
| PBA Enabled Flags               | ENUM8  | 1    | n/a  | Enables/disables the pixel binning (features). See #argus_pba_flags_t for details.                                                                                                                                                      |
| Averaging Mode                  | ENUM8  | 1    | n/a  | Selects the averaging mode for the PBA. See #argus_pba_averaging_mode_t for details.                                                                                                                                                    |
| Pre-filter Mask                 | HEX32  | 4    | n/a  | The pixel pre-filter mask determines the pixels that are dismissed by the PBA. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argus_map) for the corresponding x-y-indices. |
| Absolute Amplitude Threshold    | UQ12.4 | 2    | LSB  | Absolute Amplitude Threshold in LSB. Lower values are dismissed.                                                                                                                                                                        |
| Relative Amplitude Threshold    | UQ0.8  | 1    | %    | Relative Amplitude Threshold in % of max value. Lower values are dismissed.                                                                                                                                                             |
| Absolute Minimum Distance Scope | UQ1.15 | 2    | m    | Absolute Minimum Distance Scope Filter Threshold Value in LSB. Distances with larger values (compared to current minimum distance) are dismissed.                                                                                       |
| Relative Minimum Distance Scope | UQ0.8  | 1    | %    | Relative Minimum Distance Scope Filter Threshold Value in % of the current minimum distance. Distances with larger values (compared to current minimum distance) are dismissed                                                          |

\see

-   #Argus_GetConfigurationPixelBinning
-   #Argus_SetConfigurationPixelBinning

### SPI Configuration {#cmd_cfg_spi}

Gets or sets the SPI configuration parameters.

| Caption / Name               | Type   | Size | Unit | Comment                                                   |
| ---------------------------- | ------ | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x58 (basic); 0xD8 (extended)                             |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| SPI Baud Rate                | UINT32 | 4    | 0    | Baud Rate in BaudPerSeconds.                              |

@note The available baud rates for SPI interfaces depend on the platform.

@note In `v1.4.4` the SPI slave parameters is removed in favor of the device
address. Note further that multiple devices may share a single SPI instance.
Thus, changing the SPI baud rate for one device may affect all other device too!

Available Baud Rates:

-   NXP MKL46z Baud Rates: e.g. 12 MHz, 6 MHz, 3 MHz, 1.5 MHz, ...
-   NXP MKL17z Baud Rates: e.g. 6 MHz, 3 MHz, 1.5 MHz, ...
-   STM32F401RE Baud Rates: e.g. 21 MHz, 10.5 MHz, 5.25 MHz, ...

### UART Configuration {#cmd_cfg_uart}

Gets or sets the UART configuration (e.g. baud rate).

| Caption / Name               | Type   | Size | Unit | Comment                                                   |
| ---------------------------- | ------ | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x59 (basic); 0xD9 (extended)                             |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| UART Baud Rate               | UINT32 | 4    | 0    | Baud Rate in BaudPerSeconds.                              |

@note The device responds with an acknowledge before changing the UART baud
rate.

@note The UART configuration is only available for the STM32F401RE Evaluation
Board. The board is initialized with 1 Mbps upon reset.

@note The address byte is ignored as all devices share the same UART connection.
Thus, changing the UART baud rate for one device will affect all other device
too!

Available Baud Rates:

-   115200 bps
-   500000 bps
-   1000000 bps (1 Mbps), default after MCU reset/power on.
-   2000000 bps (2 Mbps)

## Calibration Commands {#explorer_app_cmd_cal}

### Global Range Offset {#cmd_cal_range_offset}

Gets or sets the global range offset.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x61 (basic); 0xE1 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Range Offset                 | Q0.15 | 2    | m    | The global range offset in meters.                        |

\see

-   #Argus_GetCalibrationGlobalRangeOffsets
-   #Argus_SetCalibrationGlobalRangeOffsets

### Pixel Range Offsets {#cmd_cal_offsets}

Gets or sets the pixel-to-pixel range offset table.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x67 (basic); 0xE7 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |
| Offset Values (x, y)         | Q0.15 | 64   | m    | The pixel offsets table. Ordering: X Index -> Y Index.    |

Note on indices:

-   x: The pixel x-index (0-7)
-   y: The pixel y-index (0-3)

\see

-   #Argus_GetCalibrationPixelRangeOffsets
-   #Argus_SetCalibrationPixelRangeOffsets

### Pixel Range Offsets - Reset Offset Table {#cmd_cal_offsets_rst}

Command to reset the range offset table to the factory calibrated default
values.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x68 (basic); 0xE8 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_ResetCalibrationPixelRangeOffsets

### Range Offsets Calibration Sequence - Sample Time {#cmd_cal_offsets_smpl_time}

Gets or sets the range offset calibration sequence sample time.

| Caption / Name               | Type   | Size | Unit | Comment                                                                                                                          |
| ---------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x69 (basic); 0xE9 (extended)                                                                                                    |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                                                        |
| Sample Time                  | UINT16 | 2    |      | The measurement sample acquisition time for executing the offset calibration sequence and generate the offset data. Units: msec. |

\see

-   #Argus_GetCalibrationRangeOffsetSequenceSampleTime
-   #Argus_SetCalibrationRangeOffsetSequenceSampleTime

### Crosstalk Compensation - Vector Table {#cmd_cal_xtalk_vec}

Gets or sets the crosstalk compensation vector table.

| Caption / Name                | Type  | Size | Unit | Comment                                                                                             |
| ----------------------------- | ----- | ---- | ---- | --------------------------------------------------------------------------------------------------- |
| Command                       | UINT8 | 1    |      | 0x62 (basic); 0xE2 (extended)                                                                       |
| Address (extended mode only)  | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                           |
| Crosstalk Vector (f, x, y, a) | Q11.4 | 256  | LSB  | The crosstalk vector table. Ordering: Frequency Index (A/B) -> X Index -> Y Index -> Sin/Cos Index. |

Note on indices:

-   f: The frequency index for A- and B-frames respectively (A = 0; B = 1)
-   x: The pixel x-index (0-7)
-   y: The pixel y-index (0-3)
-   a: The sin/cos index (sin = 0; cos = 1)

\see

-   #Argus_GetCalibrationCrosstalkVectorTable
-   #Argus_SetCalibrationCrosstalkVectorTable

### Crosstalk Compensation - Reset Vector Table {#cmd_cal_xtalk_rst}

Command to reset the crosstalk vector table to the factory calibrated default
values.

| Caption / Name               | Type  | Size | Unit | Comment                                                   |
| ---------------------------- | ----- | ---- | ---- | --------------------------------------------------------- |
| Command                      | UINT8 | 1    |      | 0x63 (basic); 0xE3 (extended)                             |
| Address (extended mode only) | UINT8 | 1    |      | Extended frame address byte. Skipped in basic frame mode. |

\see

-   #Argus_ResetCalibrationCrosstalkVectorTable

### Crosstalk Calibration Sequence - Sample Time {#cmd_cal_xtalk_smpl_time}

Gets or sets the crosstalk calibration sequence sample time.

| Caption / Name               | Type   | Size | Unit | Comment                                                                                                                                |
| ---------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x64 (basic); 0xE4 (extended)                                                                                                          |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                                                              |
| Sample Time                  | UINT16 | 2    |      | The measurement sample acquisition time for executing the crosstalk calibration sequence and generate the crosstalk data. Units: msec. |

\see

-   #Argus_GetCalibrationCrosstalkSequenceSampleTime
-   #Argus_SetCalibrationCrosstalkSequenceSampleTime

### Crosstalk Calibration Sequence - Maximum Amplitude Threshold {#cmd_cal_xtalk_max_ampl}

Gets or sets the crosstalk calibration sequence maximum amplitude threshold.

| Caption / Name               | Type   | Size | Unit | Comment                                                             |
| ---------------------------- | ------ | ---- | ---- | ------------------------------------------------------------------- |
| Command                      | UINT8  | 1    |      | 0x65 (basic); 0xE5 (extended)                                       |
| Address (extended mode only) | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.           |
| Maximum Amplitude Threshold  | UQ12.4 | 2    | LSB  | The maximum amplitude threshold for crosstalk calibration sequence. |

\see

-   #Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold
-   #Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold

### Pixel-2-Pixel Crosstalk Compensation Parameters {#cmd_cal_xtalk_p2p}

Gets or sets the pixel-2-pixel crosstalk calibration parameter values.

| Caption / Name                          | Type   | Size | Unit | Comment                                                                                                                                                                            |
| --------------------------------------- | ------ | ---- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                                 | UINT8  | 1    |      | 0x66 (basic); 0xE6 (extended)                                                                                                                                                      |
| Address (extended mode only)            | UINT8  | 1    |      | Extended frame address byte. Skipped in basic frame mode.                                                                                                                          |
| Enabled                                 | BOOL8  | 1    |      | Enables/disables the pixel-2-pixel crosstalk compensation algorithm.                                                                                                               |
| Kc Factor - Sine Component              | Q3.12  | 2    |      | The sine component of the Kc factor that determines the amount of the total signal on all pixels influences the individual signal of each pixel.                                   |
| Kc Factor - Cosine Component            | Q3.12  | 2    |      | The cosine component of the Kc factor that determines the amount of the total signal on all pixels influences the individual signal of each pixel.                                 |
| Ref. Pixel Kc Factor - Sine Component   | Q3.12  | 2    |      | The sine component of the Kc factor that determines the amount of the total signal on all pixels influences the individual signal of each pixel. Value for reference pixel only.   |
| Ref. Pixel Kc Factor - Cosine Component | Q3.12  | 2    |      | The cosine component of the Kc factor that determines the amount of the total signal on all pixels influences the individual signal of each pixel. Value for reference pixel only. |
| Relative Threshold                      | UQ0.8  | 1    |      | The relative threshold determines when the compensation is active for each individual pixel.                                                                                       |
| Absolute Threshold                      | UQ12.4 | 2    |      | The absolute threshold determines the minimum total crosstalk amplitude that is required for the compensation to become active.                                                    |

\see

-   #Argus_GetCalibrationCrosstalkPixel2Pixel
-   #Argus_SetCalibrationCrosstalkPixel2Pixel
