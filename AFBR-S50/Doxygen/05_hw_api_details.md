Command Details {#hw_api_cmd_details}
=====================================

The following section contains the current command details as implemented in the _ExplorerApp_. The implementation can be found in the "Sources\explorer_app\api" folder of the "AFBR_S50_ExplorerApp_KL46z" project. See also @ref hw_api.

# Generic Commands {#hw_api_cmd_generic}

## Invalid Command {#cmd_invalid}

| Caption / Name | Type  | Size  | Unit | Comment              |
| -------------- | ----- | ----- | ---- | -------------------- |
| Command        | UINT8 | 1     |      | 0x00                 |
| Status         | UINT8 | 1 / 0 |      | Slave to Master only |

## Acknowledge {#cmd_ack}

| Caption / Name       | Type  | Size | Unit | Comment |
| -------------------- | ----- | ---- | ---- | ------- |
| Command              | UINT8 | 1    |      | 0x0A    |
| Acknowledged Command | UINT8 | 1    |      |         |

## Not Acknowledge {#cmd_nak}

| Caption / Name                 | Type   | Size | Unit | Comment |
| ------------------------------ | ------ | ---- | ---- | ------- |
| Command                        | UINT8  | 1    |      | 0x0B    |
| Not Acknowledged Command       | UINT8  | 1    |      |         |
| Serial Status / Reason for NAK | UINT16 | 2    |      |         |

## Log Message {#cmd_log}

| Caption / Name    | Type               | Size | Unit  | Comment |
| ----------------- | ------------------ | ---- | ----- | ------- |
| Command           | UINT8              | 1    |       | 0x06    |
| Time Stamp [sec]  | UINT32             | 4    | sec   |         |
| Time Stamp [µsec] | UINT16             | 2    | ms/16 |         |
| Message           | CHAR[] (= UINT8[]) | n/a  |       |         |

## Test Message {#cmd_test}

| Caption / Name | Type    | Size | Unit | Comment                                                             |
| -------------- | ------- | ---- | ---- | ------------------------------------------------------------------- |
| Command        | UINT8   | 1    |      | 0x04                                                                |
| Test message   | UINT8[] | n/a  |      | The returned message does also contain the CRC of the sent message. |

## MCU/Software Reset {#cmd_reset}

| Caption / Name | Type   | Size | Unit | Comment    |
| -------------- | ------ | ---- | ---- | ---------- |
| Command        | UINT8  | 1    |      | 0x08       |
| Safety Code    | UINT32 | 4    |      | 0xDEADC0DE |

## Software Version {#cmd_sw}

| Caption / Name               | Type                   | Size | Unit | Comment                                                                              |
| ---------------------------- | ---------------------- | ---- | ---- | ------------------------------------------------------------------------------------ |
| Command                      | UINT8                  | 1    |      | 0x0C                                                                                 |
| Version Software             | UINT32                 | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                          |
| Software Build Number String | CHAR[14] (= UINT8[14]) | n/a  |      | (Added in v1.2.0) Software Build Number as String (14 Digits; e.g. "20200101123456") |

@note This command can be used to determine the current software version of the connected device. However, the command has changed as the build number was added to the command for v1.2.0 and newer versions. In order to determine the version, it is sufficient to encode only the version number. If the version is at least v1.2.0, the build number can also be extracted from the command data array.

## Module Type {#cmd_module}

| Caption / Name      | Type  | Size | Unit | Comment                                         |
| ------------------- | ----- | ---- | ---- | ----------------------------------------------- |
| Command             | UINT8 | 1    |      | 0x0E                                            |
| Module Type/Version | UINT8 | 1    |      | Module type number. See #argus_module_version_t |
| Chip Type/Version   | UINT8 | 1    |      | Module type number. See #argus_chip_version_t   |
| Laser Type          | UINT8 | 1    |      | Module type number. See #argus_laser_type_t     |

## Module UID {#cmd_uid}

| Caption / Name | Type   | Size | Unit | Comment                      |
| -------------- | ------ | ---- | ---- | ---------------------------- |
| Command        | UINT8  | 1    |      | 0x0F                         |
| Module UID     | UINT24 | 3    |      | Unique Identification Number |

## Software Information / Identification {#cmd_info}

| Caption / Name                  | Type               | Size | Unit | Comment                                                                                                                                              |
| ------------------------------- | ------------------ | ---- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                         | UINT8              | 1    |      | 0x05                                                                                                                                                 |
| Version Software (Explorer App) | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Version Argus API               | UINT32             | 4    |      | Major [31:24], Minor [23:16], Bugfix [15:0]                                                                                                          |
| Module Type/Version             | UINT8              | 1    |      | Module type number. See #argus_module_version_t                                                                                                      |
| Chip Type/Version               | UINT8              | 1    |      | Module type number. See #argus_chip_version_t                                                                                                        |
| Laser Type                      | UINT8              | 1    |      | Module type number. See #argus_laser_type_t                                                                                                          |
| Module UID                      | UINT24             | 3    |      | Unique Identification Number                                                                                                                         |
| Software ID String              | CHAR[] (= UINT8[]) | n/a  |      | Software ID String incl. Build Number (Format: "AFBR-S50 Explorer App - [BuildNumber]"; w/ build number containing 14 digits, e.g. "20200101123456") |

@note This command can be used to determine the current software version of the connected device. However, the command has changed, e.g. the build number was added to the command for v1.2.0 and newer versions. In order to determine the version, it is sufficient to encode only the first version number. If the version is not current, the remaining command may differ. In case of different version, please refer to the corresponding API documentation to find the correct command formats.

# Device Control Commands {#hw_api_cmd_ctrl}

## Measurement: Trigger Single Shot {#cmd_ctrl_trigger}

| Caption / Name | Type  | Size | Unit | Comment |
| -------------- | ----- | ---- | ---- | ------- |
| Command        | UINT8 | 1    |      | 0x10    |

## Measurement: Start Auto {#cmd_ctrl_start}

| Caption / Name | Type  | Size | Unit | Comment |
| -------------- | ----- | ---- | ---- | ------- |
| Command        | UINT8 | 1    |      | 0x11    |

## Measurement: Stop {#cmd_ctrl_stop}

| Caption / Name | Type  | Size | Unit | Comment |
| -------------- | ----- | ---- | ---- | ------- |
| Command        | UINT8 | 1    |      | 0x12    |

## Run Calibration {#cmd_ctrl_cal}

| Caption / Name          | Type  | Size | Unit | Comment                                                                        |
| ----------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------ |
| Command                 | UINT8 | 1    |      | 0x18                                                                           |
| Calibration Sequence ID | ENUM8 | 1    | n/a  | The selected calibration measurement sequence. See table below.                |
| Optional Parameters     | n/a   | >=0  | n/a  | Optional parameters for the individual calibration sequences. See table below. |

### Calibration Sequence Enumerator

| Value | Name                                    | Parameters                                                   | Description                                                |
| ----- | --------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------------- |
| 2     | Crosstalk Calibration (Mode A)          | n/a                                                          | Crosstalk calibration sequence for measurement mode A.     |
| 3     | Crosstalk Calibration (Mode B)          | n/a                                                          | Crosstalk calibration sequence for measurement mode B.     |
| 5     | Pixel Range Offset Calibration (Mode A) | optional: Calibration Target Distance (Type: Q9.22; Unit: m) | Range offsets calibration sequence for measurement mode A. |
| 6     | Pixel Range Offset Calibration (Mode B) | optional: Calibration Target Distance (Type: Q9.22; Unit: m) | Range offsets calibration sequence for measurement mode B. |

## Device Reinitialize {#cmd_ctrl_reinit}

| Caption / Name | Type  | Size | Unit | Comment |
| -------------- | ----- | ---- | ---- | ------- |
| Command        | UINT8 | 1    |      | 0x19    |

# Measurement Data Commands {#hw_api_cmd_data}

## Raw Measurement Data Set {#cmd_data_raw}

| Caption / Name                | Type     | Size | Unit        | Comment                                                                                                                                                                                                                                                                                                                                             |
| ----------------------------- | -------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Command                       | UINT8    | 1    |             | 0x30                                                                                                                                                                                                                                                                                                                                                |
| Status                        | HEX16    | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                                                                                                                    |
| Timestamp                     | UINT48   | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                                                                                                                |
| Measurement Frame State Flags | HEX16    | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                                                                                                         |
| Digital Integration Depth     | UINT16   | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                                                                                                         |
| Analog Integration Depth      | UQ10.6   | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods).                                                                                       |
| Optical Power                 | UQ12.4   | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                                                                                                        |
| Pixel Gain                    | UINT8    | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                                                                                                         |
| Enabled Pixel Mask            | HEX32    | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices.                                                                                                               |
| Enabled ADC Channel Mask      | HEX32    | 4    | n/a         | The ADC channel enabled mask determines the misc. ADC channels that are enabled and converted by the ADC muxing. The mask is channel based (starting from channel 32), i.e. the n-th bit represents the n-th ADC channel. See the channel mapping for the corresponding channel purpose.                                                            |
| Phase Count                   | UINT8    | 1    | #           | Number of phase steps.                                                                                                                                                                                                                                                                                                                              |
| ADC Samples                   | UINT24[] | 396  | LSB         | Raw sampling data from the pixel field. The 22 LSBs determine the raw 22-bit ADC readouts and the two MSBs determine the saturation flags. The values are ordered in increasing channel number where different phase steps are gathered (i.e. \f$idx = 4 n + p\f$). Disabled values (see Enabled Pixel Mask and Enabled ADC Channel Mask) are skipped. |

## Measurement Data Set (1D + 3D) - Debug {#cmd_data_full_dbg}

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                                                                                                             |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Command                       | UINT8     | 1    |             | 0x31                                                                                                                                                                                                                                                                                                                                                |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                                                                                                                    |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                                                                                                                |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                                                                                                         |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                                                                                                         |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods).                                                                                       |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                                                                                                        |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                                                                                                         |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices.                                                                                                               |
| Enabled ADC Channel Mask      | HEX32     | 4    | n/a         | The ADC channel enabled mask determines the misc. ADC channels that are enabled and converted by the ADC muxing. The mask is channel based (starting from channel 32), i.e. the n-th bit represents the n-th ADC channel. See the channel mapping for the corresponding channel purpuse.                                                            |
| Phase Count                   | UINT8     | 1    | #           | Number of phase steps.                                                                                                                                                                                                                                                                                                                              |
| ADC Samples                   | UINT24[]  | 396  | LSB         | Raw sampling data from the pixel field. The 22 LSBs determine the raw 22-bit ADC readouts and the two MSBs determine the saturation flags. The values are ordered in increasing channel number where different phase steps are gathered (i.e. \f$idx = 4 n + p\f$). Disabled values (see Enabled Pixel Mask and Enabled ADC Channel Mask) are skipped. |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                     |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                             |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                         |
| Phase (x, y)                  | UQ1.15[,] | 64   | ?           | Phase values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                                                                                                             |
| Status (ref)                  | HEX8      | 1    | m           | Status flags (see #argus_px_status_t) for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                  |
| Range (ref)                   | Q9.14     | 3    | m           | Range for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                                  |
| Amplitude (ref)               | UQ12.4    | 2    |             | Amplitude for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                              |
| Phase (ref)                   | UQ1.15    | 2    | ?           | Phase for the reference pixel (Skipped if disabled, see Enabled ADC Channel Mask).                                                                                                                                                                                                                                                                  |
| 1D Range (binned)             | Q9.14     | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                                                                                                                    |
| 1D Amplitude (binned)         | UQ12.4    | 2    |             | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                                                                                                                |
| VDD                           | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDD                                                                                                                                                                                                                                                                                                               |
| VDDL                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDDL                                                                                                                                                                                                                                                                                                              |
| VSUB                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VSUB                                                                                                                                                                                                                                                                                                              |
| IAPD                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for IAPD                                                                                                                                                                                                                                                                                                              |
| TEMP                          | Q11.4     | 2    | °C          | Auxiliary measurement results for TEMP                                                                                                                                                                                                                                                                                                              |
| BGL                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the background light value.                                                                                                                                                                                                                                                                                                 |

## Measurement Data Set (1D + 3D) {#cmd_data_full}

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0x32                                                                                                                                                                                                                                                          |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices.                         |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                               |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                       |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                   |
| 1D Range (binned)             | Q9.14     | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                              |
| 1D Amplitude (binned)         | UQ12.4    | 2    |             | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                          |
| VDD                           | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDD                                                                                                                                                                                                                         |
| VDDL                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VDDL                                                                                                                                                                                                                        |
| VSUB                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for VSUB                                                                                                                                                                                                                        |
| IAPD                          | UQ12.4    | 2    | LSB         | Auxiliary measurement results for IAPD                                                                                                                                                                                                                        |
| TEMP                          | Q11.4     | 2    | °C          | Auxiliary measurement results for TEMP                                                                                                                                                                                                                        |
| BGL                           | UQ12.4    | 2    | LSB         | Auxiliary estimation of the background light value.                                                                                                                                                                                                           |

## 3D Measurement Data Set - Debug {#cmd_data_3d_dbg}

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0x33                                                                                                                                                                                                                                                          |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices.                         |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                               |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                       |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                   |
| Phase (x, y)                  | UQ1.15[,] | 64   | ?           | Phase values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                       |

## 3D Measurement Data Set {#cmd_data_3d}

| Caption / Name                | Type      | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | --------- | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8     | 1    |             | 0x34                                                                                                                                                                                                                                                          |
| Status                        | HEX16     | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48    | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16     | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16    | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6    | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4    | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8     | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| Enabled Pixel Mask            | HEX32     | 4    | n/a         | The pixel enabled mask determines the pixels that are enabled and converted by the ADC muxing. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices.                         |
| Status (x, y)                 | HEX8[,]   | 32   | m           | Status flags for each enabled pixel, see #argus_px_status_t. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                               |
| Range (x, y)                  | Q9.14[,]  | 96   | m           | Range values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                       |
| Amplitude (x, y)              | UQ12.4[,] | 64   |             | Amplitude values for each enabled pixel. The values are ordered in increasing x and y indices (i.e. \f$n = 4 x + y\f$). Disabled values (see Enabled Pixel Mask) are skipped.                                                                                   |

## 1D Measurement Data Set - Debug {#cmd_data_1d_dbg}

| Caption / Name                | Type   | Size | Unit        | Comment                                                                                                                                                                                                                                                       |
| ----------------------------- | ------ | ---- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                       | UINT8  | 1    |             | 0x35                                                                                                                                                                                                                                                          |
| Status                        | HEX16  | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0                                                                                                                                                                              |
| Timestamp                     | UINT48 | 6    | sec;µsec/16 | Contains the measurement start time.                                                                                                                                                                                                                          |
| Measurement Frame State Flags | HEX16  | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.                                                                                                                                                                                   |
| Digital Integration Depth     | UINT16 | 2    | #           | The digital integration depth determines the digital averaging depth of a single measurement frame. This is the averaging sample count i.e. the number of repeated analog measurements for each phase step.                                                   |
| Analog Integration Depth      | UQ10.6 | 2    | #           | The analog integration depth determines the number of 128-bit laser patterns that are sent for a single correlation cycle. The value can be either a multiple of a full pattern or a fraction of a single pattern (i.e. a multiple of single pulses/periods). |
| Optical Power                 | UQ12.4 | 2    | mA          | The optical output power is determined by the laser current.                                                                                                                                                                                                  |
| Pixel Gain                    | UINT8  | 1    |             | The pixel gain determines the sensitivity of the ToF cells.                                                                                                                                                                                                   |
| 1D Pixel Count                | UINT8  | 1    | #           | The number of pixels considered for the 1D range value.                                                                                                                                                                                                       |
| Saturated Pixel Count         | UINT8  | 1    | #           | The number of pixels with saturation flags set.                                                                                                                                                                                                               |
| 1D Range (binned)             | Q9.14  | 3    | m           | 1D range as determined by the binning algorithm.                                                                                                                                                                                                              |
| 1D Amplitude (binned)         | UQ12.4 | 2    | LSB         | 1D amplitude as determined by the binning algorithm.                                                                                                                                                                                                          |
| 1D Phase (binned)             | UQ1.15 | 2    | ?           | 1D phase as determined by the binning algorithm.                                                                                                                                                                                                              |

## 1D Measurement Data Set {#cmd_data_1d}

| Caption / Name                | Type   | Size | Unit        | Comment                                                                          |
| ----------------------------- | ------ | ---- | ----------- | -------------------------------------------------------------------------------- |
| Command                       | UINT8  | 1    |             | 0x36                                                                             |
| Status                        | HEX16  | 2    | n/a         | Provides information about the measurement status. OK = 0; ERROR < 0; STATUS > 0 |
| Timestamp                     | UINT48 | 6    | sec;µsec/16 | Contains the measurement start time.                                             |
| Measurement Frame State Flags | HEX16  | 2    | n/a         | The state of the current measurement frame. See #argus_state_t for details.      |
| 1D Range (binned)             | Q9.14  | 3    | m           | 1D range as determined by the binning algorithm.                                 |
| 1D Amplitude (binned)         | UQ12.4 | 2    |             | 1D amplitude as determined by the binning algorithm.                             |

# Configuration Commands {#hw_api_cmd_cfg}

## Data Output Mode {#cmd_cfg_output_mode}

| Caption / Name               | Type  | Size | Unit | Comment                                                            |
| ---------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------ |
| Command                      | UINT8 | 1    |      | 0x41                                                               |
| Measurement Data Output Mode | ENUM8 | 1    | n/a  | The measurement data output mode. See the table below for details. |

### Measurement Data Output Mode Enumerator

| Value | Name                    | Description                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----- | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1     | Streaming Raw Data      | When in 'Raw Data Streaming Mode', the software is streaming the raw measurement data only, i.e. the raw correlation samples per pixel.                                                                                                                                                                                                                                                                       |
| 2     | Streaming Debug Data    | When in 'Debug Data Streaming Mode', the software is streaming all available measurement data from the 1D and 3D measurements, i.e. raw correlation sampling data as well as the range, phase and amplitude values per pixel (3D) and from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided. If enabled, also the reference pixel results are available. |
| 3     | Streaming Full Data     | When in 'Full Data Streaming Mode', the software is streaming all essential measurement data from the 1D and 3D measurements, i.e. range and amplitude values per pixel (3D) as well as from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided.                                                                                                           |
| 4     | Streaming 3D Debug Data | When in '3D Debug Data Streaming Mode', the software is streaming all available measurement data from the 3D measurements, i.e. the range, phase and amplitude values per pixel (3D). Additional information about the measurement frame is also provided.                                                                                                                                                    |
| 5     | Streaming 3D Data       | When in '3D Data Streaming Mode', the software is streaming all essential measurement data from the 3D measurements, i.e. range and amplitude values per pixel (3D). Additional information about the measurement frame is also provided.                                                                                                                                                                     |
| 6     | Streaming 1D Debug Data | When in '1D Debug Data Streaming Mode', the software is streaming all available measurement data from the 1D measurements, i.e. the range, phase and amplitude values from the pixel binning algorithm (1D). Additional information about the measurement frame is also provided.                                                                                                                             |
| 7     | Streaming 1D Data       | When in '1D Data Streaming Mode', the software is streaming all essential measurement data from the 1D measurements, i.e. range and amplitude values from the pixel binning algorithm (1D).                                                                                                                                                                                                                   |

## Measurement Mode {#cmd_cfg_mode}

| Caption / Name   | Type  | Size | Unit | Comment                                                           |
| ---------------- | ----- | ---- | ---- | ----------------------------------------------------------------- |
| Command          | UINT8 | 1    |      | 0x42                                                              |
| Measurement Mode | ENUM8 | 1    | n/a  | Gets or sets the measurement mode. See #argus_mode_t for details. |

## Frame Time {#cmd_cfg_frame_time}

| Caption / Name                  | Type   | Size | Unit | Comment                                                                                            |
| ------------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------- |
| Command                         | UINT8  | 1    |      | 0x43                                                                                               |
| Frame Time (inverse Frame Rate) | UINT32 | 4    | µsec | The measurement frame time (1 / "frame rate") in usec. The interval to trigger measurement frames. |

## Dual Frequency Mode {#cmd_cfg_dfm}

| Caption / Name            | Type  | Size | Unit | Comment                                                                                          |
| ------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------------------------ |
| Command                   | UINT8 | 1    |      | 0x44                                                                                             |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details.                                   |
| Dual Frequency Mode       | ENUM8 | 1    | n/a  | Selects the Dual-Frequency Mode (DFM). See See #argus_dfm_mode_t or the table below for details. |

### Dual Frequency Mode Enumerator

| Value | Name    | Description                                                                       |
| ----- | ------- | --------------------------------------------------------------------------------- |
| 0     | 1x Mode | Single Frequency Measurement Mode (w/ 1x Unambiguous Range). The DFM is disabled. |
| 1     | 4x Mode | 4X Dual Frequency Measurement Mode (w/ 4x Unambiguous Range).                     |
| 2     | 8x Mode | 8X Dual Frequency Measurement Mode (w/ 8x Unambiguous Range).                     |

## Smart Power Save Mode {#cmd_cfg_sps}

| Caption / Name            | Type  | Size | Unit | Comment                                                        |
| ------------------------- | ----- | ---- | ---- | -------------------------------------------------------------- |
| Command                   | UINT8 | 1    |      | 0x45                                                           |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details. |
| Smart Power Save Enabled  | BOOL8 | 1    | n/a  | Enables the Smart Power Save Feature.                          |

## Shot Noise Monitor Mode {#cmd_cfg_snm}

| Caption / Name            | Type  | Size | Unit | Comment                                                                                          |
| ------------------------- | ----- | ---- | ---- | ------------------------------------------------------------------------------------------------ |
| Command                   | UINT8 | 1    |      | 0x46                                                                                             |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details.                                   |
| Shot Noise Monitor Mode   | ENUM8 | 1    | n/a  | Selects the Shot-Noise-Monitor (SNM) mode. See #argus_snm_mode_t or the table below for details. |

### Shot Noise Monitor Mode Enumerator

| Value | Name             | Description                                                            |
| ----- | ---------------- | ---------------------------------------------------------------------- |
| 0     | Static (Indoor)  | Static Shot Noise Monitoring Mode, optimized for indoor applications.  |
| 1     | Static (Outdoor) | Static Shot Noise Monitoring Mode, optimized for outdoor applications. |
| 2     | Dynamic          | Dynamic Shot Noise Monitoring Mode.                                    |

## Dynamic Configuration Adaption {#cmd_cfg_dca}

| Caption / Name                                     | Type   | Size | Unit | Comment                                                                                                                                                                                                        |
| -------------------------------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                                            | UINT8  | 1    |      | 0x52                                                                                                                                                                                                           |
| Measurement Mode Selector                          | ENUM8  | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details.                                                                                                                                                 |
| Enabled Flags                                      | HEX8   | 1    | n/a  | DCA features enable flags. Bit 0: DCA feature enable, Bit 1: use average amplitude                                                                                                                             |
| Saturated Pixel Threshold for Linear Response      | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an linear decrease of integration energy.                                                                                                       |
| Saturated Pixel Threshold for Exponential Response | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an exponential decrease of integration energy.                                                                                                  |
| Saturated Pixel Threshold for Reset Response       | UINT8  | 1    | #    | The maximum allowed overflow pixels before the DCA will invoke an integration energy reset. Use 32 to disable.                                                                                                 |
| Target Amplitude                                   | UQ12.4 | 2    | LSB  | The target amplitude. The DCA will try to linearity approach the target value.                                                                                                                                 |
| Low Amplitude Threshold                            | UQ12.4 | 2    | LSB  | The minimum allowed amplitude before the integration energy will be increased.                                                                                                                                 |
| High Amplitude Threshold                           | UQ12.4 | 2    | LSB  | The maximum allowed amplitude before the integration energy will be decreased.                                                                                                                                 |
| Nominal Integration Depth                          | UQ10.6 | 2    | #    | The nominal analog integration depth. This determines the start value or the static value depending on the Enabled setting.                                                                                    |
| Min. Integration Depth                             | UQ10.6 | 2    | #    | The minimum analog integration depth, i.e. the minimum pattern count per sample.                                                                                                                               |
| Max. Integration Depth                             | UQ10.6 | 2    | #    | The maximum analog integration depth, i.e. the maximum pattern count per sample.                                                                                                                               |
| Nominal Optical Power                              | UQ12.4 | 2    | mA   | The nominal optical output power.This determines the start value or the static value depending on the Enabled setting.                                                                                         |
| Min. Optical Power                                 | UQ12.4 | 2    | mA   | The minimum optical output power, i.e. the minimum OMA value.                                                                                                                                                  |
| Nominal Pixel Gain                                 | UINT8  | 1    | n/a  | The nominal pixel gain value for the default DCA gain stage. This determines the start value or the static value depending on the Enabled setting.                                                             |
| Low Pixel Gain                                     | UINT8  | 1    | n/a  | The low pixel gain value for the DCA low gain stage.                                                                                                                                                           |
| High Pixel Gain                                    | UINT8  | 1    | n/a  | The high pixel gain value for the DCA high gain stage.                                                                                                                                                         |
| Power Saving Ratio                                 | UQ0.8  | 1    | n/a  | Determines the percentage of the full available frame time that is not exploited for digital integration. Thus the device is idle within the specified portion of the frame time and does consume less energy. |

## Pixel Binning {#cmd_cfg_pba}

| Caption / Name                  | Type   | Size | Unit | Comment                                                                                                                                                                                                               |
| ------------------------------- | ------ | ---- | ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Command                         | UINT8  | 1    |      | 0x54                                                                                                                                                                                                                  |
| Measurement Mode Selector       | ENUM8  | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details.                                                                                                                                                        |
| Enabled Flags                   | HEX8   | 1    | n/a  | Enables/disables the pixel binning (features). See #argus_pba_flags_t for details.                                                                                                                                    |
| Averaging Mode                  | HEX8   | 1    | n/a  | Selects the averaging mode for the PBA. See #argus_pba_averaging_mode_t for details.                                                                                                                                  |
| Pre-filter Mask                 | HEX32  | 4    | n/a  | The pixel pre-filter mask determines the pixels that are dismissed by the PBA. The mask is channel based, i.e. the n-th bit represents the n-th ADC channel. See the [pixel mapping](@ref argusmap) for the corresponding x-y-indices. |
| Absolute Amplitude Threshold    | UQ12.4 | 2    | LSB  | Absolute Amplitude Threshold in LSB. Lower values are dismissed.                                                                                                                                                      |
| Relative Amplitude Threshold    | UQ0.8  | 1    | %    | Relative Amplitude Threshold in % of max value. Lower values are dismissed.                                                                                                                                           |
| Absolute Minimum Distance Scope | UQ1.15 | 2    | m    | Absolute Minimum Distance Scope Filter Threshold Value in LSB. Distances with larger values (compared to current minimum distance) are dismissed.                                                                     |
| Relative Minimum Distance Scope | UQ0.8  | 1    | %    | Relative Minimum Distance Scope Filter Threshold Value in % of the current minimum distance. Distances with larger values (compared to current minimum distance) are dismissed                                        |

## SPI Configuration {#cmd_cfg_spi}

| Caption / Name | Type   | Size | Unit | Comment                                      |
| -------------- | ------ | ---- | ---- | -------------------------------------------- |
| Command        | UINT8  | 1    |      | 0x58                                         |
| SPI Slave      | ENUM8  | 1    | 0    | SPI Slave Enumeration: 0: intern, 1: extern, |
| SPI Baud Rate  | UINT32 | 4    | 0    | Baud Rate in BaudPerSeconds.                 |

# Calibration Commands {#hw_api_cmd_cal}

## Global Range Offset {#cmd_cal_range_offset}

| Caption / Name            | Type  | Size | Unit | Comment                                                        |
| ------------------------- | ----- | ---- | ---- | -------------------------------------------------------------- |
| Command                   | UINT8 | 1    |      | 0x61                                                           |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details. |
| Range Offset              | Q9.22 | 4    | m    | The global range offset in meters.                             |

## Pixel Range Offsets {#cmd_cal_offsets}

| Caption / Name            | Type  | Size | Unit | Comment                                                        |
| ------------------------- | ----- | ---- | ---- | -------------------------------------------------------------- |
| Command                   | UINT8 | 1    |      | 0x67                                                           |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details. |
| Offset Values (x, y)      | Q0.15 | 64   | m    | The pixel offsets table. Ordering: X Index -> Y Index.         |

Note on indices:

 - x: The pixel x-index (0-7)
 - y: The pixel y-index (0-3)
 .

## Pixel Range Offsets - Reset Table {#cmd_cal_offsets_rst}

| Caption / Name            | Type  | Size | Unit | Comment                                                        |
| ------------------------- | ----- | ---- | ---- | -------------------------------------------------------------- |
| Command                   | UINT8 | 1    |      | 0x68                                                           |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details. |

## Range Offsets Calibration Sequence - Sample Count {#cmd_cal_offsets_smpl_ct}

| Caption / Name | Type   | Size | Unit | Comment                                                  |
| -------------- | ------ | ---- | ---- | -------------------------------------------------------- |
| Command        | UINT8  | 1    |      | 0x69                                                     |
| Sample Count   | UINT16 | 2    |      | The number of sample per offset calibration measurement. |

## Crosstalk Compensation - Vector Table {#cmd_cal_xtalk_vec}

| Caption / Name                | Type  | Size | Unit | Comment                                                                                             |
| ----------------------------- | ----- | ---- | ---- | --------------------------------------------------------------------------------------------------- |
| Command                       | UINT8 | 1    |      | 0x62                                                                                                |
| Measurement Mode Selector     | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details.                                      |
| Crosstalk Vector (f, x, y, a) | Q9.22 | 256  | LSB  | The crosstalk vector table. Ordering: Frequency Index (A/B) -> X Index -> Y Index -> Sin/Cos Index. |

Note on indices:

 - f: The frequency index for A- and B-frames respectively (A = 0; B = 1)
 - x: The pixel x-index (0-7)
 - y: The pixel y-index (0-3)
 - a: The sin/cos index (sin = 0; cos = 1)
 .

## Crosstalk Compensation - Reset Vector Table {#cmd_cal_xtalk_rst}

| Caption / Name            | Type  | Size | Unit | Comment                                                        |
| ------------------------- | ----- | ---- | ---- | -------------------------------------------------------------- |
| Command                   | UINT8 | 1    |      | 0x63                                                           |
| Measurement Mode Selector | ENUM8 | 1    | n/a  | The addressed measurement mode. See #argus_mode_t for details. |

## Crosstalk Calibration Sequence - Sample Count {#cmd_cal_xtalk_smpl_ct}

| Caption / Name | Type   | Size | Unit | Comment                                                     |
| -------------- | ------ | ---- | ---- | ----------------------------------------------------------- |
| Command        | UINT8  | 1    |      | 0x64                                                        |
| Sample Count   | UINT16 | 2    |      | The number of sample per crosstalk calibration measurement. |

## Crosstalk Calibration Sequence - Maximum Amplitude Threshold {#cmd_cal_xtalk_max_ampl}

| Caption / Name              | Type   | Size | Unit | Comment                                                             |
| --------------------------- | ------ | ---- | ---- | ------------------------------------------------------------------- |
| Command                     | UINT8  | 1    |      | 0x65                                                                |
| Maximum Amplitude Threshold | UQ12.4 | 2    | LSB  | The maximum amplitude threshold for crosstalk calibration sequence. |

## Pixel-2-Pixel Crosstalk Compensation Parameters {#cmd_cal_xtalk_p2p}

| Caption / Name            | Type   | Size | Unit | Comment                                                                                                                         |
| ------------------------- | ------ | ---- | ---- | ------------------------------------------------------------------------------------------------------------------------------- |
| Command                   | UINT8  | 1    |      | 0x66                                                                                                                            |
| Measurement Mode Selector | ENUM8  | 1    |      | The addressed measurement mode.                                                                                                 |
| Enabled                   | BOOL8  | 1    |      | Enables/disables the pixel-2-pixel crosstalk compensation algorithm.                                                            |
| Kc Factor                 | UQ0.8  | 1    |      | The Kc factor that determines the amount of the total signal on all pixels influences the individual signal of each pixel.      |
| Relative Threshold        | UQ0.8  | 1    |      | The relative threshold determines when the compensation is active for each individual pixel.                                    |
| Absolute Threshold        | UQ12.4 | 2    |      | The absolute threshold determines the minimum total crosstalk amplitude that is required for the compensation to become active. |
