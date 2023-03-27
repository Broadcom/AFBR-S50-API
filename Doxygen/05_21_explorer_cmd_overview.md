# Command Overview {#explorer_app_cmd_overview}

The following section contains the current command details as implemented in the
@ref explorer_app. The implementation can be found in the
`Sources\ExplorerApp\api` folder of the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API).

See @ref explorer_app_cmd_details for detailed description of all available
commands.

## Generic Commands {#explorer_app_cmds_generic}

| Caption                                                | Byte | Type      | Comment                                                                                                                                                                           |
| ------------------------------------------------------ | ---- | --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Invalid Command](@ref cmd_invalid)                    | 0x00 | -         | Reserved! Invalid Command;                                                                                                                                                        |
| [Acknowledge (ACK)](@ref cmd_ack)                      | 0x0A | auto/push | Slave does acknowledge the successful reception of the last command.                                                                                                              |
| [Not Acknowledge (NAK)](@ref cmd_nak)                  | 0x0B | auto/push | Slave does not-acknowledge the successful reception of the last command.                                                                                                          |
| [Ping Command](@ref cmd_ping)                          | 0x01 | get       | A ping message the is sent from the master and reflected by the slave. Used to establish or test the UART connection.                                                             |
| [Log Message](@ref cmd_log)                            | 0x06 | auto/push | An event/debug log message sent from the slave to inform the user.                                                                                                                |
| [Test Message](@ref cmd_test)                          | 0x04 | set / get | Sending a test message to the slave that will be echoed in order to test the interface. The slave will echo the exact message including the CRC values from the original message. |
| [MCU/Software Reset](@ref cmd_reset)                   | 0x08 | cmd       | Invokes the software reset command.                                                                                                                                               |
| [Software Version](@ref cmd_sw)                        | 0x0C | get       | Gets the current software version number.                                                                                                                                         |
| [Module Type](@ref cmd_module)                         | 0x0E | get       | Gets the module information, incl. module type with version number, chip version and laser type.                                                                                  |
| [Module UID](@ref cmd_uid)                             | 0x0F | get       | Gets the chip/module unique identification number.                                                                                                                                |
| [Software Information / Identification](@ref cmd_info) | 0x05 | get       | Gets the information about current software and device (e.g. version, device id, device family, ...)                                                                              |

## Device Control Commands {#explorer_app_cmds_ctrl}

| Caption                                           | Byte | Type | Comment                                                                                                                                         |
| ------------------------------------------------- | ---- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| [Measurement: Single Shot](@ref cmd_ctrl_trigger) | 0x10 | cmd  | Executes a single shot measurement.                                                                                                             |
| [Measurement: Start Auto](@ref cmd_ctrl_start)    | 0x11 | cmd  | Starts the automatic, time-scheduled measurements with given frame rate.                                                                        |
| [Measurement: Stop](@ref cmd_ctrl_stop)           | 0x12 | cmd  | Stops the time-scheduled measurements (after the current frame finishes).                                                                       |
| [Measurement: Abort](@ref cmd_ctrl_abort)         | 0x13 | cmd  | Aborts the current measurements immediately.                                                                                                    |
| [Calibration: Run](@ref cmd_ctrl_cal)             | 0x18 | cmd  | Executes a calibration sequence.                                                                                                                |
| [Re-Initialize Device](@ref cmd_ctrl_reinit)      | 0x19 | cmd  | Invokes the device (re-)initialization command. Resets and reinitializes the API + ASIC with given config. (e.g. after unintended power cycle). |

## Measurement Data Commands {#explorer_app_cmds_data}

The modes of measurement data streaming. The actual data might depend on device
configuration and calibration.

| Caption                                                          | Byte | Type            | Comment                                                                                |
| ---------------------------------------------------------------- | ---- | --------------- | -------------------------------------------------------------------------------------- |
| [Measurement Data Set (1D + 3D) - Debug](@ref cmd_data_full_dbg) | 0x31 | get / auto/push | Gets a measurement data set containing all the available data.                         |
| [Measurement Data Set (1D + 3D)](@ref cmd_data_full)             | 0x32 | get / auto/push | Gets a measurement data set containing the essential data.                             |
| [3D Measurement Data Set - Debug](@ref cmd_data_3d_dbg)          | 0x33 | get / auto/push | Gets a 3D measurement data set containing all the available data per pixel.            |
| [3D Measurement Data Set](@ref cmd_data_3d)                      | 0x34 | get / auto/push | Gets a 3D measurement data set containing the essential data per pixel.                |
| [1D Measurement Data Set - Debug](@ref cmd_data_1d_dbg)          | 0x35 | get / auto/push | Gets a 1D measurement data set containing all the available distance measurement data. |
| [1D Measurement Data Set](@ref cmd_data_1d)                      | 0x36 | get / auto/push | Gets a 1D measurement data set containing the essential distance measurement data.     |

## Configuration Commands {#explorer_app_cmds_cfg}

| Caption                                            | Byte | Type      | Comment                                                                               |
| -------------------------------------------------- | ---- | --------- | ------------------------------------------------------------------------------------- |
| [Data Output Mode](@ref cmd_cfg_output_mode)       | 0x41 | set / get | The measurement data output mode. (Hardware API only)                                 |
| [Measurement Mode](@ref cmd_cfg_mode)              | 0x42 | set / get | Gets or sets the measurement mode.                                                    |
| [Frame Time (Frame Rate)](@ref cmd_cfg_frame_time) | 0x43 | set / get | Gets or sets the measurement frame time (i.e. inverse frame rate).                    |
| [Dual Frequency Mode](@ref cmd_cfg_dfm)            | 0x44 | set / get | Gets or sets the dual frequency mode.                                                 |
| [Smart Power Save Mode](@ref cmd_cfg_sps)          | 0x45 | set / get | Gets or sets the smart power saving feature enabled flag.                             |
| [Shot Noise Monitor Mode](@ref cmd_cfg_snm)        | 0x46 | set / get | Gets or sets the shot noise monitor mode.                                             |
| [Crosstalk Monitor Mode](@ref cmd_cfg_xtm)         | 0x47 | set / get | Gets or sets the crosstalk monitor mode.                                              |
| [Dynamic Configuration Adaption](@ref cmd_cfg_dca) | 0x52 | set / get | Gets or sets the full dynamic configuration adaption (DCA) feature configuration set. |
| [Pixel Binning Algorithm](@ref cmd_cfg_pba)        | 0x54 | set / get | Gets or sets the pixel binning algorithm (PBA) feature configuration.                 |
| [SPI Configuration](@ref cmd_cfg_spi)              | 0x58 | set / get | Gets or sets the SPI configuration (e.g. baud rate).                                  |
| [UART Configuration](@ref cmd_cfg_uart)            | 0x59 | set / get | Gets or sets the UART configuration (e.g. baud rate). Only for UART version.          |

## Calibration Commands {#explorer_app_cmds_cal}

| Caption                                                                     | Byte | Type      | Comment                                                                              |
| --------------------------------------------------------------------------- | ---- | --------- | ------------------------------------------------------------------------------------ |
| [Global Range Offset](@ref cmd_cal_range_offset)                            | 0x61 | set / get | Gets or sets the global range offset calibration parameter.                          |
| [Pixel Range Offsets](@ref cmd_cal_offsets)                                 | 0x67 | set / get | Gets or sets the pixel-to-pixel range offset table.                                  |
| [Pixel Range Offsets - Reset](@ref cmd_cal_offsets_rst)                     | 0x68 | cmd       | Resets the pixel-to-pixel range offset table to factory calibrated default values.   |
| [Range Offsets Cal. Sequence - Sample Time](@ref cmd_cal_offsets_smpl_time) | 0x69 | set / get | Gets or sets the range offset calibration sequence sample time.                      |
| [Crosstalk Compensation - Vector Table](@ref cmd_cal_xtalk_vec)             | 0x62 | set / get | Gets or sets the crosstalk compensation vector table.                                |
| [Crosstalk Compensation - Reset](@ref cmd_cal_xtalk_rst)                    | 0x63 | cmd       | Resets the crosstalk compensation vector table to factory calibrated default values. |
| [Crosstalk Cal. Sequence - Sample Time](@ref cmd_cal_xtalk_smpl_time)       | 0x64 | set / get | Gets or sets the crosstalk calibration sequence sample time.                         |
| [Crosstalk Cal. Sequence - Max. Amplitude](@ref cmd_cal_xtalk_max_ampl)     | 0x65 | set / get | Gets or sets the crosstalk calibration sequence maximum amplitude threshold.         |
| [Pixel-2-Pixel Crosstalk Compensation](@ref cmd_cal_xtalk_p2p)              | 0x66 | set / get | Gets or sets the pixel-2-pixel crosstalk calibration parameter values.               |
