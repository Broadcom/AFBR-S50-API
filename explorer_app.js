var explorer_app =
[
    [ "Build And Run the Explorer App", "explorer_app.html#explorer_app_build", null ],
    [ "Serial Communication Interface", "explorer_app.html#explorer_app_sci", [
      [ "Introduction", "explorer_app.html#explorer_app_sci_intro", null ],
      [ "Architecture", "explorer_app.html#explorer_app_sci_architecture", null ],
      [ "Hardware Layer", "explorer_app.html#explorer_app_hw_layer", [
        [ "UART", "explorer_app.html#explorer_app_uart", null ],
        [ "SPI", "explorer_app.html#explorer_app_spi", null ],
        [ "I2C", "explorer_app.html#explorer_app_i2c", null ]
      ] ],
      [ "Command Protocols", "explorer_app.html#explorer_app_cmd_protocols", [
        [ "Master-to-Slave Transfer", "explorer_app.html#explorer_app_cmd_m2s", null ],
        [ "Slave-to-Master Transfer", "explorer_app.html#explorer_app_cmd_s2m", null ]
      ] ],
      [ "Command Byte Format", "explorer_app.html#explorer_app_cmd_format", null ],
      [ "Addressing Mechanism", "explorer_app.html#explorer_app_cmd_addressing", null ],
      [ "Command Types", "explorer_app.html#explorer_app_cmd_types", null ],
      [ "Data Frame Format", "explorer_app.html#explorer_app_frame_format", [
        [ "Byte Stuffing Algorithm", "explorer_app.html#explorer_app_byte_stuffing", [
          [ "Byte Stuffing Algorithm for Sending Data", "explorer_app.html#autotoc_md38", null ],
          [ "Byte Stuffing Algorithm for Receiving Data", "explorer_app.html#autotoc_md39", null ]
        ] ],
        [ "Error checking: 8-bit CRC", "explorer_app.html#explorer_app_crc", null ]
      ] ]
    ] ],
    [ "Command Definitions", "explorer_app.html#autotoc_md40", [
      [ "Overview", "explorer_app.html#autotoc_md41", null ],
      [ "Details", "explorer_app.html#autotoc_md42", null ]
    ] ],
    [ "Python Example on Using the SCI Interface", "explorer_app.html#explorer_app_python_example", null ],
    [ "Command Overview", "explorer_app_cmd_overview.html", [
      [ "Generic Commands", "explorer_app_cmd_overview.html#explorer_app_cmds_generic", null ],
      [ "Device Control Commands", "explorer_app_cmd_overview.html#explorer_app_cmds_ctrl", null ],
      [ "Measurement Data Commands", "explorer_app_cmd_overview.html#explorer_app_cmds_data", null ],
      [ "Configuration Commands", "explorer_app_cmd_overview.html#explorer_app_cmds_cfg", null ],
      [ "Calibration Commands", "explorer_app_cmd_overview.html#explorer_app_cmds_cal", null ]
    ] ],
    [ "Command Details", "explorer_app_cmd_details.html", [
      [ "Generic Commands", "explorer_app_cmd_details.html#explorer_app_cmd_generic", [
        [ "Invalid Command", "explorer_app_cmd_details.html#cmd_invalid", null ],
        [ "Acknowledge", "explorer_app_cmd_details.html#cmd_ack", null ],
        [ "Not Acknowledge", "explorer_app_cmd_details.html#cmd_nak", null ],
        [ "Ping Command", "explorer_app_cmd_details.html#cmd_ping", null ],
        [ "Log Message", "explorer_app_cmd_details.html#cmd_log", null ],
        [ "Test Message", "explorer_app_cmd_details.html#cmd_test", null ],
        [ "MCU/Software Reset", "explorer_app_cmd_details.html#cmd_reset", null ],
        [ "Software Version", "explorer_app_cmd_details.html#cmd_sw", null ],
        [ "Module Type / Version", "explorer_app_cmd_details.html#cmd_module", null ],
        [ "Module UID", "explorer_app_cmd_details.html#cmd_uid", null ],
        [ "Software Information / Identification", "explorer_app_cmd_details.html#cmd_info", [
          [ "Single Device Version (Basic Mode ore Address == 0)", "explorer_app_cmd_details.html#autotoc_md43", null ],
          [ "Multi Device Version (Address != 0)", "explorer_app_cmd_details.html#autotoc_md44", null ]
        ] ]
      ] ],
      [ "Device Control Commands", "explorer_app_cmd_details.html#explorer_app_cmd_ctrl", [
        [ "Measurement: Trigger Single Shot", "explorer_app_cmd_details.html#cmd_ctrl_trigger", null ],
        [ "Measurement: Start Auto", "explorer_app_cmd_details.html#cmd_ctrl_start", null ],
        [ "Measurement: Stop", "explorer_app_cmd_details.html#cmd_ctrl_stop", null ],
        [ "Measurement: Abort", "explorer_app_cmd_details.html#cmd_ctrl_abort", null ],
        [ "Run Calibration", "explorer_app_cmd_details.html#cmd_ctrl_cal", [
          [ "Calibration Sequence Enumerator", "explorer_app_cmd_details.html#autotoc_md45", null ]
        ] ],
        [ "Device Reinitialize", "explorer_app_cmd_details.html#cmd_ctrl_reinit", null ]
      ] ],
      [ "Measurement Data Commands", "explorer_app_cmd_details.html#explorer_app_cmd_data", [
        [ "Measurement Data Set (1D + 3D) - Debug", "explorer_app_cmd_details.html#cmd_data_full_dbg", null ],
        [ "Measurement Data Set (1D + 3D)", "explorer_app_cmd_details.html#cmd_data_full", null ],
        [ "3D Measurement Data Set - Debug", "explorer_app_cmd_details.html#cmd_data_3d_dbg", null ],
        [ "3D Measurement Data Set", "explorer_app_cmd_details.html#cmd_data_3d", null ],
        [ "1D Measurement Data Set - Debug", "explorer_app_cmd_details.html#cmd_data_1d_dbg", null ],
        [ "1D Measurement Data Set", "explorer_app_cmd_details.html#cmd_data_1d", null ]
      ] ],
      [ "Configuration Commands", "explorer_app_cmd_details.html#explorer_app_cmd_cfg", [
        [ "Data Output Mode", "explorer_app_cmd_details.html#cmd_cfg_output_mode", [
          [ "Measurement Data Output Mode Enumerator", "explorer_app_cmd_details.html#autotoc_md46", null ]
        ] ],
        [ "Measurement Mode", "explorer_app_cmd_details.html#cmd_cfg_mode", null ],
        [ "Frame Time", "explorer_app_cmd_details.html#cmd_cfg_frame_time", null ],
        [ "Dual Frequency Mode", "explorer_app_cmd_details.html#cmd_cfg_dfm", [
          [ "Dual Frequency Mode Enumerator", "explorer_app_cmd_details.html#autotoc_md47", null ]
        ] ],
        [ "Smart Power Save Mode", "explorer_app_cmd_details.html#cmd_cfg_sps", null ],
        [ "Shot Noise Monitor Mode", "explorer_app_cmd_details.html#cmd_cfg_snm", [
          [ "Shot Noise Monitor Mode Enumerator", "explorer_app_cmd_details.html#autotoc_md48", null ]
        ] ],
        [ "Crosstalk Monitor Mode", "explorer_app_cmd_details.html#cmd_cfg_xtm", null ],
        [ "Dynamic Configuration Adaption", "explorer_app_cmd_details.html#cmd_cfg_dca", null ],
        [ "Pixel Binning", "explorer_app_cmd_details.html#cmd_cfg_pba", null ],
        [ "SPI Configuration", "explorer_app_cmd_details.html#cmd_cfg_spi", null ],
        [ "UART Configuration", "explorer_app_cmd_details.html#cmd_cfg_uart", null ]
      ] ],
      [ "Calibration Commands", "explorer_app_cmd_details.html#explorer_app_cmd_cal", [
        [ "Global Range Offset", "explorer_app_cmd_details.html#cmd_cal_range_offset", null ],
        [ "Pixel Range Offsets", "explorer_app_cmd_details.html#cmd_cal_offsets", null ],
        [ "Pixel Range Offsets - Reset Offset Table", "explorer_app_cmd_details.html#cmd_cal_offsets_rst", null ],
        [ "Range Offsets Calibration Sequence - Sample Time", "explorer_app_cmd_details.html#cmd_cal_offsets_smpl_time", null ],
        [ "Crosstalk Compensation - Vector Table", "explorer_app_cmd_details.html#cmd_cal_xtalk_vec", null ],
        [ "Crosstalk Compensation - Reset Vector Table", "explorer_app_cmd_details.html#cmd_cal_xtalk_rst", null ],
        [ "Crosstalk Calibration Sequence - Sample Time", "explorer_app_cmd_details.html#cmd_cal_xtalk_smpl_time", null ],
        [ "Crosstalk Calibration Sequence - Maximum Amplitude Threshold", "explorer_app_cmd_details.html#cmd_cal_xtalk_max_ampl", null ],
        [ "Pixel-2-Pixel Crosstalk Compensation Parameters", "explorer_app_cmd_details.html#cmd_cal_xtalk_p2p", null ]
      ] ]
    ] ]
];