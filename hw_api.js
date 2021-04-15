var hw_api =
[
    [ "Build And Run the ExplorerApp using MCUXpresso", "hw_api.html#hw_mcuxpresso", null ],
    [ "Serial Communication Interface", "hw_api.html#hw_api_sci", [
      [ "Introduction", "hw_api.html#hw_api_sci_intro", null ],
      [ "Architecture", "hw_api.html#hw_api_sci_architecture", null ],
      [ "Hardware Layer", "hw_api.html#hw_api_hw_layer", [
        [ "UART", "hw_api.html#hw_api_uart", null ],
        [ "SPI", "hw_api.html#hw_api_spi", null ],
        [ "I2C", "hw_api.html#hw_api_i2c", null ]
      ] ],
      [ "Command Protocols", "hw_api.html#hw_api_cmd_protocols", [
        [ "Master to slave transfer", "hw_api.html#hw_api_cmd_m2s", null ],
        [ "Slave to master transfer", "hw_api.html#hw_api_cmd_s2m", null ]
      ] ],
      [ "Command Byte Format", "hw_api.html#hw_api_cmd_format", null ],
      [ "Command Types", "hw_api.html#hw_api_cmd_types", null ],
      [ "Data Frame Format", "hw_api.html#hw_api_frame_format", [
        [ "Byte Stuffing Algorithm", "hw_api.html#hw_api_byte_stuffing", [
          [ "Byte Stuffing Algorithm for Sending Data", "hw_api.html#autotoc_md17", null ],
          [ "Byte Stuffing Algorithm for Receiving Data", "hw_api.html#autotoc_md18", null ]
        ] ],
        [ "Error checking: 8-bit CRC", "hw_api.html#hw_api_crc", null ]
      ] ]
    ] ],
    [ "Command Definitions", "hw_api.html#autotoc_md19", [
      [ "Overview", "hw_api.html#autotoc_md20", null ],
      [ "Details", "hw_api.html#autotoc_md21", null ]
    ] ],
    [ "Python Example on using the SCI interface", "hw_api.html#hw_example", null ],
    [ "Command Overview", "hw_api_cmd_overview.html", [
      [ "Generic Commands", "hw_api_cmd_overview.html#hw_api_cmds_generic", null ],
      [ "Device Control Commands", "hw_api_cmd_overview.html#hw_api_cmds_ctrl", null ],
      [ "Measurement Data Commands", "hw_api_cmd_overview.html#hw_api_cmds_data", null ],
      [ "Configuration Commands", "hw_api_cmd_overview.html#hw_api_cmds_cfg", null ],
      [ "Calibration Commands", "hw_api_cmd_overview.html#hw_api_cmds_cal", null ]
    ] ],
    [ "Command Details", "hw_api_cmd_details.html", [
      [ "Generic Commands", "hw_api_cmd_details.html#hw_api_cmd_generic", [
        [ "Invalid Command", "hw_api_cmd_details.html#cmd_invalid", null ],
        [ "Acknowledge", "hw_api_cmd_details.html#cmd_ack", null ],
        [ "Not Acknowledge", "hw_api_cmd_details.html#cmd_nak", null ],
        [ "Log Message", "hw_api_cmd_details.html#cmd_log", null ],
        [ "Test Message", "hw_api_cmd_details.html#cmd_test", null ],
        [ "MCU/Software Reset", "hw_api_cmd_details.html#cmd_reset", null ],
        [ "Software Version", "hw_api_cmd_details.html#cmd_sw", null ],
        [ "Module Type", "hw_api_cmd_details.html#cmd_module", null ],
        [ "Module UID", "hw_api_cmd_details.html#cmd_uid", null ],
        [ "Software Information / Identification", "hw_api_cmd_details.html#cmd_info", null ]
      ] ],
      [ "Device Control Commands", "hw_api_cmd_details.html#hw_api_cmd_ctrl", [
        [ "Measurement: Trigger Single Shot", "hw_api_cmd_details.html#cmd_ctrl_trigger", null ],
        [ "Measurement: Start Auto", "hw_api_cmd_details.html#cmd_ctrl_start", null ],
        [ "Measurement: Stop", "hw_api_cmd_details.html#cmd_ctrl_stop", null ],
        [ "Run Calibration", "hw_api_cmd_details.html#cmd_ctrl_cal", [
          [ "Calibration Sequence Enumerator", "hw_api_cmd_details.html#autotoc_md22", null ]
        ] ],
        [ "Device Reinitialize", "hw_api_cmd_details.html#cmd_ctrl_reinit", null ]
      ] ],
      [ "Measurement Data Commands", "hw_api_cmd_details.html#hw_api_cmd_data", [
        [ "Raw Measurement Data Set", "hw_api_cmd_details.html#cmd_data_raw", null ],
        [ "Measurement Data Set (1D + 3D) - Debug", "hw_api_cmd_details.html#cmd_data_full_dbg", null ],
        [ "Measurement Data Set (1D + 3D)", "hw_api_cmd_details.html#cmd_data_full", null ],
        [ "3D Measurement Data Set - Debug", "hw_api_cmd_details.html#cmd_data_3d_dbg", null ],
        [ "3D Measurement Data Set", "hw_api_cmd_details.html#cmd_data_3d", null ],
        [ "1D Measurement Data Set - Debug", "hw_api_cmd_details.html#cmd_data_1d_dbg", null ],
        [ "1D Measurement Data Set", "hw_api_cmd_details.html#cmd_data_1d", null ]
      ] ],
      [ "Configuration Commands", "hw_api_cmd_details.html#hw_api_cmd_cfg", [
        [ "Data Output Mode", "hw_api_cmd_details.html#cmd_cfg_output_mode", [
          [ "Measurement Data Output Mode Enumerator", "hw_api_cmd_details.html#autotoc_md23", null ]
        ] ],
        [ "Measurement Mode", "hw_api_cmd_details.html#cmd_cfg_mode", null ],
        [ "Frame Time", "hw_api_cmd_details.html#cmd_cfg_frame_time", null ],
        [ "Dual Frequency Mode", "hw_api_cmd_details.html#cmd_cfg_dfm", [
          [ "Dual Frequency Mode Enumerator", "hw_api_cmd_details.html#autotoc_md24", null ]
        ] ],
        [ "Smart Power Save Mode", "hw_api_cmd_details.html#cmd_cfg_sps", null ],
        [ "Shot Noise Monitor Mode", "hw_api_cmd_details.html#cmd_cfg_snm", [
          [ "Shot Noise Monitor Mode Enumerator", "hw_api_cmd_details.html#autotoc_md25", null ]
        ] ],
        [ "Dynamic Configuration Adaption", "hw_api_cmd_details.html#cmd_cfg_dca", null ],
        [ "Pixel Binning", "hw_api_cmd_details.html#cmd_cfg_pba", null ],
        [ "SPI Configuration", "hw_api_cmd_details.html#cmd_cfg_spi", null ]
      ] ],
      [ "Calibration Commands", "hw_api_cmd_details.html#hw_api_cmd_cal", [
        [ "Global Range Offset", "hw_api_cmd_details.html#cmd_cal_range_offset", null ],
        [ "Pixel Range Offsets", "hw_api_cmd_details.html#cmd_cal_offsets", null ],
        [ "Pixel Range Offsets - Reset Table", "hw_api_cmd_details.html#cmd_cal_offsets_rst", null ],
        [ "Range Offsets Calibration Sequence - Sample Count", "hw_api_cmd_details.html#cmd_cal_offsets_smpl_ct", null ],
        [ "Crosstalk Compensation - Vector Table", "hw_api_cmd_details.html#cmd_cal_xtalk_vec", null ],
        [ "Crosstalk Compensation - Reset Vector Table", "hw_api_cmd_details.html#cmd_cal_xtalk_rst", null ],
        [ "Crosstalk Calibration Sequence - Sample Count", "hw_api_cmd_details.html#cmd_cal_xtalk_smpl_ct", null ],
        [ "Crosstalk Calibration Sequence - Maximum Amplitude Threshold", "hw_api_cmd_details.html#cmd_cal_xtalk_max_ampl", null ],
        [ "Pixel-2-Pixel Crosstalk Compensation Parameters", "hw_api_cmd_details.html#cmd_cal_xtalk_p2p", null ]
      ] ]
    ] ]
];