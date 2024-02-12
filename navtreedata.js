/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "AFBR-S50 API Reference Manual", "index.html", [
    [ "Introduction", "index.html", "index" ],
    [ "Architectural Overview", "sw_api.html", [
      [ "Overview", "sw_api.html#sw_api_overview", null ],
      [ "Operation Principle", "sw_api.html#sw_api_principle", null ],
      [ "API Modules", "sw_api.html#sw_api_modules", null ]
    ] ],
    [ "Getting Started", "getting_started.html", [
      [ "Build And Run Projects", "getting_started.html#gs_build", null ],
      [ "Using the AFBR-S50 API", "getting_started.html#gs_api", [
        [ "Initialization", "getting_started.html#autotoc_md3", null ],
        [ "Running Measurements", "getting_started.html#autotoc_md4", [
          [ "Single Measurements", "getting_started.html#autotoc_md5", null ],
          [ "Periodic Measurements", "getting_started.html#autotoc_md6", null ]
        ] ]
      ] ],
      [ "MCUXpresso IDE", "mcuxpresso.html", [
        [ "Install MCUXpresso IDE", "mcuxpresso.html#autotoc_md7", null ],
        [ "Import NXP MKL46z SDK", "mcuxpresso.html#autotoc_md8", null ],
        [ "Import a Project", "mcuxpresso.html#autotoc_md9", null ],
        [ "Build the Project", "mcuxpresso.html#autotoc_md10", null ],
        [ "Download and install the OpenSDA drivers", "mcuxpresso.html#autotoc_md11", null ],
        [ "Debug the Project with the OpenSDA Debugger", "mcuxpresso.html#autotoc_md12", null ],
        [ "Display the Measurement Values", "mcuxpresso.html#autotoc_md13", null ]
      ] ],
      [ "e² Studio IDE", "e2studio.html", [
        [ "Install Renesas e² Studio IDE", "e2studio.html#autotoc_md14", null ],
        [ "Import a Project", "e2studio.html#autotoc_md15", null ],
        [ "Build the Project", "e2studio.html#autotoc_md16", null ],
        [ "Debug the Project with a SWD debugger", "e2studio.html#autotoc_md17", null ],
        [ "Connect the UART interface to PC", "e2studio.html#autotoc_md18", null ],
        [ "Display the Measurement Values", "e2studio.html#autotoc_md19", null ]
      ] ],
      [ "STM32CubeIDE", "stm32cubeide.html", [
        [ "Install STM32CubeIDE", "stm32cubeide.html#autotoc_md20", null ],
        [ "Import a Project", "stm32cubeide.html#autotoc_md21", null ],
        [ "Build the Project", "stm32cubeide.html#autotoc_md22", null ],
        [ "Debug the Project with the STLink debugger", "stm32cubeide.html#autotoc_md23", null ],
        [ "Connect the UART interface to PC", "stm32cubeide.html#autotoc_md24", null ],
        [ "Display the Measurement Values", "stm32cubeide.html#autotoc_md25", null ]
      ] ]
    ] ],
    [ "MCU Porting Guide", "porting_guide.html", [
      [ "Introduction", "porting_guide.html#pg_introduction", null ],
      [ "Toolchain Compatibility", "porting_guide.html#pg_toolchain", null ],
      [ "Architecture Compatibility", "porting_guide.html#pg_architecture", null ],
      [ "Hardware Compatibility", "porting_guide.html#pg_hardware", null ],
      [ "Hardware Layers", "porting_guide.html#pg_hw_layers", [
        [ "S2PI (= SPI + GPIO) Layer", "porting_guide.html#pg_s2pi", [
          [ "S2PI Overview", "porting_guide.html#autotoc_md26", null ],
          [ "S2PI Initialization", "porting_guide.html#autotoc_md27", [
            [ "S2PI Pin configuration", "porting_guide.html#autotoc_md28", null ],
            [ "SPI Mode", "porting_guide.html#autotoc_md29", null ],
            [ "SPI Speed", "porting_guide.html#autotoc_md30", null ],
            [ "DMA Channels", "porting_guide.html#autotoc_md31", null ],
            [ "DMA Interrupts and Callback Function", "porting_guide.html#autotoc_md32", null ],
            [ "DMA Interrupt Priority", "porting_guide.html#autotoc_md33", null ]
          ] ]
        ] ],
        [ "Timer Layer", "porting_guide.html#pg_timer", [
          [ "Lifetime Counter (LTC)", "porting_guide.html#autotoc_md34", null ],
          [ "Periodic Interrupt Timer (PIT)", "porting_guide.html#autotoc_md35", null ]
        ] ],
        [ "Interrupt Layer", "porting_guide.html#pg_irq", [
          [ "Interrupt Priority", "porting_guide.html#autotoc_md36", null ],
          [ "Concurrency and Interrupt Locking", "porting_guide.html#autotoc_md37", null ]
        ] ],
        [ "NVM Layer", "porting_guide.html#pg_nvm", null ],
        [ "Log Layer", "porting_guide.html#pg_log", null ]
      ] ],
      [ "Verifying the ported code using the HAL Self Test", "porting_guide.html#hal_self_test", null ],
      [ "Step-by-Step Porting Guide", "porting_guide.html#pg_guide", [
        [ "Create a new project in your environment", "porting_guide.html#pg_new_project", null ],
        [ "Implement hardware interfaces", "porting_guide.html#pg_hw_interface", null ],
        [ "Link Library File", "porting_guide.html#pg_linker", null ],
        [ "Utilize the API", "porting_guide.html#pg_api", null ]
      ] ]
    ] ],
    [ "Demo Applications", "apps.html", [
      [ "Example Apps", "example_app.html", [
        [ "Simple Example", "example_app.html#simple_example_app", null ],
        [ "Advanced Example", "example_app.html#advanced_example_app", null ],
        [ "High-Speed Example", "example_app.html#high_speed_example_app", null ],
        [ "Multi-Device Example", "example_app.html#multi_device_example_app", null ]
      ] ],
      [ "Explorer App", "explorer_app.html", [
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
      ] ],
      [ "CAN App (AFBR-S50 Ref. Board)", "can_app.html", [
        [ "Flash the CAN App via USB Bootloader", "can_app.html#autotoc_md49", null ],
        [ "Build And Run the CAN App using e² Studio IDE", "can_app.html#can_app_e2Studio", null ],
        [ "Using the CAN App", "can_app.html#autotoc_md50", null ],
        [ "Communication Interface CAN", "can_app.html#autotoc_md51", [
          [ "Introduction", "can_app.html#autotoc_md52", null ],
          [ "Architecture", "can_app.html#autotoc_md53", null ],
          [ "Bus levels", "can_app.html#autotoc_md54", null ],
          [ "CAN Frames", "can_app.html#autotoc_md55", [
            [ "Data Frame", "can_app.html#autotoc_md56", null ],
            [ "Remote Frame", "can_app.html#autotoc_md57", null ],
            [ "Error Frame", "can_app.html#autotoc_md58", null ]
          ] ],
          [ "Bus Access Method", "can_app.html#autotoc_md59", null ],
          [ "Arbitration Process", "can_app.html#autotoc_md60", null ],
          [ "Error handling", "can_app.html#autotoc_md61", null ]
        ] ]
      ] ]
    ] ],
    [ "AFBR-S50 Reference Board", "reference_board.html", [
      [ "Getting Started", "reference_board.html#reference_board_getting_started", null ],
      [ "Overview", "reference_board.html#reference_board_overview", [
        [ "Powering the Board", "reference_board.html#reference_board_power", null ],
        [ "USB Connector", "reference_board.html#reference_board_usb", [
          [ "Bootloader", "reference_board.html#reference_board_bootloader", null ]
        ] ],
        [ "CAN-Bus Connector", "reference_board.html#reference_board_can", null ],
        [ "Serial (UART) Connector", "reference_board.html#reference_board_uart", null ]
      ] ]
    ] ],
    [ "API Migration Guide", "migration_guide.html", [
      [ "API Migration Guide (1.4.4 → 1.5.6)", "migration_guide.html#migration_guide_1_5", [
        [ "Overview of Changes", "migration_guide.html#autotoc_md62", null ],
        [ "Introduction of distinct global range offsets for low and high power stages", "migration_guide.html#autotoc_md63", null ]
      ] ],
      [ "API Migration Guide (1.3.5 → 1.4.4)", "migration_guide.html#migration_guide_1_4", [
        [ "Overview of Changes", "migration_guide.html#autotoc_md64", null ],
        [ "Removal of Dual-Measurement Modes And Adding of Advanced Measurement Modes", "migration_guide.html#autotoc_md65", null ],
        [ "Improved Measurement Ready Callback", "migration_guide.html#autotoc_md66", null ],
        [ "Improved Measurement Evaluate Function", "migration_guide.html#autotoc_md67", null ],
        [ "Changed Data Types for Crosstalk and Offset Tables", "migration_guide.html#autotoc_md68", null ],
        [ "Advanced Debug Data Structure for Argus_EvaluateData", "migration_guide.html#autotoc_md69", null ],
        [ "S2PI Hardware Abstraction Layer (HAL) Changes", "migration_guide.html#autotoc_md70", [
          [ "Changed HAL Functions in the S2PI layer", "migration_guide.html#autotoc_md71", null ],
          [ "Added HAL Functions in the S2PI layer", "migration_guide.html#autotoc_md72", null ]
        ] ]
      ] ]
    ] ],
    [ "Troubleshooting", "faq.html", [
      [ "Device Initialization Yields Device Not Connected (Error Code -101)", "faq.html#autotoc_md73", null ],
      [ "Decreased Device Performance or Accuracy", "faq.html#autotoc_md74", null ],
      [ "EEPROM Readout Fails (Error Code -109)", "faq.html#faq_eeprom", [
        [ "1. How to Verify EEPROM Readout Sequence", "faq.html#autotoc_md75", null ],
        [ "2. Further Tests on EEPROM Readout Sequence", "faq.html#autotoc_md76", null ]
      ] ],
      [ "The Measurement Never Finishes or Yields Timeout Error (-6)", "faq.html#autotoc_md77", [
        [ "The Measurement Finished Callback is Never Invoked", "faq.html#autotoc_md78", null ],
        [ "The Measurement Finished Callback Yield a Timeout Error (Error Code -6)", "faq.html#autotoc_md79", null ]
      ] ]
    ] ],
    [ "Topics", "topics.html", "topics" ],
    [ "Data Structures", "annotated.html", [
      [ "Data Structures", "annotated.html", "annotated_dup" ],
      [ "Data Structure Index", "classes.html", null ],
      [ "Data Fields", "functions.html", [
        [ "All", "functions.html", null ],
        [ "Variables", "functions_vars.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "Globals", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Variables", "globals_vars.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", "globals_eval" ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ],
    [ "Examples", "examples.html", "examples" ]
  ] ]
];

var NAVTREEINDEX =
[
"01__simple__example_8c.html",
"explorer__tasks_8c.html#adf5f1e7124841becf9ebe3fa0f37bbfb",
"group__argus__api.html#ggaf66073d165e8d8700fe59f1082f84c27aa0f034eb5f782bf952f54659440a5844",
"group__argus__meas.html#gabe59a112f841d59e435510d3c17c9db1",
"group__argus__xtk__cli.html#gace5ad3b17bc0d1849ecdd2e7ea0d4bb6",
"group__sci__cmd.html#ga61157a72bdcf53ee9ed1e1bedb00c350",
"structargus__meas__frame__t.html#a97d8aff21d43fd27f5d03baaa1c6625d"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';