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
    [ "Getting Started", "getting_started.html", "getting_started" ],
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
    [ "Demo Applications", "apps.html", "apps" ],
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
      [ "API Migration Guide (1.5.6 → 1.6.5)", "migration_guide.html#migration_guide_1_6", [
        [ "Overview of Data & Function Changes / Extensions / new Items", "migration_guide.html#autotoc_md62", null ],
        [ "Changes in more Detail", "migration_guide.html#autotoc_md63", null ],
        [ "Extensions and new Items in more Detail", "migration_guide.html#autotoc_md64", null ]
      ] ],
      [ "API Migration Guide (1.4.4 → 1.5.6)", "migration_guide.html#migration_guide_1_5", [
        [ "Overview of Changes", "migration_guide.html#autotoc_md65", null ],
        [ "Introduction of distinct global range offsets for low and high power stages", "migration_guide.html#autotoc_md66", null ]
      ] ],
      [ "API Migration Guide (1.3.5 → 1.4.4)", "migration_guide.html#migration_guide_1_4", [
        [ "Overview of Changes", "migration_guide.html#autotoc_md67", null ],
        [ "Removal of Dual-Measurement Modes And Adding of Advanced Measurement Modes", "migration_guide.html#autotoc_md68", null ],
        [ "Improved Measurement Ready Callback", "migration_guide.html#autotoc_md69", null ],
        [ "Improved Measurement Evaluate Function", "migration_guide.html#autotoc_md70", null ],
        [ "Changed Data Types for Crosstalk and Offset Tables", "migration_guide.html#autotoc_md71", null ],
        [ "Advanced Debug Data Structure for Argus_EvaluateData", "migration_guide.html#autotoc_md72", null ],
        [ "S2PI Hardware Abstraction Layer (HAL) Changes", "migration_guide.html#autotoc_md73", [
          [ "Changed HAL Functions in the S2PI layer", "migration_guide.html#autotoc_md74", null ],
          [ "Added HAL Functions in the S2PI layer", "migration_guide.html#autotoc_md75", null ]
        ] ]
      ] ]
    ] ],
    [ "Troubleshooting", "faq.html", [
      [ "Device Initialization Yields Device Not Connected (Error Code -101)", "faq.html#autotoc_md76", null ],
      [ "Decreased Device Performance or Accuracy", "faq.html#autotoc_md77", null ],
      [ "EEPROM Readout Fails (Error Code -109)", "faq.html#faq_eeprom", [
        [ "How to Verify EEPROM Readout Sequence", "faq.html#autotoc_md78", null ],
        [ "Further Tests on EEPROM Readout Sequence", "faq.html#autotoc_md79", null ]
      ] ],
      [ "The Measurement Never Finishes or Yields Timeout Error (-6)", "faq.html#autotoc_md80", [
        [ "The Measurement Finished Callback is Never Invoked", "faq.html#autotoc_md81", null ],
        [ "The Measurement Finished Callback Yield a Timeout Error (Error Code -6)", "faq.html#autotoc_md82", null ]
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
"explorer__tasks_8c.html#a283013b33ea94c3054713e68427315d6",
"group__argus__api.html#ggabe8846d7abef63b75e8147f4f142708ba257bb825d75628478349161d2c260ea7",
"group__argus__map.html#gaa51a0ce9b1e5dd179fbeaedfc4dd80e6",
"group__argus__xtk__cli.html#ga41ce3991e7acca3071e36d6c04dfb932",
"group__sci.html#ga90d56fe3172b9b3b5c064919f1245646",
"structargus__cfg__dca__t.html#a5622d96cdb5230d51e33e5b1a2bc4c17"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';