/*
@licstart  The following is the entire license notice for the
JavaScript code in this file.

Copyright (C) 1997-2019 by Dimitri van Heesch

This program is free software; you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License as published by
the Free Software Foundation

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

@licend  The above is the entire license notice
for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "AFBR-S50 API Reference Manual", "index.html", [
    [ "Introduction", "index.html", [
      [ "API Overview", "index.html#autotoc_md0", null ],
      [ "Getting Started", "index.html#autotoc_md1", null ],
      [ "Copyright and License", "index.html#autotoc_md2", null ]
    ] ],
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
        [ "1. How to Verify EEPROM Readout Sequence", "faq.html#autotoc_md78", null ],
        [ "2. Further Tests on EEPROM Readout Sequence", "faq.html#autotoc_md79", null ]
      ] ],
      [ "The Measurement Never Finishes or Yields Timeout Error (-6)", "faq.html#autotoc_md80", [
        [ "The Measurement Finished Callback is Never Invoked", "faq.html#autotoc_md81", null ],
        [ "The Measurement Finished Callback Yield a Timeout Error (Error Code -6)", "faq.html#autotoc_md82", null ]
      ] ]
    ] ],
    [ "Modules", "modules.html", "modules" ],
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
"explorer__tasks_8c.html#a11b72623a9601834584c5ecf08dd6e27",
"group__argus__api.html#gabe8846d7abef63b75e8147f4f142708b",
"group__argus__dca.html#ggada1413643a68f3cd36f6c0a1d0b61e91a5c045d26a29f06cc32fc757b51578e25",
"group__argus__irq.html#ga7eb2f9c9e40114afd7d079ace7c152e7",
"group__argus__status.html#gga67a0db04d321a74b7e7fcfd3f1a3f70ba083e5907c8c528607783350b760ca0bb",
"group__argus__version.html#ga118a521ea35e67d1d8b8daeabf4e0053",
"group__core__device.html#gaf561d6507fa19ca607181d62e3217198",
"group__profiler.html#ga8c097a59d344c89887c5f4cbdbcf493e",
"group__sci__frame.html#gabd264bf8c72debf4020e9e681d655e71",
"structargus__pixel__t.html#a288c43dbc8360f242babcc74f1746c52"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';