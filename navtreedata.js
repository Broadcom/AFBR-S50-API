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
  [ "AFBR-S50 SDK - Argus API Reference Manual", "index.html", [
    [ "Introduction", "index.html", [
      [ "API Overview", "index.html#autotoc_md0", null ],
      [ "Getting Started", "index.html#autotoc_md1", null ],
      [ "Porting to a new MCU platform", "index.html#autotoc_md2", null ],
      [ "Copyright and License", "index.html#autotoc_md3", null ]
    ] ],
    [ "Architectural Overview", "sw_api.html", [
      [ "Overview", "sw_api.html#sw_api_overview", null ],
      [ "Operation Principle", "sw_api.html#sw_api_principle", null ],
      [ "API Modules", "sw_api.html#sw_api_modules", null ]
    ] ],
    [ "Getting Started", "getting_started.html", [
      [ "AFBR-S50 API", "getting_started.html#gs_api", null ],
      [ "Simple Example", "getting_started.html#gs_simple_example", null ],
      [ "Advanced Example", "getting_started.html#gs_adv_example", null ],
      [ "Build And Run the Examples using MCUXpresso", "getting_started.html#gs_mcuxpresso", null ],
      [ "Troubleshooting", "getting_started.html#gs_troubleshooting", [
        [ "Error -101 (Device Not Connected)", "getting_started.html#autotoc_md4", null ]
      ] ]
    ] ],
    [ "MCU Porting Guide", "porting_guide.html", [
      [ "Introduction", "porting_guide.html#pg_introduction", null ],
      [ "Toolchain Compatibility", "porting_guide.html#pg_toolchain", null ],
      [ "Architectur Compatibility", "porting_guide.html#pg_architecture", null ],
      [ "Hardware Compatibility", "porting_guide.html#pg_hardware", null ],
      [ "Hardware Layers", "porting_guide.html#pg_hw_layers", [
        [ "S2PI (= SPI + GPIO) Layer", "porting_guide.html#pg_s2pi", [
          [ "S2PI Overview", "porting_guide.html#autotoc_md5", null ],
          [ "Initialization", "porting_guide.html#autotoc_md6", [
            [ "Pin configuration", "porting_guide.html#autotoc_md7", null ],
            [ "SPI Mode", "porting_guide.html#autotoc_md8", null ],
            [ "SPI Speed", "porting_guide.html#autotoc_md9", null ],
            [ "DMA", "porting_guide.html#autotoc_md10", null ],
            [ "DMA Interrupts and Callback Function", "porting_guide.html#autotoc_md11", null ],
            [ "DMA Interrupt Priority", "porting_guide.html#autotoc_md12", null ]
          ] ]
        ] ],
        [ "Timer Layer", "porting_guide.html#pg_timer", [
          [ "Lifetime Counter (LTC)", "porting_guide.html#autotoc_md13", null ],
          [ "Periodic Interrupt Timer (PIT)", "porting_guide.html#autotoc_md14", null ]
        ] ],
        [ "Interrupt Layer", "porting_guide.html#pg_irq", [
          [ "Interrupt Priority", "porting_guide.html#autotoc_md15", null ],
          [ "Concurrency and Interrupt Locking", "porting_guide.html#autotoc_md16", null ]
        ] ],
        [ "NVM Layer", "porting_guide.html#pg_nvm", null ],
        [ "Log Layer", "porting_guide.html#pg_log", null ]
      ] ],
      [ "Step-by-step porting guide", "porting_guide.html#pg_guide", [
        [ "Create a new project in your environment", "porting_guide.html#pg_new_project", null ],
        [ "Implement hardware interfaces", "porting_guide.html#pg_hw_interface", null ],
        [ "Link Library File", "porting_guide.html#pg_linker", null ],
        [ "Utilize the API", "porting_guide.html#pg_api", null ]
      ] ]
    ] ],
    [ "Explorer App (API Demo)", "hw_api.html", "hw_api" ],
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
        [ "Functions", "globals_func.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ],
    [ "Examples", "examples.html", "examples" ]
  ] ]
];

var NAVTREEINDEX =
[
"01_simple_example_8c-example.html",
"group__arguscal.html#ga066edd20ab5393cb47e64b60c702dd6f",
"group__argusmeas.html#gae1682b01c1ed1b837fd7d763d81e857c",
"group__status.html#gga67a0db04d321a74b7e7fcfd3f1a3f70bad687ba3633069a4b24aa98a1b3bef9d6",
"structargus__pixel__t.html#a1bab0aba97de4f50e10358c3cd2e3336"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';