# AFBR-S50 API

[日本語版 README はこちら](/README-ja.md)

## Introduction

The _AFBR-S50 API_ is the appertaining software for the [AFBR-S50 Time-of-Flight Sensor family](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors) by [Broadcom Inc](https://www.broadcom.com/).

The repository consists of the _AFBR-S50 Core Library_, a static ANSI-C library, and its accompanied header files and documentation. Additionally, example and demo projects are provided for certain processors and evaluation boards.

## Documentation

The API Reference Manual can be viewed [here](https://broadcom.github.io/AFBR-S50-API/).

## Overview

### File Structure

The repository is structured in the following way:

- `/AFBR-S50`: Contains the _AFBR-S50 API_.
  - `/Lib`: Contains the static _AFBR-S50 Core Libraries_ for multiple [Cortex-Mx](https://developer.arm.com/ip-products/processors/cortex-m) architectures.
  - `/Include`: Contains the _AFBR-S50 API_ as ANSI-C header files.
  - `/Doxygen`: Contains additional documentation files that can be used with Doxygen to generate the _API Reference Manual_.
- `/Sources`: Contains all source files.
  - `/ExampleApp`: A simple example application that streams measurement data via an UART connection.
  - `/ExplorerApp`: A more sophisticated example application that implements a serial communication interface via USB or UART to connect to the corresponding AFBR-S50 Explorer GUI (download from the official Broadcom webpage).
  - `/Platform`: The platform specific code like peripheral drivers or hardware abstraction layers.
    - `/NXP_MKLxxZ`: The platform code for the [NXP Kinetis L-Series](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus:KINETIS_L_SERIES) processors (e.g. [MKL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z) which is used by the first _AFBR-S50 Evaluation Kit_).
    <!-- - `/STM32F4xx`: The platform code for the [STM32F4 Series](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html) (e.g. [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html) which is used for the new *AFBR-S50 Evaluation Kit*). -->
- `/Projects`: Project files for several IDEs.
  - `/MCUXpressoIDE`: Project files for the [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) for all NXP processors.
  <!-- - `/STM32CubeIDE`: Project files for the [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html) for all STM32 processors. -->

### Projects

All projects are located under `/Projects/<IDE>/<PROJECT>`. The following projects are available:

- `AFBR_S50_ExampleApp_<MCU>`: A simple example application that streams measurement data via an UART connection.
- `AFBR_S50_ExplorerApp_<MCU>`: A more sophisticated example application that implements a serial communication interface via USB or UART to connect to the corresponding AFBR-S50 Explorer GUI (download from the official [Broadcom webpage](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)).

## Getting Started

### Building the Documentation

Creating the documentation is generally not required since it is already hosted via [GitHub Pages](https://broadcom.github.io/AFBR-S50-API/).

However, if an updated documentation needs to be created, the following tools are required and must be installed and setup correctly. Please refer the tools documentation on how to setup.

- [Doxygen](https://www.doxygen.nl/)
- [Graphviz](https://graphviz.org/)

After successful setup, the documentation can be created by invoking _doxygen_ from the `AFBR-S50` directory:

```bash
$ cd AFBR-S50
$ doxygen
```

Now, the `index.html` can be found at `/AFBR-S50/Documentation/html/index.html`.

### Compile and Run the NXP Examples using MCUXpresso IDE

Please refer the [_Getting Started_ section of the _API Reference Manual_](https://broadcom.github.io/AFBR-S50-API/getting_started.html#gs_mcuxpresso) for a detailed guide on how to setup the projects using the [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).

Here is a basic sketch of the required steps:

1. Clone the repository.
2. Download and install the [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).
3. Download and import the NXP MKL46z (or MKL17z) SDK into the _MCUXpresso IDE_
4. Import the project located in `/Projects/MCUXpressoIDE`
   - In _MCUXpresso IDE_ go to _Menu_ > _File_ > _Import..._ > _General_ > _Existing Projects into Workspace_ > _Next_ > Browse and select all required projects > _Finish_
   - Make sure, the "Copy projects into workspace" option is disabled.
5. Build the projects
   - In _MCUXpresso IDE_ go to _Menu_ > _Project_ > _Build All_
6. Debug the project
   - Select the project in the MCUXpressoIDE _Project Explorer_.
   - In _MCUXpresso IDE_ go to _Menu_ > _Run_ > _Debug As_ > _PEMicro probes_ > _OK_
   - When the debugger hits the breakpoint at main, go to _Menu_ > _Run_ > _Resume_

<!-- ### Compile and Run the STM Examples using STM32Cube IDE

Please refer to the [*Getting Started* Section of the *API Reference Manual*](https://broadcom.github.io/AFBR-S50-API/getting_started.html#gs_mcuxpresso) on how to setup the projects using the [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html). -->

### Porting to another MCU Platform

In order to use the _AFBR-S50 API_ on another MCU platform, refer to the generic [_Porting Guide_ in the _API Reference Manual_](https://broadcom.github.io/AFBR-S50-API/porting_guide.html).

Also refer to the special _Porting Guide_ based on a port of the _AFBR-S50 API_ to the [STM32F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) Evaluation Kit with a Cortex-M4 MCU. The guide can be found on the [Broadcom homepage](https://docs.broadcom.com/docs/AFBR-S50-SDK-Porting-Guide-to-Cortex-M4-PG).

## How to get Support

In oder to get support, please make sure the issue is related to the _AFBR-S50 API_. If the issue is related to the _AFBR-S50 Hardware_, contact the corresponding support via `support.tof[at]broadcom.com`.

If your issue relates to the _AFBR-S50 API_, please make sure you have read and understood the [API Reference Manual](https://broadcom.github.io/AFBR-S50-API/) (especially the _Troubleshooting_ section). If your issue is still there, please see if you can find an related issue in the Issue Section of the repository. Finally, feel free to open a new ticked describing your problem.

## Contributing

We highly appreciate any contribution to the _AFBR-S50 API_ project.

Ideas for contributions could be:

- documentation work,
- fixing/updating platform or example code or project files,
- porting the platform code to new processors,
- adding additional examples/demo projects,
- and much more...
  .

Please make sure your work fits well into the existing structure and coding style. Also, before you start your work, check if your planned contribution will be accepted by open a new issue that describes your planned changes. This will also help to gather information about the implementation.

## Copyright and License

The _AFBR-S50-API_ is published under the _BSD 3-Clause License_:

> Copyright (c) 2021, Broadcom Inc
> All rights reserved.
>
> Redistribution and use in source and binary forms, with or without
> modification, are permitted provided that the following conditions are met:
>
> 1. Redistributions of source code must retain the above copyright notice, this
>    list of conditions and the following disclaimer.
>
> 2. Redistributions in binary form must reproduce the above copyright notice,
>    this list of conditions and the following disclaimer in the documentation
>    and/or other materials provided with the distribution.
>
> 3. Neither the name of the copyright holder nor the names of its
>    contributors may be used to endorse or promote products derived from
>    this software without specific prior written permission.
>
> THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
> AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
> IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
> DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
> FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
> DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
> SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
> CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
> OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
> OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
