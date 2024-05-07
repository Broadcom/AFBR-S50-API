# AFBR-S50 API

## Introduction

The **AFBR-S50 API** is the appertaining software for the
[AFBR-S50 Time-of-Flight Sensor family](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)
by [Broadcom Inc](https://www.broadcom.com/).

The repository consists of the **AFBR-S50 Core Library**, a static ANSI-C
library, and its accompanied header files and documentation. Additionally,
example and demo projects are provided for certain processors and evaluation
boards.

## Documentation

The **API Reference Manual** can be viewed
[here](https://broadcom.github.io/AFBR-S50-API/).

## Overview

### File Structure

The repository is structured in the following way:

-   `/AFBR-S50`: Contains the **AFBR-S50 API**.

    -   `/Include`: Contains the **AFBR-S50 API** as ANSI-C header files.

    -   `/Lib`: Contains the static **AFBR-S50 Core Libraries** for multiple
        [Cortex-Mx](https://developer.arm.com/ip-products/processors/cortex-m)
        architectures.

    -   `/Test`: The **HAL-Self Test** suite that can be used to verify the
        ported HAL.

-   `/Doxygen`: Contains additional documentation files that can be used with
    Doxygen to generate the **API Reference Manual**.

-   `/Projects`: Project files for several IDEs.

    -   `/e2Studio`: Project files for the
        [Renesas e² Studio IDE](https://www.renesas.com/us/en/software-tool/e-studio)
        for all Renesas processors.

    -   `/MCUXpressoIDE`: Project files for the
        [NXP MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)
        for all NXP processors.

    -   `/STM32CubeIDE`: Project files for the
        [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
        for all STM32 processors.

-   `/Sources`: Contains all source files.

    -   `/CANApp`: A **CAN-Bus** interface application running on the
        [**AFBR-S50 Reference Board** by **MikroElektronika**](https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board).
        It listens to commands and streams 1D range values on the CAN-bus.

    -   `/ExampleApp`: A collection of basic example applications that
        demonstrate the usage of the **AFBR-S50 API** and stream measurement
        data via an UART connection.

    -   `/ExplorerApp`: A more sophisticated example application that implements
        a serial communication interface via USB or UART to connect to the
        corresponding **AFBR-S50 Explorer GUI** (download via the Software
        Development Kit from the
        [official Broadcom webpage](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors/afbr-s50mv85g#downloads)).

    -   `/Platform`: The platform specific code like peripheral drivers or
        hardware abstraction layers (HAL).

        -   `/NXP_MKLxxZ`: The platform code for the
            [NXP Kinetis L-Series](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus:KINETIS_L_SERIES)
            processors (e.g.
            [MKL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z)
            which is used by the
            [AFBR-S50 Evaluation Kits](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)).

        -   `/Renesas_RA4M2`: The platform code for the
            [Renesas RA4M2 series](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m2-100mhz-arm-cortex-m33-trustzone-high-integration-lowest-active-power-consumption)
            which is used for the
            [AFBR-S50 Reference Board by MikroElektronika](https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board).

        -   `/STM32F4xx`: The platform code for the
            [STM32F4 Series](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
            (e.g.
            [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)).

### Projects

All projects are located under `/Projects/<IDE>/<PROJECT>`. The following
projects are available:

-   `AFBR_S50_Example_<MCU>`: A simple example application that streams
    measurement data via an UART connection.
-   `AFBR_S50_ExplorerApp_<MCU>`: A more sophisticated example application that
    implements a serial communication interface via USB or UART to connect to
    the corresponding **AFBR-S50 Explorer GUI** (download from the official
    [Broadcom webpage](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)).
-   `AFBR_S50_CANApp_<MCU>`: A CAN interface application running on the
    [AFBR-S50 Reference Board by MikroElektronika](https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board).
    listens to commands and streams 1D range values on the CAN bus.

## Getting Started

### Required Hardware

Before you get started, you require some hardware. For an easy start, it is
recommended to use one of the following:

-   [Broadcom AFBR-S50 Evaluation Kit](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)
-   [MikroElektronika AFBR-S50 Reference Board](https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board)

### Building the Documentation

Creating the documentation is generally not required since it is already hosted
via [GitHub Pages](https://broadcom.github.io/AFBR-S50-API/).

However, if an updated documentation needs to be created, the following tools
are required and must be installed and setup correctly. Please refer the tools
documentation on how to setup.

-   [Doxygen](https://www.doxygen.nl/)
-   [Graphviz](https://graphviz.org/)

After successful setup, the documentation can be created by invoking `doxygen`
from the root directory:

```bash
$ doxygen
```

Now, the `index.html` can be found at `/Documentation/html/index.html`.

### Compile and Run the Demo Projects

Please refer the
[Getting Started](https://broadcom.github.io/AFBR-S50-API/getting_started.html)
in the
[API Reference Manual](https://broadcom.github.io/AFBR-S50-API/index.html) for a
detailed guide on how to setup the projects.

Please also refer the
[Demo Applications](https://broadcom.github.io/AFBR-S50-API/apps.html) in the
[API Reference Manual](https://broadcom.github.io/AFBR-S50-API/index.html) for
an overview of existing projects.

### Porting to another MCU Platform

In order to use the **AFBR-S50 API** on another MCU platform, refer to the
generic
[Porting Guide](https://broadcom.github.io/AFBR-S50-API/porting_guide.html) in
the [API Reference Manual](https://broadcom.github.io/AFBR-S50-API/index.html).

Also refer to the special **Porting Guide** based on a port of the **AFBR-S50
API** to the
[STM32F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
Evaluation Kit with a Cortex-M4 MCU. The guide can be found on the
[Broadcom homepage](https://docs.broadcom.com/docs/AFBR-S50-SDK-Porting-Guide-to-Cortex-M4-PG).

### Migrate from previous versions

If you have used the previous version of the **AFBR-S50 API**, please read the
[Migration guide](https://broadcom.github.io/AFBR-S50-API/migration_guide.html)
to get an overview of the changes in the API as well as the HAL compared to the
current version.

## How to get Support

In order to get support, please make sure the issue is related to the **AFBR-S50
API**. If the issue is related to the **AFBR-S50 Hardware**, contact the
corresponding support via `support.tof[at]broadcom.com`.

If your issue relates to the **AFBR-S50 API**, please make sure you have read
and understood the
[API Reference Manual](https://broadcom.github.io/AFBR-S50-API/index.html)
(especially the
[Troubleshooting](https://broadcom.github.io/AFBR-S50-API/faq.html) section). If
your issue is still there, please see if you can find an related issue in the
Issue Section of the repository. Finally, feel free to open a new ticked
describing your problem.

## Contributing

We highly appreciate any contribution to the **AFBR-S50 API** project.

Ideas for contributions could be:

-   documentation work,
-   fixing/updating platform or example code or project files,
-   porting the platform code to new processors,
-   adding additional examples/demo projects,
-   and much more...

Please make sure your work fits well into the existing structure and coding
style. Also, before you start your work, check if your planned contribution will
be accepted by opening a new issue that describes your planned changes. This will
also help to gather information about the implementation.

## Acknowledgements

The **AFBR-S50 API** is based developed using the following tools:

-   [NXP MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)
-   [ST STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
-   [Renesas e² Studio IDE](https://www.renesas.com/us/en/software-tool/e-studio)
-   printf library by [mpaland](https://github.com/mpaland/printf) and
    [eyalroz](https://github.com/eyalroz/printf)
-   [Doxygen](https://www.doxygen.nl/) and [Graphviz](https://graphviz.org/)

## Copyright and License

The **AFBR-S50-API** is published under the **BSD 3-Clause License**:

> Copyright (c) 2023, Broadcom Inc All rights reserved.
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
> 3. Neither the name of the copyright holder nor the names of its contributors
>    may be used to endorse or promote products derived from this software
>    without specific prior written permission.
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
