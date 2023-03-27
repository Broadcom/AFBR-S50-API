# Introduction {#intro}

The **AFBR-S50 SDK** is the appertaining software package for the
[AFBR-S50 Time-of-Flight Sensor family](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors)
by [Broadcom Inc](https://www.broadcom.com/). The SDK consists of evaluation
tools running with the **Evaluation** or **Reference Kits** as well as the
**AFBR-S50 Core Library** and its API for the **Broadcom Time-of-Flight sensor
devices**.

The evaluation tools contain basic examples as well as fully featured reference
applications for the **AFBR-S50 Core Library**. The applications utilize the
**AFBR-S50 API** and provide a fast getting started experience.

Beneath the example implementations, the **Explorer App** publishes the full
**AFBR-S50 sensor** functionality over **USB** or **UART** interfaces. The @ref
reference_board by **MikroElektronika**
([available here](https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board)) is
provided as an independent product that provides additional interfaces and
features such as a **CAN-bus**.

The **AFBR-S50 API** and reference implementation are running on the
[ARM Cortex-Mx](https://developer.arm.com/ip-products/processors/cortex-m/)
processor architecture. Example projects exist for
[Cortex-M0+](https://developer.arm.com/Processors/Cortex-M0-Plus),
[Cortex-M4](https://developer.arm.com/Processors/Cortex-M4) and
[Cortex-M33](https://developer.arm.com/Processors/Cortex-M33).

In order to connect and evaluate the sensor, a PC GUI, the **AFBR-S50
Explorer**, is provided. The GUI establishes a serial connection via **UART** or
**USB** to the **Explorer App** running on a **Evaluation Kit** or **Reference
Board**.

It leverages the **AFBR-S50 API and Core Library** to apply configuration and
obtain measurement results. The latter is conveniently visualized in **1D or 3D
plots**.

## API Overview

The **AFBR-S50 SDK** contains the sensor API that wraps around the core library
and provides an easy access to the sensors vast functionality to the user. A
very simple example demonstrates the basic usage of the API and serves as a
starting point for any user implementation. A comprehensive reference
implementation (namely the **Explorer App**, utilized for the evaluation kit)
unveils the complete capability of the API and coats it in a serial hardware
interface that connects via UART or USB to the **AFBR-S50 Explorer** GUI.

The **AFBR-S50 Core Library** is required to operate the **AFBR-S50 Sensor
Family** along with the users application on the same target platform. It is a
comprehensive, yet simple library that implies all the algorithms that are
required to run the sensor with nearly any microprocessor that is based on the
[ARM Cortex-Mx Architecture](https://developer.arm.com/ip-products/processors/cortex-m).
Therefore the library is implemented hardware independent and the user needs to
establish the connection from the library to the underlying hardware by
implementing the provided hardware interface layers for its platform. Apart from
that, the library is easy to use and runs basically in the background through
interrupt service routines without putting much load on the main thread such
that the CPU remains available and responsive for the user application.

There are example implementations of the **AFBR-S50 Software API** that serve,
along with the provided platform drivers, as a starting point for user
implementations. All the examples are build on the **Cortex-Mx Architecture** in
general and the following hardware is available:

-   Evaluation Kit based on the
    [NXP Kinetis MKL46z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl4x-48-mhz-usb-segment-lcd-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL4x)
    (especially the
    [NXP FRDM-KL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z)
    evaluation board).

-   Evaluation Kit based on the
    [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
    (especially the
    [STM32 NUCLEO-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
    evaluation board).

-   Reference Design files based on the
    [NXP Kinetis MKL17z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl1x-48-mhz-mainstream-small-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL1x)

-   [Reference Board](@ref reference_board) based on the
    [Renesas RA4M2](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m2-100mhz-arm-cortex-m33-trustzone-high-integration-lowest-active-power-consumption)

## Getting Started

-   Refer to the @ref getting_started section in order to build and run the
    provided projects on any of the featured platforms.

-   Also see the @ref apps section for an overview of the existing projects and
    their capabilities.

-   See the @ref porting_guide section of how to use the **AFBR-S50 API** on a
    new **Cortex-Mx** platform.

-   Read through the @ref sw_api section in to get a deeper understanding of the
    **AFBR-S50 API** and its components.

## Copyright and License

The **AFBR-S50-API** is published under the **BSD 3-Clause License**:

> Copyright (c) 2021, Broadcom Inc, All rights reserved.
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
