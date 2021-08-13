# Introduction {#intro}

The *AFBR-S50 SDK* is the appertaining software for the [AFBR-S50 Time-of-Flight Sensor family](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors) by [Broadcom Inc](https://www.broadcom.com/). The SDK consists of evaluation tools running with the Evaluation Kit as well as the *AFBR-S50 Core Library* and its API for the sensor devices.

The evaluation tools contain an fully implemented reference solution for the *AFBR-S50 Core Library*. It utilizes the corresponding software API and provides another, equivalent hardware API in order to access and control the sensor device through a serial hardware interface, such as USB or UART. The *ExplorerApp* is running on a [Cortex-M0+](https://developer.arm.com/ip-products/processors/cortex-m/cortex-m0-plus) processor such as the [NXP Kinetis L-Series](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus:KINETIS_L_SERIES) (e.g. [MKL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z) is used for the Evaluation Kit). In order to connect and evaluate the sensor, a PC GUI, the *AFBR-S50 Explorer*, is provided. The GUI establishes a serial connection via USB to the *ExplorerApp* running on the *MKL46z Cortex-M0+ by NXP* and leverages the API and core library to apply configuration and obtain measurement results. The latter is conveniently visualized in 1D or 3D plots.


## API Overview

The *AFBR-S50 SDK* contains the sensor API that wraps around the core library and provides an easy access to the sensors vast functionality to the user. A very simple example demonstrates the basic usage of the API and serves as a starting point for any user implementation. A comprehensive reference implementation (namely the *ExplorerApp*, utilized for the evaluation kit) unveils the complete capability of the API and coats it in a serial hardware interface that connects via USB to the *AFBR-S50 Explorer* GUI. The latter can easily adopted to other serial interfaces such as UART.

The *AFBR-S50 Core Library* is required to operate the *AFBR-S50 Sensor Family* along with the users application on the same target platform. It is a comprehensive, yet simple library that implies all the algorithms that are required to run the sensor with nearly any microprocessor that is based on the [ARM Cortex-Mx Architecture](https://developer.arm.com/ip-products/processors/cortex-m). Therefore the library is implemented hardware independent and the user needs to establish the connection from the library to the underlying hardware by implementing the provided hardware interface layers for its platform. Apart from that, the library is easy to use and runs basically in the background through interrupt service routines without putting much load on the main thread such that the CPU remains available and responsive for the user application.

There are example implementations of the *AFBR-S50 Software API* that serve, along with the provided platform drivers, as a starting point for user implementations. All the examples are build on the *Cortex-M0+ Architecture* in general and on the [NXP Kinetis MKL46z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl4x-48-mhz-usb-segment-lcd-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL4x) (Evaluation Kit) and [NXP Kinetis MKL17z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl1x-48-mhz-mainstream-small-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL1x) (Reference Design).

## Getting Started

If you want to implement and utilize the *AFBR-S50 Core Library* and API within your own embedded environment, go to the the [getting started](@ref getting_started) section. Make sure to read about the examples and how to get them running with the free [MCUXpresso IDE from NXP](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) in the [how to build and run](@ref gs_mcuxpresso) section. Also see the [Software API](@ref sw_api) section for an brief overview of the API functionality.

## Porting to a new MCU Platform

If you want to use your own hardware to host and run the *AFBR-S50 Core Library*, refer to the [API Porting Guide](@ref porting_guide).

## Copyright and License

The *AFBR-S50-API* is published under the *BSD 3-Clause License*:

> Copyright (c) 2021, Broadcom Inc
> All rights reserved.
> 
> Redistribution and use in source and binary forms, with or without
> modification, are permitted provided that the following conditions are met:
> 
> 1. Redistributions of source code must retain the above copyright notice, this
>   list of conditions and the following disclaimer.
> 
> 2. Redistributions in binary form must reproduce the above copyright notice,
>   this list of conditions and the following disclaimer in the documentation
>   and/or other materials provided with the distribution.
>
> 3. Neither the name of the copyright holder nor the names of its
>   contributors may be used to endorse or promote products derived from
>   this software without specific prior written permission.
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