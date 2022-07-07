# Getting Started {#getting_started}

The following section gives an brief overview on of the **AFBR-S50 Core Library
and API** and shows how to get started using the evaluation or reference
platforms and example applications.

In order to get more information about the featured applications, refer to the
@ref apps section.

If a port to another microcontroller platform is required, refer to the @ref
porting_guide section.

## Build And Run Projects {#gs_build}

To directly dive into building and debugging the featured projects, go to the
corresponding IDE getting started section:

-   \subpage mcuxpresso
-   \subpage e2studio
-   \subpage stm32cubeide

Here is an overview of the featured targets and the required IDEs:

| Board                     | MCU                                                                                                                                                                                                                                                                    | IDE               | Comment                                                                                                                                                                                                    |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| AFBR-S50 Evaluation Kit   | [NXP Kinetis MKL46z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl4x-48-mhz-usb-segment-lcd-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL4x)  | \ref mcuxpresso   | Based on the [NXP FRDM-KL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z) Evaluation Board |
| AFBR-S50 Reference Design | [NXP Kinetis MKL17z](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus/kinetis-kl1x-48-mhz-mainstream-small-ultra-low-power-microcontrollers-mcus-based-on-arm-cortex-m0-plus-core:KL1x) | \ref mcuxpresso   |                                                                                                                                                                                                            |
| AFBR-S50 Evaluation Kit   | [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)                                                                                                                                                                                 | \ref stm32cubeide | Based on the [STM32 NUCLEO-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) Evaluation Board                                                                                             |
| @ref reference_board      | [Renesas RA4M2](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m2-100mhz-arm-cortex-m33-trustzone-high-integration-lowest-active-power-consumption)                                                                       | \ref e2studio     | Provided by **MikroElektronika**                                                                                                                                                                           |

## Using the AFBR-S50 API {#gs_api}

@note All **AFBR-S50 API** related functions, definitions and constants have a
prefix `Argus_` which is essentially an alias or working title for the
**AFBR-S50 Time-of-Flight Sensor** device.

The **AFBR-S50 Core Library** is provided as a static ANSI-C library file
(`lib*.a`) and the corresponding API is provided as ANSI-C header files (`*.h`).
After setting up the linker to link the library, it is sufficient to include the
main header in the `AFBR-S50/Include` folder, `argus.h`:

```C
#include "argus.h"
```

### Initialization

The API utilizes an abstract handler object that contains all internal states
for a single time-of-flight sensor device. In this way, it is possible to use
the same API with more than a single device. After including the header file,
the handler object must be created by calling the #Argus_CreateHandle function
to obtain a pointer to the newly allocated object. This is done via the standard
library function:

```C
void * malloc(size_t size)
```

If it is required to use a different function, create and overwrite the weakly
linked method and implement your own memory allocation algorithm:

```C
void * Argus_Malloc(size_t size) { /* ... */ }
```

After creation of the handler object, the **AFBR-S50** module must be
initialized with the corresponding handler object:

```C
status_t status = Argus_Init(hnd, SPI_SLAVE);
```

Note that all peripheral modules must be ready to be used before executing any
API function. So make sure to initialize the board and its peripherals before
initializing the API via #Argus_Init.

After calling #Argus_Init, the device is ready to run and has been setup with
default configuration and calibration data. Use the provided API functions to
customize the given default configuration to the needs and requirements of the
application.

Note that #Argus_Init returns #STATUS_OK on successful initialization. In case
of any non-zero return value, refer to the \ref faq section

### Running Measurements

There are two possibilities to operate the device:

-   Trigger single measurements and poll the device status.
-   Utilize a periodic interrupt timer to periodically start measurements
    autonomously.

#### Single Measurements

Using the simple polling method, the measurements are triggered by the main
thread by calling the #Argus_TriggerMeasurement function any time a new
measurement should be started. A new measurement frame is started and after
reading the data from the device, the callback is invoked to inform the host
application about the data ready event.

In the meantime, the host application can either poll the module status using
the #Argus_GetStatus method or execute other tasks and wait for the measurement
data ready callback.

After finishing the measurement cycle, the #Argus_EvaluateData function must be
called to obtain measurement data like range and signal quality from the raw
readout data. Note that it is mandatory to call the evaluation method after each
completed measurement cycle. Otherwise, the internal raw data buffer is kept
occupied and no new measurements can be triggered anymore. The module contains a
double buffer architecture, which allows to start the next measurement and
evaluate the current measurement data while the device already executes the next
measurement frame.

After evaluation, the #argus_results_t data structure is filled with all
measurement results that can be processed now be processed by the host
application depending on the users needs.

Please note that the laser safety module might refuse to restart a measurement
at the time the function is called. This is due to timing constraints dictated
by the laser safety rules. In this case, the function does return with
#STATUS_ARGUS_POWERLIMIT instead. Use the \link #Argus_SetConfigurationFrameTime
frame time \endlink and similar parameters to adjust the required pause times
via the \link #argus_cfg configuration API methods\endlink.

An example implementation is shown in the @ref simple_example_app section.

#### Periodic Measurements

A more advanced way of operating the device is to leverage from a **Periodic
Interrupt Timer (PIT)** and invoke new measurement cycles periodically and
autonomously from its interrupt service routine.

The timer is implemented in the \link #argus_timer timer\endlink interface.
Instead of calling the #Argus_TriggerMeasurement function periodically from the
main thread, the measurement cycles are initiated via the
#Argus_StartMeasurementTimer method. The measurement cycles are restarted in an
autonomous way from the periodic interrupt with specified period.

Every time, a new raw measurement data set is ready, the measurement data ready
callback is invoked by the API to inform the main thread about the event.

Similar to the previous method, the #Argus_EvaluateData function must be called
before the data can be used. Note that not calling the function will lead to
measurements are not restarted before the evaluation method is called and the
data buffers is freed. In the same manner, a slow data evaluation or much user
code to delay the data evaluation method might decrease the measurement frame
rate.

An example implementation is shown in the @ref advanced_example_app section.
