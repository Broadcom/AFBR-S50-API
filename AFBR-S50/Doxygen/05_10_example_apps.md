# Example Apps {#example_app}

These are basic demo applications that serve as starting point for the customers
own implementation. The example applications are kept simple and are documented
well within the source code in order to make it easy to follow and understand
their purpose. Each example application shows different use cases of the
**AFBR-S50 API**. The following apps are currently implemented:

-   The \ref simple_example_app demonstrates the most basic usage of the API. It
    triggers new measurement cycles from the main thread.

-   The \ref advanced_example_app demonstrates a more advanced usage of the API.
    It leverages from the periodic interrupt timer (PIT) module and triggers
    measurements from a periodic interrupt service routine outside of the scope
    of the main thread.

-   The \ref high_speed_example_app demonstration the high-framerate measurement
    modes of the AFBR-S50 API. Note that adequate hardware is required to
    achieve really high measurement rates.

# Simple Example {#simple_example_app}

The **Simple Example** demonstrates the most basic usage of the API. It triggers
new measurement cycles from the main thread.

The API calls the #Argus_TriggerMeasurement method in the main thread loop in
order to invoke another measurement cycle. The code is polling the module status
to wait for the measurement data to be ready. Once a new raw measurement data
set is ready, the #Argus_EvaluateData method is called to obtain meaningful
values like range and signal quality. The data values of the measurement cycle
is streamed via an **UART** connection.

In order to observe the values, open a connection via a terminal (e.g.
[Termite](https://www.compuphase.com/software_termite.htm)). Setup the
connection using 115200 bps, 8N1, no handshake:

| Baud rate  | Data Bits | Stop Bits | Parity | Flow Control |
| ---------- | --------- | --------- | ------ | ------------ |
| 115200 bps | 8         | 1         | none   | none         |

Range values will start to occur on the terminal as soon as the program starts
its execution.

Note that the project will provide additional information via the serial stream
upon program initialization. Especially the **HAL Self Test** might be executed
optionally in oder to verify the current **HAL** implementation.

In order to get started with the project, please read the \ref gs_build section.

The example source files are located in `Sources/ExampleApp`. The actual example
must be configured using the preprocessor constants in the main file. Refer the
documentation in the source code for more information.

The example projects are located in `Project/<IDE>/AFBR_S50_Example_<MCU>`,
depending on your target. The following target/IDE combinations are provided:

| MCU           | IDE           | Path                                            | Comment                                               |
| ------------- | ------------- | ----------------------------------------------- | ----------------------------------------------------- |
| NXP MKL46z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL46z` | Runs on **FRDM-KL46Z** evaluation board by **NXP**    |
| NXP MKL17z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL17z` |                                                       |
| STM32 F401RE  | STM32CubeIDE  | `Projects/STM32CubeIDE/AFBR_S50_Example_F401RE` | Runs on **NUCLEO-F401RE** evaluation board by **STM** |

See also @ref gs_build for an overview of featured boards/MCUs/IDEs.

<!-- \include 01_simple_example.c -->

# Advanced Example {#advanced_example_app}

The **Advanced Example** demonstrates a more advanced usage of the API. It
triggers new measurement cycles from a periodic interrupt autonomously.

The API calls the #Argus_StartMeasurementTimer method once upon initialization
from the main thread in order to start autonomous measurement cycles. This
required the optional **Periodic Interrupt Timer (PIT)** (see the 
\link #argus_timer timer HAL interface\endlink) periodically from its 
interrupt service routine.

The main thread loop is idle until the measurement data ready callback is
invoked. Once a new raw measurement data set is ready, the #Argus_EvaluateData
method is called to obtain meaningful values like range and signal quality. The
data values of the measurement cycle is streamed via an **UART** connection.

In order to observe the values, open a connection via a terminal (e.g.
[Termite](https://www.compuphase.com/software_termite.htm)). Setup the
connection using 115200 bps, 8N1, no handshake:

| Baud rate  | Data Bits | Stop Bits | Parity | Flow Control |
| ---------- | --------- | --------- | ------ | ------------ |
| 115200 bps | 8         | 1         | none   | none         |

Range values will start to occur on the terminal as soon as the program starts
its execution.

Note that the project will provide additional information via the serial stream
upon program initialization. Especially the **HAL Self Test** might be executed
optionally in oder to verify the current **HAL** implementation.

In order to get started with the project, please read the \ref gs_build section.

The example source files are located in `Sources/ExampleApp`. The actual example
must be configured using the preprocessor constants in the main file. Refer the
documentation in the source code for more information.

The example projects are located in `Project/<IDE>/AFBR_S50_Example_<MCU>`,
depending on your target. The following target/IDE combinations are provided:

| MCU           | IDE           | Path                                            | Comment                                               |
| ------------- | ------------- | ----------------------------------------------- | ----------------------------------------------------- |
| NXP MKL46z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL46z` | Runs on **FRDM-KL46Z** evaluation board by **NXP**    |
| NXP MKL17z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL17z` |                                                       |
| STM32 F401RE  | STM32CubeIDE  | `Projects/STM32CubeIDE/AFBR_S50_Example_F401RE` | Runs on **NUCLEO-F401RE** evaluation board by **STM** |

See also @ref gs_build for an overview of featured boards/MCUs/IDEs.

<!-- \include 02_advanced_example.c -->

# High-Speed Example {#high_speed_example_app}

The **High-Speed Example** demonstrates the fast measurement capabilities of the
API. It is based in the @ref advanced_example_app and utilizes the **AFBR-S50**
configuration API to achieve up to **1000 measurements-per-second**.

These configuration parameters needs to be changed to prepare the device for
**High-Speed Mode**:

| Feature                   | Value      | API Call                                     | Comment                                                              |
| ------------------------- | ---------- | -------------------------------------------- | -------------------------------------------------------------------- |
| Dual-Frequency Mode (DFM) | disabled   | #Argus_SetConfigurationDFMMode               | See also @ref argus_dfm for more information on the DFM feature.     |
| Smart-Power-Save (SPS)    | disabled   | #Argus_SetConfigurationSmartPowerSaveEnabled |                                                                      |
| Frame Rate (Frame Time)   | 1 ms/frame | #Argus_SetConfigurationFrameTime             | Frame Rate is actually set in Frame Time which is the inverse value. |

In additional to the changes above, the `print_results` method in the `main.c`
file is stripped down to the essential values, which are the time elapsed since
the previous frame in milliseconds and the range value in millimeter. If more
data is sent via UART, the baud rate of the UART must be increased in order to
prevent the UART from slowing down the program flow and thus the measurement
execution.

In order to observe the values, open a connection via a terminal (e.g.
[Termite](https://www.compuphase.com/software_termite.htm)). Setup the
connection using 115200 bps, 8N1, no handshake:

| Baud rate  | Data Bits | Stop Bits | Parity | Flow Control |
| ---------- | --------- | --------- | ------ | ------------ |
| 115200 bps | 8         | 1         | none   | none         |

Range values will start to occur on the terminal as soon as the program starts
its execution.

Note that the project will provide additional information via the serial stream
upon program initialization. Especially the **HAL Self Test** might be executed
optionally in oder to verify the current **HAL** implementation.

In order to get started with the project, please read the \ref gs_build section.

The example source files are located in `Sources/ExampleApp`. The actual example
must be configured using the preprocessor constants in the main file. Refer the
documentation in the source code for more information.

The example projects are located in `Project/<IDE>/AFBR_S50_Example_<MCU>`,
depending on your target. The following target/IDE combinations are provided:

| MCU           | IDE           | Path                                            | Comment                                               |
| ------------- | ------------- | ----------------------------------------------- | ----------------------------------------------------- |
| NXP MKL46z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL46z` | Runs on **FRDM-KL46Z** evaluation board by **NXP**    |
| NXP MKL17z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_Example_KL17z` |                                                       |
| STM32 F401RE  | STM32CubeIDE  | `Projects/STM32CubeIDE/AFBR_S50_Example_F401RE` | Runs on **NUCLEO-F401RE** evaluation board by **STM** |

See also @ref gs_build for an overview of featured boards/MCUs/IDEs.

<!-- \include 02_advanced_example.c -->

TODO

---

\example 01_simple_example.c

\example 02_advanced_example.c
