# API Migration Guide {#migration_guide}

The following document describes the changes to the **AFBR-S50 API** interface
across the released versions. This is meant to assist developers in updating
their code to a newer API version.

The API is usually not changing much between the versions, however, some
features require to introduce breaking changes such that the user code needs to
be adopted too. This document mainly focusses on the breaking changes only.

If migration over multiple releases is required, please refer to each of the
corresponding sections below and apply the changes iteratively.

Please report any additional issues to the
[AFBR-S50 GitHub repository](https://github.com/broadcom/afbr-s50-api/issues).

# API Migration Guide (1.4.4 → 1.5.6) {#migration_guide_1_5}

The following document describes the changes to the **AFBR-S50 API** interface
from **v1.4.4** to **v1.5.6**. 

## Overview of Changes

The most changes to the last version of the **AFBR-S50 API, v1.5.6,** happened
under the hood. Thus, there are only very minor changes regarding the offset
calibration parameters to the API . 

## Introduction of distinct global range offsets for low and high power stages

To better compensate for the different range offsets of the low and high power
stages, a second global range offset value has been introduced. The new value is
only applied in the low power stage and thus does not affect the high power
stage. However, the API function not require two offset values and the offset 
table data structure has changed from a two-dimensional array to a
three-dimensional array.

Here are the changes in the API functions:
```c
// v1.4.4 range offset functions with a single offset value
status_t Argus_SetCalibrationGlobalRangeOffset(argus_hnd_t *hnd, q0_15_t value);
status_t Argus_GetCalibrationGlobalRangeOffset(argus_hnd_t *hnd, q0_15_t *value);

// v1.5.6 range offset functions with a two distinct offset values
status_t Argus_SetCalibrationGlobalRangeOffsets(argus_hnd_t *hnd, q0_15_t offset_low, q0_15_t offset_high);
status_t Argus_GetCalibrationGlobalRangeOffsets(argus_hnd_t *hnd, q0_15_t *offset_low, q0_15_t *offset_high);
```

Here are the corresponding changes in the data structure:
```c
// v1.4.4 range offset table data structure wrapping a two dimensional array
typedef struct argus_cal_offset_table_t
{
    q0_15_t Table[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];

} argus_cal_offset_table_t;

// v1.5.6 range offset table data structure wrapping a three dimensional array
typedef union argus_cal_offset_table_t
{
    struct
    {
        q0_15_t LowPower[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
        q0_15_t HighPower[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
    };

    q0_15_t Table[ARGUS_DCA_POWER_STAGE_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y];

} argus_cal_offset_table_t;
```

# API Migration Guide (1.3.5 → 1.4.4) {#migration_guide_1_4}

The following document describes the changes to the **AFBR-S50 API** interface
from **v1.3.5** to **v1.4.4**. 

Due to many improvements and new features in **v1.4.4**, such as multi-device
support, high-speed measurement modes and memory footprint reduction, the API
interface has changed significantly this time.

## Overview of Changes

The latest version of the **AFBR-S50 API, v1.4.4,** includes several significant
changes that improve the performance and functionality of the sensor. These
changes include:

-   **Memory Footprint Reduction**: The new version of the API reduces the
    memory footprint of the library by removing the second measurement mode from
    the API. This means that the ability to dynamically switch between Long and
    Short Range Modes has been removed. Although this feature has been removed,
    it has resulted in a more efficient and compact library. Still, the API is
    capable of switching between measurement modes, but the user must
    re-initialize the API or use a dedicated function to change the measurement
    mode. This change has also removed the measurement mode parameter from most
    of the API functions. Beneath reducing the memory footprint, this change
    also allowed the introduction of additional measurements modes such as the
    high-speed modes.

-   **High-Speed Modes**: The new version of the API includes additional
    measurement modes that are dedicated to high speed measurements. These new
    modes allow for measurement frame rate up to 3000 frames per second, making
    the sensor more versatile and applicable to a wider range of use cases.
    Please note that the maximum limit of the frame rate highly depends on the
    used microcontroller.

-   **Multi-Device Support**: The latest version of the API now allows
    simultaneous usage of multiple devices per microcontroller. This means that
    developers can now use multiple sensors with a single microcontroller, which
    can significantly reduce the cost and complexity of their applications. The
    multiple device act independently and share a single SPI and Timer
    interface. The API automatically handles the resource management within the
    API such that there is no need for the user to manage the resources. The
    only limitation is the requirement of a common frame rate if multiple
    devices are used with a single periodic interrupt timer.

## Removal of Dual-Measurement Modes And Adding of Advanced Measurement Modes

The main implication of the API change is the removal of the measurement mode
parameter in most of the API functions. In order to migrate to the new version,
simply remove the measurement mode parameter from each function call. Instead,
set the measurement mode at initialization time (via #Argus_InitMode) or use
the dedicated #Argus_SetMeasurementMode function to load the
desired measurement mode from ROM. All changes to the default parameters must be
done afterward.

**v1.3.5**: The measurement mode can be changed at any time via the
#Argus_SetMeasurementMode function. Other API calls must pass the
measurement mode parameter (`mode`).

```c
status = Argus_Init(hnd, SPI_SLAVE); // uses default (ARGUS_MODE_A)

argus_mode_t mode = ARGUS_MODE_B;
status = Argus_SetMeasurementMode(hnd, mode);
handle_error(status, "set measurement mode failed!");

status = Argus_SetConfigurationDFMMode(hnd, mode, DFM_MODE_OFF);
handle_error(status, "set DFM failed!");

status = Argus_SetConfigurationSmartPowerSaveEnabled(hnd, mode, false);
handle_error(status, "set smart power save mode failed!");

status = Argus_SetConfigurationFrameTime(hnd, 1000);
handle_error(status, "set frame time failed!");
```

**v1.4.4**: The new API must set the measurement mode
(via #Argus_SetMeasurementMode) as the first change as it reloads the
default parameters from ROM and overwrites all previous made changes. Note that
the measurement mode is loaded upon device initialization. Using
the #Argus_Init function will load the corresponding default measurement mode.
The #Argus_InitMode function is used to specify the measurement mode at
initialization time.

```c
// Initialize with default measurement mode
status = Argus_Init(hnd, SPI_SLAVE);

// Alternatively initialize with specified measurement mode
status = Argus_InitMode(hnd, SPI_SLAVE, ARGUS_MODE_HIGH_SPEED_SHORT_RANGE);

// Change the measurement mode after initialization
argus_mode_t mode = ARGUS_MODE_ARGUS_MODE_HIGH_SPEED_SHORT_RANGE;
status = Argus_SetMeasurementMode(hnd, mode);
handle_error(status, "set measurement mode failed!");

// Apply additional changes to the default parameters afterwards
status = Argus_SetConfigurationDFMMode(hnd, DFM_MODE_OFF);
handle_error(status, "set DFM failed!");

status = Argus_SetConfigurationSmartPowerSaveEnabled(hnd, false);
handle_error(status, "set smart power save failed!");

status = Argus_SetConfigurationFrameTime(hnd, 1000);
handle_error(status, "set frame time failed!");
```

Note that also the measurement mode enumeration has changed to describe the new
measurement modes better.

The old measurement mode enumeration is as follows:

```c
// v1.3.5 measurement mode enumeration
typedef enum
{
    /*! Measurement Mode A: Long Range Mode. */
    ARGUS_MODE_A

    /*! Measurement Mode B: Short Range Mode. */
    ARGUS_MODE_B

} argus_mode_t;
```

The new measurement mode enumeration is as follows. Note that new measurement
modes may be added in future releases.

```c
// v1.4.4 measurement mode enumeration
typedef enum
{
    /*! Measurement Mode: Short Range Mode. */
    ARGUS_MODE_SHORT_RANGE

    /*! Measurement Mode: Long Range Mode. */
    ARGUS_MODE_LONG_RANGE

    /*! Measurement Mode: High Speed Short Range Mode. */
    ARGUS_MODE_HIGH_SPEED_SHORT_RANGE

    /*! Measurement Mode: High Speed Long Range Mode. */
    ARGUS_MODE_HIGH_SPEED_LONG_RANGE

} argus_mode_t;
```

Note that there are additional API methods to change the measurement mode:

```c
argus_mode_t Argus_GetDefaultMeasurementMode(argus_module_version_t module);

status_t Argus_SetMeasurementMode(argus_hnd_t * hnd, argus_mode_t mode);

status_t Argus_ResetMeasurementMode(argus_hnd_t * hnd);

status_t Argus_GetMeasurementMode(argus_hnd_t * hnd, argus_mode_t * mode);
```

## Improved Measurement Ready Callback

The measurement ready callback has been updated to include the concrete handle
pointer (`argus_hnd_t*`) instead of an abstract pointer (`void*`). This removes
the need to cast from `void*` to the `argus_hnd_t*` when passing to 
the #Argus_EvaluateData method.

```c
// old callback signature with void * data as second parameter
typedef status_t (*argus_callback_t)(status_t status, void * data);

// new callback signature with argus_hnd_t * hnd as second parameter
typedef status_t (*argus_measurement_ready_callback_t)(status_t status,
                                                       argus_hnd_t * hnd);
```

In consequence, the #Argus_TriggerMeasurement and #Argus_StartMeasurementTimer
methods have changed accordingly:

```c
// old signature
status_t Argus_StartMeasurementTimer(argus_hnd_t * hnd, argus_callback_t cb);

// new signature
status_t Argus_StartMeasurementTimer(argus_hnd_t * hnd, argus_measurement_ready_callback_t cb);
```

```c
// old signature
status_t Argus_TriggerMeasurement(argus_hnd_t * hnd, argus_callback_t cb);

// new signature
status_t Argus_TriggerMeasurement(argus_hnd_t * hnd, argus_measurement_ready_callback_t cb);
```

## Improved Measurement Evaluate Function

The #Argus_EvaluateData function has been updated and the abstract `void * raw`
pointer has been remove. The API is handling the raw data buffer now internally
and the user does not need to handle the pointer any more.

```c
// old signature with void * raw as third parameter
status_t Argus_EvaluateData(argus_hnd_t * hnd, argus_results_t * res, void * raw);

// new clean signature without the void * raw as third parameter
status_t Argus_EvaluateData(argus_hnd_t * hnd, argus_results_t * res);
```

The requirement to call #Argus_EvaluateData exactly once per invocation of the
measurement ready callback is still present. The API will return an error
if #Argus_EvaluateData is called more than once. On the other hand, the API will
get stuck if #Argus_EvaluateData is not called at all. Please use the new
method #Argus_IsDataEvaluationPending to determine if raw data is available and
requires evaluation.

```c
bool Argus_IsDataEvaluationPending(argus_hnd_t * hnd);
```

Note: The #Argus_IsDataEvaluationPending does not replace the #Argus_GetStatus
function. Still, one needs to call the #Argus_GetStatus function to determine
if the measurement is ready in case of #Argus_TriggerMeasurement or to check
for timeout in case of running measurements via #Argus_StartMeasurementTimer`.
See also the API reference manual documentation of #Argus_GetStatus
and #Argus_IsDataEvaluationPending` functions for more information.

## Changed Data Types for Crosstalk and Offset Tables

The new data types `argus_cal_offset_table_t` and `argus_cal_xtalk_table_t` have
been introduced for passing offset and crosstalk tables in and out of the API.

Function affected are:

- #Argus_SetCalibrationCrosstalkVectorTable
- #Argus_GetCalibrationCrosstalkVectorTable
- #Argus_GetCrosstalkVectorTable_Callback
- #Argus_SetCalibrationPixelRangeOffsets
- #Argus_GetCalibrationPixelRangeOffsets
- #Argus_GetPixelRangeOffsets_Callback


Here is an example for the crosstalk related functions:
```c
// v1.3.5 crosstalk functions used 3-dimensional arrays
status_t Argus_SetCalibrationCrosstalkVectorTable(argus_hnd_t * hnd, argus_mode_t mode, xtalk_t value[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);

// v1.4.4 use dedicated struct containing that data
status_t Argus_SetCalibrationCrosstalkVectorTable(argus_hnd_t * hnd, argus_cal_xtalk_table_t const * value);
```

Here is an example for the offset related functions:

```c
// v1.3.5 range offset functions used 2-dimensional arrays
status_t Argus_SetCalibrationPixelRangeOffsets(argus_hnd_t * hnd, argus_mode_t mode, q0_15_t value[ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);

// v1.4.4 use dedicated struct containing that data
status_t Argus_SetCalibrationPixelRangeOffsets(argus_hnd_t * hnd, argus_cal_offset_table_t const * value);
```

## Advanced Debug Data Structure for Argus_EvaluateData

In order to obtain additional debug information without any impact on the
measurement performance, a new debug data structure (#argus_results_debug_t)
has been introduced. A new function, #Argus_EvaluateDataDebug, is added to the
API to receive an optional pointer to that structure. The debug data structure
is optional and can be set to `NULL` if not required. Please note that in that
case it is recommended to use the #Argus_EvaluateData function instead.

```c
// Evaluate data without generation of debug data
status_t Argus_EvaluateData(argus_hnd_t * hnd, argus_results_t * res);

// Evaluate data that generates additional debug data
status_t Argus_EvaluateDataDebug(argus_hnd_t * hnd, argus_results_t * res, argus_results_debug_t * dbg);
```

Please note that this debug information is usually not required and should only
be used if explicitly requested for debugging purposes.

## S2PI Hardware Abstraction Layer (HAL) Changes

In order to support multiple devices, the S2PI layer has been updated to receive
the SPI slave on every function. Further, two new functions are added that are
required for the API to prevent access conflicts between multiple devices.

### Changed HAL Functions in the S2PI layer

The following functions have an added `slave` parameter. If there is no need for
multiple device support, the parameter can be ignored (e.g. via `(void)slave;`).

```c
// Changed functions
status_t S2PI_GetStatus(s2pi_slave_t slave)
{
    (void)slave;
    // previous implementation without slave ...
}
status_t S2PI_CaptureGpioControl(s2pi_slave_t slave)
{
    (void)slave;
    // previous implementation without slave ...
}
status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave)
{
    (void)slave;
    // previous implementation without slave ...
}
status_t S2PI_Abort(s2pi_slave_t slave)
{
    (void)slave;
    // previous implementation without slave ...
}
```

### Added HAL Functions in the S2PI layer

The newly added functions in the `s2pi.h/c` module are #S2PI_TryGetMutex and
#S2PI_ReleaseMutex. If there is no need for multiple device support, the
function can be ignored and simply return #STATUS_OK.

```c
// Added functions
status_t S2PI_TryGetMutex(s2pi_slave_t slave)
{
    (void) slave;
    return STATUS_OK;
}
void S2PI_ReleaseMutex(s2pi_slave_t slave)
{
    (void) slave;
}
```

In case of multi device support, refer to the API reference documentation of
the #S2PI_TryGetMutex and #S2PI_ReleaseMutex functions for a reference
implementation of that mechanism.
