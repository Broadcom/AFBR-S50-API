# Demo Applications {#apps}

The following demo applications are provided with the **AFBR-S50 API** on order
to present its usage:

-   \subpage example_app

    These are basic demo applications that serve as starting point for the
    customers own implementation. The example applications are kept simple and
    are documented well within the source code in order to make it easy to
    follow and understand their purpose. Each example application shows
    different use cases of the **AFBR-S50 API**. The following apps are
    currently implemented:

    -   The \ref simple_example_app demonstrates the most basic usage of the
        API. It triggers new measurement cycles from the main thread.

    -   The \ref advanced_example_app demonstrates a more advanced usage of the
        API. It leverages from the periodic interrupt timer (PIT) module and
        triggers measurements from a periodic interrupt service routine outside
        of the scope of the main thread.

    -   The \ref high_speed_example_app demonstration the high-framerate
        measurement modes of the AFBR-S50 API. Note that adequate hardware is
        required to achieve really high measurement rates.

    -   The \ref multi_device_example_app demonstration of the usage of multiple
        devices from a single MCU. Note that adequate hardware is required to
        talk to multiple devices.

-   \subpage explorer_app

    The **Explorer Application** is a full featured implementation of the API
    that is used in the **AFBR-S50 Evaluation Kits**. The **Explorer
    Application** implements a sophisticated serial communication interface that
    publishes the full **AFBR-S50 API** via **USB** or **UART** interfaces. The
    **AFBR-S50 Explorer GUI** that runs on a Windows machines connects to the
    **Explorer Application**.

-   \subpage can_app

    The \ref reference_board by **MikroElektronika** contains a **CAN-Bus**
    interface which is presented with the **CAN-Bus Application**. The
    application runs on the featured **Renesas RA4M2** **Cortex-M33** processor
    and implements a basic **CAN Interface** that can be easily enhanced and
    adopted by the user.

