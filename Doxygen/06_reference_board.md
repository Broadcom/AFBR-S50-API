# AFBR-S50 Reference Board {#reference_board}

The **AFBR-S50 Reference Board** by **MikroElektronika** is a fully featured
hardware reference design that can be used to kick-start the integration of the
**AFBR-S50** sensors into the users application. The **AFBR-S50 Reference
Board** can be directly integrated into the application or used as a hardware
reference design to evaluate a custom design.

Official Link: https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board

@image html 6_1_reference_design.jpg "Fig. 6.1: The Reference Design by MikroElektronika." width=240px
@image latex 6_1_reference_design.jpg "Fig. 6.1: The Reference Design by MikroElektronika." width=0.33\textwidth

Highlight features of the **AFBR-S50 Reference Board** are

-   **2 CAN** connectors (GH Connector with 1.25mm pitch),
-   **UART** connector (GH Connector with 1.25mm pitch),
-   **USB** port for native USB connection or bootloader access (micro USB port),
-   **SWD Debug Probe** connector (10-pin header with 1.27 mm pitch),
-   **External Power Supply** (**J6**: `VEXT`) (2-pin header with 1.27 mm pitch)

The **AFBR-S50 Reference Board** is based on the
[Renesas R7FA4M2AD3CFL](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m2-100mhz-arm-cortex-m33-trustzone-high-integration-lowest-active-power-consumption)
(100 MHz **ARM® Cortex®-M33** ) chip.

@note The **AFBR-S50 Reference Board** is quite new and thus the corresponding
documentation and code projects are still under construction and will be
periodically updated. Please feel free to contribute and submit an issue or
pull-request on the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API) or reach
out to the Broadcom TOF Support Team at [support.TOF@broadcom.com](mailto:support.TOF@broadcom.com).

## Getting Started {#reference_board_getting_started}

Get started on using the board by flashing the desired firmware to the board
via the @ref reference_board_bootloader feature. Refer to the corresponding
application documentation on how to use it here: @ref apps

## Overview {#reference_board_overview}

### Powering the Board {#reference_board_power}

The following ways can be used to power the board:

-   Connect the USB connector See also: @ref reference_board_usb
-   Apply 5V at the `VEXT` pin header (**J6**). The jumper `PWR SEL` must be set
    to `VEXT`.
-   Apply 5V via a CAN connector. The jumper `PWR SEL` (**J4**) must be set to
    `CAN`. See @ref reference_board_can section for pin configuration.
-   Apply 5V via the UART connector. See @ref reference_board_uart section for
    pin configuration.

### USB Connector {#reference_board_usb}

The **AFBR-s50 Reference Board** features a Micro-USB port (**CN1**) that is
connected to the **Renesas RA4M2** USB pins. Thus, it can be used to make the
board an USB device as well as utilizing the built-in bootloader to flash new
firmware into the MCU.

#### Bootloader {#reference_board_bootloader}

The built-in bootloader is accessible via the micro-USB connector (**CN1**). In
order to boot in bootloader mode, a jumper must be set to connect pins 7 and 9
of the **J5** header (i.e. the **SWD** interface).

1.  Install
    [Renesas Flash Programmer](https://www.renesas.com/us/en/software-tool/renesas-flash-programmer-programming-gui)

2.  Disconnect the current power supply of the board.

3.  Connect pins 7 and 9 in **J5** with a jumper.
    @image html 6_2_bootloader_jumper_setting.png "Fig. 6.2: Connect pins 7 and 9 of J5 with a jumper to enable the bootloader." width=240px
    @image latex 6_2_bootloader_jumper_setting.png "Fig. 6.2: Connect pins 7 and 9 of J5 with a jumper to enable the bootloader." width=0.33\textwidth

4.  Connect the board via USB (**CN1**) to the PC.

5.  Open **Renesas Flash Programmer**

    1. Open new Project: `File` >> `New Project...`.
    2. Fill out the tabs:
        -   `Microcontroller`: `RA`
        -   `Project Name`: create your project name
        -   `Project Folder`: your project folder path
        -   `Communication Tool`: `COM Port` >> `Tool Details...`: your COM Port number
        @image html 6_3_bootloader_new_project.jpg "Fig. 6.3: Setup a new project in Renesas Flash Bootloader." width=480
        @image latex 6_3_bootloader_new_project.jpg "Fig. 6.3: Setup a new project in Renesas Flash Bootloader." width=0.5\textwidth

    3. Click `Connect`.
    4. Browse and select the binary file (`*.srec`, `*.bin`, `*.hex`, etc.) and Click `Start`.

6.  If flashing was successful, `operation completed` is displaying at the
    console.
    @image html 6_4_bootloader_successful.png "Fig. 6.4: Bootloader flashed device successfully." width=480
    @image latex 6_4_bootloader_successful.png "Fig. 6.4: Bootloader flashed device successfully." width=0.5\textwidth

7.  Disconnect the USB cable and don't forget to remove the jumper in order to
    reboot the board in normal operation mode.

@note The binary files are also available at
https://github.com/Broadcom/AFBR-S50-API/releases/latest.

### CAN-Bus Connector {#reference_board_can}

The board features two CAN-Bus connectors (**J1** and **J2**). These are 4 pin
connectors where `CANH` is on pin 2 and `CANL` is on pin 3. The connector can
also be used to power the **AFBR-S50 Reference Board** by applying 5V to pin 1.
Pin 4 functions as GND pin. Note that the `PWR SEL` jumper (**J4**) must be set
to `CAN`.

| J1 / J2   | USB-CAN |
| --------- | ------- |
| 1: 5V_CAN | N/C     |
| 2: CANH   | CANH    |
| 3: CANL   | CANL    |
| 4: GND    | GND     |

To connect the **AFBR-S50 Reference Board** via **CAN** interface to the PC, an
appropriate **CAN-to-USB** adapter is required (e.g. the
[USB to CAN Analyzer Adapter by SeeedStudio](https://www.seeedstudio.com/USB-CAN-Analyzer-p-2888.html)).

### Serial (UART) Connector {#reference_board_uart}

The board features a UART connection via the **J3** connector. It is a 6 pin
connector where TX is on pin 3 and RX is on pin 2. The connector can also be
used to power the **AFBR-S50 Reference Board** by applying 5V to pin 1. Pin 6
functions as GND pin.

Also see the @ref reference_board_power section for alternative ways on powering
the board.

To connect the **AFBR-S50 Reference Board** via **UART** interface to the PC, an
appropriate **Serial-to-USB** adapter is required (e.g. the
[FT232H Breakout Board by Adafruit](https://learn.adafruit.com/adafruit-ft232h-breakout)).

| AFBR-S50 Reference Board | Serial-to-USB FTDI Adapter |
| ------------------------ | -------------------------- |
| J3-1: 5V                 | 5V (optional)              |
| J3-2: RXD                | TX (e.g. D0 @ FT232H)      |
| J3-3: TXD                | RX (e.g. D1 @ FT232H)      |
| J3-4: n/c                | -                          |
| J3-5: n/c                | -                          |
| J3-6: GND                | GND                        |
