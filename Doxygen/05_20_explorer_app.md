# Explorer App {#explorer_app}

The **Evaluation Kit** runs with a demo application, the **Explorer App**. It
establishes an **USB** or **UART** connection to the **AFBR-S50 Explorer GUI**
that is running on a Windows PC. The **Explorer App** provides a serial
interface that allows the GUI to transfer configuration and calibration
parameters and receive measurement data.

The **Explorer App** hosts the **AFBR-S50 Core Library** and connects it to a a
**Serial Communication Interface (SCI)** to allow accessing the sensor from the
PC. The **SCI** implements an API that is equivalent to the **AFBR-S50 API**,
but accessible via a serial peripheral interface such as **USB** or **UART**.
The SCI is an communication protocol that can be ported to almost every serial
communication interface. The **Explorer App** functions as the communication
device, while an external host, e.g. the **AFBR-S50 Explorer GUI** running on a
Windows machine, is implemented as a controller.

The following chapter gives an overview about the architecture and
implementation of the **SCI** module in the **Explorer App**. After gaining a
basic understanding of the implementation, it should be an easy task to adopt
the **Explorer App** along with the interface to the user requirements and
create the host interface that can connect to the provided **SCI**.

@image html 5_1_explorerapp.png "Fig. 5.1: An overview of the Explorer App Architecture. The Explorer App is hosting the AFBR-S50 API on the one hand and a serial communication interface on the other hand to connect to an external host via serial peripheral like USB or UART. The external host would be a PC with running the AFBR-S50 Explorer GUI in case of the evaluation kit."
@image latex 5_1_explorerapp.png "Fig. 5.1: An overview of the Explorer App Architecture. The Explorer App is hosting the AFBR-S50 API on the one hand and a serial communication interface on the other hand to connect to an external host via serial peripheral like USB or UART. The external host would be a PC with running the AFBR-S50 Explorer GUI in case of the evaluation kit."

Note: from SDK version v1.4.4 onwards, the Explorer App is able to connect to
multiple devices at the same time. This can be either achieved by using multiple
evaluation kits and establishing an unique USB/UART connection to each board or
by using a multi-device adapter board and run multiple instances of the
**AFBR-S50 API** on a single MCU.

## Build And Run the Explorer App {#explorer_app_build}

The source files are located in `Sources/ExplorerApp` of the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API) and the
projects are located in `Project/<IDE>/AFBR_S50_ExplorerApp_<MCU>`, depending on
your target. The following target/IDE combinations are provided:

| MCU           | IDE           | Path                                                | Comment                                               |
| ------------- | ------------- | --------------------------------------------------- | ----------------------------------------------------- |
| NXP MKL46z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_ExplorerApp_KL46z` | Runs on **FRDM-KL46Z** evaluation board by **NXP**    |
| NXP MKL17z    | MCUXpressoIDE | `Projects/MCUXpressoIDE/AFBR_S50_ExplorerApp_KL17z` |                                                       |
| STM32 F401RE  | STM32CubeIDE  | `Projects/STM32CubeIDE/AFBR_S50_ExplorerApp_F401RE` | Runs on **NUCLEO-F401RE** evaluation board by **STM** |
| Renesas RA4M2 | e² Studio     | `Projects/e2studio/AFBR_S50_ExplorerApp_RA4M2`      | Runs on @ref reference_board by **MikroElektronika**  |

See also @ref gs_build for an overview of featured boards/MCUs/IDEs and how to
import and build the project in the corresponding IDE. Just follow the steps and
skip the steps that connect to a serial terminal. and connect via USB or UART
with a SCI controller (e.g. using the **AFBR-S50 Explorer GUI** or the
[Python example](@ref explorer_app_python_example)) instead.

## Serial Communication Interface {#explorer_app_sci}

### Introduction {#explorer_app_sci_intro}

The basic idea is to define simple commands (= 7-bit values) to access the
AFBR-S50 API. These commands are either transactions with or without data phase.
Usually, the commands with data phase are simple getter and setter commands,
e.g. "get distance" or "set frame rate".

Each data package starts by the identifier byte (= command byte), followed by a
predefined number of data bytes which may be zero. Finally, the package finishes
with a CRC byte to verify the data integrity. These byte sequences are called
data frames.

A new feature is the addressing mechanism which has been introduces with SDK
v1.4.4. The addressing mechanism allows to connect to multiple devices at the
same time. In order to distinguish between commands with and without addressing,
the 8th bit of the command byte is used. If the 8th bit is set, the command
will contain an addressing byte immediately after the command byte. Otherwise,
the command has not addressing byte and it is addressed to the default device.

The data frames are transmitted in different manner dependent on the underlying
hardware interface. However, at a higher level, the command or message layer
does not depend on the hardware interface. The communication happens between
various systems, whereby one of them needs to be the master. All other
participants are slaves which are controlled by the master. Depending on the
underlying hardware, there can be a single (e.g. UART) or multiple (e.g. SPI or
I2C) slaves. In case of the latter, the slaves are addressed by the master via
the corresponding hardware architecture.

Each command to a slave is acknowledged after successful execution. If any error
occurs, a not-acknowledge is invoked by the slave. Only a single command can be
sent to the slave at once, i.e. the slave has to (not-)acknowledge the command
before the master can send another one. A timeout can be implemented in the
master to check if the slave responds within a given time and is still alive or
if it is stuck in some invalid state.

### Architecture {#explorer_app_sci_architecture}

The architecture of the SCI (see [Fig. 5.2](@ref sci_layer_model)) consists of
several layer. Each is communicating on a specific level with its corresponding
counterpart.

The lowermost layer is the hardware layer which is the hardware abstraction
layer for the underlying hardware, e.g. UART or I2C. The hardware layer
transfers data bits over a physical link.

The second layer is the data link layer. It takes care about a reliable
transmission of data frames, i.e. a bunch of associated bytes. Therefore it is
responsible for putting together data by framing it with a start and a stop
byte. It also applies the byte stuffing and finally adds an CRC value to allow
the detection of invalid frames.

The third layer is the protocol or message layer. Its responsibility is to
transfer messages or commands via the data link layer. In order to achieve a
reliable connection, the handshaking is also implemented into this layer via
acknowledgment messages.

The last and uppermost layer is the Application or API layer. It provides high
level functionality to transfer application specific data.

@anchor sci_layer_model
@image html 5_2_sci_layer_model.png "Fig. 5.2: The SCI Layer Model."
@image latex 5_2_sci_layer_model.png "Fig. 5.2: The SCI Layer Model."

To sum up:

1. Hardware Layer: transfer bits over physical medium
2. Data Link Layer: data framing, byte stuffing and CRC
3. Message Layer: command, messages and handshaking
4. Application Layer: host application specific data

\see See also the OSI model, which was used as an reference:
https://en.wikipedia.org/wiki/OSI_model

### Hardware Layer {#explorer_app_hw_layer}

#### UART {#explorer_app_uart}

The UART interface support only point-to-point communications. It has an
independent line for sending as well as receiving data and the slave can
transmit data at any time without special actions from the master are required.
Therefore, this mode does not require an interrupt line to inform the master
about new data ready or error conditions. The data is just transmitted
immediately which means that the master must always listen to its Rx line.

The data framing is realized with byte stuffing. There are three special bytes,
the start, stop and escape bytes, that are used to determine the boundaries of a
data frame. In order to make the start and stop bytes unique and keep the full
data range per byte, the corresponding data bytes are inverted and escaped with
the escape byte.

In order to provide a handshaking mechanism, the slave acknowledges the
successful reception of an data frame (and the successful invocation of the
corresponding command) with a short ACK (= acknowledged) or NAK (=
not_acknowledged) message within a define time.

Furthermore, due to the independent TX line, the special feature of log and
error messages are supported by the UART protocol.

#### SPI {#explorer_app_spi}

The SPI interface support multiple slave mode via the chip select (CS) lines.
Data transfers can only be initiated by the master and thus an extra IRQ line is
used to give the slave a chance to call the masters attention to it. This is
however optional and the alternative method would be polling the status by the
master.

The data framing is realized via the CS. After the command byte, the data is
transferred either by on the MISO or MOSI for a read or write command
respectively.

The handshaking is implemented via the IRQ line. If an error occurs, the IRQ is
pulled to low. The master can now read the corresponding status in order to get
the root cause of the IRQ. Also the new measurement ready event is determined
via the IRQ. In addition, the acknowledgment of the successful reception and
execution of a command could be implemented via the interrupt; the master would
responsible for reading the acknowledge status.

#### I2C {#explorer_app_i2c}

The I2C interface supports multiple slave mode via the device address bytes.
Data transfers can only be initiated by the master and thus an extra IRQ line is
used to give the slave a chance to call the masters attention to it. This is
however optional and the alternative method would be polling the status by the
master.

The data framing is realized via the usual I2C protocol. Every frame is started
with the I2C start condition followed by the devices write address and then
master writes the command. In case of a write command, the data can follow
immediately. In case of an read command, the devices read address is placed
after an repeated start condition. The slave will put its data to the SDA line
afterwards.

Besides the already build-in acknowledgment mechanism form the I2C protocol, the
reception of an invalid data frame is advertised via the IRQ line. If an error
occurs, the IRQ is pulled to low. The master can now read the corresponding
status in order to get the root cause of the IRQ. Also the new measurement ready
event is determined via the IRQ. In addition, the In addition, the
acknowledgment of the successful reception and execution of a command could be
implemented via the interrupt; the master would responsible for reading the
acknowledge status.

### Command Protocols {#explorer_app_cmd_protocols}

#### Master-to-Slave Transfer {#explorer_app_cmd_m2s}

@image html 5_3_sci_master_to_slave.png "Fig. 5.3: The master to slave communication. The left side shows the UART hardware and the right side the version for SPI/I2C hardware."
@image latex 5_3_sci_master_to_slave.png "Fig. 5.3: The master to slave communication. The left side shows the UART hardware and the right side the version for SPI/I2C hardware."

In case of UART, the master simply send data via its Tx line. After processing,
the slave responds with an acknowledge or not-acknowledge signal on the masters
Rx line.

In case of SPI or I2C, the slave can not send data without the master initiating
the transfer. Thus, an additional IRQ is used to give the slave the chance to
call the masters attention. In case of no IRQ available, the master must poll
the slave on a regular basis. So after processing, the slave signals when he is
ready to send the acknowledge or not-acknowledge signal and afterwards the
master must initiate the transfer from slave to master.

#### Slave-to-Master Transfer {#explorer_app_cmd_s2m}

@image html 5_4_sci_slave_to_master.png "Fig. 5.4: The slave to master communication. The left side shows the UART hardware and the right side the version for SPI/I2C hardware."
@image latex 5_4_sci_slave_to_master.png "Fig. 5.4: The slave to master communication. The left side shows the UART hardware and the right side the version for SPI/I2C hardware."

In case of UART, the slave simply sends data via its Tx line. After receiving on
the master, the sent data is dismissed. If an error occurs, the master is
responsible to react accordingly, e.g. re-initiating the transfer by re-sending
the previous command.

In case of SPI or I2C, the slave can not send data without the master initiating
the transfer. Thus, an additional IRQ is used to give the slave the chance to
call the masters attention. In case of no IRQ available, the master must poll
the slave on a regular basis. The slave signals when he wants to send data via
the IRQ and afterwards the master must initiate the transfer from slave to
master.

### Command Byte Format {#explorer_app_cmd_format}

Every command message is identified by the first byte in a data frame. This byte
is an unique number that is mapped to a specified parameter/value/command.

The command identifier consists of 7-bit. The MSB is used to determine the
difference between an message with (MSB=1) or without (MSB=0) addressing byte.

@image html 5_5_sci_command_byte.png "Fig. 5.5: The command byte format."
@image latex 5_5_sci_command_byte.png "Fig. 5.5: The command byte format."

Reserved Command Bytes

| Byte  | Comment                           |
| ----- | --------------------------------- |
| 0x02  | ASCII: Start of text              |
| 0x03  | ASCII: End of text                |
| 0x1B  | ASCII: Escape                     |
| 0x21  | ASCII: ! - reserved for later use |
| 0x23  | ASCII: # - reserved for later use |
| 0x24  | ASCII: $ - reserved for later use |
| 0x3F  | ASCII: ? - reserved for later use |

### Addressing Mechanism {#explorer_app_cmd_addressing}

Since version 1.4.4 of the SDK, a new addressing mechanism has been introduced.
The new addressing mechanism is based on the command byte format. The MSB of the
command byte is used to determine if the message is addressed to a specific
device or not. If the MSB is set to 1, the message is addressed to a specific
device. In this case, the next byte in the data frame is the address byte. The
address byte is the byte that immediately follows the command byte. Some
commands do not require an address byte. In this case, the address byte is
omitted and the command byte is followed by the data bytes. In this case, the
MSB of the command byte is set to 0.

Sending a message without device address is considered equivalent to sending the
message with device address 0. If the device address is omitted, the message is
sent to the default device, usually the first device in the device list.

Thus, there are two kind of frames:

-   Extended Mode: MSB of command is set; the address byte follows the command byte.
-   Basic Mode: MSB of command is NOT set; no address byte is present.

### Command Types {#explorer_app_cmd_types}

There are several command types defined:

-   **Command (cmd)**: A data frame with a command byte that determines a simple
    command message that will invoke an action on the slave side. The commands
    are send from the master to the slave. The slave executes a corresponding
    function. Usually there is no data phase but in some cases there might be
    some (optional) function parameter.

-   **Setter (set)**: Command byte followed by a given sequence of data bytes
    representing the data that needs to be transferred from the master to the
    slave.

-   **Getter (get)**: A request from the master to read data from the slave. The
    actual data read phase depends a bit on the underlying hardware. While for
    SPI and I2C, the data is read directly in the context of the message, the
    response is sent as an autonomous message from the slave via its UART Tx
    line. Note that a get message is actually a command message that invokes the
    data transfer from the slave to the master in case of UART. Note that for
    some get messages there might be a short data phase for additional
    specification of the data to be read, e.g. an index number.

-   **Automatic / Autonomous Push (Auto / Push)**: A message or data transfer
    that is initiated by the slave. Depending on the hardware, the slave
    requests a data transfer by the data-ready pin or simply invokes the
    transfer in an autonomous way. The first is the case for I2C and SPI modes
    while the latter is the case for UART mode. This type of message is utilized
    by the slave to send log messages or establish an data stream of new
    measurement data to the master without the requirement of polling the line.

Thus a single command byte can have up to three different intentions.

-   **cmd**: Executing actions.
-   **get**: The usual getter for read-only data.
-   **set / get** The setter/getter combination e.g. for configuration
    parameters that can be applied to the slave and also read back.
-   **auto / push**: Data that is only send from the slave as needed without the
    request form the master, e.g. log messages or data streaming.

### Data Frame Format {#explorer_app_frame_format}

@image html 5_6_sci_data_frames_basic.png "Fig. 5.6: The basic data frame (without addressing byte) format for different hardware."
@image latex 5_6_sci_data_frames_basic.png "Fig. 5.6: The basic data frame (without addressing byte) format for different hardware."

@image html 5_7_sci_data_frames_ext.png "Fig. 5.7: The extended data frame (with addressing byte) format for different hardware."
@image latex 5_7_sci_data_frames_ext.png "Fig. 5.7: The extended data frame (with addressing byte) format for different hardware."

Each message or command that is sent over underlying hardware is put into a data
frame. A data frame is a sequence of bytes with variable length that contains
arbitrary data. Depending on the hardware, the boundaries of the data frames are
given in an unique way.

The first byte within a data frame is the command byte that uniquely determines
the purpose and thus the format of the data frame. The last byte within the data
frame is the security byte that contains a CRC value that guarantees the data
integrity of the receive frame. Between the command and CRC bytes, there is an
optional data phase of arbitrary length. The format and interpretation of the
data is determined by the command byte in the higher layers of the communication
stack and does not have any influence for the data framing. Note that the
addressing byte is equivalent to the data bytes and does not require any special
treatment here.

#### Byte Stuffing Algorithm {#explorer_app_byte_stuffing}

In case of SPI and I2C, the data framing is incorporated into the hardware
protocol itself, while for the UART interface, a software solution is
implemented. The SPI data frame is given by the chip select signal. Each data
byte that comes while the chip select line stays at low are combined to a single
frame until the chip select is released. The I2C protocol implements an embedded
start and stop signal that gives the boundaries of a single data frame.

The UART version implements the data framing in software via byte stuffing,
since there is no mechanism to detect the start of a new data package build into
the hardware interface. The ideas is to reserve some unique byte values to serve
as special control signal, e.g. start and stop bytes. Now, all bytes within a
start and a stop byte are interpreted as data bytes for a single data frame. In
order to not loose the full range of 256 valid values per data byte, an
additional escape signal is introduced. Whenever a byte value equal to the value
of start, stop or escape byte appears as a data byte within the current frame,
an escape byte is added prior to the byte in question in order to signal that
the following value is not an control signal but an ordinary data value. In
order to increase security further, escaped data bytes are inverted to make the
three control values unique.

Here are the control byte definitions:

| Name        | Byte | Comment              |
| ----------- | ---- | -------------------- |
| Start Flag  | 0x02 | ASCII: Start of text |
| Stop Flag   | 0x03 | ASCII: End of text   |
| Escape Flag | 0x1B | ASCII: Escape        |

@see https://eli.thegreenplace.net/2009/08/12/framing-in-serial-communications

##### Byte Stuffing Algorithm for Sending Data

The algorithm to send a data frame with byte stuffing:

1. Create a new data buffer with start byte included at the first position.
2. Append the data in the buffer (command and data) and add escape bytes on the
   fly, invert escaped bytes.
3. Calculate the CRC on that buffer, but ignore all escaped bytes, i.e resolve
   byte stuffing
4. Add the CRC value (and maybe escape signal) and the Stop byte.
5. Send the buffer.

##### Byte Stuffing Algorithm for Receiving Data

The algorithm to receive data with byte stuffing:

1. After detecting a start byte, start receiving all data until the subsequent
   stop byte.
2. Write the received bytes into a buffer, removing escape bytes and invert
   escaped bytes on the fly
3. Calculate and verify the CRC value.
4. Evaluate the data buffer by invoking the corresponding function belonging to
   the given command byte.

#### Error checking: 8-bit CRC {#explorer_app_crc}

In order to guarantee the data integrity of the received data frame, a CRC value
is added to each data frame.

CRC8_SAE_J1850_ZERO definitions:

| Name               | Byte |
| ------------------ | ---- |
| CRC Generator Byte | 0x1D |
| CRC Start Value    | 0x00 |

Refer to the following link to verify your implementation of the
_CRC8_SAE_J1850_ZERO_ algorithm:

\see http://www.sunshine2k.de/coding/javascript/crc/crc_js.html

## Command Definitions

### Overview

Go to:
\subpage explorer_app_cmd_overview

### Details

Go to:
\subpage explorer_app_cmd_details

## Python Example on Using the SCI Interface {#explorer_app_python_example}

Here is an example that sets some configuration parameters (i.e. data output =
1D data only; measurement frame rate = 5 Hz) and starts timer based
measurements. The range is extracted from the received data structure and
printed to the console.

The example is very basic though. The sent data frames are manually created
(i.e. data encoding, byte stuffing and CRC are hard coded). It ignores the
received Acknowledge/NotAcknowledge commands and cannot handle unexpected data
from the device.

The file runs with Python (3.6) and requires the Python serial module
(pySerial).

Please find the file in `[INSTALL_DIR]\Device\Examples\sci_python_example.py`
(default is
`C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Examples\sci_python_example.py`).

\include sci_python_example.py

\example sci_python_example.py
