# #############################################################################
# ###     Example for using the AFBR-S50 API with UART interface            ###
# #############################################################################
#
# Prepare your evaluation kit (w/ NXP MKL46z MCU) by flashing the UART binary
# to the device. Connect the OpenSDA USB port (NOT the one labeled with KL46Z)
# to your computer. Go to the Device/Binary folder install directory of you SDK
# (default: C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Binary) and copy
# the AFBR.S50.ExplorerApp.vX.X.X_KL46z_UART.bin (not the *_USB.*!!) file to
# the OpenSDA USB drive.
#
# After flashing, the device is ready to receive commands via the OpenSDA serial
# port. Go to your device manager to find out which COM port is assigned to the
# device. Type it to the "port" variable below, before starting the script.
#
# Use Python 3 to run the script. The script requires the pySerial module which
# might need to be installed.
# To install, run: "pip install pyserial"
# See: https://pyserial.readthedocs.io/en/latest/index.html
#
#
# The script sends configuration commands to set the data output mode to 1D data
# only and the frame rate to 5 Hz. After setting the configuration, the
# measurements are started and the data is extracted from the received data
# frames and printed to the console.
#
#
# Note: The CRC values are calculated manually and added before the frames are
# sent. You can use the online calculator from the following link w/
# CRC8_SAE_J1850_ZERO to obtain the CRC values for a frame:
# http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
#
# #############################################################################

import time
import serial

# input parameters
port = "COM4"
baudrate = 115200


class AFBR_S50:
    """
    Serial Communication Interface for the AFBR-S50 Device.
    Connects to the device via a UART interface.
    """

    ## SCI Start Byte
    start_byte = b"\x02"
    ## SCI Stop Byte
    stop_byte = b"\x03"
    ## SCI Escape Byte
    esc_byte = b"\x1B"
    ## SCI Acknowledge Command
    cmd_ack = 0x0A
    ## SCI Not-Acknowledge Command
    cmd_nak = 0x0B
    ## Serial Interface
    ser = None

    def __init__(self, port, baudrate):
        """!
        Initializes the class and opens a serial port w/
        "115200,8,N,1" serial settings and no timeout.

        @param port (str): The port number string, e.g. "COM1"
        @param baudrate (int): The baud rate in bauds per second, e.g. 115200
        """
        print("AFBR-S50: Open Serial Port " + port)
        self.ser = serial.Serial(port, baudrate)
        self.ser.timeout = 1.0  # seconds
        print("AFBR-S50: Serial Port is open " + port + ": " + str(self.ser.is_open))

        # discard old data
        if self.ser.inWaiting() > 0:
            self.ser.read(self.ser.inWaiting())

    def __del__(self):
        """!
        Deletes the class and closes the opened serial port.
        """
        self.ser.close()

    def write(self, tx: bytes):
        """!
        Sends a SCI message and waits for an optional answer and
        the mandatory acknowledge.

        If any answer is received, it is returned as bytearray.

        @param tx (bytes): The data message (incl. excape bytes) as byte array to be sent.
        @return Returns the received answer (ACK or NAK) as byte array. None if no answer was received.
        """
        print("Sending: " + tx.hex())
        self.ser.write(tx)
        return self.__wait_for_ack(tx[1])  # read acknowledge

    def __wait_for_ack(self, txcmd):
        """!
        Waits for an acknowledge signal for the specified command.
        If an answer is received before the acknowledge is received,
        the answer is returned as a bytearray.
        If no acknowledge or any other command is received, an
        exception is raised.
        @param txcmd (byte): The TX command byte to await an acknowledge for.
        @return Returns the received answer (ACK or NAK) as byte array. None if no answer was received.
        """
        answer = None

        while True:
            # Read until next stop byte and remove escape bytes
            rx = bytearray(self.ser.read_until(self.stop_byte))
            if len(rx) == 0:
                raise Exception("No data was read from the RX line.")

            if rx[0] != self.start_byte[0] or rx[-1] != self.stop_byte[0]:
                raise Exception("Invalid data frame received (start or stop byte missing).")

            rx = self.__remove_byte_stuffing(rx)

            # Extract command byte (first after start byte)
            rxcmd = rx[1]

            if rxcmd == txcmd:  # response received
                answer = rx

            # acknowledge signal received
            elif rxcmd == self.cmd_ack:
                ackcmd = rx[2]

                # acknowledge for the current command
                if ackcmd == txcmd:
                    return answer

                # acknowledge for any other command
                else:
                    raise Exception("Invalid ACK received")

            # not-acknowledge signal received
            elif rxcmd == self.cmd_nak:
                nakcmd = rx[2]

                # not-acknowledge for current command
                if nakcmd == txcmd:
                    raise Exception("NAK received")

                # not-acknowledge for any other command
                else:
                    raise Exception("Invalid NAK received")

    def __remove_byte_stuffing(self, rx: bytearray):
        """!
        Removes escape bytes from the incoming message if any
        @param rx (bytearray): The data message as byte array with escape bytes.
        """
        rxi = rx.split(self.esc_byte)
        rx = b""
        for i in range(1, len(rxi)):
            # invert byte after escape byte (also inverts start byte, but we don't care..)
            rxi[i][0] ^= 0xFF
        return rx.join(rxi)

    def __extract_1d_data(self, rx: bytearray):
        """!
        Extracts the 1D data values from the 1D data message.
        @param rx (bytearray): The 1D data message as byte array without escape bytes.
        @return Returns the read data as dictionary.
        """
        d = dict()

        # Extract Status:
        s = (rx[3] << 8) + rx[4]
        d["status"] = s if s < 0x8000 else s - 0x10000  # convert to signed 16-bit int

        # Extract Time Stamp
        t_sec = (rx[5] << 24) + (rx[6] << 16) + (rx[7] << 8) + rx[8]
        t_usec = (rx[9] << 8) + rx[10]
        d["timestamp"] = t_sec + t_usec * 16.0 / 1.0e6

        # Extract Range:
        r = (rx[13] << 16) + (rx[14] << 8) + rx[15]
        d["range"] = r / 16384.0  # convert from Q9.14

        # Extract Amplitude:
        a = (rx[16] << 8) + rx[17]
        d["amplitude"] = a / 16.0  # convert from UQ12.4

        # Extract Signal Quality:
        q = rx[18]
        d["signal quality"] = q

        return d

    def read_data(self):
        """!
        Reads the serial port and decodes the SCI data messages.
        Currently only 1D data messages are supported.
        If no data is pending to be read, the function immediately
        return with None. If other data than measurement data was read,
        the function returns with None.
        Otherwise it returns a dictionary with the extracted data values.
        @return Returns the read data as dictionary. None if no data has been read.
        """
        if self.ser.inWaiting() == 0:
            return None

        # Read until next stop byte and remove escape bytes
        rx = bytearray(self.ser.read_until(self.stop_byte))
        if len(rx) == 0:
            raise Exception("No data was read from the RX line.")

        if rx[0] != self.start_byte[0] or rx[-1] != self.stop_byte[0]:
            raise Exception("Invalid data frame received (start or stop byte missing).")

        rx = self.__remove_byte_stuffing(rx)

        # extract command byte (first after start byte)
        cmd = rx[1]

        if cmd == 0x06:  # Log Message
            print("Device Log: " + str(rx[8:-2]))

        elif cmd == 0xB6:  # 1D Data Set
            return self.__extract_1d_data(rx)

        else:  # Unknown or not handled here
            print("Received Unknown Data Frame: " + rx.hex())


if __name__ == "__main__":

    try:
        # Create a new instance and open a serial port connection to the device.
        s50 = AFBR_S50(port, baudrate)

        # Setting data output mode to 1D data only
        # The message is composed of:
        # [START][CMD][PARAM][CRC][STOP]
        # where:
        # [START] = 0x02; start byte
        # [CMD]   = 0x41; command byte: data streaming mode
        # [PARAM] = 0x07; parameter of command: 1d data streaming
        # [CRC]   = 0xF5; checksum: pre-calculated in online calculator
        # [STOP]  = 0x03; stop byte
        print("setting data output mode to 1d data only")
        s50.write(bytes.fromhex("02 41 07 F5 03"))

        # Setting frame time to 200000 µsec = 0x00030D40 µsec
        # The message is composed of:
        # [START][CMD][PARAM(0)]...[PARAM(N)][CRC][STOP]
        # where:
        # [START]    = 0x02; start byte
        # [CMD]      = 0x43; command byte: measurement frame time
        # [PARAM(x)] = 0x001BFC0D40; 4-bit parameter of command: 0x00 03 0D 40 w/ escape bytes
        # [CRC]      = 0x85; checksum: pre-calculated in online calculator
        # [STOP]     = 0x03; stop byte
        #
        # NOTE: the 0x03 byte must be escaped and inverted (i.e. use 0x1BFC instead of 0x03)
        #       The CRC is calculated on the original data, i.e. 0x43 00 03 0D 40 => 0x85
        print("setting frame rate to 5 Hz (i.e. frame time to 0.2 sec)")
        s50.write(bytes.fromhex("02 43 00 1B FC 0D 40 85 03"))

        # Starting measurements
        # [CMD] = 0x11; command byte: start timer based measurements
        print("starting measurements in timer based auto mode")
        s50.write(bytes.fromhex("02 11 D0 03"))

        # Read measurement data
        print("read measurement data")
        while True:
            d = s50.read_data()
            if d != None:
                print(
                    f"{d['timestamp']:10.6f} sec | "
                    + f"range: {d['range']:6.3f} m | "
                    + f"amplitude: {d['amplitude']:8.3f} | "
                    + f"signal quality: {d['signal quality']:3d} | "
                    + f"status: {d['status']:5d} |"
                )

            else:
                # do other stuff
                time.sleep(0.1)

    except KeyboardInterrupt:
        # Try to stop measurements
        # [CMD] = 0x12; command byte: stop timer based measurements
        print("stop measurements")
        s50.write(bytes.fromhex("02 12 F7 03"))