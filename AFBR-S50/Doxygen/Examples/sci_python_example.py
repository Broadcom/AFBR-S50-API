# Example for using the AFBR-S50 API with UART interface
# #############################################################################
#
#
# Prepare your evaluation kit (w/ NXP MKL46z MCU) by flashing the UART binary
# to the device. Connect the OpenSDA USB port (NOT the one labeled with KL46Z)
# to your computer. Go to the Device/Binary folder install directory of you SDK
# (default: C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Binary) and copy
# the AFBR.S50.ExplorerApp.vX.X.X_KL46z_UART.srec (not the *_USB.*!!) file to
# the OpenSDA USB drive.
#
# After flashing, the device is ready to receive commands via the OpenSDA serial
# port. Go to your device manager to find out which COM port is assigned to the
# device. Type it to the "port" variable below, before starting the script.
#
# Use Python 3 to run the script. The script requires the pySerial module which
# might need to be installed. See: https://pyserial.readthedocs.io/en/latest/index.html
#
#
# The script sends configuration commands to set the data output mode to 1D data
# only and the frame rate to 5 Hz. After setting the configuration, the
# measurements are started and the range is extracted from the received data
# frames and printed to the console.
#
#
# Note: The CRC values are calculated manually and added before the frames are
# sent. You can use the online calculator from the following link w/
# CRC8_SAE_J1850_ZERO to obtain the CRC values for a frame:
# http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
#
# #############################################################################

import serial

# input parameters
port = "COM4"
sample_count = 100


# byte stuffing definitions
start_byte = b'\x02'
stop_byte = b'\x03'
esc_byte = b'\x1B'

def write(tx: bytes):

    print("Sending: " + tx.hex())
    ser.write(tx)

    return

def read():

    # read until next stop byte
    rx = bytearray(ser.read_until(stop_byte))

    # remove escape bytes if any
    rxi = rx.split(esc_byte)
    rx = b''
    for i in range(len(rxi)):
        rxi[i][0] ^= 0xFF # invert byte after escape byte (also inverts start byte, but we don't care..)
    rx = rx.join(rxi)

    # extract command byte (first after start byte)
    cmd = rx[1]

    # interpret commands
    if cmd == 0x0A: # Acknowledge
        print ("Acknowledged Command " + str(rx[2]))

    elif cmd == 0x0B: # Not-Acknowledge
        print ("Not-Acknowledged Command " + str(rx[2]) + " - Error: " + str((rx[3] << 8) + rx[4]))

    elif cmd == 0x06: # Log Message
        print("Device Log: " + str(rx[8:-2]))

    elif cmd == 0x36: # 1D Data Set
        # Extract Range:
        r = (rx[12] << 16) + (rx[13] << 8) + rx[14]
        r = r / 16384.0 # convert from Q9.14
        print ("Range[m]: " + str(r))

    else: # Unknown or not handled here
        print ("Received Unknown: " + rx.hex())

    return rx

# open serial port w/ "11500,8,N,1", no timeout
print("Open Serial Port " + port)
with serial.Serial(port, 115200) as ser:
    print("Serial Open " + port + ": " + str(ser.is_open))

    # discard old data
    ser.timeout = 0.1
    while len(ser.read(100)) > 0: pass
    ser.timeout = None

    # setting data output mode to 1D data only
    print("setting data output mode to 1d data only")
    write(bytes.fromhex('02 41 07 F5 03'))
    read()

    # setting frame time to 200000 µsec = 0x00030D40 µsec
    # NOTE: the 0x03 must be escaped and inverted (i.e. use 0x1BFC instead of 0x03)
    print("setting frame rate to 5 Hz (i.e. frame time to 0.2 sec)")
    write(bytes.fromhex('02 43 00 1B FC 0D 40 85 03'))
    read()

    # starting measurements
    print("starting measurements in timer based auto mode")
    write(bytes.fromhex('02 11 D0 03'))
    read()

    #read measurement data
    print("read measurement data")
    for i in range(sample_count):
        read()

    # starting measurements
    print("stop measurements")
    write(bytes.fromhex('02 12 F7 03'))
    read()

    ser.close()             # close port
