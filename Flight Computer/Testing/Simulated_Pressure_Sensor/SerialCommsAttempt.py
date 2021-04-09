"""
Description:
This program is a first attempt at simulating a barometric
pressure sensor over UART serial port.

The program will read pressure values from a CSV file,
line-by-line, and transmit them over the serial port to
the other device.

Author: Jasper Yun
Date:   2021-04-09

"""


import serial

# USART3 on FC V2 uses "38400,8,N,1"
# change COM* as appropriate
# use 1 second timeout in case '\n' not received for readline
ser = serial.Serial('COM5', 38400,timeout=1)
#ser.close()
while (True):
    x = ser.read()  # read one byte
    s = ser.read(10) # read up to 10 bytes
    line = ser.readline() # read a '\n' terminated line

    print(x)
    print(s)
    print(line)
