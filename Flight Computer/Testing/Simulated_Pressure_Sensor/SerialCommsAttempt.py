"""
Description:
This program is a first attempt at simulating a barometric
pressure sensor over UART serial port.

The program will read pressure values from a CSV file,
line-by-line, and transmit them over the serial port to
the other device.

Author: Jasper Yun
Date:   2021-04-09

Pseudocode:
- open CSV and prepare to read line by line
- open serial port and verify that port opened without errors
while (True):
    wait for FC to indicate it has turned on (some start message)
    (read serial buffer and wait)

get current time (need to be precise)
get one line from CSV and have time and pressure values
    -> this is the initial 
get next line from CSV
    -> this is one ahead, so we have the time difference
       and know how long before we need to read the next value
while (True):
    check serial input to see whether FC wants a measurement
    if yes:
        transmit current pressure value
    else if message == break:
        break

    wait the appropriate amount of time
    update initial measurement
    read next line
    
"""


import serial
import csv
import time

"""
Reads an entire line from serial port and extracts text.
Assumes that the serial messages will have \r\n terminations
"""
def readSerialExtractText(serialPort):
    line = str(serialPort.readline()) # data in is of type 'bytes'
    return (line[2:-5])
    


# USART3 on FC V2 uses "38400,8,N,1"
# change COM* as appropriate
# use 1 second timeout in case '\n' not received for readline
ser = serial.Serial()
ser.baudrate = 38400
ser.port = 'COM5'

while (True):
    try:
        ser.open()
        print("serial port opened, continuing...\n")
        break
    except:
        print("serial port not found, looping...\n")
        time.sleep(1) # sleep 1 second

threshold = 0.02 # time threshold for time.sleep() to adjust for program latency

# open CSV file: https://thispointer.com/python-read-a-csv-file-line-by-line-with-or-without-header/
# each line of form: [time, pressure, pressure (MSL), pressure (AGL)
# example: ['11.1', '85929', '1368.52', '-2.50525']
# 
with open ('PressureData.csv','r') as fp:
    csv_reader = csv.reader(fp, delimiter=',')
    next(csv_reader) # discard first row with headers

    currentLine = next(csv_reader) # initialize currentLine
    nextLine = currentLine

    # wait for start message from serial port
    while (True):
        #line = str(ser.readline()) # read a '\n' terminated line
        #extracted = line[2:-5]     # convert to string and extract text from bytes
        #if (extracted == "start"):
        print("waiting for start")
        if (readSerialExtractText(ser) == "start"):
            print('leave start')
            break # break waiting loop, begin sending data :)
    
    
    while (True):
        print(ser.in_waiting)
        nextLine = next(csv_reader)
        print("nextLine = " + str(nextLine))
        deltaT = float(nextLine[0]) - float(currentLine[0])
        time.sleep(deltaT - threshold)
        #print("Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

        if (ser.in_waiting != int(0)):

            # minimize the number of bytes being read to miminize latency
            # the serial read takes a long time
            line = str(ser.read(1)) # data in is of type 'bytes'
            #print(currentLine[1])
            #ser.write(bytes(str(currentLine[1]), 'utf-8'))
            #print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

            if (line[2:-5] == "0"):
            #if (True):
                #ser.write(bytes(str(currentLine[1] + "\r\n"), 'utf-8'))
                ser.write(bytes(str(currentLine[1]), 'utf-8'))
                print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])
                #print("sent")
            
        currentLine = nextLine # update currentLine
        print("end")
    
#ser.close()
