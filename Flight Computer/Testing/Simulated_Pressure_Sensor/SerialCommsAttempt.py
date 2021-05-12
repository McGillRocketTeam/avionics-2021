"""
Description:
This program is a first attempt at simulating a barometric
pressure sensor over UART serial port.

The program will read pressure/temperature values from a CSV file,
line-by-line, and transmit them over the serial port to
the other device.

Dependencies:
    - CSV
    - PySerial
    - Time

Pseudocode:
- open serial port and verify that port opened without errors
- open CSV and prepare to read line by line

get one line from CSV and have time and pressure values
    -> this is the initial 
get next line from CSV
    -> this is one ahead, so we have the time difference
       and know how long before we need to read the next value
while (True):
    check serial input to see whether FC wants a measurement
    if yes:
        transmit pressure or temperature based on received value
    else if message == break:
        break

    wait the appropriate amount of time
    update initial line with next line
    read next line
    
"""

import serial
import csv
import time    

# USART3 on FC V2 uses "38400,8,N,1"
# change COM port as appropriate
# use 1 second timeout in case '\n' not received for readline
ser = serial.Serial()
ser.baudrate = 38400
ser.timeout = 1 # 1 second
ser.port = 'COM8'

sendAlt = "0"  # value received from microcontroller to send altitude
sendTemp = "1" # value to send temp
stopValue = "2" # value to terminate program 
threshold = 0.1 # time threshold for time.sleep() to adjust for program latency

dataFile = "PressureTempData.csv" # contains pressure and temperature data

# attempt to open serial port
while (True):
    try:
        ser.open()
        print("serial port opened, continuing...\n")
        break
    except:
        print("serial port not found, looping...\n")
        time.sleep(1) # sleep 1 second

# open CSV file: https://thispointer.com/python-read-a-csv-file-line-by-line-with-or-without-header/
# each line of form: [time, pressure, pressure (MSL), pressure (AGL), temp (C)
# example: ['11.1', '85929', '1368.52', '-2.50525', '46.03']
# for editability, use variable to indicate which data we want to send:
altDataPosition = 1   # second element in list
TempDataPosition = -1 # last element in list

with open (dataFile,'r') as fp:
    csv_reader = csv.reader(fp, delimiter=',')
    next(csv_reader) # discard first row of row headers

    currentLine = next(csv_reader) # initialize currentLine
    nextLine = currentLine

    while (True):
        # print("bytes in rx: " + str(ser.in_waiting))   # print buffer bytes for debug
        #print("bytes in tx: " + str(ser.out_waiting))
        nextLine = next(csv_reader)
        # print("nextLine = " + str(nextLine))
        # deltaT = float(nextLine[0]) - float(currentLine[0])
        deltaT = 0.2
        time.sleep(deltaT - threshold)
        #print("Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

        if (ser.in_waiting != 0):

            # minimize the number of bytes being read to miminize latency
            # the serial read takes a long time
            #line = str(ser.read(1)) # data in is of type 'bytes'

            # readline is quite slow...not sure how to work around that yet
            line = str(ser.readline())
            if len(line) > 10: # debug messages printed on uart3 will be ignored
                line = "invalid"
            
            print(line)
            
            if (line[-4] == sendAlt):
                toTransmit = str(currentLine[altDataPosition]) + "\n"

                #print(repr("to transmit: " + toTransmit)) # check transmitted message

                ser.write(bytes(toTransmit, 'utf-8'))
                print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

            elif (line[-4] == sendTemp):
                toTransmit = str(currentLine[TempDataPosition]) + "\n"

                #print(repr("to transmit: " + toTransmit)) # check transmitted message

                ser.write(bytes(toTransmit, 'utf-8'))
                print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

            elif (line[-4] == stopValue):
                ser.close() # close the port
                break
            
        currentLine = nextLine # update currentLine
        
    
