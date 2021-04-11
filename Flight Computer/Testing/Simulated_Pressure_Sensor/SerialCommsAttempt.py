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


# USART3 on FC V2 uses "38400,8,N,1"
# change COM* as appropriate
# use 1 second timeout in case '\n' not received for readline
ser = serial.Serial()
ser.baudrate = 38400
#ser.port = 'COM5'
ser.port = 'COM11'
ser.timeout = 1 # 1 second

startValue = "0" # value received from microcontroller to send altitude
stopValue = "1" # value to terminate program 
threshold = 0.02 # time threshold for time.sleep() to adjust for program latency

while (True):
    try:
        ser.open()
        print("serial port opened, continuing...\n")
        break
    except:
        print("serial port not found, looping...\n")
        time.sleep(1) # sleep 1 second

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
        break # this aint working let's just bypass
    
        #line = str(ser.readline()) # read a '\n' terminated line
        #extracted = line[2:-5]     # convert to string and extract text from bytes
        print("waiting for start")
        print("bytes in rx buffer: " + str(ser.in_waiting))
        print("bytes in tx buffer: " + str(ser.out_waiting))
        
        if (ser.in_waiting != 0):
            line = str(ser.read(1)) # data in is of type 'bytes'
            print(line)
            if (line == startValue):
                print('leave start')
                break # break waiting loop, begin sending data :)
        time.sleep(0.5)
    
    
    while (True):
        print("bytes in rx: " + str(ser.in_waiting))
        #print("bytes in tx: " + str(ser.out_waiting))
        nextLine = next(csv_reader)
        print("nextLine = " + str(nextLine))
        deltaT = float(nextLine[0]) - float(currentLine[0])
        time.sleep(deltaT - threshold)
        #print("Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

        if (ser.in_waiting != 0):

            # minimize the number of bytes being read to miminize latency
            # the serial read takes a long time

            #line = str(ser.read(1)) # data in is of type 'bytes'
            line = str(ser.readline())
            if len(line) > 10:
                line = "invalid"
            
            print(line)
            #print(currentLine[1])
            #ser.write(bytes(str(currentLine[1]), 'utf-8'))
            #print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])

            if (line[-4] == startValue):
            #if (True):
                #ser.write(bytes(str(currentLine[1] + "\r\n"), 'utf-8'))
                toTransmit = str(currentLine[1]) + "\n"

                print(repr("to transmit: " + toTransmit))

                ser.write(bytes(toTransmit, 'utf-8'))
                print("sent: \t Time: " + currentLine[0] + "\t Pressure: " + currentLine[1])
                #print("sent")
            elif (line[-4] == stopValue):
                ser.close() # close the port
                break
        currentLine = nextLine # update currentLine
        
    
