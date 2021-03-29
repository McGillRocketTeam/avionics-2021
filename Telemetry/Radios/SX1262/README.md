# SX1262 Breakout Interfacing

This library is meant to be used with an Arduino/Teensy microcontroller.

### Structure:
##### Library:
- In **sx1262.h**, we specify the SX1262 class, the class functions and some macro and typedefs which are used to interact with the SX1262 transceiver.
- In **sx1262.cpp**, we specify the functions and what they do.
##### Examples:
- **SX1262_TestBuffer**: Used to check if interfacing with the device is working, it simply writes and reads the from the data buffer.
- **Simple_SX1262_RX**: Used to receive LoRa packets.
- **Simple_SX1262_TX**: Used to transmit LoRa packets.
