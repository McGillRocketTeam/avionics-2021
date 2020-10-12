/*
 * error codes:
 *     w: wake error
 *     q: get signal quality error
 *     s: sleep error
 *     u: too many sleep errors, moving on without sleeping
 *     v: iridium transmit error
 * 
 * success codes:
 *     x: wake
 *     r: signal quality
 *     t: sleep
 * 
 * error and success codes are always lowercase, signal quality values are always UPPERCASE
 * 
 * Iridium error codes:
 * 
   ISBD_SUCCESS 0
   ISBD_ALREADY_AWAKE 1
   ISBD_SERIAL_FAILURE 2
   ISBD_PROTOCOL_ERROR 3
   ISBD_CANCELLED 4
   ISBD_NO_MODEM_DETECTED 5
   ISBD_SBDIX_FATAL_ERROR 6
   ISBD_SENDRECEIVE_TIMEOUT 7
   ISBD_RX_OVERFLOW 8
   ISBD_REENTRANT 9
   ISBD_IS_ASLEEP 10
   ISBD_NO_SLEEP_PIN 11
   ISBD_NO_NETWORK 12
   ISBD_MSG_TOO_LONG 13

   Teensy 3.5 pinout: https://forum.pjrc.com/attachment.php?attachmentid=13444&d=1522315482&thumb=1
 */

#include "IridiumSBD.h"
#include "time.h"
#include <SPI.h>
#include <SD.h>

#define DESKTOP_DIAGNOSTICS true
#define SLEEP_PIN     30
#define GREEN         34
#define RED           31
#define YELLOW        35
#define BUZZER        9
#define SUCCESS_TONE  3000 // freq of tone upon successful transmission
#define rbserial Serial3 // pins RX/TX = 7/8

IridiumSBD modem(rbserial, SLEEP_PIN); // declare IridiumSBD object

unsigned long startTime;       // only send messages every 2 minutes
const int pollInterval = 25;   // poll signal strength every 60 seconds
int iteration = 0;
const int chipSelect = 15;     // microSD breakout SPI
const String fileName = "testapples.txt";
File dataFile;

void setup() {
  // LED and Buzzer outputs
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  noTone(BUZZER); // ensure buzzer is silent

  pinMode(chipSelect, OUTPUT); // sd card SPI

  // set SPI pins for Teensy SPI0 bus
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  
  toggleGreen();  // turn GREEN on, RED off
  startupSound(); // play startup sound for confirmation
  
  Serial.begin(115200); // data rate for desktop <-> teensy
  
  // if on desktop, wait for serial to connect
  //if (DESKTOP_DIAGNOSTICS) {
  
  Serial.println("Connection established :   Desktop <-> Teensy");

  // begin RockBlock serial connection
  rbserial.begin(19200); // data rate for iridium <-> teensy
  Serial.println("Connection established :   Iridium <-> Teensy");

  
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // low power profile

  // start the RockBlock, check that no errors occur
  Serial.println("Starting modem...");
  int status = modem.begin();
  if (status != ISBD_SUCCESS) {
    toggleRed();
    Serial.print("Begin failed: error ");
    Serial.println(status);
    if (status == ISBD_NO_MODEM_DETECTED) {
      Serial.println("No modem detected: check wiring.");
    }
    Serial.println("Program terminated.\n");
    delay(500UL);
    while(true) { // blink RED if stuck here
      errorToneBlink();
    }
  } else {
    toggleGreen();
    Serial.println("Modem Ready.\n");
  }

  startupSound(); // sound again to indicate RockBlock is active
  
  // start microSD card, check for errors
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (true) {
      errorToneBlink();
    }
  }
  Serial.println("card initialized.");

  dataFile = SD.open("walktest.txt", FILE_WRITE);
  if (SD.exists("walktest.txt")) {
    dataFile.println("resetted");
    Serial.println("file exists");
    dataFile.close();
  } else {
    Serial.println("file does not exist");
  }
  toggleGreen();

  // used to determine when to check signal quality
  startTime = millis();
}

void loop() {
  wakeModem(); // ensure modem is awake
  toggleYellow(); // yellow since we're waiting until loop is entered
  
  // char array stores error messages and signal quality measurements
  // we use one array for each message group to be sent (50 bytes max)
  int arrPosition = 1; // second char of each message, first is reserved :)
  int arrSize = 48; // max size is 50 bytes per credit used, use 48 to be safe
  char sigStrengths[arrSize] = {}; // when loop resets, clear the array
  
  // set first character in array to be iteration number
  sigStrengths[0] = (char) ('A' + iteration);
  int numErrors = 0;

  // setup SD card file
  dataFile = SD.open("walktest.txt", FILE_WRITE);
  if (dataFile) { // check for error
    dataFile.println("start of loop()");
    Serial.println("start of loop()");
  } else {
    while (true) {
      Serial.println("SD open file error");
      errorToneBlink();
    }
  }
  /*
   * loop and until the char array is almost filled, then
   * store signal strength values and error messages; 
   * 
   * each loop is structured as follows: wake, record signal quality, sleep.
   * if there are errors for each task, an error char is added to the array.
   * we tolerate up to 5 errors for waking/sleeping before hanging in a while or moving on.
   */
  while (arrPosition < arrSize-3) { // leave 3 chars for message transmission error codes
    if ((millis() - startTime) % (1000 * pollInterval) == 0) {
      int wakeStatus = wakeModem();
      while (wakeStatus != 0 && numErrors < 5) { // try to wake up to 5 times
        toggleRed();
        sigStrengths[arrPosition] = 'w';
        arrPosition++;
        Serial.print("Wake error ");
        Serial.println(numErrors);
        numErrors++;
        delay(10*1000); // wait 10 seconds
      }
      if (numErrors == 4) { // Iridium doesn't wake up, time to power off and reset
        dataFile.println(String(sigStrengths) + "  -- Iridium can't wake, wait for reset...");
        while (true) {
          Serial.println("Stuck in while, can't wake");
          tone(BUZZER, 440);  // activate buzzer to notify people
          blinkRed();
          noTone(BUZZER);
          delay(1000UL);
        }
      }
      //sigStrengths[arrPosition] = 'x';
      //arrPosition++;
      numErrors = 0; 
      toggleGreen(); // green now that we're moving forward in the program
      
      int sigQuality = getSignalQuality();
      if (sigQuality != -1) {
        toggleGreen();
        sigStrengths[arrPosition] = (char) (sigQuality + 'A');
        arrPosition++;
      } else {
        toggleRed();
        sigStrengths[arrPosition] = 'q';
        arrPosition++;
      }
      if (DESKTOP_DIAGNOSTICS) { // print signal quality if on laptop
        Serial.print("Signal quality: ");
        Serial.println(sigQuality);
      }

      // put RockBlock to sleep
      while (sleepModem() != 0 && numErrors < 5) {
        toggleRed();
        sigStrengths[arrPosition] = 's';
        arrPosition++;
        Serial.print("Sleep error ");
        Serial.println(numErrors);
        numErrors++;
        delay(10*1000); // wait 10 seconds then try again
      }
      if (numErrors == 4) { // if can't sleep, then move on since RB is awake
        sigStrengths[arrPosition] = 'u';
        arrPosition++;
        Serial.println("5 sleep errors reached");
      } else {
        //sigStrengths[arrPosition] = 't';
        //arrPosition++;
        Serial.println("Sleep successful");
        toggleYellow();
      }

      // debugging
      Serial.print("Array position: ");
      Serial.println(arrPosition);
      
      numErrors = 0;
      Serial.println(String(sigStrengths));
      Serial.println();
    }
  }

  // time to send the message! wake the modem first
  wakeModem();
  toggleGreen();
  
  boolean isSendSuccess = false;
  while (arrPosition < arrSize) { // we can try to send a few times
    int sendStatus = sendMessage(sigStrengths); // must be uncommented to send to email!!
    //int sendStatus = 0; // for debugging
    if (sendStatus == -1) {
      sigStrengths[arrPosition] = 'v'; // append error character to message
      arrPosition++;
      Serial.println("Unable to send, trying again...");
    } else if (sendStatus == 0) {
      isSendSuccess = true;
      Serial.println("Sent successfully!");
      break;
    }
  }

  // write data to SD as backup, play tone to alert user of message transmission (or failure)
  if (isSendSuccess) {
    dataFile.println(String(sigStrengths));
    for (int i = 0; i < 3; i++) { // play 3 long high pitched sounds to indicate success
      tone(BUZZER, SUCCESS_TONE);
      delay(1000);
      noTone(BUZZER);
      delay(1000);
    }
  } else {
    dataFile.println(String(sigStrengths) + " -- unable to send after 6 tries");
    for (int i = 0; i < 5; i++) { // play 5 shorter low pitched sounds to indicate failure
      tone(BUZZER, 100);
      delay(500);
      noTone(BUZZER);
      delay(500);
    }
  }

  iteration++;
  dataFile.close();
  Serial.println("end of loop()");
  return;
} 

// --- below are functions for waking/sleeping/getting signal/sending message --- //

/*
 * writes a String message to the file in the SD card
 * specified by 'fileName' string at the top of the file.
 * prints messages on serial to confirm (un)successful write.
 * returns int to indicate (un)successful write.
 */

/* function doesn't work properly
int fileWrite(String message) {
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(message);
    dataFile.close();
    Serial.println(message);
    return 0;
  } else {
    Serial.println("error opening file");
    return -1;
  }
}
*/

/*
 * puts RockBlock to sleep.
 * returns:
 *     0  successful sleep
 *    -1  error occurred
 */
int sleepModem() {
  int status = modem.sleep();
  if (status == ISBD_IS_ASLEEP) { // error code, RB is already sleeping
    toggleYellow();
    Serial.println("Modem is asleep.");
    return 0;
  }
  if (status != ISBD_SUCCESS) {
    toggleRed();
    Serial.print("Sleep failed: error ");
    Serial.println(status);
    return -1;
  } else {
    toggleYellow();
    Serial.println("Modem is asleep.");
    return 0;
  }
}

/*
 * wakes RockBlock.
 * returns:
 *     0  successful wake
 *    -1  error occurred
 */
int wakeModem() {
  int status = modem.begin();
  if (status == ISBD_ALREADY_AWAKE) {// error code, RB is already awake
    toggleGreen();
    Serial.println("Modem Ready.");
    return 0;
  }
  else if (status != ISBD_SUCCESS) {
      toggleRed();
      Serial.print("Begin failed: error ");
      Serial.println(status);
      if (status == ISBD_NO_MODEM_DETECTED) {
        Serial.println("No modem detected: check wiring.\n");
      }
      return -1;
    } else {
      toggleGreen();
      Serial.println("Modem Ready.");
      return 0;
    }
}

/*
 * gets the signal quality that the RockBlock sees.
 * returns:
 *    -1    error occurred
 *    else  int from 0-5 indicating signal strength
 */
int getSignalQuality() {
  int signalQuality = -1;
  int status = modem.getSignalQuality(signalQuality);
  if (status != ISBD_SUCCESS) {
    toggleRed();
    Serial.print("SignalQuality failed: error ");
    Serial.println(status);
    return -1;
  } else {
    toggleGreen();
    Serial.print("Signal Quality (0~5): ");
    Serial.println(signalQuality);
    return signalQuality;
  }
}

/*
 * sends a message (char array) to email.
 * input: char array with message
 * returns: 
 *      0   success
 *     -1   error occurred
 */
int sendMessage(char message[60]) {
  int status = modem.sendSBDText(message);
  if (status != ISBD_SUCCESS) {
    toggleRed();
    Serial.print("Transmission failed with error code ");
    Serial.println(status);
    Serial.println();
    return -1;
  } else {
    toggleGreen();
    Serial.println("Connected to Network.\n");
    return 0;
  }
}

#if DESKTOP_DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD * device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD * device, char c)
{
  Serial.write(c);
}
#endif


// ------- below are functions for LED control ------- //

// sets GREEN on, RED off, YELLOW off
void toggleGreen() {
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
}

// sets GREEN off, RED on, YELLOW off
void toggleRed() {
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, HIGH);
  digitalWrite(YELLOW, LOW);
}

// sets GREEN off, RED off, YELLOW on
void toggleYellow() {
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, HIGH);
}

void blinkRed() {
  toggleRed();
  digitalWrite(RED, HIGH);
  delay(500UL);
  digitalWrite(RED, LOW);
  delay(500UL);
}

void startupSound() {
  tone(BUZZER, 523);
  delay(100UL);
  noTone(BUZZER);
  delay(100UL);
  tone(BUZZER, 587);
  delay(100UL);
  noTone(BUZZER);
  delay(100UL);
  tone(BUZZER, 659);
  delay(100UL);
  noTone(BUZZER);
  delay(100UL);
  noTone(BUZZER);
}

void errorToneBlink() {
  tone(BUZZER, 440);
  blinkRed();
  noTone(BUZZER);
}
