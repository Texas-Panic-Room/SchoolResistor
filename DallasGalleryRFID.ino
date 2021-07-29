/**
* "Object Placement" Puzzle
*
* This puzzle requires the player to place one or more items in the
* correct location, at which point an RFID tag embedded in the object
* is detected by a sensor.
* When all sensors read the tag of the correct object, the puzzle is solved.
*
* Demonstrates:
* - SPI interface shared between several devices
* - RFID library
*/

// DEFINES
// Provides debugging information over serial connection if defined
#define DEBUG

// LIBRARIES
// Standard SPI library comes bundled with the Arduino IDE
#include <SPI.h>
// Download and install from https://github.com/miguelbalboa/rfid
#include <MFRC522.h>

// CONSTANTS
// The number of RFID readers
const byte numReaders = 2;
// Each reader has a unique Slave Select pin
const byte ssPins[] = {2, 3};
// They'll share the same reset pin
const byte resetPin = 9;
// This pin will be driven LOW to release a lock when puzzle is solved
const byte lockPin = A0;
// The sequence of NFC tag IDs required to solve the puzzle
//const String correctIDs[] = {"a76ef2eb", "b738f3eb", "1739f3eb", "474ff1eb", "476ef2eb"};
//NFC differentiator
const int correctIDs[] = {1, 2};


int sequenceID[numReaders] = {0,0};

// GLOBALS
// Initialise an array of MFRC522 instances representing each reader
MFRC522 mfrc522[numReaders];
// The tag IDs currently detected by each reader
String currentIDs[numReaders];


/**
 * Initialisation
 */
void setup() {

  // Initialise serial communications channel with the PC
  Serial.begin(9600);
  Serial.println(F("Serial communication started"));
  
  // Set the lock pin as output and secure the lock
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, HIGH);
  
  // Initialise the SPI bus
  SPI.begin();

  for (uint8_t i=0; i<numReaders; i++) {
  
    // Initialise the reader
    // Note that SPI pins on the reader must always be connected to certain
    // Arduino pins (on an Uno, MOSI=> pin11, MISO=> pin12, SCK=>pin13)
    // The Slave Select (SS) pin and reset pin can be assigned to any pin
    mfrc522[i].PCD_Init(ssPins[i], resetPin);
    
    // Set the gain to max - not sure this makes any difference...
    // mfrc522[i].PCD_SetAntennaGain(MFRC522::PCD_RxGain::RxGain_max);
    
    #ifdef DEBUG
    // Dump some debug information to the serial monitor
    Serial.print(F("Reader #"));
    Serial.print(i);
    Serial.print(F(" initialised on pin "));
    Serial.print(String(ssPins[i]));
    Serial.print(F(". Antenna strength: "));
    Serial.print(mfrc522[i].PCD_GetAntennaGain());
    Serial.print(F(". Version : "));
    mfrc522[i].PCD_DumpVersionToSerial();
    
    // Slight delay before activating next reader
    delay(100);
    #endif
  }
  
  #ifdef DEBUG
  Serial.println(F("--- END SETUP ---"));
  #endif
}

/**
 * Main loop
 */
void loop() {

  // Assume that the puzzle has been solved
  boolean puzzleSolved = false;

  // Assume that the tags have not changed since last reading
  boolean changedValue = false;

  //set sequence array to 0
 // int sequenceID[numReaders] = {0,0};
  
  // Loop through each reader
  for (uint8_t i=0; i<numReaders; i++) {

    // Initialise the sensor
    mfrc522[i].PCD_Init();
    
  }
    // String to hold the ID detected by each sensor
    String readRFID0 = "";
    String readRFID1 = "";
    String readRFID2 = "";
    String readRFID3 = "";
    String readRFID4 = "";
    
    // If the sensor detects a tag and is able to read it
    if(mfrc522[0].PICC_IsNewCardPresent() && mfrc522[0].PICC_ReadCardSerial()) {
      // Extract the ID from the tag
      readRFID0 = dump_byte_array(mfrc522[0].uid.uidByte, mfrc522[0].uid.size);
    }
    
    // If the current reading is different from the last known reading
    if(readRFID0 != currentIDs[0]){
      // Set the flag to show that the puzzle state has changed
      changedValue = true;
      // Update the stored value for this sensor
      currentIDs[0] = readRFID0;
      if (currentIDs[0] == "c76ff2eb" || currentIDs[0] == "c739f3eb" || currentIDs[0] == "476ef2eb" || currentIDs[0] == "474ff1eb" || currentIDs[0] == "076df2eb;") {
        sequenceID[0] = 1;
      }
      else {
        sequenceID[0] = 0;
      }
    }

    // If the sensor detects a tag and is able to read it
    if(mfrc522[1].PICC_IsNewCardPresent() && mfrc522[1].PICC_ReadCardSerial()) {
      // Extract the ID from the tag
      readRFID1 = dump_byte_array(mfrc522[1].uid.uidByte, mfrc522[1].uid.size);
    }
    
    // If the current reading is different from the last known reading
    if(readRFID1 != currentIDs[1]){
      // Set the flag to show that the puzzle state has changed
      changedValue = true;
      // Update the stored value for this sensor
      currentIDs[1] = readRFID1;
      if (currentIDs[1] == "276ef2eb" || currentIDs[1] == "776ff2eb" || currentIDs[1] == "f76cf2eb") {
        sequenceID[1] = 2;
      }
      else {
        sequenceID[1] = 0;
      }
    }
    
    if(sequenceID[numReaders] = correctIDs[numReaders]) {
      puzzleSolved = true;
    }

    // Halt PICC
    mfrc522[0].PICC_HaltA();
    // Stop encryption on PCD
    mfrc522[0].PCD_StopCrypto1(); 

    // Halt PICC
    mfrc522[1].PICC_HaltA();
    // Stop encryption on PCD
    mfrc522[1].PCD_StopCrypto1(); 

  #ifdef DEBUG
  // If the changedValue flag has been set, at least one sensor has changed
  if(changedValue){
    // Dump to serial the current state of all sensors
    for (uint8_t i=0; i<numReaders; i++) {
      Serial.print(F("Reader #"));
      Serial.print(String(i));
      Serial.print(F(" on Pin #"));
      Serial.print(String((ssPins[i])));
      Serial.print(F(" detected tag: "));
      Serial.println(currentIDs[i]);
    }
    Serial.println(F("---"));
    Serial.print("Current Array: ");
    for (uint8_t i=0; i<numReaders; i++) {
      Serial.print(sequenceID[i]);
      Serial.print(", ");
    }
    Serial.println("");
  }
  #endif

  // If the puzzleSolved flag is set, all sensors detected the correct ID
  if(puzzleSolved){
    onSolve();
  }
 
  // Add a short delay before next polling sensors
  //delay(100); 
}

/**
 * Called when correct puzzle solution has been entered
 */
void onSolve(){

  #ifdef DEBUG
  // Print debugging message
  Serial.println(F("Puzzle Solved!"));
  #endif
  
  // Release the lock
  digitalWrite(lockPin, LOW);

  while(true) {
    delay(1000);
  }
  
}

/**
 * Helper function to return a string ID from byte array
 */
String dump_byte_array(byte *buffer, byte bufferSize) {
  String read_rfid = "";
  for (byte i=0; i<bufferSize; i++) {
    read_rfid = read_rfid + String(buffer[i], HEX);
  }
  return read_rfid;
}
