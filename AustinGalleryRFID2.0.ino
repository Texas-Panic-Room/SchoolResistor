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
const byte numReaders = 3;
// Each reader has a unique Slave Select pin
const byte ssPins[] = {2, 3, 4};
// They'll share the same reset pin
const byte resetPin = 9;
// This pin will be driven LOW to release a lock when puzzle is solved
const byte lockPin = A1;
const byte buzzer = A0;
bool buzz0;
bool buzz1;
bool buzzsolve;
// The sequence of NFC tag IDs required to solve the puzzle
//const String correctIDs[] = {"a76ef2eb", "b738f3eb", "1739f3eb", "474ff1eb", "476ef2eb"};

const byte correctIDs[] = {1,2, 3, 4, 5};

//photo sensor
const int photoPin = A2;
const int threshold =150; //photosensor threshold

//hall effect
const byte hall = A3;

//Artifact 0 Broken Atlas pin 2
String tag00 = "0545256a";
String tag01 = "85ca2f6a";
String tag02 = "65f0256a";
String tag03 = "5545256a";
String tag04 = "d61d7d41";
String tag05 = "86E37B41";
String tag06 = "26E37B41";
String tag07 = "56137B41";
String tag08 = "16117B41";
String tag09 = "F6127B41";

//Artifact 1 ELEPHANT pin 3
String tag10 = "05F72D6A";
String tag11 = "A5F62D6A";
String tag12 = "653D2D6A";
String tag13 = "B5372D6A";
String tag14 = "853E316A";
String tag15 = "46FF8441";
String tag16 = "C68F7741";
String tag17 = "36907741";
String tag18 = "F68E7741";
String tag19 = "568F7741";

//Artifact 2 
const String tag20 = "NA";
const String tag21 = "NA";
const String tag22 = "NA";
const String tag23 = "NA";
const String tag24 = "NA";
const String tag25 = "NA";
const String tag26 = "NA";
const String tag27 = "NA";
const String tag28 = "NA";
const String tag29 = "NA";

//Artifact 3
const String tag30 = "NA";
const String tag31 = "NA";
const String tag32 = "NA";
const String tag33 = "NA";
const String tag34 = "NA";
const String tag35 = "NA";
const String tag36 = "NA";
const String tag37 = "NA";
const String tag38 = "NA";
const String tag39 = "NA";

//Artifact 4 Extra Artifact pin 4
String tag40 = "56ee7941";
String tag41 = "b6ee7941";
String tag42 = "260e7b41";
String tag43 = "86e7b41";
String tag44 = "16e57b41";
String tag45 = "76e57b41";
String tag46 = "16187d41";
String tag47 = "76187d41";
String tag48 = "d6187d41";
String tag49 = "46f77d41";

//variable for disabling an artifact
const String none = "";



// GLOBALS
// Initialise an array of MFRC522 instances representing each reader
MFRC522 mfrc522[numReaders];
// The tag IDs currently detected by each reader
String currentIDs[5];
int sequenceID[5];


/**
 * Initialisation
 */
void setup() {

  // Initialise serial communications channel with the PC
  Serial.begin(9600);
  Serial.println(F("Serial communication started"));
  
  // Set the lock pin as output and secure the lock
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, LOW);
  pinMode(buzzer, OUTPUT);
  pinMode(hall, INPUT);
  pinMode(photoPin, INPUT);
  digitalWrite(buzzer,HIGH);
  delay(70);
  digitalWrite(buzzer, LOW);
  buzz0 = false;
  buzz1 = false;
  
  
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
  }
  
  #ifdef DEBUG
  Serial.println(F("--- END SETUP ---"));
  #endif
}

/**
 * Main loop
 */
void loop() {
 
  //Photosensor initialization
  int photo = analogRead(photoPin);
  int hallState = digitalRead(hall);

  // Assume that the puzzle has been solved
  boolean puzzleSolved = true;

  // Assume that the tags have not changed since last reading
  boolean changedValue = false;

  //Non RFID Sensors
   if (photo <= threshold) {
        sequenceID[3] = 4;
        // three short beeps

        if(!buzz1) {
          digitalWrite(buzzer, HIGH);
          delay(50);
          digitalWrite(buzzer, LOW);
          buzz1 = true;
          Serial.println("photo on");
          changedValue = true;
        }
      }
      else {
        sequenceID[3] = 0;
        buzz1 = false;
      }
      if (hallState == 0) {
        sequenceID[4] = 5;

        // four short beeps
        if(buzz0 == false){
          digitalWrite(buzzer, HIGH);
          delay(50);
          digitalWrite(buzzer, LOW);
          Serial.println("hall on");
          buzz0 = true;
          changedValue = true;
        }
      }
      else {
        sequenceID[4] = 0;
        buzz0 = false;
      } 
    
  // Loop through each reader
  for (uint8_t i=0; i<numReaders; i++) {

    // Initialise the sensor
    mfrc522[i].PCD_Init();
    
    // String to hold the ID detected by each sensor
    String readRFID[numReaders];
    
    // If the sensor detects a tag and is able to read it
    if(mfrc522[i].PICC_IsNewCardPresent() && mfrc522[i].PICC_ReadCardSerial()) {
      // Extract the ID from the tag
      readRFID[i] = dump_byte_array(mfrc522[i].uid.uidByte, mfrc522[i].uid.size);
    }
    
    // If the current reading is different from the last known reading of RFID0
    if(readRFID[i] != currentIDs[i]){
      // Set the flag to show that the puzzle state has changed
      changedValue = true;
      // Update the stored value for this sensor
      currentIDs[i] = readRFID[i];
      if (currentIDs[0] == "" ) {
        sequenceID[0] = 1;
      }
      else {
        sequenceID[0] = 0;
      }
      
      if (currentIDs[1] == "") {
        sequenceID[1] = 2;
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      else {
        sequenceID[1] = 0;
      }
      
      if (currentIDs[2] == tag40) { 
        //or function is causing a single RFID to trigger all of them...
        sequenceID[2] = 3;
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
        delay(50);
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      else if (currentIDs[2] == tag41) {
        sequenceID[2] = 3;
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
        delay(50);
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      else if (currentIDs[2] == tag42) {
        sequenceID[2] = 3;
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
        delay(50);
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      else if (currentIDs[2] == tag43) {
        sequenceID[2] = 3;
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
        delay(50);
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      else {
        sequenceID[2] = 0;
      }
    }


    // Halt PICC
    mfrc522[i].PICC_HaltA();
    // Stop encryption on PCD
    mfrc522[i].PCD_StopCrypto1(); 
  }
  for (uint8_t i=0; i<5; i++) {    
      // If the reading fails to match the correct ID for this sensor 
    if(sequenceID[i] != correctIDs[i]) {
      // The puzzle has not been solved
      puzzleSolved = false;
    }
  }

  #ifdef DEBUG
  // If the changedValue flag has been set, at least one sensor has changed
  if(changedValue){
    // Dump to serial the current state of all sensors
    for (uint8_t i=0; i<5; i++) {
      Serial.print(F("Reader #"));
      Serial.print(String(i));
      Serial.print(F(" detected tag: "));
      Serial.print(currentIDs[i]);
      Serial.print(F("---"));
      Serial.println(sequenceID[i]);
    }
    Serial.println(F("---"));
  }
  #endif

  // If the puzzleSolved flag is set, all sensors detected the correct ID
  if(puzzleSolved){
    onSolve();
    
    for (uint8_t i=0; i<5; i++) {
    sequenceID[i] = 0;
    }
    changedValue = true;
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
  if (!buzzsolve) {
    digitalWrite(buzzer, HIGH);
    delay (100);
    digitalWrite(buzzer, LOW);
    delay(50);
    digitalWrite(buzzer, HIGH);
    delay(50);
    digitalWrite(buzzer, LOW);
    delay(50);
    digitalWrite(buzzer, HIGH);
    delay(50);
    digitalWrite(buzzer, LOW);
    buzzsolve = true;
    digitalWrite(lockPin, HIGH);
    delay(10000);
    digitalWrite(lockPin, LOW);
    buzzsolve = false;
  }
  // Release the lock
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
