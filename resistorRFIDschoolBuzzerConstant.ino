#include <MFRC522.h>

const byte numReaders = 5;
//Input pin for each reader
const byte ssPins[] = {2,3,4,5,6};
//All share same reset pin
const byte resetPin = 9;
//6th "cdd79a4f",
// In order of color: yellow, purple, green, brown, orange
const String correctIDs[] = {"adcf9a4f", "dd89a4f","2dd09a4f", "edcf9a4f", "8dd79a4f"}; // currently resistor set 1 APR
//IMPORTANT: back up IDs
//IMPORTANT CHANGE THIS NUMBER WHEN YOU ADD OR REMOVE CARD IDS.
const int numyellow = 1, numpurple = 1, numgreen = 1, numbrown = 1, numorange = 1;
const String yellowIDs[] = {"0ddf9a4f"};
const String purpleIDs[] = {"4ddf9a4f"};
const String greenIDs[] = {"8ddf9a4f"};
const String brownIDs[] = {"cddf9a4f"};
const String orangeIDs[] = {"0de09a4f"};

const int buzzer = 8;

int outputPin = 7;

MFRC522 mfrc522[numReaders];

String currentIDs[numReaders];

String dump_byte_array(byte *buffer, byte bufferSize) {
  String read_rfid = "";
  for (byte i = 0; i < bufferSize; i++) {
    read_rfid = read_rfid + String(buffer[i], HEX);
  }
  return read_rfid;
}

int current_state = 0;

void setup() {
  // Initialise the SPI bus
  Serial.begin(9600);
  Serial.println("Start");
  // put your setup code here, to run once:
  pinMode(outputPin, OUTPUT);

  SPI.begin();
  for(uint8_t i = 0; i < numReaders; i++){
    //Assign pin and reset to RFID reader
    mfrc522[i].PCD_Init(ssPins[i], resetPin);

    delay(100);
  }
  
  delay(1000);
  Serial.println("Setup done");
  digitalWrite(outputPin, LOW);
}

void loop(){
  // Assume that the rfids tags are at the right readers
  boolean rfidSolved = true;

  // Assume that the tags have not changed since last reading
  boolean changedValue = false;

  // Loop through each reader
  for (uint8_t i = 0; i < numReaders; i++) {
    // Initialise the sensor
    mfrc522[i].PCD_Init();

    // String to hold the ID detected by each sensor
    String readRFID = "";

    // If the sensor detects a tag and is able to read it
    if (mfrc522[i].PICC_IsNewCardPresent() && mfrc522[i].PICC_ReadCardSerial()) {
      // Extract the ID from the tag
      readRFID = dump_byte_array(mfrc522[i].uid.uidByte, mfrc522[i].uid.size);
      Serial.println(readRFID);
    }

    if (readRFID != currentIDs[i]) {
      // Set the flag to show that the rfid state has changed
      changedValue = true;
      // Update the stored value for this sensor
      currentIDs[i] = readRFID;
    }

    //Ignore no cards read
        if(readRFID != ""){
          //Play a sound when card is read
          tone(buzzer, 1000);
          delay(500);
          noTone(buzzer);
        }
  }

  
  
  for(int i = 0; i < numReaders; i++){
    if(currentIDs[i] != correctIDs[i]){
      rfidSolved = false;
    }
  }

  if(rfidSolved){
    digitalWrite(outputPin, HIGH);
    Serial.println("Open");
    tone(buzzer, 1500);
    delay(100);
    noTone(buzzer);
    delay(50);
    tone(buzzer, 1500);
    delay(50);
    noTone(buzzer);
    delay(50);
    tone(buzzer, 1500);
    delay(50);
    noTone(buzzer);
    delay(50);
    tone(buzzer, 1500);
    delay(50);
    noTone(buzzer);
    delay(50);
  }
}
