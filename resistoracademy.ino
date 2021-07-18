/* This code is to be used with KY-024 Hall effect sensor
 * It displays both Analog and Digital values given by the sensor
 * Refer to www.surtrtech.com for more details
 */

#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

#define hall0 2          //A0 used with analog output, D2 with digital output
#define hall1 3
#define hall2 4
#define hall3 5
#define hall4 6
#define hall5 7

#define maglock 12

int halls[6] = {hall0, hall1, hall2, hall3, hall4, hall5};
int hallsLen = arr_len(halls);

bool Val0=false,Val1=false,Val2=false,Val3=false,Val4=false,Val5=false;          //Here you can store both values, the Val2 can be boolean
bool unlock = false;


void setup() {
  Serial.begin(9600);
  pinMode(hall0,INPUT);
  pinMode(hall1,INPUT);
  pinMode(hall2,INPUT);
  pinMode(hall3,INPUT);
  pinMode(hall4,INPUT);
  pinMode(hall5,INPUT);

  pinMode(maglock, OUTPUT);

}

void loop() {
  
for(int i = 0; i < hallsLen; i++){
    Serial.print("Hall");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(digitalRead(halls[i]));
  }

   Serial.print("-----------------------------");

   delay(1000);

   if(!Val0 && !Val1 && !Val2 && !Val3 && !Val4 && !Val5){
    unlock = true;
   }

   else {
    unlock = false;
   }

  if (!unlock) {
    digitalWrite(maglock, LOW);
  }
  else{
    digitalWrite(maglock, HIGH);
    Serial.println("\n solved \n");
    delay(90000);
    digitalWrite(maglock, LOW);
    unlock = false;
  }
} 
