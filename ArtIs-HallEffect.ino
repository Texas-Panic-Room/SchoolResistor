#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

const int hall0 = 2;
const int hall1 = 3;
const int hall2 = 4;
const int hall3 = 5;
const int hall4 = 6;
const int hall5 = 7;
const int hall6 = 8;
const int hall7 = 9;
const int maglock =  12;
const int led = 13;
int halls[3] = {hall0, hall1, hall2};
int hallsLen = arr_len(halls);

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < hallsLen; i++){
    pinMode(halls[i], INPUT);
  }
  pinMode(maglock, OUTPUT);
  pinMode(led, OUTPUT);
}

void loop(){
  bool aligned = true;
  for(int i = 0; i < hallsLen; i++){
    Serial.print("Hall");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(digitalRead(halls[i]));
    if(digitalRead(halls[i])){
      aligned = false;
    }
  }
  
  if (aligned) {     
    digitalWrite(maglock, HIGH);  
    digitalWrite(led, HIGH);
  } 
  else {
    digitalWrite(maglock, LOW);
    digitalWrite(led, LOW);
  }
}
