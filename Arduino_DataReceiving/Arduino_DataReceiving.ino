#include <SoftwareSerial.h>

int BTx = 2;
int BRx = 3;
int value = 0;

// Diode : 6~10
// Starting From 10

SoftwareSerial bluetooth(2,3);

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  
  if(bluetooth.available()){
    
    value = bluetooth.parseInt();
    
    if(value == 100) {
      TurnOnLED(HIGH,LOW,LOW,LOW,LOW);
    }
    else if(value == 101) {
      TurnOnLED(HIGH,HIGH,LOW,LOW,LOW);
    }
    else if(value == 102) {
      TurnOnLED(HIGH,HIGH,HIGH,LOW,LOW);
    }
    else if(value == 203) {
      TurnOnLED(HIGH,HIGH,HIGH,HIGH,LOW);
    }
    else if(value == 204) {
      TurnOnLED(HIGH,HIGH,HIGH,HIGH,HIGH);
    }
    else {
      TurnOnLED(LOW,LOW,LOW,LOW,LOW);
    }
  }
}

void TurnOnLED(int Pin1, 
               int Pin2, 
               int Pin3, 
               int Pin4, 
               int Pin5)
{
  digitalWrite(10,Pin1);
  digitalWrite(9,Pin2);
  digitalWrite(8,Pin3);
  digitalWrite(7,Pin4);
  digitalWrite(6,Pin5);
}
