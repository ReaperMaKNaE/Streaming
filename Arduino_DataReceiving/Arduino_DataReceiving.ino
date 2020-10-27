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
  // The pins which allow PWM is 5, 6, 9, 10, 11
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  // Always 5V pin
  pinMode(8, OUTPUT);
  digitalWrite(8,HIGH);
}

void loop() {  
  if(bluetooth.available()){
    
    value = bluetooth.parseInt();
    bluetooth.write(value);
    analogWrite(5, 0);
    analogWrite(6, value);
    analogWrite(9, 0);
    analogWrite(10, value);
    
    /*
    if(value == 100) {
    }
    else if(value == 101) {
    }
    else if(value == 102) {
    }
    else if(value == 203) {
    }
    else if(value == 204) {
    }
    else {
      analogWrite(5, LOW);
      analogWrite(6, LOW);
      analogWrite(9, LOW);
      analogWrite(10, LOW);
    }
    */
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
