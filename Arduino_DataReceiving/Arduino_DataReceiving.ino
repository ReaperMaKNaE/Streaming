#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo;

int BTx = 2;
int BRx = 3;
int value = 0;

int servoPin = 12;

SoftwareSerial bluetooth(2,3);

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  // The pins which allow PWM is 5, 6, 9, 10, 11
  // 5, 6 -> motor
  // 9, 10 -> motor2
  // 11 -> servo
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  servo.attach(servoPin);
}

void loop() {
  if(bluetooth.available()){
    value = bluetooth.parseInt();
    analogWrite(5, 0);
    analogWrite(6, value);
    analogWrite(10, 0);
    analogWrite(11, value);
    servo.write(value);
  }
}
