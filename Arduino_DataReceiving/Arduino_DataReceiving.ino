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
    int i = 0;
    if(bluetooth.available()){
    value = bluetooth.parseInt();
        if(value == 0){
            analogWrite(5, 0);
            analogWrite(6, 0);
            analogWrite(10, 0);
            analogWrite(11, 0);
            servo.write(0);
        }
        else if(value == 210){
            analogWrite(5, 0);
            analogWrite(6, 100);
            analogWrite(10, 0);
            analogWrite(11, 100);
            delay(500);
            analogWrite(5, 0);
            analogWrite(6, 0);
            analogWrite(10, 0);
            analogWrite(11, 0);
            delay(500);
            for(i = 1; i<=5; i++){
                analogWrite(5, 0);
                analogWrite(6, 10*i);
                analogWrite(10, 0);
                analogWrite(11, 10*i);
                delay(100);
            }
        }
        else {
            if(value > 90){
                for(i = 90; i < value; i ++){
                    servo.write(i)
                }
            }
            else{
                for(i = 90; i > value; i --){
                    servo.write(i)
                }
            }
        }
    }
}
