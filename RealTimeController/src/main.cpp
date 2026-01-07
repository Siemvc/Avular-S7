#include <Arduino.h>

const int pinBucketDown = 29;
const int pinBucketUp = 28;
const int pinPotmeter = 27;

const int deadband = 4; //Deadband for position control
const int minPWM = 60;
const int maxPWM = 245;
const int maxPotValue = 785; //785
const int minPotValue = 30; //30

const int kp = 5; //Proportional gain for control

int targetPosition = 0;
int currentPosition = 0;
int inputValue = 0;
int inputMMValue = 0; //mm
int speed = 0; //PWM value for motor control


void setup(){
  Serial.begin(115200);

  pinMode(pinBucketDown, OUTPUT);
  pinMode(pinBucketUp, OUTPUT);
  pinMode(pinPotmeter, INPUT);
  //Teensy 4.1 standard 10 bit ADC resolution(0-1023)
  analogReadResolution(10);

  currentPosition = analogRead(pinPotmeter);
  targetPosition = currentPosition; //Stay in place at startup

}


void MotorControl(int speed){
  if(speed > 0){
    analogWrite(pinBucketUp, speed);
    analogWrite(pinBucketDown, 0);
    Serial.printf("Up PWM: %d, ", speed);
    }
  else if(speed < 0){
    analogWrite(pinBucketDown, -speed);
    analogWrite(pinBucketUp, 0);
    Serial.printf("Down PWM: %d, ", -speed);
   }
   else if(speed == 0){
    analogWrite(pinBucketDown, 0);
    analogWrite(pinBucketUp, 0);
    Serial.print("Stopped, ");
    }
}

void loop(){
  if(Serial.available() > 0){
    inputMMValue = Serial.parseInt();
    if(inputMMValue >= 0){
      inputValue = map(inputMMValue, 0, 100, minPotValue, maxPotValue);
      if(inputValue <= maxPotValue && inputValue >= minPotValue){
        Serial.printf("Received speed: %d\n", inputValue);
        targetPosition = inputValue;
      }
      else{
        Serial.printf("Input value %d out of range (%d - %d). Ignored.\n", inputMMValue, 0, 100);
        delay(1000);
      }
    }
  }
    currentPosition = analogRead(pinPotmeter);
    Serial.printf("Current: %d, Target: %d\n", currentPosition, targetPosition);
    int positionError = targetPosition - currentPosition;
    if(abs(positionError) > deadband){
      speed = positionError * kp;

      //Constrain speed to min and max PWM values
      if(speed > 0){
        speed = constrain(speed, minPWM, maxPWM);
      }
      else{
        speed = constrain(speed, -maxPWM, -minPWM);
      }
      MotorControl(speed);
    }
    else{
      MotorControl(0); //Within deadband, stop motor
    }
    
  delay(100); //Small delay for stability
  }


