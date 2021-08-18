/**
 * @file RobotDriver.cpp
 * @author Philipp Hermann <phil12hermann@gmail.com>
 * @date 2021-08-14
 * @version alpha1.2
 * @brief Defines classes for controlling a low cost robot arm for education purposes. This implementation is for learners and thus not complete.
 * 
 * @copyright Copyright (c) 2021 Philipp Hermann
 */

#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "RobotDriver.h"

float square(float x){
    return x*x;
}

void ServoMotor::begin(void){
    Serial.begin(9600);
    Serial.println("Initializing servo. Servo number: "+servonum);
    driver.begin();
    driver.setPWMFreq(125);
    delay(10);
    if(servomindeg<0) pwmOffset=625.0;
    else pwmOffset=480.0;
    drive(0);
    return;
}

bool ServoMotor::drive(float deg){
    Serial.begin(9600);
    if(deg<0||deg>servomaxdeg){
        Serial.println(F("Can't reach requested position, check borders!"));
        return 1;
    }
    float pwm=pwmOffset+deg*(servomax-servomin)/(servomaxdeg-servomindeg);
    Serial.println("Setting PWM to "+pwm);
    driver.setPWM(servonum, 0, pwm);
    return 0;
}

void Greifer::openGripper(void){
    drive(opened);
    return;
}

void Greifer::closeGripper(void){
    drive(closed);
    return;
}

void SchrittMotor::begin(void){
    StepperMotor->setSpeed(speed);
    pinMode(home, INPUT);
    homeAxis();
}

void SchrittMotor::homeAxis(void){
  Serial.println(F("Homing y-axis in progress..."));
  while(digitalRead(home)==false){
    StepperMotor->step(-10);
    delay(10);
  }
  Serial.println(F("First contact detected, moving away."));
  StepperMotor->step(mm);
  Serial.println(F("Testing switch again."));
  while(digitalRead(home)==false){
    StepperMotor->step(-1);
    delay(5);
  }
  Serial.println(F("Second contact detected, moving away."));
  StepperMotor->step(mm);
  Serial.println(F("Setting home position."));
  position=0;
  Serial.println(F("Homing successful."));
  return;
}

void