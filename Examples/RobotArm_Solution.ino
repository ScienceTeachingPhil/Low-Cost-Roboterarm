/**
 * @file Roboterarm.ino
 * @author Philipp Hermann (phil12hermann@gmail.com)
 * @brief Arduino sketch that controls a low cost robot arm via the library "RobotDriver.h". For this library is designed to learn about kinematics, a function calculating hinge angles is to be implemented in this sketch.
 * @version alpha1.2
 * @date 2021-08-14
 * 
 * @copyright Copyright (c) 2021 Philipp Hermann
 * 
 */

#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "RobotDriver.h"







void calculateAngles(float* angles, float x, float z, float shoulderHeight, float xUpperLimit){

  /*
  * TODO:
  * Implement this function. It calculates the angles that the shoulder and ellbow servos need to have to reach a point with the coordinates {x,y}.
  * The angles are to be saved to the array "angles" (2 elements, first one is the shoulder angle, second one the ellbow) via pointer.
  */

  float height_z = z-shoulderHeight;
  Serial.print("Height over pivet point in mm: ");
  Serial.println(height_z);
  Serial.flush();
  
  float length_e = sqrt(square(x) + square(height_z));
  Serial.print("Vector length in mm: ");
  Serial.println(length_e);
  Serial.flush();

  float height_e = 0.5*sqrt(4*square(0.5*xUpperLimit)-square(length_e));
  Serial.print("Height of triangle in mm: ");
  Serial.println(height_e);
  Serial.flush();
  
  float gamma = 180/PI*asin(height_e/(0.5*xUpperLimit));
  Serial.print("Base angles of triangle in deg: ");
  Serial.println(gamma);
  Serial.flush();
  
  float beta = 2*gamma;
  Serial.print("Top angle of triangle in deg: ");
  Serial.println(beta);
  Serial.flush();
   
  float alpha=0;
  if(height_z==0) alpha=gamma;
  else if(height_z<0) alpha=gamma-(180/PI*acos(x/length_e));
  else alpha=gamma+(180/PI*acos(x/length_e));
  Serial.print("Shoulder angle in deg: ");
  Serial.println(alpha);
  Serial.flush();
        
  if(alpha<-30||alpha>90||beta<0||beta>120){
    Serial.println("Result of kinematic equation: parameters outside limits.");
    Serial.flush();
    return;
  }
           
  *angles = alpha;
  *(angles+1) = beta;
  return;
}

void setup(void){

}

void loop(void){
    
}