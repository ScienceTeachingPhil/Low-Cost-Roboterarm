/**
 * @file RobotDriver.cpp
 * @author Philipp Hermann <phil12hermann@gmail.com>
 * @date 2021-08-18
 * @version alpha1.5
 * @brief Defines classes for controlling a low cost robot arm for education purposes. This implementation is for learners and thus not complete.
 * 
 * @copyright Copyright (c) 2021 Philipp Hermann
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
    Serial.println(F("ServoMotor::begin()"));
    Serial.begin(9600);
    Serial.print(F("Initializing servo. Servo number: "));
    Serial.println(servonum);
    Serial.flush();
    driver.begin();
    driver.setPWMFreq(125);
    delay(10);
    if(servomindeg<0) pwmOffset=625.0;
    else pwmOffset=480.0;
    drive(0);
    return;
}

bool ServoMotor::drive(float deg){
    Serial.println(F("ServoMotor::drive()"));
    Serial.begin(9600);
    if(deg<0||deg>servomaxdeg){
        Serial.println(F("Can't reach requested position, check borders!"));
        return 1;
    }
    float pwm=pwmOffset+deg*(servomax-servomin)/(servomaxdeg-servomindeg);
    Serial.print("Setting PWM to ");
    Serial.println(pwm);
    driver.setPWM(servonum, 0, pwm);
    return 0;
}

void Greifer::openGripper(void){
    Serial.println(F("Greifer::openGripper()"));
    drive(opened);
    return;
}

void Greifer::closeGripper(void){
    Serial.println(F("Greifer::closeGripper()"));
    drive(closed);
    return;
}

void SchrittMotor::begin(void){
    Serial.println(F("SchrittMotor::begin()"));
    StepperMotor->setSpeed(speed);
    pinMode(home, INPUT);
    homeAxis();
}

void SchrittMotor::homeAxis(void){
  Serial.println(F("SchrittMotor::homeAxis()"));
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
  Serial.flush();
  position=0;
  Serial.println(F("Homing successful."));
  Serial.flush();
  return;
}

bool SchrittMotor::driveStepper(float pos){
    Serial.println(F("SchrittMotor::driveStepper()"));
    Serial.flush();
    if(pos==position) return 0;
    else if(pos<=limit&&pos>=0&&position<=limit&&position>=0){
        Serial.print("Moving y to ");
        Serial.println(pos);
        StepperMotor->step((pos-position)*mm);
        position=pos;
        return 0;
    }
    else if(position>limit||position<0){
        Serial.println(F("Position out of limits. Homing y."));
        homeAxis();
        return 1;
    }
    else Serial.println(F("Requested position of y out of limits."));
    return 1;
}

void Roboterarm::begin(void){
    Serial.println(F("Roboterarm::begin()"));
    Schulter->begin();
    Ellenbogen->begin();
    Endeffektor->begin();
    Linearachse->begin();
    shoulderPosition=0;
    Schulter->drive(shoulderPosition);
    ellbowPosition=0;
    Ellenbogen->drive(ellbowPosition);
    return;
}

void Roboterarm::drive(float x, float y, float z, bool e){
    Serial.println(F("Roboterarm::drive()"));
    if(x<0||x>xUpperLimit){
        Serial.println(F("Check your x value."));
        return;
    }
    else if(y<0||y>yUpperLimit){
        Serial.println(F("Check your y value."));
        return;
    }
    else if(z<0||z>zUpperLimit){
        Serial.println(F("Check your z value."));
        return;
    }

    (*calculateAngles)(angles, x, z, shoulderHeight, xUpperLimit);
    Serial.println("Returned.");
    Serial.flush();

    Serial.print(F("Shoulder angle: "));
    Serial.println(angles[0]);
    Serial.print(F("Ellbow angle: "));
    Serial.println(angles[1]);

    Serial.println("Checking Angles...");
    Serial.flush();
    if(angles[0]<shoulderAngleLow||angles[0]>shoulderAngleHigh){
        Serial.println(F("Shoulder angle ist too high or too low."));
        return;
    }
    else if(angles[1]<ellbowAngleLow||angles[1]>ellbowAngleHigh){
        Serial.println(F("Ellbow angle ist too high or too low."));
        return;
    }

    Serial.println("Driving Servos...");
    Serial.flush();
    char flag=0;
    float shoulderChange = angles[0]-shoulderPosition;
    float ellbowChange = angles[1]-ellbowPosition;
    float ratio=0;
    if(shoulderChange==0){
        flag+=Ellenbogen->drive(angles[1]);
        goto end;
    }
    else if(ellbowChange==0){
        flag+=Schulter->drive(angles[0]);
        goto end;
    }
    else ratio = ellbowChange/shoulderChange;

    if(ratio>0){
        while(shoulderPosition<angles[0]){
            ++shoulderPosition;
            ellbowPosition+=ratio;
            Schulter->drive(shoulderPosition);
            Ellenbogen->drive(ellbowPosition);
            delay(100);
        }
        flag+=Schulter->drive(angles[0]);
        flag+=Ellenbogen->drive(angles[1]);
        shoulderPosition=angles[0];
        ellbowPosition=angles[1];
        delay(100);
        goto end;
    }
    else if(shoulderChange<0&&ellbowChange>0){
        while(shoulderPosition>angles[0]){
            --shoulderPosition;
            ellbowPosition-=ratio;
            Schulter->drive(shoulderPosition);
            Ellenbogen->drive(ellbowPosition);
            delay(100);
        }
        flag+=Schulter->drive(angles[0]);
        flag+=Ellenbogen->drive(angles[1]);
        shoulderPosition=angles[0];
        ellbowPosition=angles[1];
        delay(100);
        goto end;
    }
    else if(shoulderChange>0&&ellbowChange<0){
        while(shoulderPosition<angles[0]){
            ++shoulderPosition;
            ellbowPosition+=ratio;
            Schulter->drive(shoulderPosition);
            Ellenbogen->drive(ellbowPosition);
            delay(100);
        }
        flag+=Schulter->drive(angles[0]);
        flag+=Ellenbogen->drive(angles[1]);
        shoulderPosition=angles[0];
        ellbowPosition=angles[1];
        delay(100);
        goto end;
    }
    else if(shoulderChange<0&&ellbowChange<0){
        while(shoulderPosition>angles[0]){
            --shoulderPosition;
            ellbowPosition-=ratio;
            Schulter->drive(shoulderPosition);
            Ellenbogen->drive(ellbowPosition);
            delay(100);
        }
        flag+=Schulter->drive(angles[0]);
        flag+=Ellenbogen->drive(angles[1]);
        shoulderPosition=angles[0];
        ellbowPosition=angles[1];
        delay(100);
        goto end;
    }
    
    end:
        if(flag>0){
            Serial.println(F("Something went wrong, aborting movement to requested position!"));
            return;
        }
        else{
            flag+=Linearachse->driveStepper(y);
            if(e==1) Endeffektor->closeGripper();
            else Endeffektor->openGripper();
            return;
        }
}