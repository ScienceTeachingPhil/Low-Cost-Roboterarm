/**
 * @file RobotDriver.h
 * @author Philipp Hermann (phil12hermann@gmail.com)
 * @brief Contains prototypes for classes controlling a low cost robot arm for education purposes.
 * @version alpha1.5
 * @date 2021-08-18
 * 
 * @copyright Copyright (c) 2021 Philipp Hermann
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef RobotDriver_h
#define RobotDriver_h

#include <Arduino.h>
#include <Stepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

float square(float x);

class ServoMotor{
    protected:
        float pwmOffset;
        char servonum;
        const float servomin;
        const float servomax;
        const float servomindeg;
        const float servomaxdeg;
        Adafruit_PWMServoDriver& driver;

    public:
        void begin(void);
        bool drive(float deg);
        ServoMotor(int servonum, Adafruit_PWMServoDriver& driver, float servomin, float servomax, float servomindeg, float servomaxdeg): servonum(servonum), driver(driver), servomin(servomin), servomax(servomax), servomindeg(servomindeg), servomaxdeg(servomaxdeg){
            Serial.println(F("ServoMotor()"));
        }
        ~ServoMotor(){
            Serial.println(F("~ServoMotor()"));
        }
};

class Greifer: public ServoMotor{
    private:
        const float closed;
        const float opened;
    
    public:
        Greifer(char servonum, Adafruit_PWMServoDriver& driver, float servomin, float servomax, float servomindeg, float servomaxdeg, float closed, float opened): ServoMotor(servonum, driver, servomin, servomax, servomindeg, servomaxdeg), closed(closed), opened(opened){
            Serial.println(F("Greifer()"));
        }
        ~Greifer(){
            Serial.println(F("~Greifer()"));
        }
        void openGripper(void);
        void closeGripper(void);
};

class SchrittMotor{
    private:
        Stepper* StepperMotor;
        const float spu;
        const float limit;
        const float speed;
        const float mm;
        float position;
        const char home;
        const char pin1;
        const char pin2;
        const char pin3;
        const char pin4;

    public:
        SchrittMotor(float spu, float limit, float speed, float mm, char home, char pin1, char pin2, char pin3, char pin4): spu(spu), limit(limit), speed(speed), mm(mm), home(home), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4){
            Serial.println(F("SchrittMotor()"));
            position = -1;
            StepperMotor=new Stepper(spu, pin1, pin2, pin3, pin4);
        }
        ~SchrittMotor(){
            Serial.println(F("~SchrittMotor()"));
            delete StepperMotor;
        }
        void begin(void);
        void homeAxis(void);
        bool driveStepper(float pos);
};

class Roboterarm{
    private:
        Adafruit_PWMServoDriver ServoDriver;
        ServoMotor* Schulter;
        ServoMotor* Ellenbogen;
        Greifer* Endeffektor;
        SchrittMotor* Linearachse;
        const void (*calculateAngles)(float*, float, float, float, float);
        const float shoulderHeight;
        const float xUpperLimit;
        const float yUpperLimit;
        const float zUpperLimit;
        const float shoulderAngleLow;
        const float shoulderAngleHigh;
        const float ellbowAngleLow;
        const float ellbowAngleHigh;
        float shoulderPosition;
        float ellbowPosition;

    public:
        float angles[2] = {0, 0};
        Roboterarm(float shoulderHeight, float xUpperLimit, float yUpperLimit, float zLimit, void (*calculateAngles_pointer)(float*, float, float, float, float),
         float shoulderPWMlow, float shoulderPWMhigh, float shoulderAngleLow, float shoulderAngleHigh, char shoulderPin,
         float ellbowPWMlow, float ellbowPWMhigh, float ellbowAngleLow, float ellbowAngleHigh, char ellbowPin,
         float gripperPWMlow, float gripperPWMhigh, float gripperAngleLow, float gripperAngleHigh, float gripperAngleClosed, float GripperAngleOpened, char gripperPin,
         float yAxisSPR, char yAxisSwitch, float yAxisSpeed, float yAxisMMPR, char yAxisPin1, char yAxisPin2, char yAxisPin3, char yAxisPin4): 
         shoulderHeight(shoulderHeight), xUpperLimit(xUpperLimit), yUpperLimit(yUpperLimit), zUpperLimit(shoulderHeight+zLimit), calculateAngles(calculateAngles_pointer),
         shoulderAngleLow(shoulderAngleLow), shoulderAngleHigh(shoulderAngleHigh), ellbowAngleLow(ellbowAngleLow), ellbowAngleHigh(ellbowAngleHigh){
            Serial.println(F("Roboterarm()"));
            ServoDriver = Adafruit_PWMServoDriver();
            Schulter = new ServoMotor(shoulderPin, ServoDriver, shoulderPWMlow, shoulderPWMhigh, shoulderAngleLow, shoulderAngleHigh);
            Ellenbogen = new ServoMotor(ellbowPin, ServoDriver, ellbowPWMlow, ellbowPWMhigh, ellbowAngleLow, ellbowAngleHigh);
            Endeffektor = new Greifer(gripperPin, ServoDriver, gripperPWMlow, gripperPWMhigh, gripperAngleLow, gripperAngleHigh, gripperAngleClosed, GripperAngleOpened);
            Linearachse = new SchrittMotor(yAxisSPR, yUpperLimit, yAxisSpeed, yAxisSPR/yAxisMMPR, yAxisSwitch, yAxisPin1, yAxisPin2, yAxisPin3, yAxisPin4);
        }
        ~Roboterarm(){
            Serial.println(F("~Roboterarm()"));
            delete Schulter;
            delete Ellenbogen;
            delete Endeffektor;
            delete Linearachse;
        }
        void begin(void);
        void drive(float x, float y, float z, bool e);
};
#endif