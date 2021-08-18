/**
 * @file Roboterarm.ino
 * @author Philipp Hermann (phil12hermann@gmail.com)
 * @brief Arduino sketch that controls a low cost robot arm via the library "RobotDriver.h". For this library is designed to learn about kinematics, a function calculating hinge angles is to be implemented in this sketch.
 * @version alpha1.5
 * @date 2021-08-18
 * 
 * @copyright Copyright (c) 2021 Philipp Hermann
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "RobotDriver.h"

void calculateAngles(float* angles, float x, float z, float shoulderHeight, float xMax){
  Serial.println(z);
  Serial.println(shoulderHeight);
  float height_z = z-shoulderHeight;
  Serial.println(height_z);
  /*height_z is now the height over/under the shoulder pivet point.
  z was the height over the base of the robot.*/

  float length_e = sqrt(square(x) + square(height_z));
  Serial.println(length_e);
  //Calculating the length of a vector (base of the triangle)

  float height_e = 0.5*sqrt(4*square(0.5*xMax)-square(length_e));
  Serial.println(height_e);
  /*Calculating the height over the base of an isosceles triangle. xMax is a + b, 
  so we have to divide by 2 to get one of a or b.*/

  float gamma = 180/PI*asin(height_e/(0.5*xMax));
  Serial.println(gamma);
  //Calculating the base angles of the triangle in degrees (asin returns radian!)

  float beta = 2*gamma;
  Serial.println(beta);
  /*Normaly, beta would be 180°-2*gamma, but since we defined the "real" 180° as 
  our home position and thus 0°, we are left with just 2*gamma.*/
  
  /*Now we rotate the triangle around the shoulder with 
  the angle between the vector e and the x-Axis.*/
  float alpha=0;
  if(height_z==0) alpha=gamma;
  /*if e||x we don't have to rotate the triangle*/
  else if(height_z<0) alpha=gamma-(180/PI*acos(x/length_e));
  /*else, if e points down, we have to substract the angle 
  between e and x from the base angle of the triangle*/
  else alpha=gamma+(180/PI*acos(x/length_e));
  /*else (i.e. e points up) we need to add the angle between e and x
  to the base angle of the triangle*/
  Serial.println(alpha);
  Serial.println("Saving to array");
           
  *angles = alpha;
  *(angles+1) = beta;
  Serial.println("Saved to array");
  //save both servo angles to the array
  Serial.println("Returning");
  Serial.flush();
  return;
}

Roboterarm robot(116, 300.0, 380.0, 300.0, &calculateAngles,
    450, 1030, -39, 90, 0,          //shoulder
    480, 1000, 0, 120, 1,           //ellbow
    450, 1500, -60, 90, 60, 30, 2,  //gripper
    2048, 2, 10, 32, 3, 5, 4, 6);   //y-axis
/*these are example values from the developer of this sketch.
For explanation on how to get to these values for your robot,
check your worksheet.*/

void setup(){
  Serial.begin(9600);
  //serial communication is established
  robot.begin();
  //robot is getting initialized
}

void loop() {
  robot.drive(190, 10, 150, 0);
  //robot drives to {190, 50, 150} and opens gripper
  delay(2000);
  robot.drive(250, 10, 116, 0);
  //robot drives to {200, 100, 100} and closes gripper
  delay(2000);
  robot.drive(190, 10, 350, 0);
  delay(2000);
  robot.drive(250, 10, 50, 0);
  delay(2000);
  robot.drive(190, 10, 150, 0);
}