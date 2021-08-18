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

#include <Wire.h>
//TODO: include necessary librarys!

void calculateAngles(float* angles, float x, float z, float shoulderHeight, float xUpperLimit){
  /*
  * TODO:
  * Implement this function. It calculates the angles that the shoulder and ellbow servos need to have to reach a point with the coordinates {x,y}.
  * The angles are to be saved to the array "angles" (2 elements, first one is the shoulder angle, second one the ellbow) via pointer.
  */
}

//TODO: fill in the missing values with help of the manual on your worksheet
Roboterarm robot(, , , , &calculateAngles,  //basic stuff
, , , , ,                                   //shoulder
, , , , ,                                   //ellbow
, , , , , , ,                               //gripper
, , , , , , , );                            //y-axis

void setup(void){
  //TODO: begin serial communication and initialize the robot arm
}

void loop(void){
  //TODO: program the movements of your robot
}