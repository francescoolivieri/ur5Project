#include <math.h>
#include "custom_joint_pub.hpp"
#include "ur5_kinematics.h"
#include "robot.hpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  init();

  Robot robot;

  robot.move_gripper(10);

  //     Vector6d q_des0;
  // q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
 
  //   /*********************************************************************************************************/

  //   Vector3d endPos = { -0.4, -0.2, 0.7};
  //   Vector3d firstPos = { -0.0011723, -0.199997, 0.566109};
  //   Vector3d secondPos = { -0.136458, -0.146217, 0.566109};
  //   Vector3d endOrient = { 0, 0, 0};
  //   Vector3d posArc = {0.182291, -0.201256, 0.566109}; //current pos of end effetocr
  //   double delta = 1/steps;
  //   cout <<  delta << endl;
    


  //    inverseDiffKinematicsUr5(q_des0, endPos, endOrient, 0, 1, delta, LIN);
  
     //MatrixXd points(1,3);
     //points << 0,0,0;
     
    cout << "HELo" << endl;

  return 0;
}
