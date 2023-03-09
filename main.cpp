#include <math.h>
#include "robot.hpp"
#include <iostream>
using namespace std;

int main(int argc, char **argv){
    
    init();
    //cout << "here" << endl;
    ros::Rate loop_rate(loop_frequency);
    Vector3d zero;
    zero << 0, 0, 0;
    Vector6d q_des0;


    Robot robot;
    //robot.move_gripper(70);
    
    Vector3d endOrient = { 0, 0, 0};
    //Vector3d endPos = worldToRobot({ 0.9, 0.6, 0.87+0.2});
    Vector3d endPos = { -0.4, 0, 0.7};

    

    robot.move(endPos, endOrient);

    
    /*
    endPos << worldToRobot({0.9, 0.6, 0.87});
    robot.move(endPos, endOrient);

    robot.move_gripper(10);

    endPos << worldToRobot({0.9, 0.6, 1.0});
    robot.move(endPos, endOrient);*/

    cout << "HELo" << endl;
    return 0;

}
