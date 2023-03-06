#include <math.h>
#include "custom_joint_pub.hpp"
#include "ur5_kinematics.h"
#include "robot.hpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv){
    
    init();
    ros::Rate loop_rate(loop_frequency);
    Vector3d zero;
    zero << 0, 0, 0;
    Vector6d q_des0;

    //q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
    //q_des0 << -0.380411 , -1.52801 , -1.86065 , -1.42134 , -1.58563  , -1.10542;
    q_des0 << 0.516664, -0.952288 , -2.24017 , -1.59347 , -1.50309 , -1.99897;
    RowVectorXd th(6);

    Vector3d testPos = { -0.306541, 0.00912967, 0.599758};
    Vector3d firstPos = { -0.0011723, -0.199997, 0.566109};
    Vector3d secondPos = { -0.136458, -0.146217, 0.566109};

    Vector3d endOrient = { 0, 0, 0};

    Vector3d endPos = { -0.4, -0.3, 0.7};
    Vector3d startPos = {0.182291, -0.201256, 0.566109}; //current pos of end effector

     //Vector3d endPos1 = { -0.2, -0.3, 0.6};
     //Vector3d endPos1 = { -0.3, -0.4, 0.6};
     //Vector3d endOrient = { 0 , 0, 0};

     double samples = 50;
     //double delta = 1/samples;
     //double samples = 10;
     double delta = 1/steps;
     cout <<  delta << endl;


    
     MatrixXd tot_trajectory = inverseDiffKinematicsUr5(q_des0, endPos, endOrient);

    for(int i = 0; i < 7 ; i++){
        send_des_jstate(q_des0, zero);
        
        loop_rate.sleep();
        ros::spinOnce();
    }
     ros::Duration(3).sleep();


    
     for(int i=0; i< tot_trajectory.rows(); i++){
         //MatrixXd mat = directKinematicsUr5(tot_trajectory.block<1,6>(i,0));
         send_des_jstate(tot_trajectory.block<1,6>(i,0), zero);
         ros::spinOnce();
         //loop_time++;
         loop_rate.sleep();
         //points.conservativeResize(points.rows()+1, points.cols());
         //points.block<1,3>(points.rows()-1, 0) = point;
     }

    cout << "HELo" << endl;
    return 0;

}
