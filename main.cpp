#include <math.h>
#include "custom_joint_pub.hpp"
#include "ur5_kinematics.h"
//#include "robot.hpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv){

    /*
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;
   
  
   JointStateVector q_des0;
   JointStateVector q_current;

   q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
     //q_des0 << -2.72186 , -1.1391 , -2.44604  , -1.12765 , -1.57099 ,  1.15094;
    // q_des0 << -3.10976 ,  -1.5341 , -1.92959 , -0.988641 , -1.66503  ,  1.79526;
   //q_des0 << -2.29748, -1.07838, -2.47464, -1.15936,  -1.5708, 0.726684;
   //q_des0 <<  -2.21729 ,-1.18481, -2.09685, -1.43074,  -1.5708, 0.646493;
   //pub_des_jstate_sim_rt.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node, "/command", 1));

  //node.getParam("/real_robot", real_robot);
  if (true)
  {
      pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
  } else {
      pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);
  }

  ros::Rate loop_rate(loop_frequency);
  jointState_msg_sim.position.resize(6);
  jointState_msg_sim.velocity.resize(6);
  jointState_msg_sim.effort.resize(6);
  jointState_msg_robot.data.resize(6);*/
  //q_des0 
  
  //initFilter(q_des0);
  //JointStateVector amp;
  //JointStateVector freq;
  //amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
  //freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0;
  //VectorXd th(6);
  //th <<   1.48357, -1.0231, 0.549017, -1.09672, 1.5708, 0.0872224;*/
  /* MY CODE */
    /*********************************************************************************************************/
    RowVectorXd th(6);
    //VectorXd thi(6);
    
    //scale facto == 1 quindi abbastanza inutile
    
    /*---------------------*/
        //table coordinates: Y = 0.15 X= 0.1 Z= 0.7

         //to user max= X=-0.4
     /*------------------*/

    Vector3d testPos = { -0.306541, 0.00912967, 0.599758};
    Vector3d firstPos = { -0.0011723, -0.199997, 0.566109};
    Vector3d secondPos = { -0.136458, -0.146217, 0.566109};

    Vector3d endOrient = { 0, 0, 0};

    Vector3d endPos = { 0.4, -0.3, 0.7};
    Vector3d startPos = {0.182291, -0.201256, 0.566109}; //current pos of end effector

     //Vector3d endPos1 = { -0.2, -0.3, 0.6};
     //Vector3d endPos1 = { -0.3, -0.4, 0.6};
     //Vector3d endOrient = { 0 , 0, 0};

     double samples = 50;
     //double delta = 1/samples;
     //double samples = 10;
     double delta = 1/steps;
     cout <<  delta << endl;



     MatrixXd tot_trajectory = inverseDiffKinematicsUr5(q_des0, endPos, endOrient, 0, 1, delta, LIN);

     MatrixXd points(1,3);
     points << 0,0,0;

     send_des_jstate(q_des0);
     ros::spinOnce();
     ros::Duration(3).sleep();


    /*
     for(int i=0; i< tot_trajectory.rows(); i++){
         //MatrixXd mat = directKinematicsUr5(tot_trajectory.block<1,6>(i,0));
         send_des_jstate(tot_trajectory.block<1,6>(i,0));
         ros::spinOnce();
         //loop_time++;
         loop_rate.sleep();
         //points.conservativeResize(points.rows()+1, points.cols());
         //points.block<1,3>(points.rows()-1, 0) = point;
     }*/

    cout << "HELo" << endl;
    return 0;

}
