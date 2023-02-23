#include <math.h>
#include "custom_joint_publisher.h"
#include "ur5_kinematics.h"
#include <iostream>
#include <ros_impedance_controller/generic_float.h>

using namespace std;

bool soft_gripper;

double mapToGripperJoints(int diameter){
    if(soft_gripper){
        int D0 = 40;
        int L = 60;
        double delta = 0.5*(diameter - D0);

        return atan2(delta, L);
    }else{
        return (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "custom_joint_publisher");

    /*---starting config of the joints---*/
  
  JointStateVector q_des0;
  JointStateVector q_current;

  //q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
    //q_des0 << -2.72186 , -1.1391 , -2.44604  , -1.12765 , -1.57099 ,  1.15094;

  q_des0 << -2.29748, -1.07838, -2.47464, -1.15936,  -1.5708, 0.726684;
  //q_des0 <<  -2.21729 ,-1.18481, -2.09685, -1.43074,  -1.5708, 0.646493;
  //pub_des_jstate_sim_rt.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node, "/command", 1));

  //node.getParam("/real_robot", real_robot);
    ros::NodeHandle node;
    node.getParam("/soft_gripper", soft_gripper);


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
  jointState_msg_robot.data.resize(8);


    /*********************************************************************************************************/
    

    //scale facto == 1 quindi abbastanza inutile
    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }

    

    vector<double> peddie(8);
    peddie = {-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558, 2*M_PI, M_PI};
    jointState_msg_robot.data = peddie;

    ros::ServiceClient client = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");

    ros_impedance_controller::generic_float::Request req;
    ros_impedance_controller::generic_float::Response res;

    req.data = 50;

    double gripQ = mapToGripperJoints(50);
    jointState_msg_robot.data[6] = gripQ;
    jointState_msg_robot.data[7] = gripQ;
    pub_des_jstate.publish(jointState_msg_robot);

    VectorXd th(6);
    //th = InverseDiffKinematicsUr5Quaternions({}, {0,0,0}, jointState_msg_robot.data.);

    for(int j=0 ; j<2000 ; j++){
       // jointState_msg_robot.data[6] -= 0.01;
       // jointState_msg_robot.data[7] -= 0.01;

        /* REAL ROBOT
        req.data -= 0.01;
        
        client.call(req, res);

        if(!res.ack) ROS_ERROR("GRIPPER FAIL");
        */

       if(j > 1500){
            double gripQ = mapToGripperJoints(50);
            jointState_msg_robot.data[6] = gripQ;
            jointState_msg_robot.data[7] = gripQ;
       }

        pub_des_jstate.publish(jointState_msg_robot);
        //rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}