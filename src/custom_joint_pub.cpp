#include "custom_joint_pub.hpp"


void init(){

    int argc = 1;
    char* argv[] = {(char *)"custom_joint_pub", NULL};

    ros::init(argc, argv, "custom_joint_pub");

    ros::NodeHandle node;
    
    
    node.getParam("/real_robot", real_robot);
    node.getParam("/soft_gripper", soft_gripper);
    node.getParam("/gripper_sim", gripper_sim);
    
    cout << "--GRIPPER CHECK---" << endl;
    cout << gripper_sim << endl << endl;
    cout << soft_gripper << endl;
    cout << "------------------" << endl;

    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);

    client_gripper = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");

    if(gripper_sim && !real_robot){
        if(soft_gripper){
            jointState_msg_robot.data.resize(8);
        }else{
            jointState_msg_robot.data.resize(9);
        }
    }else{
        jointState_msg_robot.data.resize(6);
    }
}

void robot_send_des_jstate(const Vector6d & joint_pos, const double diameter, const bool move_gripper){
  
  if(real_robot){
    for (int i = 0; i < joint_pos.size(); i++)
      {
        jointState_msg_robot.data[i] = joint_pos[i];
      }
    
    if(gripper_sim && move_gripper){
        robot_move_gripper(diameter);
    }

    pub_des_jstate.publish(jointState_msg_robot);

  }else{
    cout << "Function call wrongly (robot_send_des_jstate) " << endl;
  }
}

void robot_move_gripper(const double diameter){

  if(real_robot && gripper_sim){
    ros_impedance_controller::generic_float::Request req;
    ros_impedance_controller::generic_float::Response res;

    req.data = diameter;

    /* !!! HAVE TO BE CALLED ONLY ONCE !!! */
    client_gripper.call(req, res); 

    if(!res.ack) ROS_ERROR("GRIPPER FAIL");
  }

}

void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_pos){

    
    for (int i = 0; i < joint_pos.size(); i++)
    {
      jointState_msg_robot.data[i] = joint_pos[i];
    }

    /* GRIPPER MANAGEMENT */


    
    if(gripper_sim){
      int j=0;
      if(soft_gripper){
        for (int i = joint_pos.size() ; i < joint_pos.size()+gripper_pos.size()-1 ; i++)
        {
          jointState_msg_robot.data[i] = gripper_pos[j++];
        }
      }else{
        for (int i = joint_pos.size() ; i < joint_pos.size()+gripper_pos.size(); i++)
        {
          jointState_msg_robot.data[i] = gripper_pos[j++];
        }
      }
    }

    /* SEND MESSAGE */

    cout << joint_pos.transpose() << endl;

    pub_des_jstate.publish(jointState_msg_robot);
}

Vector9d receive_jstate(){
    sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");

    Vector9d v;

    if(soft_gripper){
      v << msg->position[4], msg->position[3], msg->position[0], msg->position[5], msg->position[6], msg->position[7], msg->position[1], msg->position[2], 0;
    }else{
      v << msg->position[5], msg->position[4], msg->position[0], msg->position[6], msg->position[7], msg->position[8], msg->position[1], msg->position[2], msg->position[3];
    }

    return v;
}