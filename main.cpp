#include <math.h>
#include "custom_joint_publisher.h"
#include "ur5_kinematics.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;

  //pub_des_jstate_sim_rt.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node, "/command", 1));

  //node.getParam("/real_robot", real_robot);

  if (real_robot)
  {
      pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);

  } else {
      pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);
  }

  ros::Rate loop_rate(loop_frequency);

  jointState_msg_sim.position.resize(6);
  jointState_msg_sim.velocity.resize(6);
  jointState_msg_sim.effort.resize(6);
  jointState_msg_robot.data.resize(6);

  //q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
  
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
    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }

    th << 0, 0, 0, 0, 0, 0;
    //thi << 0, 0, 0, 0, 0, 0;

    //MatrixXd support = DirectKinematicsUr5(thi);
    //Vector3d startPos = support.block<3,1>(0,3);
    //Vector3d startPos = {0.3, 0.3, 0.1};
    Vector3d startPos = { 0.151850 , 0.190809, 0.5};
    Vector3d startOrient = { 0 , 0, 0};


    //cout << startPos << endl;

    //endPos << startPos(0)+0.05, startPos(1)+0.05, startPos(2)+0.05;
    //Vector3d endPos2 =  worldToRobot(endPos);
    Vector3d endPos = { 0.3, 0.3, 0.5};
    Vector3d endOrient = { 0 , 0, 0};

    double samples = 500;
    double delta = 1/samples;
    
    //cout << toRotationMatrix(startOrient) << endl;

    MatrixXd thi = InverseKinematicsUr5(startPos, toRotationMatrix(startOrient)); //we obtain the matrix of all possible solutions
    
    th = thi.block<1,6>(0,0);
    
    //MatrixXd result = InverseDiffKinematicsUr5(th, startPos, endPos, startOrient, endOrient, 0, 1, delta);

    /*
    for(int j=0; j<samples; j++){
        //ROS_INFO("Hello");
        rate.sleep();
        cout << result.block<1,6>(j,0) << endl;
    }*/

    /********************************************************************************************/
  
    q_des0 << th(0), th(1), th(2), th(3), th(4), th(5);
  /*
  for(int i=0 ; i<6 ; i++){
      dist[i] *= SCALE_FACTOR;
      a[i] *= SCALE_FACTOR;
  }*/

  //Vector3d pe_2 = DirectKinematicsUr5(th); // test inverse kin

  //cout << pe_2-pe << endl;

  
  /*
    int i=0;
    while (ros::ok())
    {

        //1- step reference
        if (loop_time < 100.)
        {
            cout<< "inir" << endl;
            q_des = q_des0;
            cout << q_des << endl;
            
            cout << endl;
            if(loop_time < 2)
                send_des_jstate(q_des);

        } else {

            JointStateVector delta_q;
            
            //th = result.block<1,6>(i,0);


            q_des << result(i,0), result(i,1), result(i,2), result(i,3), result(i,4), result(i,5);
            
            
            i++;
            cout << endl;
            //cout << i << endl;
            if(i == samples){
                break;
            }
            //q_des = secondOrderFilter(q_des0 + delta_q, loop_frequency, 5.);
            send_des_jstate(q_des);
        }
        

        //2- sine reference
        // q_des = q_des0.array() + amp.array()*(2*M_PI*freq*loop_time).array().sin();

        ros::spinOnce();
        loop_time++;
        loop_rate.sleep();

    }

    cout << "HELo" << endl;*/

  return 0;
}