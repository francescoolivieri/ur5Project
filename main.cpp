#include <math.h>
#include "custom_joint_publisher.h"
#include "ur5_kinematics.h"
#include <iostream>



using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;

    /*---starting config of the joints---*/
  
  JointStateVector q_des0;
  JointStateVector q_current;

 q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
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
  jointState_msg_robot.data.resize(6);

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
    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }

    /*---------------------*/
        //table coordinates: Y = 0.15 X= 0.1 Z= 0.7

        //to user max= X=-0.4
    /*------------------*/
    

    //MatrixXd support = DirectKinematicsUr5(thi);
    //Vector3d startPos = support.block<3,1>(0,3);
    //Vector3d startPos = {0.3, 0.3, 0.1};
    //Vector3d startPos = { 0.151850 , 0.190809, 0.5};
    //Vector3d startOrient = { 0 , 0, 0};


    //cout << startPos << endl;

    //endPos << startPos(0)+0.05, startPos(1)+0.05, startPos(2)+0.05;
    //Vector3d endPos2 =  worldToRobot(endPos);
    //Vector3d endPos = { -0.4, 0, 0.79}; //contains maximum depth

    Vector3d endPos = { -0.4, -0.2, 0.7};

    Vector3d firstPos = { -0.0011723, -0.199997, 0.566109};
    Vector3d secondPos = { -0.136458, -0.146217, 0.566109};

    Vector3d endOrient = { 0, 0, 0};

    Vector3d posArc = {0.182291, -0.201256, 0.566109}; //current pos of end effetocr

    //Vector3d endPos1 = { -0.2, -0.3, 0.6};
    //Vector3d endPos1 = { -0.3, -0.4, 0.6};
    //Vector3d endOrient = { 0 , 0, 0};

    //double samples = 10;
    double delta = 1/steps;

    cout <<  delta << endl;
    


    //cout << toRotationMatrix(startOrient) << endl;

    //MatrixXd thi = InverseKinematicsUr5(startPos, toRotationMatrix(startOrient)); //we obtain the matrix of all possible solutions
    
    //th = thi.block<1,6>(0,0);

    //th << q_des0(0), q_des0(1), q_des0(2), q_des0(3), q_des0(4), q_des0(5);


    
    //MatrixXd result = InverseDiffKinematicsUr5(th,  endPos, endOrient, 0, 1, delta);
    //MatrixXd result = InverseDiffKinematicsUr5(th,  endPos, endOrient, 0, 1, delta);

    
    MatrixXd tot_trajectory =  InverseDiffKinematicsUr5(q_des0, endPos, endOrient, 0, 1, delta, LIN);

    MatrixXd points(1,3);
    points << 0,0,0;
    
    send_des_jstate(q_des0);
    ros::spinOnce();
    ros::Duration(3).sleep();

    
    for(int i=0; i< tot_trajectory.rows(); i++){
        MatrixXd mat = DirectKinematicsUr5(tot_trajectory.block<1,6>(i,0));
        Vector3d point = mat.block<3,1>(0,3);
        send_des_jstate(tot_trajectory.block<1,6>(i,0));
        ros::spinOnce();
        //loop_time++;
        loop_rate.sleep();
        points.conservativeResize(points.rows()+1, points.cols());
        points.block<1,3>(points.rows()-1, 0) = point;
    }

    

    /*
    MatrixXd mat = DirectKinematicsUr5(q_des0);
    Vector3d vect = mat.block<3,1>(0,3);

    cout << vect << endl;
    cout << tangentialPoint(vect) << endl;*/
   

    /*
    for(int k = 0; k < tot_trajectory.rows()-1; k++){

        MatrixXd mat = DirectKinematicsUr5(tot_trajectory.block<1,6>(k,0));
        inPos = mat.block<3,1>(0,3);
        cout << sqrt(inPos(0)*inPos(0) + inPos(1)*inPos(1))<< endl;

    }*/



    //MatrixXd tot_trajectory(trajectory1.rows()+trajectory2.rows(), trajectory1.cols());
    //tot_trajectory << trajectory1, trajectory2;


    //MatrixXd  trajectory2 = InverseDiffKinematicsUr5(q_des0, endPos, endOrient, 0, 1, delta);

    /*
    for(int j=0; j<samples; j++){
        //ROS_INFO("Hello");
        rate.sleep();
        cout << result.block<1,6>(j,0) << endl;
    }*/

    /********************************************************************************************/
  
    //q_des0 << th(0), th(1), th(2), th(3), th(4), th(5);
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
        if (loop_time < 200.)
        {
            if(loop_time < 2)
                send_des_jstate(q_des0);

        } else {

            JointStateVector delta_q;
            
            //th = result.block<1,6>(i,0);

            if( (i % (int) samples) == 0 ){
                cout << endl;
                cout << endl;
                cout << endl;
                cout << "passaggio di traiettoria" << endl;
                cout << endl;
                cout << endl;
                cout << endl;
            }

            q_des << tot_trajectory(i,0), tot_trajectory(i,1), tot_trajectory(i,2), tot_trajectory(i,3), tot_trajectory(i,4), tot_trajectory(i,5);
            
            
            i++;
            //cout << endl;
            //cout << i << endl;
            if(i == tot_trajectory.rows()-1 ){
                q_des0 = q_des;
                break;
            }
            send_des_jstate(q_des);
        }
        

        //2- sine reference
        // q_des = q_des0.array() + amp.array()*(2*M_PI*freq*loop_time).array().sin();

        ros::spinOnce();
        loop_time++;
        loop_rate.sleep();

    }*/
    cout << "HELo" << endl;

  return 0;
}