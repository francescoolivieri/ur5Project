#include "robot.hpp"

Joints::Joints(){
    this->set_joints_from_robot();
}

Joints::Joints(Vector6d q_arm){
    this->set_new(q_arm);
}

Joints::Joints(Vector6d q_arm, Eigen::Vector3d q_gripper){
    this->set_new(q_arm, q_gripper);
}

void Joints::set_new(Vector6d q_arm, Eigen::Vector3d q_gripper){
    this->shoulder_pan = q_arm(0);
    this->shoulder_lift = q_arm(1);
    this->elbow = q_arm(2);
    this->wrist_1 = q_arm(3);
    this->wrist_2 = q_arm(4);
    this->wrist_3 = q_arm(5);
    this->gripper_1 = q_gripper(0);
    this->gripper_2 = q_gripper(1);
    this->gripper_3 = q_gripper(2);
}

void Joints::set_new(Vector6d q_arm){
    this->shoulder_pan = q_arm(0);
    this->shoulder_lift = q_arm(1);
    this->elbow = q_arm(2);
    this->wrist_1 = q_arm(3);
    this->wrist_2 = q_arm(4);
    this->wrist_3 = q_arm(5);
}

void Joints::set_new(Eigen::Vector3d q_gripper){
    this->gripper_1 = q_gripper(0);
    this->gripper_2 = q_gripper(1);
    this->gripper_3 = q_gripper(2);
}

Vector6d Joints::get_arm(){
    Vector6d v;
    v << this->shoulder_pan, this->shoulder_lift, this->elbow, this->wrist_1, this->wrist_2, this->wrist_3;
    return v;
}

Vector3d Joints::get_gripper(){
    Eigen::Vector3d v;
    v << this->gripper_1, this->gripper_2, this->gripper_3;
    return v;
}

void Joints::set_joints_from_robot(){
    Vector9d v = receive_jstate();

    Vector6d v_arm = v.block<6,1>(0,0); 
    Vector3d v_gripper = v.block<3,1>(6,0);
    
    this->set_new(v_arm);
    this->set_new(v_gripper);
}

Robot::Robot(){
    this->joints = Joints();
}

Robot::Robot(Joints joints){
    this->joints = joints;
}
        
Robot::Robot(Vector6d q_arm, Vector3d q_gripper){
    this->joints = Joints(q_arm, q_gripper);
}

Robot::Robot(Vector6d q_arm){
    this->joints = Joints(q_arm);
}

double Robot::mapToGripperJoints(double diameter){
    if(soft_gripper){
        int D0 = 40;
        int L = 60;
        double delta = 0.5*(diameter - D0);

        return atan2(delta, L);
    }else{
        return (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    }
}

void Robot::set_new_gripper_position(double diameter){

    double q = mapToGripperJoints(diameter);

    Vector3d v;
    v << q, q, q;

    this->joints.set_new(v);
    this->gripper_diameter = diameter;
}

void Robot::move_gripper(double diameter){

    if(real_robot){
        robot_move_gripper(diameter);
    }else{
        /* CAN BE DONE GRADUALLY */
        this->set_new_gripper_position(diameter);

        send_des_jstate(this->joints.get_arm(), this->joints.get_gripper());
    }
}

void Robot::move(Vector3d finalPos, Vector3d finalOrient){
    ros::Rate loop_rate(loop_frequency);

    double samples = 50;

    double delta = 1/steps;

    //cout << this->joints.get_arm() << endl;

    VectorXd startConf(6);
    startConf << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
    
    MatrixXd tot_trajectory = inverseDiffKinematicsUr5(startConf, finalPos, finalOrient);
    

    //this->joints.get_arm()
    
     for(int i=0; i< tot_trajectory.rows(); i++){
         //MatrixXd mat = directKinematicsUr5(tot_trajectory.block<1,6>(i,0));
         send_des_jstate(tot_trajectory.block<1,6>(i,0), this->joints.get_gripper());
         ros::spinOnce();
         //loop_time++;
         loop_rate.sleep();
         //points.conservativeResize(points.rows()+1, points.cols());
         //points.block<1,3>(points.rows()-1, 0) = point;
     }

    this->joints.set_joints_from_robot();
}