#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "custom_joint_pub.hpp"
#include "ur5_kinematics.h"

/* SET THIS CONSTANT TO TRUE IF YOU WANT TO UPDATE JOINTS FROM
    THE REAL ROBOT */
#define UPDATE_FROM_ROBOT 0

using namespace std;
using namespace Eigen;

//#define GRIPPER_LENGTH 0.12

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 9, 1> Vector9d;

bool real_robot;
bool soft_gripper;
bool gripper_sim;

class Joints{
    public:
        double shoulder_pan;
        double shoulder_lift;
        double elbow;
        double wrist_1;
        double wrist_2;
        double wrist_3;

        double gripper_1;
        double gripper_2;
        double gripper_3;

        Joints();
        Joints(Vector6d q_arm);
        Joints(Vector6d q_arm, Vector3d q_gripper);

        void set_new(Vector6d q_arm, Vector3d q_gripper);
        void set_new(Vector6d q_arm);
        void set_new(Vector3d q_gripper);

        void set_joints_from_robot();

        Vector6d get_arm();
        Vector3d get_gripper();

};

class Robot{
    private:
       // vector<double> alp = {0., M_PI/2, 0., 0., M_PI/2, -M_PI/2};
       // vector<double> dist = {0.1625, 0., 0., 0.1333, 0.0997, 0.0996+GRIPPER_LENGTH};
       // vector<double> a = {0., 0., -0.425, -0.3922, 0., 0.};

        Joints joints;
        double gripper_diameter;  // can be useful to se the actual state of the gripper

        double mapToGripperJoints(double diameter);
    
    public:
        Robot();
        Robot(Joints joints);
        Robot(Vector6d q_arm, Vector3d q_gripper);
        Robot(Vector6d q_arm);

        void move(Vector3d finalPos, Vector3d finalOrient);
        void rotate(Vector3d finalPos, Vector3d finalOrient);
        void move_gripper(double diameter);
        string get_string_nearest_model(vector<string> models_list);

        void set_new_gripper_position(double diameter);
    
};

#endif