/**
 * @file robot.hpp
 * @author Federico Adami, Francesco Olivieri, Pirrinz Perroneddesi
 * @brief This file contains two classes that are crucial. They provide to the final user a good level of abstraction and an easy management of the ur5 robotic arm.
 * @version 0.1
 * @date 2023-06-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "custom_joint_pub.hpp"
#include "link_attacher.hpp"
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

static double working_height = Mathutils::robotToWorld({0, 0, 1.1})(2);
static double grasping_height = Mathutils::worldToRobot({0, 0, 0.89})(2);
static double releasing_height = Mathutils::worldToRobot({0, 0, 0.92})(2); //0.94
static double height_offset = 0.22;

/**
 * @class Joints
 * @brief Class representing the robot's joints, used to both store and update.
 * 
 * @remarks It might change behaviour, if UPDATE_FROM_ROBOT is defined all the joints will be updated according their actual state published on the ROS topic: "/ur5/joint_states".
 */
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

        /**
         * @brief Construct a new Joints object.
         * It sets all the joints to their actual values.
         * 
         * @remarks This function could be blocking, if UPDATE_FROM_ROBOT is defined (waits for messages).
         */
        Joints();

        /**
         * @brief Construct a new Joints object-
         * It sets all the joints of ur5-arm to the specified passed by argument.
         * 
         * @param q_arm Vector (6 elements) of arm joints
         */
        Joints(Vector6d q_arm);

        /**
         * @brief Construct a new Joints object.
         * It sets all the joint of ur5-arm and ur5-gripper to the specified passed by argument.
         * 
         * @param q_arm Vector (6 elements) of arm joints
         * @param q_gripper Vector (3 elements) of gripper joints
         */
        Joints(Vector6d q_arm, Vector3d q_gripper);

        /**
         * @brief Set new arm and gripper set of joints.
         * 
         * @param q_arm Vector (6 elements) of arm joints
         * @param q_gripper Vector (3 elements) of gripper joints
         */
        void set_new(Vector6d q_arm, Vector3d q_gripper);

        /**
         * @brief Set new arm joints
         * 
         * @param q_arm Vector (6 elements) of arm joints
         */
        void set_new(Vector6d q_arm);

        /**
         * @brief Set new gripper joints
         * 
         * @param q_gripper Vector (3 elements) of gripper joints
         */
        void set_new(Vector3d q_gripper);

        /**
         * @brief Update the joints taking the values from the proper ROS topic 
         * 
         * @remarks This function waits until a message is received on the proper topic (/ur5/joint_states)
         */
        void set_joints_from_robot();
        
        /**
         * @brief Get the ur5-arm joints
         * 
         * @return Vector6d Vector of 6 elements representing the joints of the ur5-arm in the following order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
         */
        Vector6d get_arm();

        /**
         * @brief Get the ur5-gripper joints
         * 
         * @return Vector3d Vector of 3 elements representing the joints of the gripper
         */
        Vector3d get_gripper();

};

/**
 * @class Robot
 * @brief Class representing a robot, it provides various functionalities to use it with ease.
 */
class Robot{
    private:
       // vector<double> alp = {0., M_PI/2, 0., 0., M_PI/2, -M_PI/2};
       // vector<double> dist = {0.1625, 0., 0., 0.1333, 0.0997, 0.0996+GRIPPER_LENGTH};
       // vector<double> a = {0., 0., -0.425, -0.3922, 0., 0.};

        Joints joints;
        double gripper_diameter;

        /**
         * @brief Function used to calculate the joints of the gripper starting from the diameter value
         * 
         * @param diameter The diameter value to move the gripper to.
         * @return double new value of joints of the gripper
         */
        double mapToGripperJoints(double diameter);
    
    public:

        /**
         * @brief Default constructor for Robot class.
         * It sets the joints using the default constructor
         */
        Robot();

        /**
         * @brief Construct a new Robot object by giving the actual set of joints
         * 
         * @param joints Set of joints that should represent the actual state of the joints
         */
        Robot(Joints joints);

        /**
         * @brief Construct a new Robot object by giving the actual set of joints separated in arm and gripper joints
         * 
         * @param q_arm Vector (6 elements) of arm joints
         * @param q_gripper Vector (3 elements) of gripper joints
         */
        Robot(Vector6d q_arm, Vector3d q_gripper);

        /**
         * @brief Construct a new Robot object by giving the actual set of joints of the ur5-arm
         * 
         * @param q_arm Vector (6 elements) of arm joints
         */
        Robot(Vector6d q_arm);

        /**
         * @brief Retrieves the joints of the wrist (last 3 joints)
         * 
         * @return Vector3d Joints of the wrist
         */
        Vector3d get_wrist();

        /**
         * @brief Retrieves the joints of the shoulder (first 3 joints)
         * 
         * @return Vector3d Joints of the wrist
         */
        Vector3d get_shoulder();

        /**
         * @brief Moves the robot to the specified final position and orientation.
         * 
         * @param finalPos Vector representing the final position from the point of view of the robot
         * @param finalOrient Vector representing the final orientation
         * 
         * @remarks Position must be given from the point of view of the robot base
         */
        void move(Vector3d finalPos, Vector3d finalOrient);

        /**
         * @brief Rotates the robot to the specified final orientation.
         * 
         * @param finalPos Vector representing the final position
         * @param finalOrient Vector representing the final orientation
         * 
         * @remarks Position must be given from the point of view of the robot base
         */
        void rotate(Vector3d finalPos, Vector3d finalOrient);

        /**
         * @brief Moves the gripper to the specified diameter value.
         *
         * @param diameter The diameter value to move the gripper to.
         */
        void move_gripper(double diameter);

        /**
         * @brief Retrieves the name of the model closest to the end effector
         *
         * @param models_list List of models to look into
         * @return The name of the model nearest to the end effector.
         */
        string get_string_nearest_model(vector<string> models_list);

        /**
         * @brief Sets the gripper to a new position based on the given diameter.
         * 
         * @param diameter The diameter value determining the new gripper position.
         */
        void set_new_gripper_position(double diameter);

        /**
         * @brief Sets the block in an upright position using the UR5 robot arm.
         *
         * This function moves the UR5 robot arm to manipulate a block and bring it into an upright position.
         * 
         *
         * @param model_pose Vector of 3 elements representing the pose .
         * @param model_rotation Vector of 3 elements representing the rotation of the block (yaw, pitch, roll).
         *
         * @remarks This function assumes that the block is initially placed at a position where it can be easily reached by the robot arm.
         * @remarks Position must be given from the point of view of the robot base
         */
        void set_block_up_right(Vector3d model_pose, Vector3d model_rotation);
    
};

#endif