/**
 * @file custom_joint_pub.hpp
 * @author Federico Adami, Francesco Olivieri, Eddie Veronese
 * @brief Set of functions that are useful to manage ros communication
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CUSTOM_JOINT_PUB_HPP
#define CUSTOM_JOINT_PUB_HPP

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <ros_impedance_controller/generic_float.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include "link_attacher.hpp"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 9, 1> Vector9d;

static double loop_time = 0.;
static double loop_frequency = 250.;

extern bool real_robot;
extern bool soft_gripper;
extern bool gripper_sim;

static ros::Publisher pub_des_jstate;
static ros::ServiceClient client_gripper;

static sensor_msgs::JointState jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_robot;

/**
 * @brief Function that initializes: ROS, the params of the robot, publishers, subscribers and service clients
 * 
 * @remarks Can be blocking.
 */
void init();

/**
 * @brief Function that sends to the robot the new joints configuration
 * 
 * @param joint_pos Joints configuration of the arm
 * @param gripper_pos Joints configuration of the gripper
 */
void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_pos);

/* !!! GRIPPED SHOULD BE MOVED ONCE AT TIME ON THE REAL ROBOT -> WHEN YOU ARRIVED AT DESTINATION !!! */
/**
 * @brief Function that sends to the real robot the new joints configuration
 * 
 * @param joint_pos Joint configuration of the arm
 * @param diameter Diameter desired of the gripper
 * @param move_gripper Boolean that indicates the desire to open/close the gripper
 */
void robot_send_des_jstate(const Vector6d & joint_pos, const double diameter, const bool move_gripper);

/**
 * @brief Function that sends the request to move the gripper to the robot
 * 
 * @param diameter Opening diameter desired
 */
void robot_move_gripper(const double diameter);

/**
 * @brief Function that receives the joints configuration from the robot
 * 
 * @return Vector9d Actual joint configuration received from the robot
 * 
 * @remarks Can be blocking.
 */
Vector9d receive_jstate();

#endif