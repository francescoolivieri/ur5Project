#ifndef CUSTOM_JOINT_PUB_HPP
#define CUSTOM_JOINT_PUB_HPP

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <ros_impedance_controller/generic_float.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 9, 1> Vector9d;

static double loop_time = 0.;
static double loop_frequency = 60.;

extern bool real_robot;
extern bool soft_gripper;
extern bool gripper_sim;

// Publishers
//std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_des_jstate_sim_rt;
static ros::Publisher pub_des_jstate;
static ros::Subscriber sub_jstate;

static sensor_msgs::JointState jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_robot;

void init();
void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_pos);

/* !!! GRIPPED SHOULD BE MOVED ONCE AT TIME ON THE REAL ROBOT -> WHEN YOU ARRIVED AT DESTINATION !!! */
void robot_send_des_jstate(const Vector6d & joint_pos, const double diameter, const bool move_gripper);
void robot_move_gripper(const double diameter);

Vector9d receive_jstate();

#endif