#ifndef LINK_ATTACHER_HPP
#define LINK_ATTACHER_HPP

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>
#include "gazebo_ros_link_attacher/Attach.h"
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace Eigen;
using namespace std;

static const char ur5_model[] = "ur5";
static const char ur5_hand_link[] = "hand_1_link";
static const char table_model[] = "tavolo";
static const char table_link[] = "link";

extern ros::ServiceClient attach_client;
extern ros::ServiceClient detach_client;
extern ros::ServiceClient get_model_client;
extern ros::ServiceClient get_list_models_client;

void attach(const char* model1, const char* link1, const char* model2, const char* link2);
void detach(const char* model1, const char* link1, const char* model2, const char* link2);
Vector3d get_pose(string model_name);
void get_list_models(vector<string> &list_models);

#endif