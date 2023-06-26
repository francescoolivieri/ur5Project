/**
 * @file link_attacher.hpp
 * @author Federico Adami, Francesco Oliveri, Eddie Veronese
 * @brief It provides a useful set of functions that manages the linker service
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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

extern vector<string> models_list;

/**
 * @brief Function that send a request to create a dynamic link to attach the two models specified
 * 
 * @param model1 Name of the first model
 * @param link1 Name of the link in the first model
 * @param model2 Name of the second model
 * @param link2 Name of the link in the second model
 */
void attach(const char* model1, const char* link1, const char* model2, const char* link2);

/**
 * @brief Function that send a request to delete a dynamic link to detach the two models specified
 * 
 * @param model1 Name of the first model
 * @param link1 Name of the link in the first model
 * @param model2 Name of the second model
 * @param link2 Name of the link in the second model
 */
void detach(const char* model1, const char* link1, const char* model2, const char* link2);

/**
 * @brief Function that retrieves the position of the model specified
 * 
 * @param model_name Name of the model
 * @return Vector3d Position of the model 
 */
Vector3d get_pose(string model_name);

/**
 * @brief Function that retrieves the orientation of the model specified
 * 
 * @param model_name Name of the model
 * @return Vector3d Orientation of the model 
 */
Vector3d get_orientation(string model_name);

/**
 * @brief Function that retrieves the list of the models (filter to find only lego models)
 * 
 * @param list_models List of models present
 * 
 * @remarks This function has a filter to retrieve only lego models
 */
void get_list_models(vector<string> &list_models);

#endif