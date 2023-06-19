#include "link_attacher.hpp"

ros::ServiceClient attach_client;
ros::ServiceClient detach_client;
ros::ServiceClient get_model_client;
ros::ServiceClient get_list_models_client;
gazebo_ros_link_attacher::Attach link_attacher_srv;

void attach(const char* model1, const char* link1, const char* model2, const char* link2)
{
    link_attacher_srv.request.model_name_1 = model1;
    link_attacher_srv.request.link_name_1 = link1;
    link_attacher_srv.request.model_name_2 = model2;
    link_attacher_srv.request.link_name_2 = link2;
    if (!attach_client.call(link_attacher_srv))
        ROS_INFO_STREAM("Attach failed");
}

void detach(const char* model1, const char* link1, const char* model2, const char* link2)
{
    link_attacher_srv.request.model_name_1 = model1;
    link_attacher_srv.request.link_name_1 = link1;
    link_attacher_srv.request.model_name_2 = model2;
    link_attacher_srv.request.link_name_2 = link2;
    if (!detach_client.call(link_attacher_srv))
        ROS_INFO_STREAM("Detach failed");
}

Vector3d get_pose(string model_name){
    gazebo_msgs::GetModelState gms;
    gms.request.model_name=model_name.c_str();
    if (!get_model_client.call(gms))
        ROS_ERROR("Get model pose failed");

    Vector3d pose;
    pose << gms.response.pose.position.x , gms.response.pose.position.y, gms.response.pose.position.z;
    return pose;
}

void get_list_models(vector<string> &list_models){
    gazebo_msgs::GetWorldProperties gwp;
    if (!get_list_models_client.call(gwp))
        ROS_INFO_STREAM("Get list models failed");

    for(int i=0; i<gwp.response.model_names.size(); i++){
        if(gwp.response.model_names[i][0] == 'x'){
            list_models.push_back(gwp.response.model_names[i]);
        }
    }
}