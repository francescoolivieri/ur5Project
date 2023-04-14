#include "blocks.hpp"

#include <ros_impedance_controller/finals.h>
#include <ros_impedance_controller/final.h>

Lego::Lego(){
   // this->type = "NULL";
}

Lego::Lego(string type, double x_base, double y_base, double z_base, double yaw, double pitch, double roll){
    this->type = type;
    this->x_base = x_base;
    this->y_base = y_base;
    this->z_base = z_base;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
}

Blocks::Blocks(){
    ros_impedance_controller::finals::ConstPtr msg = ros::topic::waitForMessage<ros_impedance_controller::finals>("/messaggi");

    Lego new_list[msg->length];
    this->list = new_list;
    for(int i=0 ; i<msg->length ; i++){
        list[i] = Lego(msg->finals[i].type, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll );
    }

    this->size = msg->length;
}

void Blocks::update_blocks_pos(){
    this->list = NULL;

    ros_impedance_controller::finals::ConstPtr msg = ros::topic::waitForMessage<ros_impedance_controller::finals>("/messaggi");

    Lego new_list[msg->length];
    this->list = new_list;
    for(int i=0 ; i<msg->length ; i++){
        list[i] = Lego(msg->finals[i].type, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll );
    }

    this->size = msg->length;
}

Lego Blocks::get_block(int index){
    if(this->size > 0)
        return list[index];
    else
        return Lego();
}