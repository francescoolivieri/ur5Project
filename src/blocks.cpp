#include "blocks.hpp"

#include <ros_impedance_controller/finals.h>
#include <ros_impedance_controller/final.h>

Lego::Lego(){
   
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

Models::Models(){
    ros_impedance_controller::finals::ConstPtr msg = ros::topic::waitForMessage<ros_impedance_controller::finals>("/messaggi");

    Lego new_list[msg->length];
    this->list = new_list;
    for(int i=0 ; i<msg->length ; i++){
        list[i] = Lego(msg->finals[i].type, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll );
    }

    this->length = msg->length;
}

void Models::update_blocks_pos(){
    this->list = NULL;

    ros_impedance_controller::finals::ConstPtr msg = ros::topic::waitForMessage<ros_impedance_controller::finals>("/messaggi");

    Lego new_list[msg->length];
    this->list = new_list;
    for(int i=0 ; i<msg->length ; i++){
        list[i] = Lego(msg->finals[i].type, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll );
    }

    this->length = msg->length;
}

Lego Models::get_block(int index){
    if(this->length > 0)
        return list[index];
    else
        return Lego();
}

Lego Models::get_block(string model_name){
    for (int i=0; i<length ; i++){
        if( model_name == list[i].type )
            return list[i];
    }
     return Lego();
}

int Models::get_lenght(){
    return length;
}