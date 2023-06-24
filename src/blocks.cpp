#include "blocks.hpp"
#include <cstdlib>


#include <final_msgs/finals.h>
#include <final_msgs/final.h>

Lego::Lego(){
   
}

Lego::Lego(string classe, double x_base, double y_base, double z_base, double yaw, double pitch, double roll){
    this->classe = classe;
    this->x_base = x_base;
    this->y_base = y_base;
    this->z_base = z_base;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
}


Models::Models(){
    final_msgs::finals::ConstPtr msg = ros::topic::waitForMessage<final_msgs::finals>("/messaggi");

    this->list = new Lego[msg->length];
    for(int i = 0; i < msg->length; i++){
        list[i] = Lego(msg->finals[i].classe, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll);
    }

    this->length = msg->length;
}

void Models::update_blocks_pos(){
    delete[] this->list; // Dealloca la memoria precedente

    final_msgs::finals::ConstPtr msg = ros::topic::waitForMessage<final_msgs::finals>("/messaggi");

    this->list = new Lego[msg->length];
    for(int i = 0; i < msg->length; i++){
        list[i] = Lego(msg->finals[i].classe, msg->finals[i].x_base, msg->finals[i].y_base, msg->finals[i].z_base, msg->finals[i].yaw, msg->finals[i].pitch, msg->finals[i].roll);
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
        if( model_name == list[i].classe )
            return list[i];
    }
     return Lego();
}

int Models::get_length(){
    return length;
}