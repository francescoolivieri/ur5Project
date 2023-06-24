#ifndef BLOCKS_HPP
#define BLOCK_HPP

#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"

using namespace std;

static ros::Subscriber sub_blocks;

class Lego{
    public:
        string classe;
        double x_base;
        double y_base;
        double z_base;
        double yaw;
        double pitch;
        double roll;

        Lego();
        Lego(string classe, double x_base, double y_base, double z_base, double yaw, double pitch, double roll);
};

class Blocks{
    private:
        Lego *list;
        int length;
        //vector<Block> list;

    public:
        Blocks();

        void update_blocks_pos();
        Lego get_block(int index);
        Lego get_block(string model_name);
        int get_length();

};

#endif