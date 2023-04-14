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
        string type;
        double x_base;
        double y_base;
        double z_base;
        double yaw;
        double pitch;
        double roll;

        Lego();
        Lego(string type, double x_base, double y_base, double z_base, double yaw, double pitch, double roll);
};

class Blocks{
    private:
        Lego *list;
        int size;
        //vector<Block> list;

    public:
        Blocks();

        void update_blocks_pos();
        Lego get_block(int index);

};

#endif