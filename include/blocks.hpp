#ifndef BLOCKS_HPP
#define BLOCK_HPP

#include <string>
#include <vector>
#include "ros/ros.h"

using namespace std;

static ros::Subscriber sub_blocks;

class Block{
    public:
        string type;
        double x_base;
        double y_base;
        double z_base;
        double yaw;
        double pitch;
        double roll;

        Block();
        Block(string type, double x_base, double y_base, double z_base, double yaw, double pitch, double roll);
};

class Blocks{
    private:
        vector<Block> list;

    public:
        Blocks();

        void update_blocks_pos();
        Block get_block(int index);

};

#endif