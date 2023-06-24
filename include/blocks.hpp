#ifndef BLOCKS_HPP
#define BLOCK_HPP

#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"

using namespace std;

static ros::Subscriber sub_blocks;

/**
 * @class Lego
 * @brief Class representing a Lego object
 */
class Lego{
    public:
        string classe;
        double x_base;
        double y_base;
        double z_base;
        double yaw;
        double pitch;
        double roll;

        /**
         * @brief Default Constructor         * 
         */
        Lego();

        /**
         * @brief Construct a new Lego object
         * 
         * @param type Name of the Lego
         * @param x_base Position of the Lego on the x-axis
         * @param y_base Position of the Lego on the y-axis
         * @param z_base Position of the Lego on the z-axis
         * @param yaw Orientation of the Lego on the z-axis
         * @param pitch Orientation of the Lego on the y-axis
         * @param roll Orientation of the Lego on the x-axis
         * 
         * @remarks Position must be given from the point of view of the robot base
         */
        Lego(string classe, double x_base, double y_base, double z_base, double yaw, double pitch, double roll);
};

/**
 * @class Models
 * @brief Class that retrieves 
 */
class Models{
    private:
        Lego *list;
        int length;
        
    public:

        /**
         * @brief Default Constructor.
         * Class 
         * 
         */
        Models();

        void update_blocks_pos();
        Lego get_block(int index);
        Lego get_block(string model_name);
        int get_length();


};

#endif