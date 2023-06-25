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
        string type;    // Name of the model
        double x_base;  // Position of the Lego on the x-axis
        double y_base;  // Position of the Lego on the y-axis
        double z_base;  // Position of the Lego on the z-axis
        double yaw;     // Orientation of the Lego on the z-axis
        double pitch;   // Orientation of the Lego on the y-axis
        double roll;    // Orientation of the Lego on the x-axis

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
        Lego(string type, double x_base, double y_base, double z_base, double yaw, double pitch, double roll);
};

/**
 * @class Models
 * @brief Class used to retrieves and store models info.
 */
class Models{
    private:
        Lego *list; // List of Lego found by the vision
        int length; // Length of the array
        
    public:

        /**
         * @brief Default Constructor.
         * Retrieves from the topic the list of Lego found by the vision.
         * 
         * @remarks This function waits until a message is received on the proper topic
         */
        Models();

        /**
         * @brief Retrieves from the topic the list of Lego found by the vision.
         * 
         * @remarks This function waits until a message is received on the proper topic
         */
        void update_blocks_pos();

        /**
         * @brief Get the Lego object in the list at the index specified.
         * 
         * @param index Array index
         * @return Lego Lego Class at the index specified
         */
        Lego get_block(int index);

        /**
         * @brief Get the Lego object in the list with model name equal to the one passed by argument.
         * 
         * @param model_name Name of the model I'm looking for
         * @return Lego Lego with the same name of the parameter passed
         */
        Lego get_block(string model_name);

        /**
         * @brief Get the length of the list of Lego
         * 
         * @return int Length of the list of Lego
         */
        int get_lenght();

};

#endif