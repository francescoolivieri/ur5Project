/**
 * @file main.cpp
 * @author Federico Adami, Francesco Olivieri, Eddie Veronese
 * @brief 
 * @version 0.1
 * @date 2023-06-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <math.h>
#include "robot.hpp"
#include "blocks.hpp"
#include "link_attacher.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace Mathutils;

vector<string> models_list;

int main(int argc, char **argv){
    init();
    
    Robot robot;

    cout << "FIRST STEP: " ;
    Vector3d model_pose = get_pose("x1-y3-z2");
    cout << " OK " << endl;

    cout << "SECOND STEP: " ;
    Vector3d model_orient = get_orientation("x1-y3-z2");
    cout << "OK " << endl;

    cout << "THIRD STEP: ";
    robot.set_block_up_right(worldToRobot(model_pose), model_orient); 
    cout << "OK " << endl;

    /*
    robot.move( worldToRobot(model_pose) , {0,0,0});
    get_list_models(models_list);
    string model = robot.get_string_nearest_model(models_list);
    attach("ur5", "hand_1_link", model.c_str(), "link");

    model_pose = get_pose("x1-y2-z2");
    model_pose[2] += 0.2;

    robot.move( worldToRobot(model_pose), {0,0,0});
    attach("tavolo", "link", model.c_str(), "link");
    detach("ur5", "hand_1_link", model.c_str(), "link");

    model_pose[2] += 0.3;
    robot.move( worldToRobot(model_pose), {0,0,0});
    */

    cout << "HELo" << endl;
    return 0;

}