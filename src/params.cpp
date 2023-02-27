#include "params.hpp"

bool real_robot(){
    return p_real_robot;
}

bool soft_gripper(){
    return p_soft_gripper;
}

bool gripper_sim(){
    return p_gripper_sim;
}

bool* set_real_robot(){
    return &p_real_robot;
}

bool* set_soft_gripper(){
    return &p_soft_gripper;
}

bool* set_gripper_sim(){
    return &p_gripper_sim;
}