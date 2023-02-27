#ifndef ROBOT_HPP
#define ROBOT_HPP

bool p_real_robot = false;
bool p_soft_gripper = true;
bool p_gripper_sim = true;

bool real_robot();
bool soft_gripper();
bool gripper_sim();

bool* set_real_robot();
bool* set_soft_gripper();
bool* set_gripper_sim();

#endif