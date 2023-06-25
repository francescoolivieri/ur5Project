/**
 * @file ur5_kinematics.h
 * @author Federico Adami, Francesco Olivieri, Eddie Veronese
 * @brief Library that contains all the functions and utilities to handle the kinematics of the ur5 properly
 * @version 0.1
 * @date 2023-06-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef UR5_KINEMATICS_H
#define UR5_KINAMETICS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>
#include <ros/console.h>

#define SCALE_FACTOR 1
#define GRIPPER_LENGTH 0.15 // needs to be replaced with 0.12 later (0 is just for testing)
#define RADIUS 0.20

static double steps = 400;


typedef enum {
    SIN,
    CIRC,
    LIN
}mode;

using namespace std;
using namespace Eigen;

/**
 * @namespace Kinematics
 * @brief Contains all the functions related with the kinematic of the robot arm.
 */
namespace Kinematics{

    /**
     * @brief This function calculate the direct kinematics of the ur5.
     * 
     * @param th Vector of 6 elements that contains the state of the ur5-joints
     * @return Matrix4d Matrix representing orientation and position of the end-effector 
     */
    Matrix4d directKinematicsUr5(VectorXd th);

    /**
     * @brief This function calculate the inverse kinematics of the ur5.
     * 
     * @param pe Position of the end effector
     * @param Re Orientation of the end effector
     * @return MatrixXd Represent the 8 different configurations of joints to achieve the position and orientation specified (every row is a different configuration)
     */
    MatrixXd inverseKinematicsUr5(Vector3d pe, Matrix3d Re);

    /**
     * @brief This function calculates the attractive force based on position error (normalized).
     * 
     * @param error Difference between where actual position and final position
     * @return Vector3d Attractive force towards the final postion I want to achieve
     */
    Vector3d attrForce_pos(Vector3d error);

    /**
     * @brief This function calculates the repulsive force that keeps the end effector away from the point of shoulder singularity
     * 
     * @param xe Current end effector position
     * @return Vector3d Repulsive force
     */
    Vector3d repulForce(Vector3d xe);

    /**
     * @brief This function calculates the desired position of the end effector, it combines the attractive force towards the target position and the repulsive force from point of singularity.
     * 
     * @param xe Current end effector position
     * @param xf Target end effector position
     * @return Vector3d Desired position of the end effector (step towards the target position)
     */
    Vector3d desPos(Vector3d xe, Vector3d xf);

    /**
     * @brief This function calculates the desired orientation of the end effector
     * 
     * @param phie Current end effector orientation
     * @param phif Target end effector orientation 
     * @return Vector3d Desired orientation of the end effector (step towards the target orientation)
     */
    Vector3d desOrient(Vector3d phie, Vector3d phif);

    /**
     * @brief This function calculates the attractive force based on orientation error (normalized).
     * 
     * @param error Difference between where actual position and final orientation
     * @return Vector3d Attractive force towards the final orientation I want to achieve
     */
    Vector3d attrForce_orient(Vector3d error);

    VectorXd dotQ(RowVectorXd qk, Vector3d xe, Vector3d xd, Matrix3d Re, Vector3d phid);
    MatrixXd inverseDiffKinematicsUr5(VectorXd th, Vector3d endPos, Vector3d endOrientation);
    MatrixXd jointSpace_kinematics(VectorXd qk, Vector3d endPos, Vector3d endOrient );
    VectorXd nearest_config(VectorXd qk, MatrixXd val);


    /**
     * @brief This function calculate the error between the actual position and the desired one
     * 
     * @param xe Current end effector position
     * @param xd Desired end effector position
     * @return Vector3d Position error
     */
    Vector3d positionError(Vector3d xe, Vector3d xd);

    /**
     * @brief This function calculates the orentation error between the desired and the current one. (Provided by Focchi)
     * 
     * @param w_R_e Current orientation of the end effector represented as a rotation matrix
     * @param w_R_d Desider orientation of the end effector represented as a rotation matrix
     * @return Vector3d Oritentation error
     */
    Vector3d orientationError(Matrix3d w_R_e, Matrix3d w_R_d);


    //Quaternions

    /* This set of functions are not recommended,  */
    MatrixXd inverseDiffKinematicsUr5Quaternions(VectorXd q_k, Vector3d endPos, Vector3d endOrient);
    VectorXd ComputeErrorQuaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des);
    VectorXd dotQquaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des, Vector3d v_des, Vector3d w_des);
}

/**
 * @namespace Mathutils
 * @brief Contains all the mathematical tools useful for the calculations about the ur5-robot
 */
namespace Mathutils{

    /**
     * @brief Function that calculates the change of base frame starting from a world frame point to a point with respect to the base frame of the robot
     * 
     * @param p World frame point
     * @return Vector3d Robot base frame point
     */
    Vector3d worldToRobot(Vector3d p);

    /**
     * @brief Function that calculates the change of base frame starting from a robot base frame point to a point with respect to the world frame
     * 
     * @param p Robot base frame point 
     * @return Vector3d World frame point
     */
    Vector3d robotToWorld(Vector3d p);

    /**
     * @brief Get the T i object
     * 
     * @param i 
     * @param th 
     * @return Matrix4d 
     */
    Matrix4d getT_i(int i, double th);
    MatrixXd ur5Jacobian(VectorXd th);

    Matrix3d eulerToRotationMatrix(Vector3d euler); //euler angles in x, y, z
    double vectMagnitude(const Vector3d &v);
    double quatMagnitude(const Quaterniond &q);
    double centerDist(Vector3d p);
    int touchCenterCircle(Vector3d start_pos, Vector3d end_pos);
    Quaterniond EulerToQuaternion(Vector3d euler);
    Vector3d tangentialPoint(Vector3d start_pos);
}

//variables
static double value;
static bool first = true;

static vector<double> alp = {0., M_PI/2, 0., 0., M_PI/2, -M_PI/2};
static vector<double> dist = {0.1625, 0., 0., 0.1333, 0.0997, 0.0996+GRIPPER_LENGTH};
static vector<double> a = {0., 0., -0.425, -0.3922, 0., 0.}; // ?


static Matrix4d t_60 = Matrix4d::Identity(); 
static Matrix4d t0b = Matrix4d::Identity();  
static Matrix4d te6 = Matrix4d::Identity();  

typedef Matrix<double, 6, 1> Vector6d;


#endif