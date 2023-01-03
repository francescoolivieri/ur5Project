#ifndef UR5_KINEMATICS_H
#define UR5_KINAMETICS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>

#define SCALE_FACTOR 1
#define GRIPPER_LENGTH 0.12

using namespace std;
using namespace Eigen;


//methods
Matrix4d DirectKinematicsUr5(VectorXd th);
MatrixXd InverseKinematicsUr5(Vector3d pe, Matrix3d Re);
Matrix4d getT_i(int i, double th);

Vector3d worldToRobot(Vector3d p);
Vector3d robotToWorld(Vector3d p);

MatrixXd ur5Jacobian(VectorXd th);
Vector3d TrajectoryPosition(double currentTime, double totalDuration, Vector3d startPos, Vector3d endPos);
Vector3d TrajectoryOrientation(double currentTime, double totalDuration, Vector3d startOrient, Vector3d endOrient);

VectorXd JointAngularVelocity(RowVectorXd qk, Vector3d xe, Vector3d xd, Vector3d vd, Vector3d phie, Vector3d phid, Vector3d phiddot );

MatrixXd InverseDiffKinematicsUr5(RowVectorXd th, Vector3d endPos, Vector3d endOrientation,  double tMin, double tMax, double DeltaT);
Matrix3d toRotationMatrix(Vector3d euler); //euler angles in x, y, z

//variables

bool first = true;

vector<double> alp = {0., M_PI/2, 0., 0., M_PI/2, -M_PI/2};
vector<double> dist = {0.1625, 0., 0., 0.1333, 0.0997, 0.0996+GRIPPER_LENGTH};
vector<double> a = {0., 0., -0.425, -0.3922, 0., 0.}; // ?


Matrix4d t_60 = Matrix4d::Identity();
Matrix4d t0b = Matrix4d::Identity();
Matrix4d te6 = Matrix4d::Identity();


#endif