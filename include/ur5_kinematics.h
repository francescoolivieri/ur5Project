#ifndef UR5_KINEMATICS_H
#define UR5_KINAMETICS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>

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


namespace Kinematics{
    Matrix4d directKinematicsUr5(VectorXd th);
    MatrixXd inverseKinematicsUr5(Vector3d pe, Matrix3d Re);

    Vector3d attrForce_pos(Vector3d error);
    Vector3d repulForce(Vector3d xe);
    Vector3d desPos(Vector3d xe, Vector3d xf);
    Vector3d desOrient(Vector3d phie, Vector3d phif);
    Vector3d attrForce_orient(Vector3d error);

    VectorXd dotQ(RowVectorXd qk, Vector3d xe, Vector3d xd, Matrix3d Re, Vector3d phid);
    MatrixXd inverseDiffKinematicsUr5(VectorXd th, Vector3d endPos, Vector3d endOrientation);
    //VectorXd q_dott0(VectorXd qk);
    Vector3d velocity(Vector3d xe, Vector3d xd, double delta);
    MatrixXd jointSpace_kinematics(VectorXd qk, Vector3d endPos, Vector3d endOrient );
    VectorXd nearest_config(VectorXd qk, MatrixXd val);
    Vector3d positionError(Vector3d xe, Vector3d xd);
    Vector3d orientationError(Matrix3d w_R_e, Matrix3d w_R_d);


    //Quaternions
    MatrixXd inverseDiffKinematicsUr5Quaternions(VectorXd q_k, Vector3d endPos, Vector3d endOrient);
    VectorXd ComputeErrorQuaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des);
    VectorXd dotQquaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des, Vector3d v_des, Vector3d w_des);
}

namespace Mathutils{

    Vector3d worldToRobot(Vector3d p);
    Vector3d robotToWorld(Vector3d p);
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