#include <math.h>
#include "custom_joint_publisher.h"
#include "ur5_kinematics.h"
#include <iostream>

using namespace std;

int main () {

    VectorXd th(6);

    Vector3d startPos =  {0.3, 0.3, 0.1};
    Vector3d startOrient = {0, 0, 0};

    Vector3d endPos = {-0.1, 0.5, 0.5};
    Vector3d endOrient = {0, M_PI/2, 0};

    MatrixXd thi = InverseKinematicsUr5(startPos, toRotationMatrix(startOrient));

    th = thi.block<1,6>(0,0);

    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }


    
    cout << InverseDiffKinematicsUr5(thi, endPos, endOrient, 0, 1, 0.5);
    
    

    return 0;
}