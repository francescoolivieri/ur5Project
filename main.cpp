#include <math.h>
#include "custom_joint_publisher.h"
#include "ur5_kinematics.h"
#include <iostream>

using namespace std;

int main () {

    VectorXd th(6);

    th <<   1.48357, -1.0231, 0.549017, -1.09672, 1.5708, 0.0872224;

    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }
    
    Vector3d pe;
    pe << 0.574422, 1.02901, 1.06601; // desired point

    MatrixXd th_m = InverseKinematicsUr5(pe);
    th << th_m(0,0), th_m(0,1), th_m(0,2), th_m(0,3), th_m(0,4), th_m(0,5); // choose one line of possible th

    cout << th << endl << endl;

    Vector3d pe_2 = DirectKinematicsUr5(th); // test inverse kin

    cout << pe_2-pe << endl;

    return 0;
}