#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>

#define SCALE_FACTOR 1
#define GRIPPER_LENGTH 0.12

using namespace std;
using namespace Eigen ;

Vector3d DirectKinematicsUr5(VectorXd th);
MatrixXd InverseKinematicsUr5(Vector3d pe);
Matrix4d getT_i(int i, double th);

vector<double> alp = {0., M_PI/2, 0., 0., M_PI/2, -M_PI/2};
vector<double> dist = {0.1625, 0., 0., 0.1333, 0.0997, 0.0996+GRIPPER_LENGTH};
vector<double> a = {0., 0., -0.425, -0.3922, 0., 0.}; // ?
VectorXd th(6);

Matrix4d t_60 = Matrix4d::Identity();
Matrix4d t0b = Matrix4d::Identity();
Matrix4d te6 = Matrix4d::Identity();

bool first = true;

Vector3d worldToRobot(Vector3d p);

Vector3d robotToWorld(Vector3d p);s

MatrixXd ur5Jacobian(VectorXd th){
    MatrixXd J(6,6);

    VectorXd J1(6), J2(6), J3(6), J4(6), J5(6), J6(6);


    J1 << dist[4]*(cos(th(0))*cos(th(4)) + cos(th(1)+th(2)+th(3))*sin(th(0))*sin(th(4))) + dist[2]*cos(th(0)) + dist[3]*cos(th(0)) - a[3]*cos(th(1) + th(2))*sin(th(0)) - a[2]*cos(th(1))*sin(th(0)) - dist[4]*sin(th(1)+th(2)+th(3))*sin(th(0)),
        dist[4]*(cos(th(4))*sin(th(0)) - cos(th(1)+th(2)+th(3))*cos(th(0))*sin(th(4))) + dist[2]*sin(th(0)) + dist[3]*sin(th(0)) + a[3]*cos(th(1)+th(2))*cos(th(0)) + a[2]*cos(th(0))*cos(th(1)) + dist[4]*sin(th(1)+th(2)+th(3))*cos(th(0)),
        0, 0, 0, 1;

    J2 << -cos(th(0))*(a[3]*sin(th(1)+th(2)) + a[2]*sin(th(1)) + dist[4]*(sin(th(1)+th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3)) ) - dist[4]*sin(th(4))*(cos(th(1)+th(2))*sin(th(3)) + sin(th(1)+th(2))*cos(th(3)) ) ),
        -sin(th(0))*(a[3]*sin(th(1) + th(2)) + a[2]*sin(th(1)) + dist[4]*(sin(th(1) + th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3))) - dist[4]*sin(th(4))*(cos(th(1) + th(2))*sin(th(3)) + sin(th(1) + th(2))*cos(th(3)))),
        a[3]*cos(th(1) + th(2)) - (dist[4]*sin(th(1) + th(2) + th(3) + th(4)))/2 + a[2]*cos(th(1)) + (dist[4]*sin(th(1) + th(2) + th(3) - th(4)))/2 + dist[4]*sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;

    J3 << cos(th(0))*(dist[4]*cos(th(1) + th(2) + th(3)) - a[3]*sin(th(1) + th(2)) + dist[4]*sin(th(1) + th(2) + th(3))*sin(th(4))),
        sin(th(0))*(dist[4]*cos(th(1) + th(2) + th(3)) - a[3]*sin(th(1) + th(2)) + dist[4]*sin(th(1) + th(2) + th(3))*sin(th(4))),
        a[3]*cos(th(1) + th(2)) - (dist[4]*sin(th(1) + th(2) + th(3) + th(4)))/2 + (dist[4]*sin(th(1) + th(2) + th(3) - th(4)))/2 + dist[4]*sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;

    J4 << dist[4]*cos(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4))),
        dist[4]*sin(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4))),
        dist[4]*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3)) - sin(th(1) + th(2) + th(3) + th(4))/2),
        sin(th(0)),
        -cos(th(0)),
        0;
    
    J5 << -dist[4]*sin(th(0))*sin(th(4)) - dist[4]*cos(th(1) + th(2) + th(3))*cos(th(0))*cos(th(4)),
     dist[4]*cos(th(0))*sin(th(4)) - dist[4]*cos(th(1) + th(2) + th(3))*cos(th(4))*sin(th(0)),
     -dist[4]*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3) + th(4))/2),
     sin(th(1) + th(2) + th(3))*cos(th(0)),
     sin(th(1) + th(2) + th(3))*sin(th(0)),
    -cos(th(1) + th(2) + th(3));

    J6 << 0,0,0,
        cos(th(4))*sin(th(0)) - cos(th(1) + th(2) + th(3))*cos(th(0))*sin(th(4)),
        -cos(th(0))*cos(th(4)) - cos(th(1) + th(2) + th(3))*sin(th(0))*sin(th(4)),
        -sin(th(1) + th(2) + th(3))*sin(th(4));

    J.block<6,1>(0,0) = J1; 
    J.block<6,1>(0,1) = J2; 
    J.block<6,1>(0,2) = J3; 
    J.block<6,1>(0,3) = J4; 
    J.block<6,1>(0,4) = J5; 
    J.block<6,1>(0,5) = J6; 

    return J;
}

MatrixXd InverseDiffKinematicsUr5(Vector3d ve, Vector3d omegae, RowVectorXd th, double tMin, double tMax, double DeltaT){
    vector<double> t;
    VectorXd dotqk(6);
    RowVectorXd qk(6),qk1(6);
    MatrixXd q(1,6);

    int i=0;
    while( (tMin+i*DeltaT)<=tMax ){
        t.push_back(tMin+i*DeltaT);
        i++;
    }
    
    for(double i : t){
        cout << i << " ";
    }
    cout << endl;

    qk = th;  // .transpose()
    q.block<1,6>(0,0) = qk;

    for(i=0; i<t.size() ; i++){
        MatrixXd J = ur5Jacobian(qk.transpose());
        VectorXd V(6);

        ve << -0.2*sin(t[i]), 0 ,0.2*cos(t[i]);
        omegae << 0, 0, 0;

        V.block<3,1>(0,0) = ve;  // calcolate al momento
        V.block<3,1>(3,0) = omegae;

        dotqk = J.inverse() * V;

        qk1 = qk + dotqk.transpose()*DeltaT;
        q.conservativeResize(q.rows()+1, q.cols());
        q.block<1,6>(q.rows()-1, 0) = qk1;
        qk = qk1;
    }

    return q;
}
