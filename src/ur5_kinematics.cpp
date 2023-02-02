#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>
#include "ur5_kinematics.h"

//using namespace Eigen;

Vector3d worldToRobot(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return (t0b*pe).block<3,1>(0,0);
}

Vector3d robotToWorld(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return (t0b*pe).block<3,1>(0,0);
}


Matrix4d DirectKinematicsUr5(VectorXd th){
    int i=0;

    t_60 = Matrix4d::Identity();

    /*
    t0b << 1, 0, 0, 0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;*/

    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    

    t_60 *= t0b; // base frame transf.

    for(i=0 ; i<6 ; i++){
        t_60 *= getT_i(i, th[i]);
    }

    return t_60;
}


MatrixXd InverseKinematicsUr5(Vector3d pe, Matrix3d Re){
    
    // transf. of pe with respect of robot base frame
    Vector4d pe_t;  // temp of pe in 4d 
    pe_t << pe(0), pe(1), pe(2), 1;


    /*
    t0b << 1, 0, 0, 0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;*/

    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;


    pe_t = t0b * pe_t;  

    pe = pe_t.block<3,1>(0,0);

    t_60 = Matrix4d::Identity();
    t_60.block<3,1>(0,3) = pe; // sostituisco nuovo point in t_60
    t_60.block<3,3>(0,0) = Re;
    
    
    // th1
    Vector4d p50 = t_60 * Vector4d(0.0,0.0,-dist[5],1.0);
    double th1_1 = real( atan2(p50(1),p50(0)) + acos(std::complex<double>(dist[3]/hypot(p50(1),p50(0)), 0.0)).real() ) + M_PI/2;
    double th1_2 = real( atan2(p50(1),p50(0)) - acos(std::complex<double>(dist[3]/hypot(p50(1),p50(0)), 0.0)).real() ) + M_PI/2;

    // th5
    double th5_1 = real( acos(complex<double>((pe(0)*sin(th1_1) - pe(1)*cos(th1_1)-dist[3]), 0.0) / dist[5]) );
    double th5_2 = -real( acos(complex<double>((pe(0)*sin(th1_1) - pe(1)*cos(th1_1)-dist[3]), 0.0) / dist[5]) );
    double th5_3 = real( acos(complex<double>((pe(0)*sin(th1_2) - pe(1)*cos(th1_2)-dist[3]), 0.0) / dist[5]) );
    double th5_4 = -real( acos(complex<double>((pe(0)*sin(th1_2) - pe(1)*cos(th1_2)-dist[3]), 0.0) / dist[5]) );

    Matrix4d t_06 = t_60.inverse(); // ?
    Vector3d Xhat = t_06.block<3,1>(0,0);
    Vector3d Yhat = t_06.block<3,1>(0,1);

    // th6
    double th6_1 = real( atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))) / sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1) ) );
    double th6_2 = real( atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))) / sin(th5_2), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_2) ) );
    double th6_3 = real( atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))) / sin(th5_3), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_3) ) );
    double th6_4 = real( atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))) / sin(th5_4), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_4) ) );

    Matrix4d T41m = getT_i(0,th1_1).inverse() * t_60 * (getT_i(5,th6_1)).inverse() * getT_i(4,th5_1).inverse();
    Vector3d p41_1 = T41m.block<3,1>(0,3);
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41m = getT_i(0,th1_1).inverse() * t_60 * (getT_i(5,th6_2)).inverse() * getT_i(4,th5_2).inverse() ;
    Vector3d p41_2 = T41m.block<3,1>(0,3);
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    T41m = getT_i(0,th1_2).inverse() * t_60 * (getT_i(5,th6_3)).inverse() * getT_i(4,th5_3).inverse() ;
    Vector3d p41_3 = T41m.block<3,1>(0,3);
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    T41m =  getT_i(0,th1_2).inverse() * t_60 * (getT_i(5,th6_4)).inverse() * getT_i(4,th5_4).inverse() ;
    Vector3d p41_4 = T41m.block<3,1>(0,3);
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    // th3   
    double th3_1 = acos( complex<double>((pow(p41xz_1,2)-pow(a[2],2)-pow(a[3],2)) / (2*a[2]*a[3]), 0.0)).real();
    double th3_2 = acos( complex<double>((pow(p41xz_2,2)-pow(a[2],2)-pow(a[3],2)) / (2*a[2]*a[3]), 0.0)).real() ;
    double th3_3 = acos( complex<double>((pow(p41xz_3,2)-pow(a[2],2)-pow(a[3],2)) / (2*a[2]*a[3]), 0.0)).real() ;
    double th3_4 = acos( complex<double>((pow(p41xz_4,2)-pow(a[2],2)-pow(a[3],2)) / (2*a[2]*a[3]), 0.0)).real() ;

    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    // th2
    double th2_1 = real( atan2(-p41_1(2), -p41_1(0))-asin((-a[3]*sin(th3_1))/p41xz_1) );
    double th2_2 = real( atan2(-p41_2(2), -p41_2(0))-asin((-a[3]*sin(th3_2))/p41xz_2) );
    double th2_3 = real( atan2(-p41_3(2), -p41_3(0))-asin((-a[3]*sin(th3_3))/p41xz_3) );
    double th2_4 = real( atan2(-p41_4(2), -p41_4(0))-asin((-a[3]*sin(th3_4))/p41xz_4) );

    double th2_5 = real( atan2(-p41_1(2), -p41_1(0))-asin((a[3]*sin(th3_1))/p41xz_1) );
    double th2_6 = real( atan2(-p41_2(2), -p41_2(0))-asin((a[3]*sin(th3_2))/p41xz_2) );
    double th2_7 = real( atan2(-p41_3(2), -p41_3(0))-asin((a[3]*sin(th3_3))/p41xz_3) );
    double th2_8 = real( atan2(-p41_4(2), -p41_4(0))-asin((a[3]*sin(th3_4))/p41xz_4) );

    Matrix4d T43m =  getT_i(2, th3_1).inverse() * getT_i(1,th2_1).inverse() * getT_i(0,th1_1).inverse() * t_60 * getT_i(5,th6_1).inverse() * getT_i(4,th5_1).inverse() ;
    Vector3d Xhat43 = T43m.block<3,1>(0,0);
    double th4_1 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_2).inverse() * getT_i(1,th2_2).inverse() * getT_i(0,th1_1).inverse() * t_60 * getT_i(5,th6_2).inverse() * getT_i(4,th5_2).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_2 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_3).inverse() * getT_i(1,th2_3).inverse() * getT_i(0,th1_2).inverse() * t_60 * getT_i(5,th6_3).inverse() * getT_i(4,th5_3).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_3 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_4).inverse() * getT_i(1,th2_4).inverse() * getT_i(0,th1_2).inverse() * t_60 * getT_i(5,th6_4).inverse() * getT_i(4,th5_4).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_4 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m = getT_i(2, th3_5).inverse() * getT_i(1,th2_5).inverse() * getT_i(0,th1_1).inverse() * t_60 * getT_i(5,th6_1).inverse() * getT_i(4,th5_1).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_5 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_6).inverse() * getT_i(1,th2_6).inverse() * getT_i(0,th1_1).inverse() * t_60 * getT_i(5,th6_2).inverse() * getT_i(4,th5_2).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_6 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_7).inverse() * getT_i(1,th2_7).inverse() * getT_i(0,th1_2).inverse() * t_60 * getT_i(5,th6_3).inverse() * getT_i(4,th5_3).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_7 = real( atan2(Xhat43(1), Xhat43(0)) );

    T43m =  getT_i(2, th3_8).inverse() * getT_i(1,th2_8).inverse() * getT_i(0,th1_2).inverse() * t_60 * getT_i(5,th6_4).inverse() * getT_i(4,th5_4).inverse() ;
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_8 = real( atan2(Xhat43(1), Xhat43(0)) );

    MatrixXd M_th(8,6);

    M_th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;


    return M_th;
}

// NB: IF i == 0 -> t_i == t_10 or i == 1 -> t_i == t_21
Matrix4d getT_i(int i, double th){
    // better if store matrices

    Matrix4d t_i = Matrix4d::Identity();

    Matrix3d R;
    Vector3d T;

    // rotation matrix
    R << cos(th), -sin(th), 0,
        sin(th)*cos(alp[i]), cos(th)*cos(alp[i]), (-sin(alp[i])), 
        sin(th)*sin(alp[i]), cos(th)*sin(alp[i]), cos(alp[i]);

    // translation 
    T << a[i], (-dist[i] * sin(alp[i])), dist[i] * cos(alp[i]);
        
    // composing affine transformation
    t_i.block<3,3>(0,0) = R;
    t_i.block<3,1>(0,3) = T;

    return t_i;
}

Vector3d TrajectoryPosition(double currentTime, double totalDuration, Vector3d startPos, Vector3d endPos){

    Vector3d desiredPos;
    desiredPos = (currentTime/totalDuration)*endPos + (1-(currentTime/totalDuration))*startPos;

    return desiredPos;
}

Vector3d TrajectoryPositionSinusoidal(double currentTime, double totalDuration, Vector3d startPos, Vector3d endPos){

    Vector3d desiredPos;

    //desiredPos = (currentTime/totalDuration)*endPos + (1-(currentTime/totalDuration))*startPos;

    double sinArgument = (currentTime/totalDuration)*M_PI - M_PI/2;
    desiredPos = ((sin(sinArgument)+1)/2)*(endPos-startPos) + startPos;

    return desiredPos;
}

Vector3d TrajectoryOrientation(double currentTime, double totalDuration, Vector3d startOrient, Vector3d endOrient){

    Vector3d desiredOrient;
    desiredOrient = (currentTime/totalDuration)*endOrient+(1-(currentTime/totalDuration))*startOrient;

    //cout << desiredOrient << endl;
    //cout << endl;

    return desiredOrient;
}

VectorXd JointAngularVelocity(RowVectorXd qk, Vector3d xe, Vector3d xd, Vector3d vd, Matrix3d Re, Vector3d phie, Vector3d phid, Vector3d phiddot ){
    
    VectorXd dotQ;
    MatrixXd Jac;
    MatrixXd Ja;   
    Matrix3d T;
    MatrixXd Ta(6,6);
    Matrix3d Kp;
    Matrix3d Kphi;
    VectorXd V(6);
    Matrix3d w_R_d;
    Vector3d errorOrientation;
    Vector3d omegaDot;

    w_R_d = toRotationMatrix(phid);
    errorOrientation = computeOrientationError(Re, w_R_d);
    

    Jac = ur5Jacobian(qk.transpose());
    double psi = phid[0];
    double theta = phid[1];
    double phi = phid[2];

    T << cos(theta)*cos(phi), -sin(phi), 0,
         cos(theta)*sin(phi), cos(phi), 0,
         -sin(theta), 0, 1;

    omegaDot = T*phiddot;

    cout << errorOrientation << endl;
    cout << endl;


    /*Ta.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    Ta.block<3,3>(0,3) = MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,0) = MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,3) = T;*/



    

//do kp and kphi have arbitrary values?
    Kp = MatrixXd::Identity(3,3)*5; 
    Kphi = MatrixXd::Identity(3,3)*0.1; 

   

    V.block<3,1>(0,0) = vd + Kp*(xd-xe);
    V.block<3,1>(3,0) = omegaDot + Kphi*(errorOrientation);

    dotQ = (Jac+ MatrixXd::Identity(6,6)*(1e-06)).inverse()*V;

    //cout << dotQ << endl;

    return dotQ;
}


//th is to substitute with startPos
MatrixXd InverseDiffKinematicsUr5(RowVectorXd th,Vector3d startPos, Vector3d endPos, Vector3d startOrientation, Vector3d endOrientation,  double tMin, double tMax, double DeltaT){
    vector<double> t;
    VectorXd dotqk;
    RowVectorXd qk(6),qk1(6);
    MatrixXd q(1,6);

    //Vector6d V;
    Vector3d phie;
    Vector3d xd;
    Vector3d previousXd;
    Vector3d phid;
    Vector3d previousPhid;

    //Vector3d startOrientation;
    //Vector3d startPos;

    double totalDuration = tMax - tMin;

//roto-trasl matrix from end effector to base frame
    Matrix4d t_f0;

//parts of trasl and rotation of t_f0
    Vector3d xe;
    Matrix3d Re;

// desired end-effector tangential and angular velocities
    Vector3d vd;
    Vector3d phiddot;

    int i=0;
    while( (tMin+i*DeltaT)<=tMax ){
        t.push_back(tMin+i*DeltaT);
        i++;
    }
    
    for(double i : t){
        //cout << i << " ";
    }
    //cout << endl;


    qk = th;  // .transpose()
    q.block<1,6>(0,0) = qk;

//calculating initial position and orientation
    t_f0 = DirectKinematicsUr5(qk.transpose());
    //startPos = t_f0.block<3,1>(0,3);
    Re = t_f0.block<3,3>(0,0);
    //startOrientation = Re.eulerAngles(0,1,2);


    //xd = TrajectoryPosition(0, totalDuration, startPos, endPos);
    //previousXd = TrajectoryPosition(0-DeltaT, totalDuration, startPos, endPos);

    /*----sinusoidal profile trajectory -----*/
    xd = TrajectoryPositionSinusoidal(0, totalDuration, startPos, endPos);
    previousXd = TrajectoryPositionSinusoidal(0-DeltaT, totalDuration, startPos, endPos);

    phid = TrajectoryOrientation(0, totalDuration, startOrientation, endOrientation);
    previousPhid = TrajectoryOrientation(0-DeltaT, totalDuration, startOrientation, endOrientation);

    //cout << startOrientation << endl;
    //cout << endOrientation<< endl;

    for(i=0; i<t.size() ; i++){
        
        t_f0 = DirectKinematicsUr5(qk);
        xe = t_f0.block<3,1>(0,3);
        Re = t_f0.block<3,3>(0,0);
        //phie = Re.eulerAngles(0,1,2);//initially 0,1,2

        //if(i==2) cout << Re << endl;
        //if(i==3) cout << Re << endl;

        
        //xd = TrajectoryPosition(t[i], totalDuration, startPos, endPos);
        xd = TrajectoryPositionSinusoidal(t[i], totalDuration, startPos, endPos);
        phid = TrajectoryOrientation(t[i], totalDuration, startOrientation, endOrientation);

        previousXd = TrajectoryPositionSinusoidal(t[i-1], totalDuration, startPos, endPos);
        previousPhid = TrajectoryOrientation(t[i-1], totalDuration, startOrientation, endOrientation);

        vd = (xd-previousXd)/DeltaT;
        phiddot = (phid-previousPhid)/DeltaT;

        //cout << "desired point" << phid << endl;
        //cout << "effective point " << phie << endl;
        //cout << phie << endl;
        //cout << endl;
        //cout <<endl;

        //V.block<3,1>(0,0) = xe; 
        //V.block<3,1>(3,0) = phie;
        dotqk = JointAngularVelocity(qk, xe, xd, vd, Re, phie, phid, phiddot);
        //dotqk = J.inverse() * V;

        qk1 = qk + dotqk.transpose()*DeltaT;
        q.conservativeResize(q.rows()+1, q.cols());
        q.block<1,6>(q.rows()-1, 0) = qk1;
        qk = qk1;
    }

    return q;
}

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

Matrix3d toRotationMatrix(Vector3d euler){

    double psi = euler(0);    //yaw X
    double theta = euler(1);  //pitch Y
    double phi = euler(2);    //roll Z

    Matrix3d R;

    R << cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
         sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),
         -sin(theta), cos(theta)*sin(psi), cos(theta)*cos(phi);

    return R;

}

VectorXd q_dott0(VectorXd qk){
    VectorXd q_dott0(6);
    int k0 = 20;

    q_dott0 = -k0/6*(qk/(2*M_PI));

    return q_dott0;
}

VectorXd JointAngularVelocityRedundancy(RowVectorXd qk, Vector3d xe, Vector3d xd, Vector3d vd, Vector3d phie, Vector3d phid, Vector3d phiddot ){
    
    VectorXd dotQ;
    MatrixXd Jac;
    MatrixXd Ja;   
    Matrix3d T;
    MatrixXd Ta(6,6);
    Matrix3d Kp;
    Matrix3d Kphi;
    VectorXd V(6);

    Jac = ur5Jacobian(qk.transpose());
    double psi = phie[0];
    double theta = phie[1];
    double phi = phie[2];

    T << cos(theta)*cos(phi), -sin(phi), 0,
         cos(theta)*sin(phi), cos(phi), 0,
         -sin(theta), 0, 1;

    Ta.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    Ta.block<3,3>(0,3) = MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,0) = MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,3) = T;

    Ja = Ta.inverse()*Jac;

    

//do kp and kphi have arbitrary values?
    Kp = MatrixXd::Identity(3,3)*5; 
    Kphi = MatrixXd::Identity(3,3)*0.1;

   

    V.block<3,1>(0,0) = vd + Kp*(xd-xe);
    V.block<3,1>(3,0) = phiddot + Kphi*(phid-phie);

    
    

    dotQ = (Ja+ MatrixXd::Identity(6,6)*(1e-06)).completeOrthogonalDecomposition().pseudoInverse()*V + (MatrixXd::Identity(6,6)-Ja.completeOrthogonalDecomposition().pseudoInverse()*Ja)*q_dott0(qk.transpose());

    return dotQ;
}

Vector3d computeOrientationError(Matrix3d w_R_e, Matrix3d w_R_d){

    Vector3d error;
    Matrix3d e_R_d = w_R_e.transpose()*w_R_d;

    
    double cos_dtheta = (e_R_d(0,0) + e_R_d(1,1) + e_R_d(2,2) - 1)/2;

    Vector3d toNormalize; 

    toNormalize << e_R_d(2,1) - e_R_d(1,2), e_R_d(0,2) - e_R_d(2,0), e_R_d(1,0) - e_R_d(0,1); 

    
    double sin_dtheta = toNormalize.norm()*0.5;

    double dtheta = atan2(sin_dtheta, cos_dtheta);

    
    if(dtheta == 0){

        error = {0, 0, 0};

    }else{


        Vector3d axis = 1/(2*sin_dtheta)*toNormalize;
        error = w_R_e * axis * dtheta;
    }

    

    return error;

}

// standard inv. diff. kin. using quaternion 
VectorXd InverseDiffKinematicsUr5Quaternions(Vector3d pos_des, Vector3d orient_des, VectorXd q_k, Vector3d v_des, Vector3d w_des){
    int iter_max = 1000;
   
    VectorXd q_des(6);

    // set q as current set of joint
    VectorXd q = q_k;

    // coeff. matrix building
    MatrixXd K = MatrixXd::Identity(6,6);
    Matrix3d K_p = Matrix3d::Identity()*5;
    Matrix3d K_o = Matrix3d::Identity()*5;
    K.block<3,3>(0,0) = K_p;
    K.block<3,3>(3,3) = K_o; 

    // speed matrix building
    VectorXd speed_control(6);
    speed_control << v_des, w_des;
    
    for(int i=0 ; i<iter_max ; i++){
        // Jacobian calc.
        MatrixXd J = ur5Jacobian(q);

        VectorXd err_vect = ComputeErrorQuaternion(q, pos_des, orient_des);

        // calc. next set of joint base on error
        q = q + 0.05 * (J + MatrixXd::Identity(6,6) * 0.001 ).inverse() * ( speed_control + (K * err_vect) );

        for (int j = 0; j < q.size(); j++) {
            q(j) = atan2(imag(exp(1i * q(j))), real(exp(1i * q(j))));
        }

        if(vectMagnitude(err_vect.block<3,1>(0,0))<1e-4 && vectMagnitude(err_vect.block<3,1>(3,0))<1e-4){
            
            cout << "Finded in " << i << " iterations" << endl;

            q_des = q;
            return q_des;
        }
    }    

    return q;
}

// position and orientation error using quaternion
VectorXd ComputeErrorQuaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des){
    // express orientation in quaternion
    Quaterniond quat_orient_des = EulerToQuaternion(orient_des);

    // calc. forward kin to get position and orientation
    Matrix4d curr_fwk = DirectKinematicsUr5(q);

    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);

    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Quaterniond quat_orient_curr(curr_rot);   // calc. quaternion

    // errors calculation
    Vector3d err_pos = pos_des - curr_pos;
    Vector3d err_orient = quat_orient_curr.w() * quat_orient_des.vec() - quat_orient_des.w() * quat_orient_des.vec() - quat_orient_des.vec().cross(quat_orient_curr.vec()) ;

    // build 6x1 vector
    VectorXd err_vect(6);
    err_vect << err_pos, err_orient;

    cout << err_pos << endl << err_orient << endl;

    return err_vect;
}

// returns the J*(...)
VectorXd JointAngularVelocityQuaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des, Vector3d v_des, Vector3d w_des){
    
    // Jacobian calc.
    MatrixXd J = ur5Jacobian(q);

    // express orientation in quaternion
    Quaterniond quat_orient_des = EulerToQuaternion(orient_des);

    // coeff. matrix building
    MatrixXd K = MatrixXd::Identity(6,6);

    Matrix3d K_p = Matrix3d::Identity()*5;
    Matrix3d K_o = Matrix3d::Identity()*5;
    K.block<3,3>(0,0) = K_p;
    K.block<3,3>(3,3) = K_o; 

    // speed vector building
    VectorXd speed_control(6);
    speed_control << v_des, w_des;

    VectorXd err_vect = ComputeErrorQuaternion(q, pos_des, orient_des);
        
    // calc. next set of joint base on error
    return J.inverse() * ( speed_control + (K * err_vect) );
}

Quaterniond EulerToQuaternion(Vector3d euler){
    Matrix3d R = Matrix3d::Identity();

    R = toRotationMatrix({0,0,euler(2)}) * toRotationMatrix({0,euler(1),0}) * toRotationMatrix({euler(0),0,0});

    Quaterniond q(R);

    return q;

}

double quatMagnitude(const Quaterniond &q) {
    return sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
}

double vectMagnitude(const Vector3d &v) {
    return sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}