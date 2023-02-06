#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>
#include "ur5_kinematics.h"




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

MatrixXd TrajectoryPosition(double number_steps, Vector3d startPos, Vector3d endPos, mode traj_type){


    MatrixXd path(1,3);
    Vector3d desiredPos;


    double number_steps1 = 60; // to be put out of formula
    double step_length = 1/number_steps;

    

    for(int i=0; i< number_steps; i++){

    
        switch (traj_type){

            case LIN:
                {
                    desiredPos = (i*step_length)*endPos + (1-(i*step_length))*startPos;

                    
                    break;
                } 

            case CIRC:
                {   

                    double a1;
                    double a2;
                    a1 = atan2(startPos(1), startPos(0));
                    a2 = atan2(endPos(1), endPos(0));
                    if(startPos(0)<0){
                    a1 = a1;
                    }

                    if(endPos(0)<0){
                        a2 = a2 ;
                    }

                    double a21 = a2-a1;
                    step_length = a21/number_steps;

                    desiredPos(0) = RADIUS*cos(a21*(i*step_length)+a1);
                    desiredPos(1) = RADIUS*sin(a21*(i*step_length)+a1);
                    desiredPos(2) = startPos(2);

                    break;
                }
        }

        path.conservativeResize(path.rows()+1, path.cols());
        path.block<1,3>(path.rows()-1, 0) = desiredPos;
    }

    return path;
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

    //cout << errorOrientation << endl;
    //cout << endl;


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
MatrixXd InverseDiffKinematicsUr5(VectorXd th, Vector3d endPos, Vector3d endOrientation,  double tMin, double tMax, double DeltaT, mode traj_type){
    
    MatrixXd curr_fwk = DirectKinematicsUr5(th);
    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);
    Vector3d curr_rot_euler = curr_rot.eulerAngles(0,1,2);

    MatrixXd path_points = pathGeneration(th, endPos, endOrientation);

    MatrixXd q(1,6);
    Vector3d xd;
    Vector3d phid;
    Vector3d previousXd;
    Vector3d previousPhid;
    Vector3d vd;
    Vector3d phiddot;
    VectorXd qk = th;
    
    
    for(int j=1; j<path_points.rows(); j++ ){

        xd = path_points.block<1,3>(j,0);
        previousXd = path_points.block<1,3>(j-1,0);

        //valori fissi in futuro da creare traj orientamento end effector
        phid = endOrientation;
        previousPhid = curr_rot_euler; 

        vd = (xd-previousXd)/DeltaT;
        phiddot = (phid-previousPhid)/DeltaT;

        MatrixXd partial_trajectory = InverseDiffKinematicsUr5Quaternions(xd, endOrientation, qk, vd, phiddot);

        qk = partial_trajectory.block<1,6>(partial_trajectory.rows()-1, partial_trajectory.cols());
        q.conservativeResize(q.rows()+partial_trajectory.rows(), q.cols());
        q << q, partial_trajectory;
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
MatrixXd InverseDiffKinematicsUr5Quaternions(Vector3d pos_des, Vector3d orient_des, VectorXd q_k, Vector3d v_des, Vector3d w_des){
    int iter_max = 1000;
   
    MatrixXd joints_config(1,6);
    joints_config = q_k.transpose();
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

        joints_config.conservativeResize(joints_config.rows()+1, joints_config.cols());
        joints_config.block<1,6>(joints_config.rows()-1, 0) = q;

        if(vectMagnitude(err_vect.block<3,1>(0,0))<1e-4 && vectMagnitude(err_vect.block<3,1>(3,0))<1e-4){
            
            cout << "Finded in " << i << " iterations" << endl;

            break;

        }
    }    

    return joints_config;
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

    cout << err_orient << endl;

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

    Matrix3d K_p = Matrix3d::Identity()*2;
    Matrix3d K_o = Matrix3d::Identity()*16;
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

int touchCenterCircle(Vector3d start_pos, Vector3d end_pos){
    const Vector2d center_circle = {0, 0};
    const double radius = 0.17;


    Vector3d diff = end_pos-start_pos;

    double delta;

    delta = 4*((diff(0)*start_pos(0)+diff(1)*start_pos(1))*(diff(0)*start_pos(0)+diff(1)*start_pos(1)))-4*(diff(0)*diff(0)+diff(1)*diff(1))*(start_pos(0)*start_pos(0)+start_pos(1)*start_pos(1)-radius*radius);

    // if the line connecting start-end goes into the circle
    if(delta > 0 ){

        for(double t=0; t<=1 ; t=t+0.001){
            Vector3d p = start_pos + (end_pos - start_pos) * t;

            // check if p is inside the circle
            if( pow(p(0), 2) + pow(p(1), 2) - pow(radius, 2) < 0 ){
                return true;
            }
        }
    }

    return false;

}



int nearestViaPoint(Vector3d end_pos, Vector3d pos1, Vector3d pos2){
    double dist1 = (end_pos - pos1).norm();
    double dist2 = (end_pos - pos2).norm();

    if(dist1 < dist2) return 0;
    else return 1;
}

Vector3d tangentialPoint(Vector3d start_pos){

    double m;
    Vector3d tan_pos;

    if(start_pos(0)<0){

        m = ((2*start_pos(0)*start_pos(1)) - sqrt(4*start_pos(0)*start_pos(0)*start_pos(1)*start_pos(1)-4*(start_pos(0)*start_pos(0)-RADIUS*RADIUS)*(-RADIUS*RADIUS+start_pos(1)*start_pos(1))))/(2*(start_pos(0)*start_pos(0)-RADIUS*RADIUS));

    }else{

        m = ((2*start_pos(0)*start_pos(1)) + sqrt(4*start_pos(0)*start_pos(0)*start_pos(1)*start_pos(1)-4*(start_pos(0)*start_pos(0)-RADIUS*RADIUS)*(-RADIUS*RADIUS+start_pos(1)*start_pos(1))))/(2*(start_pos(0)*start_pos(0)-RADIUS*RADIUS));

    }
    cout << "m value :" << endl;
    cout << m << endl;
    tan_pos(0) = (m*start_pos(0)- start_pos(1))/(m+ (1/(m+0.001)));
    tan_pos(1) = (-1/(m+0.001))*tan_pos(0);
    tan_pos(2) = start_pos(2);

    return tan_pos;
}

MatrixXd pathGeneration(VectorXd q_current, Vector3d end_pos, Vector3d end_orient){

    //endOirent da togliere non viene usata
    MatrixXd curr_fwk = DirectKinematicsUr5(q_current);
    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);

    MatrixXd result(1,3);

    if(false){

        MatrixXd result = TrajectoryPosition(60, curr_pos, end_pos, LIN);

        return result;

    }else{

        MatrixXd path1;
        MatrixXd path2;
        MatrixXd path3;

        Vector3d first_pos;
        Vector3d second_pos;

        first_pos = tangentialPoint(curr_pos);
        second_pos = tangentialPoint(end_pos);

        cout << "seond pos" << endl;
        cout << first_pos << endl;

        
        path1 = TrajectoryPosition(60, curr_pos, end_pos, LIN);

        path2 = TrajectoryPosition(60, curr_pos, end_pos, CIRC);

        path3 = TrajectoryPosition(60, curr_pos, end_pos,  LIN);

        MatrixXd result(path1.rows()*3, path1.cols());
        result << path1, path2, path3;

        return result;
    }

}

MatrixXd completeTrajectory(VectorXd q_current, Vector3d end_pos, Vector3d end_orient, double delta){

    MatrixXd curr_fwk = DirectKinematicsUr5(q_current);
    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);

    //cout << "curr pos" << endl;
    //cout << curr_pos << endl;

    //const Vector3d via_point1 = {0.2, -0.3, 0.6};
    //const Vector3d via_point2 = {-0.2, -0.3, 0.6};

    Vector3d first_pos;
    Vector3d second_pos;

    if(false){

        MatrixXd result;
        result = InverseDiffKinematicsUr5(q_current, end_pos, end_orient, 0, 1, delta, LIN);

        return result;

    }else{

        MatrixXd trajectory1;
        MatrixXd trajectory2;
        MatrixXd trajectory3;

        first_pos = tangentialPoint(curr_pos);
        //second_pos = tangentialPoint(end_pos);

        cout << "seond pos" << endl;
        cout << first_pos << endl;

        //cout << sqrt(first_pos(0)*first_pos(0) + first_pos(1)*first_pos(1));

        
        trajectory1 = InverseDiffKinematicsUr5(q_current, first_pos, end_orient, 0, 1, delta, LIN);
        RowVectorXd q_des1 = trajectory1.block<1,6>(trajectory1.rows()-1, 0);

        trajectory2 = InverseDiffKinematicsUr5(q_des1.transpose(), second_pos , end_orient, 0, 1, delta, CIRC);
        RowVectorXd q_des2 = trajectory2.block<1,6>(trajectory2.rows()-1, 0);

        trajectory3 = InverseDiffKinematicsUr5(q_des2.transpose(), end_pos , end_orient, 0, 1, delta, LIN);

        MatrixXd result(trajectory1.rows()*2, trajectory1.cols());
        result << trajectory1, trajectory2;

        return result;

        //return MatrixXd::Identity(3,3);

    }
}
