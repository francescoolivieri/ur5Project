#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>
#include <ur5_kinematics.h>


Vector3d worldToRobot(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, -0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;

    return (t0b*pe).block<3,1>(0,0);
}

Vector3d robotToWorld(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, 0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;

    return (t0b*pe).block<3,1>(0,0);
}



Vector3d DirectKinematicsUr5(VectorXd th){
    int i=0;

    t_60 = Matrix4d::Identity();

    t0b << 1, 0, 0, 0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;

    t_60 *= t0b; // base frame transf.

    for(i=0 ; i<6 ; i++){
        t_60 *= getT_i(i, th[i]);
    }
    
    return t_60.block<3,1>(0,3);
}


MatrixXd InverseKinematicsUr5(Vector3d pe){
    
    // transf. of pe with respect of robot base frame
    Vector4d pe_t;  // temp of pe in 4d 
    pe_t << pe(0), pe(1), pe(2), 1;


    t0b << 1, 0, 0, -0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;


    pe_t = t0b * pe_t;  

    pe = pe_t.block<3,1>(0,0);

    t_60 = Matrix4d::Identity();
    t_60.block<3,1>(0,3) = pe; // sostituisco nuovo point in t_60
    
    
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

