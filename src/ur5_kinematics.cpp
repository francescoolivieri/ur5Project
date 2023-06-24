
#include "ur5_kinematics.h"

using namespace Mathutils;
using namespace Kinematics;

double deltaT = 0.001;

Vector3d Mathutils::worldToRobot(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, -0.5,
        0, -1, 0, 0.35,
        0, 0, -1, 1.75,
        0, 0,  0,  1;

    return (t0b*pe).block<3,1>(0,0);
}

Vector3d Mathutils::robotToWorld(Vector3d p){
    Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    t0b << 1, 0, 0, 0.5,
        0, -1, 0, 0.35,
        0, 0, -1, 1.75,
        0, 0, 0, 1;

    return (t0b*pe).block<3,1>(0,0);
}


Matrix4d Kinematics::directKinematicsUr5(VectorXd th){
    int i=0;

    t_60 = Matrix4d::Identity();

    
    /*
    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0, //parte da eliminare
        0, 0, 0, 1;
    

    t_60 *= t0b; // base frame transf.*/

    for(i=0 ; i<6 ; i++){
        t_60 *= getT_i(i, th[i]);
    }

    return t_60;
}


MatrixXd Kinematics::inverseKinematicsUr5(Vector3d pe, Matrix3d Re){
    
    // transf. of pe with respect of robot base frame
    Vector4d pe_t;  // temp of pe in 4d 
    pe_t << pe(0), pe(1), pe(2), 1;


    /*
    t0b << 1, 0, 0, 0.499992,
        0, -1, 0, 0.349988,
        0, 0, -1, 1.749994,
        0, 0, 0, 1;

    t0b << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;


    pe_t = t0b * pe_t;  */

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
Matrix4d Mathutils::getT_i(int i, double th){
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

Vector3d Kinematics::attrForce_pos(Vector3d error){
    return -0.001*(error)/error.norm();
}


Vector3d Kinematics::attrForce_orient(Vector3d error){
    return -(error)/error.norm();
}

Vector3d Kinematics::repulForce(Vector3d xe){

    double distance = sqrt(xe(0)*xe(0) + xe(1)*xe(1));
    if(distance < RADIUS){
        Vector3d pot = 0.001*(xe/distance);
        pot(2) = 0;
        return pot;

    }else{
        
        return MatrixXd::Zero(3,1);
    }
}

Vector3d Kinematics::desPos(Vector3d xe, Vector3d xf){

    Vector3d errPos = xe - xf;

    if(errPos.norm() > 0.0001){
        if(errPos.norm() > 0.1){
            return xe + (attrForce_pos(errPos) + repulForce(xe));
        }else{
            return xe - 0.03*errPos;
        }
    }

    return xe;
}


Vector3d Kinematics::desOrient(Vector3d phie, Vector3d phif){
    Vector3d errOrient = phie - phif;
    Vector3d phid;
    
    if(errOrient.norm() > 0.0001){
        if(errOrient.norm() > 0.01){

            phid = phie + 0.9*attrForce_orient(errOrient);
        }else{
            phid = phie - 0.3*errOrient;
        }
    }else{
        
        phid = phie;

    }

    return phid;
}

VectorXd Kinematics::nearest_config(VectorXd qk, MatrixXd val){
    VectorXd min_config = val.block<1,6>(0,0);
    VectorXd diff_min = qk - min_config;
    VectorXd diff;
    VectorXd candidate;

    for(int i=1; i < val.rows(); i++){
        candidate = val.block<1,6>(i,0);
        diff = qk -candidate;
        diff << diff(0)*2, diff(1)*2, diff(2)*2, diff(3), diff(4), diff(5);
        if(diff.norm() < diff_min.norm()){    // looking for the nearest configuration
            min_config = candidate;
            diff_min = diff;
        }
    }

    return min_config;
}

MatrixXd Kinematics::jointSpace_kinematics(VectorXd qk, Vector3d endPos, Vector3d endOrient ){

    MatrixXd val = inverseKinematicsUr5(endPos, eulerToRotationMatrix(endOrient));
    VectorXd endConfig = nearest_config(qk, val);
    /*to be added to gripper getter*/
    for (int j = 0; j < qk.size(); j++) {
            qk(j) = atan2(imag(exp(1i * qk(j))), real(exp(1i * qk(j))));
    }
    VectorXd qNext = qk;
    VectorXd error = endConfig - qNext;
    MatrixXd joints_config = qk.transpose();


    // check if orientations are correct
    Matrix3d rot;
    Vector3d pos;
    MatrixXd tmp = directKinematicsUr5(endConfig);
    rot = tmp.block<3,3>(0,0);
    pos = tmp.block<3,1>(0,3);

    /*final euler orientation and position*/
    /*
    cout << "final euler orientation and position" << endl;
    cout << endOrient << endl;
    cout << eulerToRotationMatrix(endOrient).eulerAngles(0,1,2) << endl << endl;
    cout << rot.eulerAngles(0,1,2) << endl << endl;
    cout << pos << endl;
    */

    /* fill the matrix with the middle configurations */
    int iter = 0;
    while (error.norm() > 0.005)
    {
        qNext = qNext + 5*deltaT* error/error.norm();
        joints_config.conservativeResize(joints_config.rows() + 1, joints_config.cols());
        joints_config.block<1,6>(joints_config.rows()-1, 0) = qNext.transpose();
        error = endConfig - qNext;
        // cout << error.norm() << endl;
        iter++;
    }
    cout << "numero di iterazioni: " << endl;
    cout << iter << endl;
    return joints_config;
}



VectorXd Kinematics::dotQ(RowVectorXd qk, Vector3d xe, Vector3d xd, Matrix3d Re, Vector3d phid){

    MatrixXd Jac;
    double Kp = 0.8;
    double Kphi = 6;
    VectorXd V(6);
    Matrix3d w_R_d;
    Vector3d errorOrientation;
    Vector3d errorPosition;

    w_R_d = eulerToRotationMatrix(phid);
    errorOrientation = orientationError(Re, w_R_d);
    errorPosition = positionError(xe, xd);
    
    Jac = ur5Jacobian(qk.transpose());

    V.block<3,1>(0,0) = Kp*errorPosition/deltaT;
    V.block<3,1>(3,0) = Kphi*errorOrientation;

    return (Jac + MatrixXd::Identity(6,6)*(0.001)).inverse()*V;

}

double Mathutils::centerDist(Vector3d p){
    return sqrt(p(0)*p(0)+p(1)+p(1));
}



//th is to substitute with startPos
MatrixXd Kinematics::inverseDiffKinematicsUr5(VectorXd th, Vector3d endPos, Vector3d endOrientation){
    
    MatrixXd curr_fwk = directKinematicsUr5(th);
    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);
    Vector3d curr_rot_euler = curr_rot.eulerAngles(0,1,2);

    MatrixXd joints_config = th.transpose();
    Vector3d xd;
    Vector3d xe;
    Matrix3d Re;
    Vector3d eule;
    VectorXd qk = th;
    VectorXd q = th;
    VectorXd dotq;
    
    int iter = 0;
    Vector3d distancePos = endPos - curr_pos;
    Vector3d distanceOrient = endOrientation - curr_rot_euler;
    
    /* Check on error from actual pos. to desired pos. and number of iterations */
    while( (distancePos.norm() > 0.001  || distanceOrient.norm() > 0.01 ) && iter < 1500){ 

        /* Calc. direct kin. to know my actual position and orientation in space */
        curr_fwk = directKinematicsUr5(qk);
        Re = curr_fwk.block<3,3>(0,0);
        xe = curr_fwk.block<3,1>(0,3);
        eule = Re.eulerAngles(0,1,2); 

        xd = desPos(xe, endPos);
        
        dotq = dotQ(qk, xe, xd, Re, endOrientation);
        
        qk = q + dotq * deltaT;
        q = qk;
        
        joints_config.conservativeResize(joints_config.rows() + 1, joints_config.cols());
        joints_config.block<1,6>(joints_config.rows()-1, 0) = qk.transpose();

        distancePos = endPos - xe;
        distanceOrient = orientationError(Re, eulerToRotationMatrix(endOrientation));

        iter++;
    }
    
    ROS_DEBUG("numero iterazioni: %d", iter );

    return joints_config;
}

MatrixXd Mathutils::ur5Jacobian(VectorXd th){
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

Matrix3d Mathutils::eulerToRotationMatrix(Vector3d euler){

    double psi = euler(0);    //yaw X psi
    double theta = euler(1);  //pitch Y theta
    double phi = euler(2);    //roll Z phi 

    Matrix3d R;

    R << cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
         sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),
         -sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi);

    return R;

}

/*
VectorXd q_dott0(VectorXd qk){
    VectorXd q_dott0(6);
    int k0 = 20;

    q_dott0 = -k0/6*(qk/(2*M_PI));

    return q_dott0;
}*/

Vector3d Kinematics::orientationError(Matrix3d w_R_e, Matrix3d w_R_d){

    Vector3d error;
    Vector3d toNormalize; 
    Matrix3d e_R_d = w_R_e.transpose()*w_R_d;

    double cos_dtheta = (e_R_d(0,0) + e_R_d(1,1) + e_R_d(2,2) - 1)/2;
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

Vector3d Kinematics::positionError(Vector3d xe, Vector3d xd){

    Vector3d error = xd - xe;
    
    if(error.norm()>0.1){
        error = 0.1*error.normalized();
    }

    return error;
}

// standard inv. diff. kin. using quaternion 
MatrixXd Kinematics::inverseDiffKinematicsUr5Quaternions(VectorXd q_k, Vector3d endPos, Vector3d endOrient){
    int iter_max = 1000;
   
    MatrixXd joints_config(1,6);
    joints_config = q_k.transpose();

    VectorXd q = q_k;

    
    MatrixXd curr_fwk = directKinematicsUr5(q);
    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);
    Vector3d curr_rot_euler = curr_rot.eulerAngles(0,1,2);
    //VectorXd q_des(6);

    Vector3d xe;
    Matrix3d Re;
    Vector3d eule;

    Vector3d xd;
    Vector3d phid;
    Vector3d vd;
    Vector3d phiddot;

    // set q as current set of joint
    

    // coeff. matrix building
    MatrixXd K = MatrixXd::Identity(6,6);
    Matrix3d K_p = Matrix3d::Identity()*0;
    Matrix3d K_o = Matrix3d::Identity()*7;
    K.block<3,3>(0,0) = K_p;
    K.block<3,3>(3,3) = K_o; 

    // speed matrix building
    VectorXd speed_control(6);
    

    Vector3d distancePos = endPos - curr_pos;
    Vector3d distanceOrient = endOrient - curr_rot_euler;
    int i;
    for(i=0 ; i<iter_max ; i++){

        
        curr_fwk = directKinematicsUr5(q);
        Re = curr_fwk.block<3,3>(0,0);
        xe = curr_fwk.block<3,1>(0,3);
        eule = Re.eulerAngles(0,1,2);

        xd = desPos(xe, endPos);
        phid = desOrient(eule, endOrient);
        
        vd = (xd-xe)/0.001;
        phiddot = (phid-eule)/0.001;
        speed_control << vd, phiddot;

        // Jacobian calc.
        MatrixXd J = ur5Jacobian(q);
        
        VectorXd err_vect = ComputeErrorQuaternion(q, endPos, endOrient);

        // calc. next set of joint base on error
        q = q + 0.001 * (J + MatrixXd::Identity(6,6) * 0.001 ).inverse() * ( speed_control + (K * err_vect) );

        
        for (int j = 0; j < q.size(); j++) {
            q(j) = atan2(imag(exp(1i * q(j))), real(exp(1i * q(j))));
        }

        joints_config.conservativeResize(joints_config.rows()+1, joints_config.cols());
        joints_config.block<1,6>(joints_config.rows()-1, 0) = q;

        if(vectMagnitude(err_vect.block<3,1>(0,0))<1e-4 && vectMagnitude(err_vect.block<3,1>(3,0))<1e-4){
            
            cout << "Finded in " << i << " iterations" << endl;
            return joints_config;
            break;

        }

        distancePos = endPos - xe;
        distanceOrient = endOrient - eule;
        /*
        cout << "-------errors-------" << endl;
        cout << distancePos.norm() << endl << endl;
        //cout << err_orient << endl << endl;
        cout << distanceOrient.norm() << endl << endl;
        cout << xe << endl << endl;
        cout << eule << endl << endl;
        cout << "--------------------" << endl;*/
    } 
    
          

    //return joints_config;
    return joints_config;
}

// position and orientation error using quaternion
VectorXd Kinematics::ComputeErrorQuaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des){
    // express orientation in quaternion
    Quaterniond quat_orient_des = EulerToQuaternion(orient_des);

    // calc. forward kin to get position and orientation
    Matrix4d curr_fwk = directKinematicsUr5(q);

    Vector3d curr_pos = curr_fwk.block<3,1>(0,3);

    Matrix3d curr_rot = curr_fwk.block<3,3>(0,0);
    Quaterniond quat_orient_curr(curr_rot);   // calc. quaternion

    // errors calculation
    Vector3d err_pos = pos_des - curr_pos;
    Vector3d err_orient = quat_orient_curr.w() * quat_orient_des.vec() - quat_orient_des.w() * quat_orient_des.vec() - quat_orient_des.vec().cross(quat_orient_curr.vec()) ;

    // build 6x1 vector
    Vector3d err_vect;
    err_vect << err_orient;

    /*s
    cout << "-------errors-------" << endl;
    cout << err_pos.norm() << endl << endl;
    //cout << err_orient << endl << endl;
    cout << err_orient.norm() << endl << endl;
    cout << "--------------------" << endl;*/

    return err_vect;
}

// returns the J*(...)
VectorXd Kinematics::dotQquaternion(VectorXd q, Vector3d pos_des, Vector3d orient_des, Vector3d v_des, Vector3d w_des){
    
    // Jacobian calc.
    MatrixXd J = ur5Jacobian(q);

    // express orientation in quaternion
    Quaterniond quat_orient_des = EulerToQuaternion(orient_des);

    // coeff. matrix building
    MatrixXd K = MatrixXd::Identity(6,6);

    Matrix3d K_p = Matrix3d::Identity()*0;
    Matrix3d K_o = Matrix3d::Identity()*0;
    K.block<3,3>(0,0) = K_p;
    K.block<3,3>(3,3) = K_o; 

    //w_des = {0, 0 , 0};
    // speed vector building
    VectorXd speed_control(6);
    speed_control << 7*v_des,17*w_des;
    //speed_control << w_des, w_des;

    VectorXd err_vect = ComputeErrorQuaternion(q, pos_des, orient_des);
        
    // calc. next set of joint base on error
    return J.inverse() * ( speed_control + (K * err_vect) );
}

Quaterniond Mathutils::EulerToQuaternion(Vector3d euler){
    Matrix3d R = Matrix3d::Identity();

    R = eulerToRotationMatrix({0,0,euler(2)}) * eulerToRotationMatrix({0,euler(1),0}) * eulerToRotationMatrix({euler(0),0,0});

    Quaterniond q(R);

    return q;

}

double Mathutils::quatMagnitude(const Quaterniond &q) {
    return sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
}

double Mathutils::vectMagnitude(const Vector3d &v) {
    return sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}

int Mathutils::touchCenterCircle(Vector3d start_pos, Vector3d end_pos){
    const Vector2d center_circle = {0, 0};
    const double radius = RADIUS;

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

Vector3d Mathutils::tangentialPoint(Vector3d start_pos){

    double m;
    double distance_from_center = sqrt(start_pos(0)*start_pos(0) + start_pos(1)*start_pos(1));
    Vector3d tan_pos;

    if(distance_from_center > RADIUS){

        if(start_pos(0)<0){

            m = ((2*start_pos(0)*start_pos(1)) - sqrt(4*start_pos(0)*start_pos(0)*start_pos(1)*start_pos(1)-4*(start_pos(0)*start_pos(0)-RADIUS*RADIUS)*(-RADIUS*RADIUS+start_pos(1)*start_pos(1))))/(2*(start_pos(0)*start_pos(0)-RADIUS*RADIUS));

        }else{
            
            
            m = ((2*start_pos(0)*start_pos(1)) + sqrt(4*start_pos(0)*start_pos(0)*start_pos(1)*start_pos(1)-4*(start_pos(0)*start_pos(0)-RADIUS*RADIUS)*(-RADIUS*RADIUS+start_pos(1)*start_pos(1))))/(2*(start_pos(0)*start_pos(0)-RADIUS*RADIUS));
        }

        cout << "m value :" << endl;
        cout << m << endl << endl;
        tan_pos(0) = (m*start_pos(0)- start_pos(1))/(m+ (1/(m+0.001)));
        tan_pos(1) = (-1/(m+0.001))*tan_pos(0);
        tan_pos(2) = start_pos(2);

    }else{

        m = start_pos(1)/start_pos(0);

        tan_pos(0) = (m*start_pos(0)- start_pos(1))/(m+ (1/(m+0.001)));
        tan_pos(1) = (-1/(m+0.001))*tan_pos(0);
        tan_pos(2) = start_pos(2);

    }
    
    return tan_pos;
}

