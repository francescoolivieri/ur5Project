#include "robot.hpp"

vector<string> models_list;

bool real_robot;
bool soft_gripper;
bool gripper_sim;

Joints::Joints(){
    #if UPDATE_FROM_ROBOT
        this->set_joints_from_robot();
    #else
        Vector6d startArmPos;
        startArmPos << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;

        Vector3d startGripPos;
        if(soft_gripper){
            startGripPos << 0, 0, 0;
        }else{
            startGripPos << 1.8, 1.8, 1.8;
        }

        this->set_new( startArmPos, startGripPos );
    #endif
}

Joints::Joints(Vector6d q_arm){
    this->set_new(q_arm);
}

Joints::Joints(Vector6d q_arm, Eigen::Vector3d q_gripper){
    this->set_new(q_arm, q_gripper);
}

void Joints::set_new(Vector6d q_arm, Eigen::Vector3d q_gripper){
    this->shoulder_pan = q_arm(0);
    this->shoulder_lift = q_arm(1);
    this->elbow = q_arm(2);
    this->wrist_1 = q_arm(3);
    this->wrist_2 = q_arm(4);
    this->wrist_3 = q_arm(5);
    this->gripper_1 = q_gripper(0);
    this->gripper_2 = q_gripper(1);
    this->gripper_3 = q_gripper(2);
}

void Joints::set_new(Vector6d q_arm){
    this->shoulder_pan = q_arm(0);
    this->shoulder_lift = q_arm(1);
    this->elbow = q_arm(2);
    this->wrist_1 = q_arm(3);
    this->wrist_2 = q_arm(4);
    this->wrist_3 = q_arm(5);
}

void Joints::set_new(Eigen::Vector3d q_gripper){
    this->gripper_1 = q_gripper(0);
    this->gripper_2 = q_gripper(1);
    this->gripper_3 = q_gripper(2);
}

Vector6d Joints::get_arm(){
    Vector6d v;
    v << this->shoulder_pan, this->shoulder_lift, this->elbow, this->wrist_1, this->wrist_2, this->wrist_3;
    return v;
}

Vector3d Joints::get_gripper(){
    Eigen::Vector3d v;
    v << this->gripper_1, this->gripper_2, this->gripper_3;
    return v;
}

void Joints::set_joints_from_robot(){
    Vector9d v = receive_jstate();

    Vector6d v_arm = v.block<6,1>(0,0); 
    Vector3d v_gripper = v.block<3,1>(6,0);
    
    this->set_new(v_arm);
    this->set_new(v_gripper);
}

Robot::Robot(){
    this->joints = Joints();
}

Robot::Robot(Joints joints){
    this->joints = joints;
}
        
Robot::Robot(Vector6d q_arm, Vector3d q_gripper){
    this->joints = Joints(q_arm, q_gripper);
}

Robot::Robot(Vector6d q_arm){
    this->joints = Joints(q_arm);
}

Vector3d Robot::get_wrist(){
    Vector3d wrist;
    wrist << this->joints.wrist_1, this->joints.wrist_2, this->joints.wrist_3;

    return wrist;
}

Vector3d Robot::get_shoulder(){
    Vector3d shoulder;
    shoulder << this->joints.shoulder_pan, this->joints.shoulder_lift, this->joints.elbow;

    return shoulder;
}

double Robot::mapToGripperJoints(double diameter){
    if(soft_gripper){
        int D0 = 40;
        int L = 60;
        double delta = 0.5*(diameter - D0);

        return atan2(delta, L);
    }else{
        return (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    }
}

void Robot::set_new_gripper_position(double diameter){

    double q = mapToGripperJoints(diameter);

    Vector3d v;
    v << q, q, q;

    this->joints.set_new(v);
    this->gripper_diameter = diameter;
}

void Robot::move_gripper(double diameter){

    if(real_robot){
        robot_move_gripper(diameter);
    }else{
        /* CAN BE DONE GRADUALLY */
        this->set_new_gripper_position(diameter);

        send_des_jstate(this->joints.get_arm(), this->joints.get_gripper());
    }
}

void Robot::move(Vector3d finalPos, Vector3d finalOrient){
    ros::Rate loop_rate(loop_frequency);

    /* Calc. configurations to arrive at the final pos. & orient. */
    MatrixXd tot_trajectory = Kinematics::inverseDiffKinematicsUr5(this->joints.get_arm(), finalPos, finalOrient);

    
    /* Send config. to the robot */
    int i;
     for(i=0; i< tot_trajectory.rows(); i++){
         send_des_jstate(tot_trajectory.block<1,6>(i,0), this->joints.get_gripper());
         ros::spinOnce();
         loop_time++;
         loop_rate.sleep();
     }

    /* Update actual joint state */
    #if UPDATE_FROM_ROBOT
        this->joints.set_joints_from_robot();
    #else
        Vector6d v = tot_trajectory.block<1,6>(i-1,0).transpose();
        this->joints.set_new( v );
    #endif
}

void Robot::rotate(Vector3d finalPos, Vector3d finalOrient){
    ros::Rate loop_rate(loop_frequency);
    VectorXd startConf(6);
    
    /* Calc. configurations to arrive at the final orientation */
    MatrixXd tot_trajectory = Kinematics::jointSpace_kinematics(this->joints.get_arm(), finalPos, finalOrient);

    
    /* Send config. to the robot */
    int i;
    for(i=0; i< tot_trajectory.rows(); i++){
         send_des_jstate(tot_trajectory.block<1,6>(i,0), this->joints.get_gripper());
         ros::spinOnce();
         loop_time++;
         loop_rate.sleep();
    }

    /* Update actual joint state */
    #if UPDATE_FROM_ROBOT
        this->joints.set_joints_from_robot();
    #else
        Vector6d v = tot_trajectory.block<1,6>(i-1,0).transpose();
        this->joints.set_new( v );
    #endif
}

string Robot::get_string_nearest_model(vector<string> models_list){
    if(models_list.size() == 0){
        ROS_ERROR("list of models is empty");
        return "err";
    }

    Vector3d ef_pos = Mathutils::robotToWorld(Kinematics::directKinematicsUr5(this->joints.get_arm()).block<3,1>(0,3));

    string nearest_model = models_list[0];
    double nearest_distance = (ef_pos - get_pose(nearest_model)).norm();

    double current_distance;
    string current_model;
    for(int i=1; i<models_list.size(); i++){
        current_model = models_list[i];
        current_distance = (ef_pos - get_pose(current_model)).norm();


        if(current_distance < nearest_distance){
            nearest_distance = current_distance;
            nearest_model = current_model;
        }
    }
    return nearest_model;
}

void Robot::set_block_up_right(Vector3d model_pose, Vector3d model_rotation){
    vector<string> models_list;
    get_list_models(models_list);

    bool reversed = false;

    string handled_model;

    double roll = model_rotation(2);
    double pitch = model_rotation(1);
    double yaw = model_rotation(0); 

    if( (abs(roll) < 0.5 && abs(pitch) < 0.5) || ((0.9 < (abs(pitch)/M_PI) && (abs(pitch)/M_PI) < 1.1) && (0.9 < (abs(roll)/M_PI) && (abs(roll)/M_PI) < 1.1)) ){
        cout << "Block already up right! " << endl;
    }else{

        /* upside down */
        if( ((0.9 < (abs(roll)/M_PI) && (abs(roll)/M_PI) < 1.1)  ||  (0.9 < (abs(pitch)/M_PI) && (abs(pitch)/M_PI) < 1.1) ) && !(0.9 < (abs(pitch)/M_PI_2) && (abs(pitch)/M_PI_2) < 1.1) ){
            
            move({model_pose(0), model_pose(1), working_height}, {0, 0, -yaw});

            move_gripper(50);

            move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -yaw});


            handled_model = get_string_nearest_model(models_list);
            attach("ur5", "hand_1_link", handled_model.c_str(), "link");

            move({model_pose(0), model_pose(1), working_height}, {0, 0, 0}); 
            rotate({model_pose(0), model_pose(1), working_height}, {M_PI_2, M_PI, 0});
            move({model_pose(0), model_pose(1), releasing_height}, {M_PI_2, M_PI, 0}); 

            detach("ur5", "hand_1_link", handled_model.c_str(), "link");
            move({model_pose(0), model_pose(1), working_height}, {M_PI_2, M_PI, 0});
            rotate({model_pose(0), model_pose(1), working_height}, {0,0,0});
            
            reversed = true;
        }else if( (0.9 < (abs(pitch)/M_PI_2) && (abs(pitch)/M_PI_2) < 1.1) ){  /* lying on the side */
            move_gripper(75);

           if( pitch > 0 ){
                if( (roll<0 && yaw<0) || (roll>0 && yaw>0) ){
                    move({model_pose(0), model_pose(1), working_height}, {0, 0, -(roll-yaw)});
                    

                    if( roll-yaw < 1.57 && roll-yaw > -1.57 ) {

                        move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll-yaw)});
                    }else{

                        move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll-yaw)});
                        reversed = true;
                    }

                }else{
                    move({model_pose(0), model_pose(1), working_height}, {0, 0, -(roll+yaw)});

                    if( (roll+yaw < 1.57 && roll+yaw > -1.57) || abs(roll+yaw)>4.71 ){

                        move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll+yaw)});
                    }else{

                        move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll+yaw)});
                        reversed = true;
                    }

                }

           }else{
                move({model_pose(0), model_pose(1), working_height}, {0, 0, -(roll+yaw)});

                if( (roll+yaw < 1.57 && roll+yaw > -1.57) ){
                    move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll+yaw)});
                    reversed = true;
                }else{
                    move({model_pose(0), model_pose(1), grasping_height}, {0, 0, -(roll+yaw)});
                }

           }

            handled_model = get_string_nearest_model(models_list);
            attach("ur5", "hand_1_link", handled_model.c_str(), "link");
            move({model_pose(0), model_pose(1), working_height}, {0, 0, 0});
            rotate({model_pose(0), model_pose(1), working_height}, {-M_PI_2, 0, M_PI_2});
            move({model_pose(0), model_pose(1), releasing_height}, {-M_PI_2, 0, M_PI_2});  

        
            detach("ur5", "hand_1_link", handled_model.c_str(), "link");
            move({model_pose(0), model_pose(1), working_height}, {-M_PI_2, 0, M_PI_2});
            rotate({model_pose(0), model_pose(1), working_height}, {0,0,0});

        }else{
            cout << "Block Position & Orientation NOT Handle, Aborting..." << endl;
            return;
        }


        if(reversed == true){
                move({model_pose(0), model_pose(1), releasing_height}, {0,0,0});
                attach("ur5", "hand_1_link", handled_model.c_str(), "link");

                move({model_pose(0), model_pose(1), working_height}, {0,0,0});
                rotate({model_pose(0), model_pose(1), working_height}, {0,0,M_PI});
                move({model_pose(0), model_pose(1), releasing_height}, {0,0,M_PI});
                detach("ur5", "hand_1_link", handled_model.c_str(), "link");

                move({model_pose(0), model_pose(1), working_height}, {0,0,M_PI});
            }

        rotate({model_pose(0), model_pose(1), working_height}, {M_PI_2,M_PI,0});
        move({model_pose(0), model_pose(1), releasing_height}, {M_PI_2,M_PI,0});

        attach("ur5", "hand_1_link", handled_model.c_str(), "link");
        move({model_pose(0), model_pose(1), working_height}, {M_PI_2,M_PI,0});
        rotate({model_pose(0), model_pose(1), working_height}, {0,0,0});
        move({model_pose(0), model_pose(1), releasing_height}, {0,0,0});
        
        detach("ur5", "hand_1_link", handled_model.c_str(), "link");
        move({model_pose(0), model_pose(1), working_height}, {0,0,0});
    }
}