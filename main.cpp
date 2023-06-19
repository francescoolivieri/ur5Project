#include <math.h>
#include "robot.hpp"
#include "blocks.hpp"
#include "link_attacher.hpp"
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv){
    
    init();
    /*
    //cout << "here" << endl;
    ros::Rate loop_rate(loop_frequency);
    Vector3d zero;
    zero << 0, 0, 0;
    Vector6d q_des0;
    Vector3d phie = {0,0,0};

    
    Robot robot;
    //robot.move_gripper(70);
    
    Vector3d endOrient = {0, 0, 0};
    //Vector3d endPos = worldToRobot({ 0.9, 0.6, 0.87+0.2});
    Vector3d endPos = { -0.4, -0.3, 0.6};

    robot.move(endPos, endOrient);

    endOrient = {0, 0, M_PI/2};
    endPos = {-0.4, -0.3, 0.8};
    robot.rotate(endPos, endOrient);
*/
    vector<string> models_list; //= {"x1-y2-z2", "x1-y3-z2"};
    Robot robot;
    Vector3d endPos = { -0.4, -0.3, 0.6};
    Vector3d endOrient = {0, 0, M_PI_2};

    get_list_models(models_list);

    /*
    for(int i =0; i< models_list.size(); i++){
        cout << models_list[i] << endl;
    }*/
    
    endPos << worldToRobot({0.7, 0.7, 1.1});
    robot.move(endPos, endOrient);
    robot.move_gripper(70);

    endPos << worldToRobot({0.7, 0.7, 0.91});
    robot.move(endPos, endOrient);
    robot.move_gripper(70);

    string grabbed_model = robot.get_string_nearest_model(models_list);
    attach("ur5", "hand_1_link", grabbed_model.c_str(), "link");

    //robot.move_gripper(15);
    
    robot.move(worldToRobot({0.72, 0.68, 1.1}), endOrient);
    endOrient << -M_PI_2, 0, M_PI_2;
    robot.rotate(worldToRobot({0.72, 0.68, 1.1}), endOrient);
    robot.move(worldToRobot({0.72, 0.68, 0.92}), endOrient);

    //attach("x1-y2-z2", "link", grabbed_model.c_str(), "link");

    detach("ur5", "hand_1_link", grabbed_model.c_str(), "link");
    robot.move(worldToRobot({0.74, 0.68, 1.1}), endOrient);
    robot.rotate(worldToRobot({0.74, 0.68, 1.1}), {0,0,0});
    robot.rotate(worldToRobot({0.7, 0.65, 1.1}), {M_PI_2,M_PI,0});
    robot.move(worldToRobot({0.7, 0.65, 0.91}), {M_PI_2,M_PI,0});
    robot.move(worldToRobot({0.7, 0.69, 0.91}), {M_PI_2,M_PI,0});
    //robot.move_gripper(40);
    attach("ur5", "hand_1_link", grabbed_model.c_str(), "link");
    robot.move(worldToRobot({0.7, 0.7, 1.1}), {0,0,0});

    /*
    robot.move(worldToRobot({0.74, 0.68, 0.93}), {0,0,0});

    //robot.move_gripper(15);
    //detach("x1-y2-z2", "link", grabbed_model.c_str(), "link");
    attach("ur5", "hand_1_link", grabbed_model.c_str(), "link");
    
    robot.move(worldToRobot({0.72, 0.68, 1.1}), {0,0,0});
    
    robot.move(worldToRobot({0.7, 0.7, 0.88}), {-M_PI_2,0,0});

    
    detach("ur5", "hand_1_link", grabbed_model.c_str(), "link");
    //attach("x1-y2-z2", "link", grabbed_model.c_str(), "link");
    robot.move(worldToRobot({0.7, 0.7, 1.0}), {-M_PI_2,0,0});
    robot.rotate(worldToRobot({0.7, 0.7, 1.1}), {0,0,0});*/

    /*
    endPos << worldToRobot({0.7, 0.7, 1.3});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.7, 0.7, 0.93});
    robot.move(endPos, endOrient);

    detach("ur5", "hand_1_link", grabbed_model.c_str(), "link");
    attach("tavolo", "link", grabbed_model.c_str(), "link");

    grabbed_model.clear();

    robot.move_gripper(50);

    endPos << worldToRobot({0.7, 0.7, 1.1});
    robot.move(endPos, endOrient);*/

/*
    endPos << worldToRobot({0.7, 0.6, 1.24});
    robot.move(endPos, endOrient);

     endPos << worldToRobot({0.5, 0.6, 1.24});
    robot.move(endPos, endOrient);

     endPos << worldToRobot({0.4, 0.6, 1.24});
    robot.move(endPos, endOrient);


     endPos << worldToRobot({0.4, 0.6, 1.14});
    robot.move(endPos, endOrient);

     endPos << worldToRobot({0.6, 0.6, 1.24});
    robot.move(endPos, endOrient);

     endPos << worldToRobot({0.9, 0.6, 1.24});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.4, 0.6, 1.24});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.3, 0.6, 1.24});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.5, 0.5, 0.94});
    robot.move(endPos, {0,-M_PI/2, 0});

    cin ;

    endPos << worldToRobot({0.9, 0.6, 1.14});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.3, 0.9, 0.9});
    robot.move(endPos, endOrient);

    endPos << worldToRobot({0.5, 0.6, 1.24});
    robot.move(endPos, endOrient);*/

    cout << "HELo" << endl;
    return 0;

}
