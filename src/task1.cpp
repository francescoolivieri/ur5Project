#include "task.hpp"
using namespace Mathutils;

void task1(Robot robot, Models blocks)
{
    cout << "Task number 1"<< endl;


        
        int length=blocks.get_length();
        if(length>1){
            cout << "Troppi blocchi" << endl;
            return;
        }

    Lego lego_found = blocks.get_block(0);
    cout << "Classe del blocco:" << lego_found.classe << endl;
    const char* modelStr=lego_found.classe.c_str();
    Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
    Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});

    if((strcmp(modelStr, "X2-Y2-Z2") == 0) || (strcmp(modelStr, "X2-Y2-Z2-FILLET") == 0)){
        robot.move_gripper(70);
    }else{

    robot.move_gripper(60);
    }

    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
    string model = robot.get_string_nearest_model(models_list);
    cout << model << endl;
    attach("ur5", "hand_1_link", model.c_str(), "link");

    //switch case per pos finale
        double releaseX;
        double releaseY;
        
        if (strcmp(modelStr, "X1-Y1-Z2") == 0) {
            releaseX = 0.95;
            releaseY = 0.75;
        } else if (strcmp(modelStr, "X1-Y2-Z1") == 0) {
            releaseX = 0.95;
            releaseY = 0.60;
        } else if (strcmp(modelStr, "X1-Y2-Z2") == 0) {
            releaseX = 0.95;
            releaseY = 0.45;
        } else if (strcmp(modelStr, "X1-Y2-Z2-CHAMFER") == 0) {
            releaseX = 0.95;
            releaseY = 0.30;
        } else if (strcmp(modelStr, "X1-Y2-Z2-TWINFILLET") == 0) {
            releaseX = 0.90;
            releaseY = 0.75;
        } else if (strcmp(modelStr, "X1-Y3-Z2") == 0) {
            releaseX = 0.90;
            releaseY = 0.60;
        } else if (strcmp(modelStr, "X1-Y3-Z2-FILLER") == 0) {
            releaseX = 0.90;
            releaseY = 0.45;
        } else if (strcmp(modelStr, "X1-Y4-Z1") == 0) {
            releaseX = 0.90;
            releaseY = 0.30;
        } else if (strcmp(modelStr, "X1-Y4-Z2") == 0) {
            releaseX = 0.85;
            releaseY = 0.35;
        } else if (strcmp(modelStr, "X2-Y2-Z2") == 0) {
            releaseX = 0.85;
            releaseY = 0.60;
        } else if (strcmp(modelStr, "X2-Y2-Z2-FILLET") == 0) {
            releaseX = 0.85;
            releaseY = 0.45;
        } else {
            cout << "Class error" << endl;
        }


    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});
    robot.move(worldToRobot({releaseX, releaseY, 1.1}), {0,0,0});
    robot.move(worldToRobot({releaseX, releaseY, 0.87}), {0,0,0});
    attach("tavolo", "link", model.c_str(), "link");
    detach("ur5", "hand_1_link", model.c_str(), "link");
    robot.move(worldToRobot({releaseX, releaseY, 1.1}), {0,0,0});

    robot.move_gripper(15);

    cout << "Finito"<< endl;
}
