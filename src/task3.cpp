#include "task.hpp"
using namespace Mathutils;

void task3(Robot robot, Models blocks)
{
    cout << "Task number 3"<< endl;

        int length=blocks.get_length();
        cout << "Lunghezza:" << length << endl;


        double x1_y1_z2_height=0.87;
        double x1_y2_z1_height=0.87;
        double x1_y2_z2_height=0.87;
        double x1_y2_z2_chamfer_height=0.87;
        double x1_y2_z2_twinfillet_height=0.87;
        double x1_y3_z2_height=0.87;
        double x1_y3_z2_fillet_height=0.87;
        double x1_y4_z1_height=0.87;
        double x1_y4_z2_height=0.87;
        double x2_y2_z2_height=0.87;
        double x2_y2_z2_fillet_height=0.87;


        for(int i=0; i<length; i++){
            Lego lego_found = blocks.get_block(i);
            cout << "Classe del blocco:" << lego_found.classe << endl;
            const char* modelStr=lego_found.classe.c_str();
            Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
            Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};

            Vector3d lego_rot_inv={lego_found.yaw, -(lego_found.pitch), lego_found.roll};
            
            robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});

            

            if((strcmp(modelStr, "X2-Y2-Z2") == 0) || (strcmp(modelStr, "X2-Y2-Z2-FILLET") == 0)){
                robot.move_gripper(70);
            }else{

            robot.move_gripper(60);
            }

            robot.set_block_up_right(worldToRobot(lego_pos), lego_rot_inv);

            if(lego_found.pitch>0.5 || lego_found.pitch <-0.5){
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,0});
            }else{
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
            }
            string model = robot.get_string_nearest_model(models_list);
            cout << model << endl;
            attach("ur5", "hand_1_link", model.c_str(), "link");

            double height=0;

            double releaseX;
            double releaseY;

            if (strcmp(modelStr, "X1-Y1-Z2") == 0) {
                releaseX = 0.95;
                releaseY = 0.75;
                height=x1_y1_z2_height;
                x1_y1_z2_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y2-Z1") == 0) {
                releaseX = 0.95;
                releaseY = 0.60;
                height=x1_y2_z1_height;
                x1_y2_z1_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y2-Z2") == 0) {
                releaseX = 0.95;
                releaseY = 0.45;
                height=x1_y2_z2_height;
                x1_y2_z2_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y2-Z2-CHAMFER") == 0) {
                releaseX = 0.95;
                releaseY = 0.30;
                height=x1_y2_z2_chamfer_height;
                x1_y2_z2_chamfer_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y2-Z2-TWINFILLET") == 0) {
                releaseX = 0.90;
                releaseY = 0.75;
                height=x1_y2_z2_twinfillet_height;
                x1_y2_z2_twinfillet_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y3-Z2") == 0) {
                releaseX = 0.90;
                releaseY = 0.60;
                height=x1_y3_z2_height;
                x1_y3_z2_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y3-Z2-FILLER") == 0) {
                releaseX = 0.90;
                releaseY = 0.45;
                height=x1_y3_z2_fillet_height;
                x1_y3_z2_fillet_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y4-Z1") == 0) {
                releaseX = 0.90;
                releaseY = 0.30;
                height=x1_y4_z1_height;
                x1_y4_z1_height+=0.056;
            } else if (strcmp(modelStr, "X1-Y4-Z2") == 0) {
                releaseX = 0.85;
                releaseY = 0.75;
                height=x1_y4_z2_height;
                x1_y4_z2_height+=0.056;
            } else if (strcmp(modelStr, "X2-Y2-Z2") == 0) {
                releaseX = 0.85;
                releaseY = 0.60;
                height=x2_y2_z2_height;
                x2_y2_z2_height+=0.056;
            } else if (strcmp(modelStr, "X2-Y2-Z2-FILLET") == 0) {
                releaseX = 0.85;
                releaseY = 0.45;
                height=x2_y2_z2_fillet_height;
                x2_y2_z2_fillet_height+=0.056;
            } else {
                cout << "Class error" << endl;
            }


            robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});
            robot.move(worldToRobot({releaseX, releaseY, 1.1}), {0,0,0});
            robot.move(worldToRobot({releaseX, releaseY, height}), {0,0,0});
            attach("tavolo", "link", model.c_str(), "link");
            detach("ur5", "hand_1_link", model.c_str(), "link");
            robot.move(worldToRobot({releaseX, releaseY, 1.1}), {0,0,0});
            robot.move_gripper(15);
        }

        cout << "Finito"<< endl;
}
