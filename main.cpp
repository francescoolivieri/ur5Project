/**
 * @file main.cpp
 * @author Federico Adami, Francesco Olivieri, Eddie Veronese
 * @brief 
 * @version 0.1
 * @date 2023-06-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <math.h>
#include "robot.hpp"
#include "blocks.hpp"
#include "link_attacher.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace Mathutils;

vector<string> models_list;

int main(int argc, char **argv){
    init();

    int n_task=0;
    if(argc>1){
        n_task=atoi(argv[1]);
    }
    
    Robot robot;

    Blocks blocks = Blocks();

    switch (n_task)
    {
    case 1:{
        cout << "Task number 1"<< endl;

        //task1(blocks, robot)
        
        int length=blocks.get_length();
        if(length>1){
            cout << "Troppi blocchi" << endl;
            break;
        }

        

        
        Lego lego_found = blocks.get_block(0);
        cout << "Classe del blocco:" << lego_found.classe << endl;
        const char* modelStr=lego_found.classe.c_str();
        Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
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
                releaseY = 0.75;
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
        
        
        
        break;
    }
    case 2:{

        cout << "Task number 2"<< endl;
        int length=blocks.get_length();
        cout << "Lunghezza:" << length << endl;
        for(int i=0; i<length; i++){
            Lego lego_found = blocks.get_block(i);
            cout << "Classe del blocco:" << lego_found.classe << endl;
            const char* modelStr=lego_found.classe.c_str();
            Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
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
           
            cout << modelStr << endl;
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
            } else if (strcmp(modelStr, "xX-Y3-Z2") == 0) {
                releaseX = 0.90;
                releaseY = 0.60;
            } else if (strcmp(modelStr, "X1-Y3-Z2-FILLER") == 0) {
                releaseX = 0.85;
                releaseY = 0.35;
            } else if (strcmp(modelStr, "X1-Y4-Z1") == 0) {
                releaseX = 0.90;
                releaseY = 0.30;
            } else if (strcmp(modelStr, "X1-Y4-Z2") == 0) {
                releaseX = 0.85;
                releaseY = 0.60;
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
        }
        cout << "Finito"<< endl;
        break;
    }
    case 3:{
        cout << "Task number 3"<< endl;

        int length=blocks.get_length();
        cout << "Lunghezza:" << length << endl;

        /*
        
        */
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
            Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
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

            double height=0;
            //switch case per pos finale
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
        break;
    }
    case 4:{
        cout << "Task number 4"<< endl;
        int length=blocks.get_length();
        cout << "Lunghezza:" << length << endl;

        bool already_pick=false;
        

        //conto i blocchi presenti
        for(int i=0; i<length; i++){

            Lego lego_found = blocks.get_block(i);
            const char* modelStr=lego_found.classe.c_str();
            //se è lungo 4 lo prendo
            if (strcmp(modelStr, "X1-Y4-Z2") == 0) {

                if(!already_pick){
                    cout << "Classe del blocco:" << lego_found.classe << endl;
                    Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
                    Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                    Vector3d lego_rot2 = {lego_found.yaw, lego_found.pitch, lego_found.roll};
                    
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                    robot.move_gripper(60);
                    robot.set_block_up_right(lego_pos, lego_rot2);
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
                    string model = robot.get_string_nearest_model(models_list);
                    cout << model << endl;
                    attach("ur5", "hand_1_link", model.c_str(), "link");
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});

                    robot.move(worldToRobot({0.95, 0.5, 1.1}), {0,0,0});
                    robot.move(worldToRobot({0.95, 0.5, 0.87}), {0,0,0});
                    attach("tavolo", "link", model.c_str(), "link");
                    detach("ur5", "hand_1_link", model.c_str(), "link");
                    robot.move(worldToRobot({0.95, 0.5, 1.1}), {0,0,0});
                    robot.move_gripper(15);
                    already_pick=true;
                }else{
                    cout << "Classe del blocco:" << lego_found.classe << endl;
                    Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
                    Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                    Vector3d lego_rot2 = {lego_found.yaw, lego_found.pitch, lego_found.roll};
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                    robot.move_gripper(60);
                    robot.set_block_up_right(lego_pos, lego_rot2);
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
                    string model = robot.get_string_nearest_model(models_list);
                    cout << model << endl;
                    attach("ur5", "hand_1_link", model.c_str(), "link");
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});

                    robot.move(worldToRobot({0.88, 0.5, 1.1}), {0,0,0});
                    robot.move(worldToRobot({0.88, 0.5, 0.87}), {0,0,0});
                    attach("tavolo", "link", model.c_str(), "link");
                    detach("ur5", "hand_1_link", model.c_str(), "link");
                    robot.move(worldToRobot({0.85, 0.5, 1.1}), {0,0,0});
                    robot.move_gripper(15);
                }
              
            }
        }

        for(int i=0; i<length; i++){

            Lego lego_found = blocks.get_block(i);
            const char* modelStr=lego_found.classe.c_str();
            //se è lungo 3 lo prendo
            if (strcmp(modelStr, "X1-Y3-Z2") == 0) {
                cout << "Classe del blocco:" << lego_found.classe << endl;
                Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
                Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                robot.move_gripper(60);
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
                string model = robot.get_string_nearest_model(models_list);
                cout << model << endl;
                attach("ur5", "hand_1_link", model.c_str(), "link");
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});

                robot.move(worldToRobot({0.915, 0.5, 1.1}), {0,0,0});
                robot.move(worldToRobot({0.915, 0.5, 1.1}), {0,0,M_PI_2});
                robot.move(worldToRobot({0.915, 0.5, 0.925}), {0,0,M_PI_2});
                attach("tavolo", "link", model.c_str(), "link");
                detach("ur5", "hand_1_link", model.c_str(), "link");
                robot.move(worldToRobot({0.915, 0.5, 1.1}), {0,0,0});
                robot.move_gripper(15);

            }
        }

        for(int i=0; i<length; i++){

            Lego lego_found = blocks.get_block(i);
            const char* modelStr=lego_found.classe.c_str();
            //se è lungo 1 lo prendo
            if (strcmp(modelStr, "X1-Y1-Z2") == 0) {
                cout << "Classe del blocco:" << lego_found.classe << endl;
                Vector3d lego_pos = {lego_found.x_base, lego_found.y_base, lego_found.z_base};
                Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                robot.move_gripper(60);
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 0.87}), {0,0,lego_rot(2)});
                string model = robot.get_string_nearest_model(models_list);
                cout << model << endl;
                attach("ur5", "hand_1_link", model.c_str(), "link");
                robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,0});

                robot.move(worldToRobot({0.915, 0.5, 1.1}), {0,0,0});
                robot.move(worldToRobot({0.915, 0.5, 0.98}), {0,0,0});
                attach("tavolo", "link", model.c_str(), "link");
                detach("ur5", "hand_1_link", model.c_str(), "link");
                robot.move(worldToRobot({0.915, 0.5, 1.1}), {0,0,0});
                robot.move_gripper(15);

            }
        }
        
        
        cout << "Finito"<< endl;
        break;
    }
    default:{
        cout << "Non valid task number" << endl;
        break;
    }
    }

   // robot.set_block_up_right(lego_pos);    

    cout << "HELo" << endl;
    return 0;

}