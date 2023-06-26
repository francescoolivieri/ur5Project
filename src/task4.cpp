#include "task.hpp"
using namespace Mathutils;

void task4(Robot robot, Models blocks)
{
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
                    Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
                    Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                    Vector3d lego_rot2 = {lego_found.yaw, lego_found.pitch, lego_found.roll};
                    
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                    robot.move_gripper(60);
                    robot.set_block_up_right(worldToRobot(lego_pos), lego_rot2);
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
                    Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
                    Vector3d lego_rot = {lego_found.roll, lego_found.pitch, lego_found.yaw};
                    Vector3d lego_rot2 = {lego_found.yaw, lego_found.pitch, lego_found.roll};
                    robot.move(worldToRobot({lego_pos(0), lego_pos(1), 1.1}), {0,0,lego_rot(2)});
                    robot.move_gripper(60);
                    robot.set_block_up_right(worldToRobot(lego_pos), lego_rot2);
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
                Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
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
                Vector3d lego_pos = {lego_found.x_base+0.015, lego_found.y_base, lego_found.z_base};
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
}
