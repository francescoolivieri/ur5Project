#include <math.h>
#include "robot.hpp"
#include "blocks.hpp"
#include "link_attacher.hpp"
#include <iostream>
#include <string>
using namespace std;

vector<string> models_list;

int main(int argc, char **argv){
    
    init();
    
    Robot robot;
    Vector3d endPos = { -0.4, -0.3, 0.6};
    Vector3d endOrient = {0, 0, M_PI_2};

    Vector3d model_pose = get_pose("x1-y3-z2");
    robot.set_block_up_right(model_pose);

    

    cout << "HELo" << endl;
    return 0;

}
