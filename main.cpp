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
#include "link_attacher.hpp"
#include <iostream>
#include <string>
#include "task.hpp"



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

    Models blocks = Models();


    switch (n_task)
    {
    case 1:{
        
       
        task1(robot, blocks);
        break;
    }
    case 2:{

        task2(robot, blocks);
        break;
    }
    case 3:{

        task3(robot, blocks);
        break;
    }
    case 4:{

        task4(robot, blocks);
        break;
    }
    default:{
        cout << "Non valid task number" << endl;
        break;
    }
    }

 

    cout << "HELo" << endl;
    return 0;

}