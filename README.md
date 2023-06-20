# ur5Project

How is structured:
    -custom libraries in src and include to implement functions
    -main.cpp file where functions can be called and tested

USAGE:
    -to use this code you need to replace "locosi/cpp" contents with this repo 
    -to execute main: rosrun lab_paolopoli main

TO_DO:
    -in order to better organize code is necessary to create classes and possibly separate differnatial kinematics functions from normal   kinmatics functions
    
NOTE:
    -avoid using global variables in main.cpp (th varaible was moved locally)
    
SIDE-NOTES:
    In order to make rotations of blocks work you need:
    -reduce gravity at -3 [m/s^2] 
    -set all the params of the objets to 0.2 of inertia
    -set block mass at 0.01
    (other params configs are possible this is the first one that worked for me)

