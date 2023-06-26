# ur5Project
IMPORTANT: to run this project you need: [locosim](https://github.com/mfocchi/locosim), [gazebo_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) and these [blocks](https://drive.google.com/drive/folders/16mWvEDk631fzUIbqhHXUmVg8F2yUmw3h?usp=sharing).

## Project Structure
This project contains the following core folders:
- docs: contains the doxygen files, here you can see the generated documentation (can be easily updated using the doxygen command)
- include: contains all the libraries files (.hpp)
- src: contains all the source files (.cpp) exception for the main.cpp


## How to Build
Download this project, place it in src folder and then perform the following command:

```
~/ros_ws$ catkin_make install
```

## How to Run
1. run the python script ```ur5_generic.py``` (you can find it in locosim/robot_control/lab_exercises/lab_palopoli)
2. start this command:
```
rosrun lab_palopoli main x
```
where x is the number of the task we want to perform.

    

