# ur5Project
IMPORTANT: to run this project you need: [locosim](https://github.com/mfocchi/locosim), [gazebo_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) and these [blocks](https://drive.google.com/drive/folders/16mWvEDk631fzUIbqhHXUmVg8F2yUmw3h?usp=sharing).

## Project Structure
This project contains the following core folders:
- docs: contains the doxygen files, here you can see the generated documentation (can be easily updated using the doxygen command)
- include: contains all the libraries files (.hpp)
- src: contains all the source files (.cpp) exception for the main.cpp

Every document in the include folder has been documented using doxygen, to see more details on every files' purpose you should consider to use that tool. 

## How to Build
Download this project, place it in src folder inside locosim (ros workspace) and then perform the following command:

```
~/ros_ws$ catkin_make install
```

## How to Run
1. make sure you include in your tavolo.world file the following directive: ```<plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>```
2. run the python script ```ur5_generic.py``` (you can find it in locosim/robot_control/lab_exercises/lab_palopoli)
3. launch this command from the ros workspace:
```
rosrun lab_palopoli main x
```
where x is the number of the task we want to perform.

    

