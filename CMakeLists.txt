cmake_minimum_required(VERSION 2.8.3)
project(lab_palopoli)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs ros_impedance_controller)
find_package(Eigen3 3.3 REQUIRED)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(main main.cpp)

add_library(lib src/ur5_kinematics.cpp src/custom_joint_pub.cpp src/robot.cpp src/blocks.cpp src/link_attacher.cpp src/task1.cpp src/task2.cpp src/task3.cpp src/task4.cpp)  
target_link_libraries(main lib ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})


install(TARGETS main lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
