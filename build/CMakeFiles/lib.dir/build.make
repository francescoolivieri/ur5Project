# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lib.dir/flags.make

CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o: ../src/ur5_kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o -c /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/ur5_kinematics.cpp

CMakeFiles/lib.dir/src/ur5_kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/src/ur5_kinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/ur5_kinematics.cpp > CMakeFiles/lib.dir/src/ur5_kinematics.cpp.i

CMakeFiles/lib.dir/src/ur5_kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/src/ur5_kinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/ur5_kinematics.cpp -o CMakeFiles/lib.dir/src/ur5_kinematics.cpp.s

CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o: ../src/custom_joint_pub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o -c /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/custom_joint_pub.cpp

CMakeFiles/lib.dir/src/custom_joint_pub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/src/custom_joint_pub.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/custom_joint_pub.cpp > CMakeFiles/lib.dir/src/custom_joint_pub.cpp.i

CMakeFiles/lib.dir/src/custom_joint_pub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/src/custom_joint_pub.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/src/custom_joint_pub.cpp -o CMakeFiles/lib.dir/src/custom_joint_pub.cpp.s

# Object files for target lib
lib_OBJECTS = \
"CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o" \
"CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o"

# External object files for target lib
lib_EXTERNAL_OBJECTS =

devel/lib/liblib.so: CMakeFiles/lib.dir/src/ur5_kinematics.cpp.o
devel/lib/liblib.so: CMakeFiles/lib.dir/src/custom_joint_pub.cpp.o
devel/lib/liblib.so: CMakeFiles/lib.dir/build.make
devel/lib/liblib.so: CMakeFiles/lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library devel/lib/liblib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lib.dir/build: devel/lib/liblib.so

.PHONY : CMakeFiles/lib.dir/build

CMakeFiles/lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lib.dir/clean

CMakeFiles/lib.dir/depend:
	cd /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build /home/fed/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/cpp/build/CMakeFiles/lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lib.dir/depend

