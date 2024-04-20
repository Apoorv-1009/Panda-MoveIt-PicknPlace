# Panda-MoveIt-PicknPlace

Pick and Place Script with MoveIt

Video link: https://youtu.be/FNqIYG1AAtQ

This script demonstrates pick and place functionality using MoveIt in ROS 2. It utilizes the MoveIt MoveGroup Interface to control the motion of a robotic arm and gripper. Below are the libraries used in this script:

- `memory`: Provides utilities for managing dynamic memory allocation.
- `unistd.h`: Header file providing access to the POSIX operating system API.
- `vector`: Implements a dynamic array data structure.
- `chrono`: Provides time-related functionality.
- `thread`: Enables multi-threading in C++.
- `rclcpp`: ROS Client Library for C++.
- `moveit/move_group_interface/move_group_interface.h`: MoveIt C++ API for controlling robot motion.

## Instructions for Use

1. **Launch Rviz**: Use the following command to launch Rviz for visualization:

   ```bash
   ros2 launch panda_config demo.launch.py
   ```

2. **Run the Pick and Place Script**: Execute the pick and place script using the following command:

   ```bash
   ros2 run hello_moveit hello_moveit
   ```

---
