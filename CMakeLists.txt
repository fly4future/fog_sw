# This CMakeLists is needed for integration with CLIon
# Please keep it here
cmake_minimum_required(VERSION 3.14)
project("FOG_SW")

add_subdirectory(ros2_ws/src/orbslam_node)
add_subdirectory(ros2_ws/src/px4_mavlink_ctrl)
add_subdirectory(ros2_ws/src/px4_msgs)
add_subdirectory(ros2_ws/src/px4_ros_com)
add_subdirectory(ros2_ws/src/realsense-ros/realsense2_camera_msgs)
add_subdirectory(ros2_ws/src/realsense-ros/realsense2_camera)
add_subdirectory(ros2_ws/src/realsense-ros/realsense2_description)
