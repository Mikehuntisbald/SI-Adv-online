cmake_minimum_required(VERSION 3.14)
project(pc_array)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloud2Array.msg"
  DEPENDENCIES sensor_msgs
)

# This macro ensures modules and global scripts declared therein get installed
# See: http://wiki.ros.org/catkin/CMakeLists.txt#Installing
ament_package()


