# filepath: /home/aims/aims_ws/src/aims_controller/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(aims_controller)

# Find ament macros and libraries used in the package
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(os REQUIRED)
find_package(cv2 REQUIRED)
find_package(ultralytics REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Adds the controller_node.py executable
add_executable(controller_node ./src/aims_controller/controller_node.py)
# Determines what dependencies are needed for the controller_node.py executable
ament_target_dependencies(controller_node
  rclpy
  os
  std_msgs
)

# Adds the cv_node.py executable
add_executable(cv_node ./src/aims_controller/cv_node.py)
# Determines what dependencies are needed for the cv_node.py executable
ament_target_dependencies(cv_node
  rclpy
  os
  std_msgs
  sensor_msgs
  cv_bridge
  cv2
  ultralytics
)

"""
# Adds the receiver_node.py executable
add_executable(receiver_node ./src/aims_controller/receiver_node.py)
# Determines what dependencies are needed for the receiver_node.py executable
ament_target_dependencies(receiver_node
  rclpy
  std_msgs
)

# Adds the aims_launch.py executable
add_executable(aims_launch ./src/launch/aims_launch.py)
# Determines what dependencies are needed for the aims_launch.py executable
ament_target_dependencies(aims_launch
  launch
  launch_ros
)

# Install the Python modules and executables
install(
  TARGETS controller_node cv_node receiver_node aims_launch
  DESTINATION lib/${PROJECT_NAME} # the path implicitly has ~/aims_ws/install/aims_controller before it
)
"""

# Install Python executables
install(
  PROGRAMS
    src/aims_controller/controller_node.py
    src/aims_controller/cv_node.py
    src/aims_controller/receiver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY src/launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the .xml files
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME} # the path implicitly has ~/aims_ws/install/aims_controller before it
)

# Allows package to be used in other packages.
ament_package()