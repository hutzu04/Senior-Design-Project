cmake_minimum_required(VERSION 3.5)
project(rqt_aims)

# Find ament macros and libraries used in the package
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rqt_gui REQUIRED)
find_package(rqt_gui_py REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(os REQUIRED)
find_package(re REQUIRED)
find_package(serial REQUIRED)
find_package(time REQUIRED)
find_package(threading REQUIRED)
find_package(subprocess REQUIRED)
find_package(rospkg REQUIRED)
find_package(signal REQUIRED)
find_package(logging REQUIRED)

# Adds the convert_g_to_can.py executable
add_executable(convert_g_to_can ./src/rqt_aims/convert_g_to_can.py)
# Determines what dependencies are needed for the convert_g_to_can.py executable
ament_target_dependencies(convert_g_to_can
  os
  re
)

# Adds the send_g_code.py executable
add_executable(send_g_code ./src/rqt_aims/send_g_code.py)
# Determines what dependencies are needed for the send_g_code.py executable
ament_target_dependencies(send_g_code
  rclpy
  os
  re
  serial
  time
  threading
)

# Adds the aims.py executable
add_executable(aims ./src/rqt_aims/aims.py)
# Determines what dependencies are needed for the aims.py executable
ament_target_dependencies(aims
  os
  rclpy
  rospkg
  subprocess
  re
  signal
  serial
  threading
  time
  logging
)

# Install the Python modules and executables
install(
  TARGETS convert_g_to_can send_g_code aims
  DESTINATION lib/${PROJECT_NAME} # the path implicitly has ~/aims_ws/install/rqt_aims before it
)

# Install the resource folder
install(
  DIRECTORY ./resource
  DESTINATION share/${PROJECT_NAME}/resource # the path implicitly has ~/aims_ws/install/rqt_aims before it
)

# Install the .xml files
install(
  FILES package.xml plugin.xml
  DESTINATION share/${PROJECT_NAME} # the path implicitly has ~/aims_ws/install/rqt_aims before it
)

# Allows package to be used in other packages.
ament_package()