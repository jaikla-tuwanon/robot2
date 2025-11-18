cmake_minimum_required(VERSION 3.5)
project(control_rbkairos)

# -------------------------
# Find dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_ign_gazebo REQUIRED)

# -------------------------
# Install Python package
# -------------------------
ament_python_install_package(${PROJECT_NAME})

# -------------------------
# Install launch, URDF, config
# -------------------------
install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------
# Install worlds (เฉพาะไฟล์ .world)
# -------------------------
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# -------------------------
# Install executable scripts
# -------------------------
install(
  PROGRAMS
    control_rbkairos/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Finalize package
# -------------------------
ament_package()
