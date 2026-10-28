cmake_minimum_required(VERSION 3.8)

project(samarth_arm_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# -----------------------------
# Trajectory publisher
# -----------------------------

add_executable(
  trajectory_publisher
  src/trajectory_publisher.cpp
)

ament_target_dependencies(
  trajectory_publisher
  rclcpp
  trajectory_msgs
  sensor_msgs
)


# -----------------------------
# Custom PID controller
# -----------------------------

add_executable(
  pid_controller
  src/pid_controller.cpp
)

ament_target_dependencies(
  pid_controller
  rclcpp
  sensor_msgs
  std_msgs
)


# -----------------------------
# Install C++ executables
# -----------------------------

install(
  TARGETS
    trajectory_publisher
    pid_controller

  DESTINATION
    lib/${PROJECT_NAME}
)


# -----------------------------
# Install Python scripts
# -----------------------------

install(
  PROGRAMS
    scripts/joint_logger.py

  DESTINATION
    lib/${PROJECT_NAME}
)


# -----------------------------
# Install configs and launches
# -----------------------------

install(
  DIRECTORY
    config
    launch

  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()
