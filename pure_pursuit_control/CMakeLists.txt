cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executables
add_executable(dummy_path_publisher src/dummy_path_publisher.py)
ament_target_dependencies(dummy_path_publisher rclpy nav_msgs geometry_msgs)

add_executable(dummy_pose_publisher src/dummy_pose_publisher.py)
ament_target_dependencies(dummy_pose_publisher rclpy geometry_msgs)

add_executable(dummy_tracking_distance_publisher src/dummy_tracking_distance_publisher.py)
ament_target_dependencies(dummy_tracking_distance_publisher rclpy std_msgs)

add_executable(pure_pursuit src/pure_pursuit.py)
ament_target_dependencies(pure_pursuit rclpy std_msgs nav_msgs geometry_msgs)

# Install
install(TARGETS
  dummy_path_publisher
  dummy_pose_publisher
  dummy_tracking_distance_publisher
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

