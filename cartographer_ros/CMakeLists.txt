# Copyright 2016 The Cartographer Authors
# Copyright 2022 Wyca Robotics (for the ROS2 conversion)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(builtin_interfaces REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(absl REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/assets_writer.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  src/node_constants.cpp
  src/node.cpp
  src/node_options.cpp
  src/offline_node.cpp
  src/playable_bag.cpp
  src/ros_log_sink.cpp
  src/ros_map.cpp
  src/ros_map_writing_points_processor.cpp
  src/sensor_bridge.cpp
  src/submap.cpp
  src/tf_bridge.cpp
  src/time_conversion.cpp
  src/trajectory_options.cpp
  src/urdf_reader.cpp
  src/metrics/family_factory.cpp
  src/metrics/internal/family.cpp
  src/metrics/internal/histogram.cpp
  )

set(dependencies
  builtin_interfaces
  cartographer
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  visualization_msgs
  rosbag2_cpp
  rosbag2_storage
  urdf
  urdfdom
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  )
target_link_libraries(${PROJECT_NAME} cartographer ${PCL_LIBRARIES})

# Executables

# 1. C++ 파일을 빌드 대상으로 추가
add_executable(tf_transform_publisher src/tf_transform_publisher.cpp)
# 2. 의존성 추가 (rclcpp, tf2_ros 등)
ament_target_dependencies(tf_transform_publisher
  rclcpp
  tf2_ros
  geometry_msgs
)
# 3. 기존 프로젝트 라이브러리와 링크
target_link_libraries(tf_transform_publisher ${PROJECT_NAME})

add_executable(cartographer_node src/node_main.cpp)
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node ${dependencies})

add_executable(cartographer_occupancy_grid_node src/occupancy_grid_node_main.cpp)
target_link_libraries(cartographer_occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_occupancy_grid_node ${dependencies})

add_executable(cartographer_offline_node src/offline_node_main.cpp)
target_link_libraries(cartographer_offline_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_offline_node ${dependencies})

add_executable(cartographer_assets_writer src/assets_writer_main.cpp)
target_link_libraries(cartographer_assets_writer ${PROJECT_NAME})
ament_target_dependencies(cartographer_assets_writer ${dependencies})

add_executable(cartographer_pbstream_map_publisher src/pbstream_map_publisher_main.cpp)
target_link_libraries(cartographer_pbstream_map_publisher ${PROJECT_NAME})
ament_target_dependencies(cartographer_pbstream_map_publisher ${dependencies})

add_executable(cartographer_pbstream_to_ros_map src/pbstream_to_ros_map_main.cpp)
target_link_libraries(cartographer_pbstream_to_ros_map ${PROJECT_NAME})
ament_target_dependencies(cartographer_pbstream_to_ros_map ${dependencies})

add_executable(cartographer_rosbag_validate src/rosbag_validate_main.cpp)
target_link_libraries(cartographer_rosbag_validate ${PROJECT_NAME})
ament_target_dependencies(cartographer_rosbag_validate ${dependencies})

if($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
  target_compile_definitions(cartographer_rosbag_validate PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cartographer_node
  cartographer_occupancy_grid_node
  cartographer_offline_node
  cartographer_assets_writer
  cartographer_pbstream_map_publisher
  cartographer_pbstream_to_ros_map
  cartographer_rosbag_validate
  tf_transform_publisher  # ✅ 요거 추가
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY configuration_files urdf launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_export_dependencies(tf2_ros)
ament_package()

# Non converted bin:
#google_binary(cartographer_dev_pbstream_trajectories_to_rosbag
#  SRCS
#  dev/pbstream_trajectories_to_rosbag_main.cc
#)

#google_binary(cartographer_dev_rosbag_publisher
#  SRCS
#  dev/rosbag_publisher_main.cc
#)


#google_binary(cartographer_dev_trajectory_comparison
#  SRCS
#  dev/trajectory_comparison_main.cc
#)


## TODO(cschuet): Add support for shared library case.
#if (${BUILD_GRPC})
#  google_binary(cartographer_grpc_node
#    SRCS
#      cartographer_grpc/node_grpc_main.cc
#  )


#  google_binary(cartographer_grpc_offline_node
#    SRCS
#      cartographer_grpc/offline_node_grpc_main.cc
#  )

#  install(PROGRAMS
#    ../scripts/cartographer_grpc_server.sh
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()
