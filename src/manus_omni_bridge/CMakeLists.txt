cmake_minimum_required(VERSION 3.10)
project(manus_omni_bridge)

# Suppress Deprecation Warnings
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(NLopt REQUIRED)

# ROS 2 Message Dependencies
find_package(manus_ros2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(omnihand_2025_node_msgs REQUIRED)
find_package(omnihand_pro_2025_node_msgs REQUIRED)

add_executable(bridge_node 
    src/manus_bridge_node.cpp 
    src/manusOmni.cpp
    src/manusOmniPro.cpp
    src/ManusUniversalPose.cpp
    src/PinocchioRetargeter.cpp
)

# Set Include Directories
target_include_directories(bridge_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    "/opt/ros/humble/include/pinocchio"
)

# Explicitly link external (non-ROS) libraries
target_link_libraries(bridge_node 
    Eigen3::Eigen
    ${pinocchio_LIBRARIES}
    NLopt::nlopt
)

# Use Ament for ROS-specific dependencies
ament_target_dependencies(bridge_node
  rclcpp
  sensor_msgs
  visualization_msgs
  manus_ros2_msgs
  omnihand_2025_node_msgs
  omnihand_pro_2025_node_msgs
)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()