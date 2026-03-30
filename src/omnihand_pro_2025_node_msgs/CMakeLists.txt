cmake_minimum_required(VERSION 3.10)
project(omnihand_pro_2025_node_msgs)
cmake_policy(SET CMP0148 OLD)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ControlMode.msg"
  "msg/CurrentReport.msg"
  "msg/CurrentThreshold.msg"
  "msg/MotorAngle.msg"
  "msg/MotorErrorReport.msg"
  "msg/MotorPos.msg"
  "msg/MotorVel.msg"
  "msg/TactileSensorData.msg"
  "msg/TactileSensor.msg"
  "msg/TemperatureReport.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
