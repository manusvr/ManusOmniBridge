cmake_minimum_required(VERSION 3.10)
project(omnihand_node)

find_package(ament_cmake REQUIRED)

# This is the magic part: it copies your launch folder to the install directory
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so"
)
install(PROGRAMS
  lib/omnihand_2025_node
  lib/omnihand_pro_2025_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
