cmake_minimum_required(VERSION 3.8)
project(threelink_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
# Install worlds files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes directory
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/payload_attach_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
