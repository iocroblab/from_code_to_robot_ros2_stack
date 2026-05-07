cmake_minimum_required(VERSION 3.8)
project(fctr_ur_package)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
