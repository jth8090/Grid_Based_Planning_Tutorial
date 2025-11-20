cmake_minimum_required(VERSION 3.5)
project(gbp_common)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    maps
    rviz
    launch
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
