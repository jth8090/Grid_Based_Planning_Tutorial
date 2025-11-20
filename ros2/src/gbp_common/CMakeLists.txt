cmake_minimum_required(VERSION 3.5)
project(gbp_common)

# 언어 표준 (기본)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ament_cmake 필수
find_package(ament_cmake REQUIRED)

# 우리가 만든 리소스 디렉토리들을 install/share/<pkg_name>/ 밑으로 복사
install(
  DIRECTORY
    maps
    rviz
    launch
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

# ament 패키지로 등록
ament_package()
