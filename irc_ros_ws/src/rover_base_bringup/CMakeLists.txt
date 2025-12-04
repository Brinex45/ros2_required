cmake_minimum_required(VERSION 3.8)
project(rover_base_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz                  #directory which needs to be installed
  DESTINATION share/${PROJECT_NAME}/      #directory where it will be installed
)

ament_package()
