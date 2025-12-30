cmake_minimum_required(VERSION 3.8)
project(rover_base_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_to_stamped src/twist_to_stamped.cpp)   #"cpp_node" name of executable
ament_target_dependencies(twist_to_stamped rclcpp geometry_msgs)         #these two lines create an executable but not install it

install(TARGETS            #installs the executable
  twist_to_stamped
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes                  #directory which needs to be installed
  DESTINATION share/${PROJECT_NAME}/      #directory where it will be installed
)

ament_package()
