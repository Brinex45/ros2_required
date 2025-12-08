cmake_minimum_required(VERSION 3.8)
project(arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irc_custom_interfaces REQUIRED)

add_executable(arm_kinematics_node src/arm_kinematic.cpp)
ament_target_dependencies(arm_kinematics_node rclcpp irc_custom_interfaces)

install(TARGETS
  arm_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
