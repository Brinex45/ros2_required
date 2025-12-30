cmake_minimum_required(VERSION 3.8)
project(irc_rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add_executable(arm_kinematics_node src/arm_kinematic.cpp)
# ament_target_dependencies(arm_kinematics_node rclcpp irc_custom_interfaces std_msgs joint_trajectory_controller)
# target_link_libraries(arm_kinematics_node Eigen3::Eigen)
# install(TARGETS
#   arm_kinematics_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY urdf meshes src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
