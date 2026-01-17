cmake_minimum_required(VERSION 3.8)
project(irc_rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(check_node src/check_node.cpp)
ament_target_dependencies(check_node rclcpp std_msgs)
# target_link_libraries(check_node Eigen3::Eigen)
install(TARGETS
  check_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
