cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(mazeSolving src/wall_follower_maze_solver.cpp)
ament_target_dependencies(mazeSolving rclcpp sensor_msgs geometry_msgs)

add_executable(mazeSolving1 src/maze_solver.cpp)
ament_target_dependencies(mazeSolving1 rclcpp sensor_msgs geometry_msgs)


install(TARGETS mazeSolving mazeSolving1
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
