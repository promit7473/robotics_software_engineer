cmake_minimum_required(VERSION 3.10)
project(module_2_assignment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)

add_executable(task1_drive_turtle src/task1_drive_turtle.cpp)
ament_target_dependencies(task1_drive_turtle rclcpp std_msgs geometry_msgs)

add_executable(multi_turtlebot3_controller src/multi_turtlebot3_controller.cpp)
ament_target_dependencies(multi_turtlebot3_controller rclcpp std_msgs geometry_msgs)

add_executable(task3_driver src/task3_driver.cpp)
ament_target_dependencies(task3_driver rclcpp std_msgs geometry_msgs)

add_executable(task4_driver src/task4_driver.cpp)
ament_target_dependencies(task4_driver rclcpp std_msgs geometry_msgs)

install(TARGETS
  task1_drive_turtle
  multi_turtlebot3_controller
  task3_driver
  task4_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})