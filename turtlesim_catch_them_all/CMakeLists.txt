cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch_them_all)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(generate_new_turtles src/generate_new_turtles.cpp)
ament_target_dependencies(generate_new_turtles rclcpp turtlesim my_robot_interfaces example_interfaces)

add_executable(hunt_turtles src/hunt_turtles.cpp)
ament_target_dependencies(hunt_turtles rclcpp turtlesim my_robot_interfaces geometry_msgs example_interfaces)

install(TARGETS
  generate_new_turtles
  hunt_turtles
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
