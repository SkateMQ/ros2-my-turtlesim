cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/LedPanelState.msg"
  "msg/NewTurtleInfo.msg"
  "srv/ComputeRectangleArea.srv"
  "srv/SetLedPanelState.srv"
  "srv/DeleteDeadTurtle.srv"
  "srv/FindNearestTurtle.srv"
  "srv/EatTurtle.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
