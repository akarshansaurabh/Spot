cmake_minimum_required(VERSION 3.8)
project(motion_planner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

add_executable(EnvironmentSetup 
src/environment/EnvironmentSetup.cpp 
src/environment/Environment_1.cpp)

add_executable(SpotServer 
src/main/Test_Server.cpp 
src/kinematics/kinematics.cpp 
src/kinematics/arm_kinematics.cpp 
src/maths/commonmathssolver.cpp
src/maths/curves.cpp
src/com/com_solver.cpp
src/rviz2/markers.cpp
src/motion_planner/arm_motion_planner.cpp)

add_executable(SpotClient 
src/main/Test_Client.cpp)

ament_target_dependencies(EnvironmentSetup
  rclcpp
  std_msgs
  kdl_parser
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  custom_interfaces
  visualization_msgs
)

ament_target_dependencies(SpotServer
  rclcpp
  std_msgs
  kdl_parser
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  custom_interfaces
  visualization_msgs
)

ament_target_dependencies(SpotClient
  rclcpp
  std_msgs
  kdl_parser
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  custom_interfaces
  visualization_msgs
)

install(TARGETS
  EnvironmentSetup
  SpotServer
  SpotClient
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Final package setup
ament_package()
