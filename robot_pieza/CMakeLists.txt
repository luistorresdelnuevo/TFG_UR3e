cmake_minimum_required(VERSION 3.5)
project(robot_pieza)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(robot_pieza src/robot1.cpp src/robot2.cpp)

#Include directories.
target_include_directories(robot_pieza PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_pieza PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  


ament_target_dependencies(
  robot_pieza
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
  "moveit_msgs"
)

install(TARGETS robot_pieza
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
