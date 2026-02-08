cmake_minimum_required(VERSION 3.8)
project(kinematic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_description REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joystick REQUIRED)
find_package(joy REQUIRED)

add_executable(kinematic src/kinematic.cpp)

ament_target_dependencies(kinematic 
  rclcpp 
  std_msgs
  sensor_msgs 
  geometry_msgs 
  robot_description 
  tf2 
  tf2_geometry_msgs
  trajectory_msgs
  controller_manager
  joint_state_broadcaster
  joystick
  joy
  )

install(TARGETS
  kinematic
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install (DIRECTORY config
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
