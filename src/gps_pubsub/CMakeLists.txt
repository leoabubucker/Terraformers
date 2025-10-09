cmake_minimum_required(VERSION 3.8)
project(gps_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS Default Packages for C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Required Package for Custom Msgs
find_package(std_msgs REQUIRED)

# Required Package for String Msgs
find_package(gps_msgs REQUIRED)

add_executable(gps_publisher src/gps_publisher.cpp src/gps_helper.cpp)
ament_target_dependencies(gps_publisher rclcpp std_msgs gps_msgs)

add_executable(gps_subscriber src/gps_subscriber.cpp src/gps_helper.cpp)
ament_target_dependencies(gps_subscriber rclcpp std_msgs gps_msgs)

install(TARGETS
  gps_publisher
  gps_subscriber
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(gps_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(gps_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
