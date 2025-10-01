cmake_minimum_required(VERSION 3.5)
project(imu_publisher_2)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp src/sensor.cpp src/serial.c src/wit_c_sdk.c)
ament_target_dependencies(talker rclcpp tutorial_interfaces std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces std_msgs)


# add_executable(imu_publisher_node
#   src/main.cpp
#   src/publisher_member_function.cpp
#   src/sensor.cpp
#   # add other cpp files if needed
# )

# ament_target_dependencies(imu_publisher_node
#   rclcpp
#   tutorial_interfaces
# )

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_package()