cmake_minimum_required(VERSION 3.5)
project(imu_pubsub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED) # for default string msgs
find_package(imu_msgs REQUIRED) # for custom msgs

add_executable(imu_publisher src/IMUPublisher.cpp src/sensor.cpp src/serial.c src/wit_c_sdk.c)
ament_target_dependencies(imu_publisher rclcpp imu_msgs std_msgs)

add_executable(imu_subscriber src/IMUSubscriber.cpp)
ament_target_dependencies(imu_subscriber rclcpp imu_msgs std_msgs)

install(TARGETS
  imu_publisher
  imu_subscriber
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(imu_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(imu_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_package()