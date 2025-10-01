#include <string>
#include <map>
#include <chrono>
#include <memory>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/imu_data.hpp"     // CHANGE
#include "imu_publisher_2/sensor.h"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    accelerationPublisher_ = this->create_publisher<tutorial_interfaces::msg::IMUData>("IMUAcceleration", 10);    // CHANGE
    anglePublisher_ = this->create_publisher<tutorial_interfaces::msg::IMUData>("IMUAngle", 10);    // CHANGE
    gyroPublisher_ = this->create_publisher<tutorial_interfaces::msg::IMUData>("IMUGyro", 10);    // CHANGE
    magnetPublisher_ = this->create_publisher<tutorial_interfaces::msg::IMUData>("IMUMagnet", 10);    // CHANGE    
    publishers =  {
    {"IMUAcceleration", accelerationPublisher_},
    {"IMUAngle", anglePublisher_},
    {"IMUGyro", gyroPublisher_},
    {"IMUMagnet", magnetPublisher_}
  };
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string topic = keys[currentKey];
    dataMatrix[0] = {Sensor::acc_x, Sensor::acc_y, Sensor::acc_z};
    dataMatrix[1] = {Sensor::angle_x, Sensor::angle_y, Sensor::angle_z};
    dataMatrix[2] = {Sensor::gyro_x, Sensor::gyro_y, Sensor::gyro_z};
    dataMatrix[3] = {Sensor::h_x, Sensor::h_y, Sensor::h_z};
  
    auto message = tutorial_interfaces::msg::IMUData();    
    
    message.x = dataMatrix[currentKey][0];
    message.y = dataMatrix[currentKey][1];
    message.z = dataMatrix[currentKey][2];                                   // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing to topic %s: x: '%f' y: '%f' z: '%f'", topic.c_str(), message.x, message.y, message.z);    // CHANGE
    publishers.at(topic) ->publish(message);
    currentKey = (currentKey + 1) % 4;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::IMUData>::SharedPtr accelerationPublisher_;         // CHANGE
  rclcpp::Publisher<tutorial_interfaces::msg::IMUData>::SharedPtr anglePublisher_;         // CHANGE
  rclcpp::Publisher<tutorial_interfaces::msg::IMUData>::SharedPtr gyroPublisher_;         // CHANGE
  rclcpp::Publisher<tutorial_interfaces::msg::IMUData>::SharedPtr magnetPublisher_;         // CHANGE
  std::map<std::string, rclcpp::Publisher<tutorial_interfaces::msg::IMUData>::SharedPtr> publishers;
  int currentKey = 0;
  std::string keys[4] = {"IMUAcceleration", "IMUAngle", "IMUGyro", "IMUMagnet"};
std::array<std::array<float, 3>, 4> dataMatrix;
size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  std::thread sensor_thread([&node]() {
    while (rclcpp::ok()) {
      Sensor::initSensor();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  rclcpp::spin(node);
  sensor_thread.join();
  rclcpp::shutdown();
  return 0;
}