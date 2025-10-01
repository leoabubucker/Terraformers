#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/imu_data.hpp"     // IGNORE?
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    accelerationSubscription_ = this->create_subscription<tutorial_interfaces::msg::IMUData>(          // CHANGE
      "IMUAcceleration", 10, std::bind(&MinimalSubscriber::acceleration_callback, this, _1));
    angleSubscription_ = this->create_subscription<tutorial_interfaces::msg::IMUData>(          // CHANGE
      "IMUAngle", 10, std::bind(&MinimalSubscriber::angle_callback, this, _1));
    gyroSubscription_ = this->create_subscription<tutorial_interfaces::msg::IMUData>(          // CHANGE
      "IMUGyro", 10, std::bind(&MinimalSubscriber::gyro_callback, this, _1));
    magnetSubscription_ = this->create_subscription<tutorial_interfaces::msg::IMUData>(          // CHANGE
      "IMUMagnet", 10, std::bind(&MinimalSubscriber::magnet_callback, this, _1));
  }

private:
  void acceleration_callback(const tutorial_interfaces::msg::IMUData::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[IMUAcceleration] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void angle_callback(const tutorial_interfaces::msg::IMUData::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[IMUAngle] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void gyro_callback(const tutorial_interfaces::msg::IMUData::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[IMUGyro] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void magnet_callback(const tutorial_interfaces::msg::IMUData::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[IMUMagnet] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::IMUData>::SharedPtr accelerationSubscription_;       // CHANGE
  rclcpp::Subscription<tutorial_interfaces::msg::IMUData>::SharedPtr angleSubscription_;       // CHANGE
  rclcpp::Subscription<tutorial_interfaces::msg::IMUData>::SharedPtr gyroSubscription_;       // CHANGE
  rclcpp::Subscription<tutorial_interfaces::msg::IMUData>::SharedPtr magnetSubscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}