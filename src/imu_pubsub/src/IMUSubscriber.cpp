#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "imu_msgs/msg/imu_data.hpp" // Error on this line may be due to incorrect VSCode config and NOT an actual error. If building and running works, ignore the error.
using std::placeholders::_1;

class IMUSubscriber : public rclcpp::Node
{
public:
  IMUSubscriber()
      : Node("imu_subscriber")
  {
    accelerationSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>( // CHANGE
        "IMUAcceleration", 10, std::bind(&IMUSubscriber::acceleration_callback, this, _1));
    angleSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>( // CHANGE
        "IMUAngle", 10, std::bind(&IMUSubscriber::angle_callback, this, _1));
    gyroSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>( // CHANGE
        "IMUGyro", 10, std::bind(&IMUSubscriber::gyro_callback, this, _1));
    magnetSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>( // CHANGE
        "IMUMagnet", 10, std::bind(&IMUSubscriber::magnet_callback, this, _1));
  }

private:
  void acceleration_callback(const imu_msgs::msg::IMUData::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[IMUAcceleration] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void angle_callback(const imu_msgs::msg::IMUData::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[IMUAngle] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void gyro_callback(const imu_msgs::msg::IMUData::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[IMUGyro] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }

  void magnet_callback(const imu_msgs::msg::IMUData::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[IMUMagnet] x: %f y: %f z: %f", msg->x, msg->y, msg->z);
  }
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr accelerationSubscription_; // CHANGE
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr angleSubscription_;        // CHANGE
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr gyroSubscription_;         // CHANGE
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr magnetSubscription_;       // CHANGE
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUSubscriber>());
  rclcpp::shutdown();
  return 0;
}