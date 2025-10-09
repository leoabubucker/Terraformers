#include "imu_pubsub/imu_helper.h"
using std::placeholders::_1;

class IMUSubscriber : public rclcpp::Node
{
public:
  IMUSubscriber()
      : Node("imu_subscriber")
  {
    accelerationSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>(
        "IMUAcceleration", 10, std::bind(&IMUSubscriber::acceleration_callback, this, _1));
    angleSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>(
        "IMUAngle", 10, std::bind(&IMUSubscriber::angle_callback, this, _1));
    gyroSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>(
        "IMUGyro", 10, std::bind(&IMUSubscriber::gyro_callback, this, _1));
    magnetSubscription_ = this->create_subscription<imu_msgs::msg::IMUData>(
        "IMUMagnet", 10, std::bind(&IMUSubscriber::magnet_callback, this, _1));
  }

private:
  void acceleration_callback(const imu_msgs::msg::IMUData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Publishing to topic [IMUAcceleration]:\nHeader Timestamp: %s\nFrame ID: %s\nx: %f\ny: %f\nz: %f", IMUHelper::format_timestamp(msg->header.stamp).c_str(),
                msg->header.frame_id.c_str(), msg->x, msg->y, msg->z);
    ss << "[IMUAcceleration]:\n"
       << "Header Timestamp: " << IMUHelper::format_timestamp(msg->header.stamp) << "\n"
       << "Frame ID: " << msg->header.frame_id << "\n"
       << "Time (UTC): " << msg->utc_time << "\n"
       << "X: " << std::fixed << std::setprecision(4) << msg->x << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n";
  }
  void
  angle_callback(const imu_msgs::msg::IMUData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Publishing to topic [IMUAngle]:\nHeader Timestamp: %s\nFrame ID: %s\nx: %f\ny: %f\nz: %f", IMUHelper::format_timestamp(msg->header.stamp).c_str(),
                msg->header.frame_id.c_str(), msg->x, msg->y, msg->z);
    ss << "[IMUAngle]:\n"
       << "Header Timestamp: " << IMUHelper::format_timestamp(msg->header.stamp) << "\n"
       << "Frame ID: " << msg->header.frame_id << "\n"
       << "Time (UTC): " << msg->utc_time << "\n"
       << "X: " << std::fixed << std::setprecision(4) << msg->x << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n";
  }

  void gyro_callback(const imu_msgs::msg::IMUData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Publishing to topic [IMUGyro]:\nHeader Timestamp: %s\nFrame ID: %s\nx: %f\ny: %f\nz: %f", IMUHelper::format_timestamp(msg->header.stamp).c_str(),
                msg->header.frame_id.c_str(), msg->x, msg->y, msg->z);
    ss << "[IMUGyro]:\n"
       << "Header Timestamp: " << IMUHelper::format_timestamp(msg->header.stamp) << "\n"
       << "Frame ID: " << msg->header.frame_id << "\n"
       << "Time (UTC): " << msg->utc_time << "\n"
       << "X: " << std::fixed << std::setprecision(4) << msg->x << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n";
  }

  void magnet_callback(const imu_msgs::msg::IMUData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Publishing to topic [IMUMagnet]:\nHeader Timestamp: %s\nFrame ID: %s\nx: %f\ny: %f\nz: %f", IMUHelper::format_timestamp(msg->header.stamp).c_str(),
                msg->header.frame_id.c_str(), msg->x, msg->y, msg->z);
    ss << "[IMUMagnet]:\n"
       << "Header Timestamp: " << IMUHelper::format_timestamp(msg->header.stamp) << "\n"
       << "Frame ID: " << msg->header.frame_id << "\n"
       << "Time (UTC): " << msg->utc_time << "\n"
       << "X: " << std::fixed << std::setprecision(4) << msg->x << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n"
       << "Y: " << std::fixed << std::setprecision(4) << msg->y << "\n";
    saveOutput();
  }
  void saveOutput()
  {
    std::ofstream OutputFile("imu_output.txt", std::ios::app);

    OutputFile << ss.str();

    OutputFile.close();

    ss.str("");
    ss.clear();
  }
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr accelerationSubscription_;
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr angleSubscription_;
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr gyroSubscription_;
  rclcpp::Subscription<imu_msgs::msg::IMUData>::SharedPtr magnetSubscription_;
  std::stringstream ss;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUSubscriber>());
  rclcpp::shutdown();
  return 0;
}