#include "imu_pubsub/imu_helper.h"
using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher()
      : Node("imu_publisher"), count_(0)
  {
    accelerationPublisher_ = this->create_publisher<imu_msgs::msg::IMUData>("IMUAcceleration", 10);
    anglePublisher_ = this->create_publisher<imu_msgs::msg::IMUData>("IMUAngle", 10);
    gyroPublisher_ = this->create_publisher<imu_msgs::msg::IMUData>("IMUGyro", 10);
    magnetPublisher_ = this->create_publisher<imu_msgs::msg::IMUData>("IMUMagnet", 10);
    publishers = {
        {"IMUAcceleration", accelerationPublisher_},
        {"IMUAngle", anglePublisher_},
        {"IMUGyro", gyroPublisher_},
        {"IMUMagnet", magnetPublisher_}};
    timer_ = this->create_wall_timer(500ms, std::bind(&IMUPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string topic = keys[currentKey];
    dataMatrix[0] = {IMUHelper::acc_x, IMUHelper::acc_y, IMUHelper::acc_z};
    dataMatrix[1] = {IMUHelper::angle_x, IMUHelper::angle_y, IMUHelper::angle_z};
    dataMatrix[2] = {IMUHelper::gyro_x, IMUHelper::gyro_y, IMUHelper::gyro_z};
    dataMatrix[3] = {IMUHelper::h_x, IMUHelper::h_y, IMUHelper::h_z};

    auto message = imu_msgs::msg::IMUData();

    message.header.stamp = this->now();
    message.header.frame_id = "imu_link";
    message.x = dataMatrix[currentKey][0];
    message.y = dataMatrix[currentKey][1];
    message.z = dataMatrix[currentKey][2];
    RCLCPP_INFO(this->get_logger(), "Publishing to topic [%s]:\nHeader Timestamp: %s\nFrame ID: %s\nx: %f\ny: %f\nz: %f", topic.c_str(), IMUHelper::formatTimestamp(message.header.stamp).c_str(),
                message.header.frame_id.c_str(), message.x, message.y, message.z);
    publishers.at(topic)->publish(message);
    currentKey = (currentKey + 1) % 4;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<imu_msgs::msg::IMUData>::SharedPtr accelerationPublisher_;
  rclcpp::Publisher<imu_msgs::msg::IMUData>::SharedPtr anglePublisher_;
  rclcpp::Publisher<imu_msgs::msg::IMUData>::SharedPtr gyroPublisher_;
  rclcpp::Publisher<imu_msgs::msg::IMUData>::SharedPtr magnetPublisher_;
  std::map<std::string, rclcpp::Publisher<imu_msgs::msg::IMUData>::SharedPtr> publishers;
  int currentKey = 0;
  std::string keys[4] = {"IMUAcceleration", "IMUAngle", "IMUGyro", "IMUMagnet"};
  std::array<std::array<float, 3>, 4> dataMatrix;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUPublisher>();
  int fd = IMUHelper::initSensor();
  std::thread sensor_thread([&node, fd]()
                            {
    while (rclcpp::ok()) {
      IMUHelper::loopIMU(fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } });
  rclcpp::spin(node);
  sensor_thread.join();
  serial_close(fd);
  rclcpp::shutdown();
  return 0;
}