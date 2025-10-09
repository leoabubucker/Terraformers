#include "gps_pubsub/gps_helper.h"
using namespace std::chrono_literals;

class GPSPublisher : public rclcpp::Node
{
public:
  GPSPublisher()
      : Node("gps_publisher"), count_(0)
  {
    gpsPublisher_ = this->create_publisher<gps_msgs::msg::GPSData>("GPSData", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&GPSPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = gps_msgs::msg::GPSData();
    message.header.stamp = this->now();
    message.header.frame_id = "gps_link";
    // message.utc_time = "004952.00";
    // message.latitude = 38.99152415;
    // message.longitude = -76.9375376;
    // message.fix_quality = 1;
    // message.num_satellites = 19;
    // message.hdop = 0.8;
    // message.altitude = 26.6381;
    // message.geoid_separation = -33.0521;
    message.utc_time = GPSHelper::utc_time;
    message.latitude = GPSHelper::latitude;
    message.longitude = GPSHelper::longitude;
    message.fix_quality = GPSHelper::fix_quality;
    message.num_satellites = GPSHelper::num_satellites;
    message.hdop = GPSHelper::hdop;
    message.altitude = GPSHelper::altitude;
    message.geoid_separation = GPSHelper::geoid_separation;
    RCLCPP_INFO(this->get_logger(), "Publishing to topic GPSData:\nHeader Timestamp: %s\nFrame ID: %s\nTime (UTC): %s\nLatitude: %.8f\nLongitude: %.8f\nFix Quality: %u\nSatellites: %u\nHDOP: %.2f\nAltitude: %.4f m\nGeoid Separation: %.4f m",
                GPSHelper::format_timestamp(message.header.stamp).c_str(),
                message.header.frame_id.c_str(),
                message.utc_time.c_str(),
                message.latitude,
                message.longitude,
                message.fix_quality,
                message.num_satellites,
                message.hdop,
                message.altitude,
                message.geoid_separation);
    gpsPublisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<gps_msgs::msg::GPSData>::SharedPtr gpsPublisher_;
  size_t count_;
  std::string format_timestamp(const builtin_interfaces::msg::Time &stamp)
  {
    // Combine seconds and nanoseconds into std::chrono::time_point
    auto total_ns = std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
    auto time_point = std::chrono::time_point<std::chrono::system_clock>(total_ns);

    // Convert to time_t for std::put_time
    std::time_t time_sec = std::chrono::system_clock::to_time_t(time_point);
    std::tm tm = *std::gmtime(&time_sec); // or std::localtime if you want local time

    // Get fractional seconds
    auto fractional_ns = stamp.nanosec % 1000000000;

    // Format to string
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setw(3) << std::setfill('0') << fractional_ns / 1000000; // milliseconds

    return oss.str();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSPublisher>();
  int fd = GPSHelper::initGPS();
  std::thread gps_thread([&node, fd]()
                         {
    while (rclcpp::ok()) {
      GPSHelper::loopGPS(fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } });
  rclcpp::spin(node);
  gps_thread.join();
  rclcpp::shutdown();
  return 0;
}