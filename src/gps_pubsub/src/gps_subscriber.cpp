#include "gps_pubsub/gps_helper.h"


using std::placeholders::_1;

class GPSSubscriber : public rclcpp::Node
{
public:
  GPSSubscriber()
      : Node("gps_subscriber")
  {
    GPSSubscription_ = this->create_subscription<gps_msgs::msg::GPSData>( // CHANGE
        "GPSData", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const gps_msgs::msg::GPSData::SharedPtr message)
  {
    RCLCPP_INFO(this->get_logger(), "Listening to topic [GPSData]:\nHeader Timestamp: %s\nFrame ID: %s\nTime (UTC): %s\nLatitude: %.8f\nLongitude: %.8f\nFix Quality: %u\nSatellites: %u\nHDOP: %.2f\nAltitude: %.4f m\nGeoid Separation: %.4f m",
                GPSHelper::format_timestamp(message->header.stamp).c_str(),
                message->header.frame_id.c_str(),
                message->utc_time.c_str(),
                message->latitude,
                message->longitude,
                message->fix_quality,
                message->num_satellites,
                message->hdop,
                message->altitude,
                message->geoid_separation);
    ss << "[GPSData]:\n"
       << "Header Timestamp: " << GPSHelper::format_timestamp(message->header.stamp) << "\n"
       << "Frame ID: " << message->header.frame_id << "\n"
       << "Time (UTC): " << message->utc_time << "\n"
       << "Latitude: " << std::fixed << std::setprecision(8) << message->latitude << "\n"
       << "Longitude: " << std::fixed << std::setprecision(8) << message->longitude << "\n"
       << "Fix Quality: " << static_cast<unsigned int>(message->fix_quality) << "\n"
       << "Satellites: " << static_cast<unsigned int>(message->num_satellites) << "\n"
       << "HDOP: " << std::fixed << std::setprecision(2) << message->hdop << "\n"
       << "Altitude: " << std::fixed << std::setprecision(4) << message->altitude << " m\n"
       << "Geoid Separation: " << std::fixed << std::setprecision(4) << message->geoid_separation << " m\n";
       saveOutput();
  }

  void saveOutput()
  {
    std::ofstream OutputFile("gps_output.txt", std::ios::app);

    OutputFile << ss.str();

    OutputFile.close();

    ss.str("");
    ss.clear();
  }
  rclcpp::Subscription<gps_msgs::msg::GPSData>::SharedPtr GPSSubscription_;
  std::stringstream ss;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSSubscriber>());
  rclcpp::shutdown();
  return 0;
}