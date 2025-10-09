#ifndef SRC_GPS_PUBSUB_INCLUDE_GPS_PUBSUB_GPS_HELPER_H_
#define SRC_GPS_PUBSUB_INCLUDE_GPS_PUBSUB_GPS_HELPER_H_

#include <stdio.h>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <unordered_set>
#include<bits/stdc++.h>
#include "builtin_interfaces/msg/time.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_data.hpp"
#include <memory>
class GPSHelper
{
public:
    static std::string utc_time;
    static double latitude;
    static double longitude;
    static uint8_t fix_quality;
    static uint8_t num_satellites;
    static float hdop;
    static float altitude;
    static float geoid_separation;
    static int initGPS();
    static void loopGPS(int fd);
    static std::string format_timestamp(const builtin_interfaces::msg::Time &stamp);



private:
    static int open_serial_port(const char *portname, int attempt);
    static int write_to_port(int fd, const char *buffer, size_t size);
    static int read_from_port(int fd, char *buffer, size_t size);
    static bool configureSerialPort(int fd, int baud_rate, int attempt);
    static bool validateChecksum(std::string str);
    static double convertNMEACoordinate(const std::string &coord_str, char direction);
    static void saveOutputRaw();
    static std::stringstream ss2;
    static int calculateChecksum(std::string str);
    static void handle_sigint(int);
    static std::atomic<bool> stopFlag;

};

#endif // SRC_GPS_PUBSUB_INCLUDE_GPS_PUBSUB_GPS_HELPER_H_
