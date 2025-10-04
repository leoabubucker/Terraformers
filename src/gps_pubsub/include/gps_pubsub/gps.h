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

class GPS
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
    static int open_serial_port(const char *portname);
    static int write_to_port(int fd, const char *buffer, size_t size);
    static int read_from_port(int fd, char *buffer, size_t size);
    static bool configureSerialPort(int fd, int baud_rate);
    static bool validateChecksum(std::string str);
    static double convertNMEACoordinate(const std::string &coord_str, char direction);
};