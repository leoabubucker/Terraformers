#include "gps_pubsub/gps.h"

std::string GPS::utc_time = "DEFAULT VALUE";
double GPS::latitude, GPS::longitude = -1.0;
uint8_t GPS::fix_quality, GPS::num_satellites = -1;
float GPS::hdop, GPS::altitude, GPS::geoid_separation = -1.0;

int GPS::open_serial_port(const char *portname)
{

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC); // gives read and write access
    if (fd < 0)
    {
        std::cerr << "Error opening " << portname << ": "
                  << strerror(errno) << std::endl;
        return -1;
    }
    else
    {
        printf("connection successful");
    };
    return fd;
}

int GPS::write_to_port(int fd, const char *buffer, size_t size)
{
    return write(fd, buffer, size);
}

int GPS::read_from_port(int fd, char *buffer, size_t size)
{

    return read(fd, buffer, size);
}

bool GPS::configureSerialPort(int fd, int baud_rate)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error from tcgetattr: " << strerror(errno)
                  << std::endl;
        return false;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no
                                                // canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 1;                         // read doesn't block
    tty.c_cc[VTIME] = 10;                       // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error from tcsetattr: " << strerror(errno)
                  << std::endl;
        return false;
    }
    return true;
}

// int notmain()
// {
//     const char *portname = "/dev/ttyUSB0";
//     int fd = open_serial_port(portname);

//     if (!configureSerialPort(fd, B115200))
//     {
//         printf("close serial port!\n");
//         return 1;
//     }
//     char buffer[100];
//     const char *msg = "CONFIG\r\n";
//     int check_port = write_to_port(fd, msg, strlen(msg));
//     const char *msg_2 = "CONFIG ANTENNA POWERON\r\n";
//     int check_port_2 = write_to_port(fd, msg_2, strlen(msg_2));

//     // GPGSV - satellites in view

//     while (1)
//     {

//         const char *msg_1 = "GPGGA\r\n";
//         int check_port_1 = write_to_port(fd, msg_1, strlen(msg_1));
//         char buffer_new[1000];
//         int n = read_from_port(fd, buffer_new, sizeof(buffer_new));
//         char buffer_new_1[1000];
//         int n_1 = read_from_port(fd, buffer_new_1, sizeof(buffer_new_1));

//         if (n < 0)
//         {
//             std::cerr << "Error reading from serial port: "
//                       << strerror(errno) << std::endl;
//         }
//         if (n == 0)
//         {
//             std::cout << "no output found\n";
//         }
//         else
//         {
//             std::string s1 = std::string(buffer_new, n);
//             std::cout << s1 << std::endl; // LOGS DATA TO CONSOLE

//             sleep(1);
//         }
//     }
//     return 0;
// }

int GPS::initGPS()
{
    // return -1; // TESTING ONLY - COMMENT TO GET REAL DATA
    const char *portname = "/dev/ttyUSB0";
    int fd = GPS::open_serial_port(portname);

    if (!GPS::configureSerialPort(fd, B115200))
    {
        printf("close serial port!\n");
        return -1;
    }
    char buffer[100];
    const char *msg = "CONFIG\r\n";
    int check_port = write_to_port(fd, msg, strlen(msg));
    const char *msg_2 = "CONFIG ANTENNA POWERON\r\n";
    int check_port_2 = write_to_port(fd, msg_2, strlen(msg_2));
    return fd;
}

void GPS::loopGPS(int fd)
{

    // GPS::utc_time = "004952.00";
    // GPS::latitude = 38.99152415;
    // GPS::longitude = -76.9375376;
    // GPS::fix_quality = 1;
    // GPS::num_satellites = 19;
    // GPS::hdop = 0.8;
    // GPS::altitude = 26.6381;
    // GPS::geoid_separation = -33.0521;
    // return;
    // TESTING ONLY ABOVE THIS LINE - COMMENT TO GET REAL DATA
    const char *msg_1 = "GPGGA\r\n";
    int check_port_1 = GPS::write_to_port(fd, msg_1, strlen(msg_1));
    char buffer_new[1000];
    int n = GPS::read_from_port(fd, buffer_new, sizeof(buffer_new));
    char buffer_new_1[1000];
    int n_1 = GPS::read_from_port(fd, buffer_new_1, sizeof(buffer_new_1));

    if (n < 0)
    {
        std::cerr << "Error reading from serial port: "
                  << strerror(errno) << std::endl;
    }
    if (n == 0)
    {
        std::cout << "no output found\n";
    }
    else
    {
        std::string str = std::string(buffer_new, n);
        std::cout << str << std::endl; // LOGS DATA TO CONSOLE
        // str = "$GNGGA,002427.00,3859.52869522,N,07656.24204695,W,1,09,4.0,22.8332,M,-33.0525,M,,*71";

        if (str.substr(0, 6) == "$GNGGA")
        {
            if (true) //checksum check - not working rn
            {
                size_t asteriskPos = str.find('*');
                if (asteriskPos != std::string::npos)
                {
                    str = str.substr(0, asteriskPos);
                }

                std::vector<std::string> splitOutput;
                std::stringstream ss(str);
                std::string token;

                std::string expectedChecksum = "";
                char directionLatitude = 'N';
                char directionLongitude = 'W';
                if (str.find("S") != std::string::npos)
                {
                    directionLatitude = 'S';
                }

                if (str.find("E") != std::string::npos)
                {
                    directionLongitude = 'E';
                }
                // Split on ','
                while (std::getline(ss, token, ','))
                {
                    splitOutput.push_back(token);
                }

                // Filter out unwanted tokens
                std::unordered_set<std::string> valsToRemove = {"N", "E", "S", "W", "M", "$GNGGA", ""};
                splitOutput.erase(
                    std::remove_if(splitOutput.begin(), splitOutput.end(),
                                   [&valsToRemove](const std::string &val)
                                   {
                                       return valsToRemove.count(val) > 0;
                                   }),
                    splitOutput.end());

                // Print the result
                // for (const auto& val : splitOutput) {
                //     std::cout << val << " ";
                // }
                // std::cout << std::endl;
                GPS::utc_time = splitOutput[0];
                GPS::latitude = convertNMEACoordinate(splitOutput[1], directionLatitude);
                GPS::longitude = convertNMEACoordinate(splitOutput[2], directionLongitude);
GPS::fix_quality = static_cast<uint8_t>(std::stoi(splitOutput[3]));
GPS::num_satellites = static_cast<uint8_t>(std::stoi(splitOutput[4]));
                GPS::hdop = std::stof(splitOutput[5]);
                GPS::altitude = std::stof(splitOutput[6]);
                GPS::geoid_separation = std::stof(splitOutput[7]);
            }
        }
    }
    // if (s1.substr(0, 6) == "$GNGGA")
    // {
    //     if (GPS::validateChecksum(s1))
    //     {
    //         std::string delimeter = ",";
    //         std::vector<std::string> splitOutput;
    //         std::string expectedChecksum = "";
    //         char directionLatitude = 'N';
    //         char directionLongitude = 'W';
    //         if (s1.find("S") != std::string::npos)
    //         {
    //             directionLatitude = 'S';
    //         }

    //         if (s1.find("E") != std::string::npos)
    //         {
    //             directionLongitude = 'E';
    //         }
    //         while (s1.find(delimeter) != std::string::npos)
    //         {
    //             s1 = s1.substr(s1.find(delimeter));
    //             if (s1.find(delimeter) != std::string::npos)
    //             {
    //                 splitOutput.push_back(s1.substr(0, s1.find(delimeter)));
    //             }
    //         }
    //         std::unordered_set<std::string> valsToRemove = {"N", "E", "S", "W", "M", "$GNGGA"};
    //         splitOutput.erase(
    //             std::remove_if(splitOutput.begin(), splitOutput.end(),
    //                            [&valsToRemove](const std::string &fruit)
    //                            {
    //                                return valsToRemove.count(fruit) > 0;
    //                            }),
    //             splitOutput.end());
    //    std::cout << s1 << std::endl; // LOGS DATA TO CONSOLE

    // System.out.println
    sleep(1);
}

bool GPS::validateChecksum(std::string str)
{
    // Check if the sentence starts with '$'
    size_t start = str.find('$');
    if (start == std::string::npos)
    {
        return false;
    }
    std::string expectedChecksum = "";
    if (str.find('*') != std::string::npos)
    {
        expectedChecksum = str.substr(str.find('*') + 1);
    }
    else
    {
        return false;
    }
    if (expectedChecksum == "")
    {
        return false;
    }

    // Stop at '*' if it exists
    size_t end = str.find('*');
    if (end == std::string::npos)
    {
        end = str.length(); // use full length if '*' is not found
    }

    // XOR all characters between $ and *
    unsigned char checksum = 0;
    for (size_t i = start + 1; i < end; ++i)
    {
        checksum ^= static_cast<unsigned char>(str[i]);
    }

    // Convert to 2-digit uppercase hex
    std::stringstream ss;
    ss << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<int>(checksum);

    return ss.str() == expectedChecksum;
}

double GPS::convertNMEACoordinate(const std::string &coord_str, char direction)
{
    if (coord_str.empty())
        return 0.0;

    // Parse degrees and minutes:
    // Latitude format: ddmm.mmmmm (2 digits degrees)
    // Longitude format: dddmm.mmmmm (3 digits degrees)
    int degrees_len = (direction == 'N' || direction == 'S') ? 2 : 3;

    // Extract degrees part (first 2 or 3 digits)
    double degrees = std::stod(coord_str.substr(0, degrees_len));
    // Extract minutes part (rest)
    double minutes = std::stod(coord_str.substr(degrees_len));

    // Convert to decimal degrees
    double decimal_degrees = degrees + (minutes / 60.0);

    // Apply negative sign for South and West
    if (direction == 'S' || direction == 'W')
    {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

std::string GPS::format_timestamp(const builtin_interfaces::msg::Time &stamp)
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