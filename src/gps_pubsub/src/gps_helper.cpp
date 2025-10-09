#include "gps_pubsub/gps_helper.h"

std::atomic<bool> GPSHelper::stopFlag{false};

std::string GPSHelper::utc_time = "DEFAULT VALUE";
double GPSHelper::latitude, GPSHelper::longitude = -1.0;
uint8_t GPSHelper::fix_quality, GPSHelper::num_satellites = -1;
float GPSHelper::hdop, GPSHelper::altitude, GPSHelper::geoid_separation = -1.0;
std::stringstream GPSHelper::ss2;
int GPSHelper::open_serial_port(const char *portname, int attempt)
{

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC); // gives read and write access
    if (fd < 0)
    {
        std::cerr << "Error opening " << portname << ": "
                  << strerror(errno) << ". Retrying in 1 Second (Atttempt " << attempt << ")" << std::endl;
        return -1;
    }
    else
    {
        printf("connection successful");
    };
    return fd;
}

int GPSHelper::write_to_port(int fd, const char *buffer, size_t size)
{
    return write(fd, buffer, size);
}

int GPSHelper::read_from_port(int fd, char *buffer, size_t size)
{

    return read(fd, buffer, size);
}

bool GPSHelper::configureSerialPort(int fd, int baud_rate, int attempt)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error from tcgetattr: " << strerror(errno)
                  << ". Retrying in 1 Second (Atttempt " << attempt << ")" << std::endl;
        return false;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no
    // canonical processing
    tty.c_oflag = 0;      // no remapping, no delays
    tty.c_cc[VMIN] = 1;   // read doesn't block
    tty.c_cc[VTIME] = 10; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
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

int GPSHelper::initGPS()
{
    // return -1; // TESTING ONLY - COMMENT TO GET REAL DATA
    signal(SIGINT, GPSHelper::handle_sigint);
    const char *portname = "/dev/ttyUSB0";
    int fd = -1;
    int openAttempt = 0;
    do
    {
        if (stopFlag)
        {
            std::cerr << "Ctrl+C detected. Exiting...\n";
            break;
        }
        openAttempt++;
        fd = GPSHelper::open_serial_port(portname, openAttempt);
        sleep(1);
    } while (fd == -1);

    bool configurePort = false;
    int readAttempt = 0;
    do
    {
        if (stopFlag)
        {
            std::cerr << "Ctrl+C detected. Exiting...\n";
            break;
        }
        readAttempt++;
        configurePort = GPSHelper::configureSerialPort(fd, B115200, readAttempt);
        sleep(1);
    } while (!configurePort);

    char buffer[100];
    const char *msg = "CONFIG\r\n";
    int check_port = write_to_port(fd, msg, strlen(msg));
    const char *msg_2 = "CONFIG ANTENNA POWERON\r\n";
    int check_port_2 = write_to_port(fd, msg_2, strlen(msg_2));
    return fd;
}

void GPSHelper::loopGPS(int fd)
{

    // GPSHelper::utc_time = "004952.00";
    // GPSHelper::latitude = 38.99152415;
    // GPSHelper::longitude = -76.9375376;
    // GPSHelper::fix_quality = 1;
    // GPSHelper::num_satellites = 19;
    // GPSHelper::hdop = 0.8;
    // GPSHelper::altitude = 26.6381;
    // GPSHelper::geoid_separation = -33.0521;
    // return;
    // TESTING ONLY ABOVE THIS LINE - COMMENT TO GET REAL DATA
    const char *msg_1 = "GPGGA\r\n";
    int check_port_1 = GPSHelper::write_to_port(fd, msg_1, strlen(msg_1));
    char buffer_new[1000];
    int n = GPSHelper::read_from_port(fd, buffer_new, sizeof(buffer_new));
    char buffer_new_1[1000];
    int n_1 = GPSHelper::read_from_port(fd, buffer_new_1, sizeof(buffer_new_1));

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

        if (str.substr(0, 6) == "$GNGGA")
        {
            GPSHelper::ss2 << str << std::endl;
            saveOutputRaw();
            if (validateChecksum(str))
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

                GPSHelper::utc_time = splitOutput[0];
                GPSHelper::latitude = convertNMEACoordinate(splitOutput[1], directionLatitude);
                GPSHelper::longitude = convertNMEACoordinate(splitOutput[2], directionLongitude);
                GPSHelper::fix_quality = static_cast<uint8_t>(std::stoi(splitOutput[3]));
                GPSHelper::num_satellites = static_cast<uint8_t>(std::stoi(splitOutput[4]));
                GPSHelper::hdop = std::stof(splitOutput[5]);
                GPSHelper::altitude = std::stof(splitOutput[6]);
                GPSHelper::geoid_separation = std::stof(splitOutput[7]);
            }
        }
    }
    sleep(1);
}

bool GPSHelper::validateChecksum(std::string str)
{
    std::string originalStr = str;
    std::string expectedChecksum = str.substr(str.find('*') + 1);
    if (str[0] == '$' && str.find('*') != std::string::npos)
    {
        str = str.substr(1);
        str = str.substr(0, str.find('*'));
        std::cout << str << std::endl
                  << expectedChecksum << std::endl
                  << GPSHelper::calculateChecksum(str) << std::endl;
        if (calculateChecksum(str) == std::atoi(expectedChecksum.c_str()))
        {
            return true;
        }
    }
    std::cerr << "ERROR: INVALID CHECKSUM, Expected: " << expectedChecksum << " Actual: " << calculateChecksum(str) << " GPSHelper Output Ignored: " << originalStr << std::endl;
    return false;
}

double GPSHelper::convertNMEACoordinate(const std::string &coord_str, char direction)
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

std::string GPSHelper::format_timestamp(const builtin_interfaces::msg::Time &stamp)
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

void GPSHelper::saveOutputRaw()
{
    std::ofstream OutputFile("gps_output_raw.txt", std::ios::app);

    OutputFile << GPSHelper::ss2.str();

    OutputFile.close();

    GPSHelper::ss2.str("");
    GPSHelper::ss2.clear();
}
int GPSHelper::calculateChecksum(std::string str)
{
    unsigned char checksum = 0;

    for (char ch : str)
    {
        checksum ^= static_cast<unsigned char>(ch);
    }

    // Convert to uppercase 2-digit hex string
    std::stringstream ss;
    ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum);
    return std::atoi(ss.str().c_str());
}

void GPSHelper::handle_sigint(int)
{
    stopFlag = true;
}