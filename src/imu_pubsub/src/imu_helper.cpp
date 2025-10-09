#include "imu_pubsub/imu_helper.h"

int IMUHelper::fd, IMUHelper::s_iCurBaud = 9600;
volatile char IMUHelper::s_cDataUpdate = 0;
const int IMUHelper::c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

float IMUHelper::acc_x, IMUHelper::acc_y, IMUHelper::acc_z, IMUHelper::angle_x, IMUHelper::angle_y, IMUHelper::angle_z, IMUHelper::gyro_x, IMUHelper::gyro_y, IMUHelper::gyro_z, IMUHelper::h_x, IMUHelper::h_y, IMUHelper::h_z = -1.0;

void IMUHelper::initSensor()
{

    /*
     *               TESTING CODE
     * 1) To publish random data (not using the sensor):
     *   a) Uncomment "IMUHelper::add_to_memory();"
     *   b) Comment the rest of this function
     * 2) To publish data from the sensor:
     *   a) Ensure sensorPath exists
     *   b) Comment "IMUHelper::add_to_memory()"
     *   c) Uncomment the rest of this function
     */
    // IMUHelper::add_to_memory();
    unsigned char *sensorPath = (unsigned char*)"/dev/ttyUSB0"; //TODO: ADD SENSOR PATH

    if((IMUHelper::fd = serial_open(sensorPath , 9600)<0))
    {
        printf("open %s fail\n", sensorPath);
        return;
    }
    else printf("open %s success\n", sensorPath);

    float fAcc[3], fGyro[3], fAngle[3];
    int i , ret;
    unsigned char cBuff[1];

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(SensorDataUpdata);

    // printf("\r\n********************** wit-motion Normal example  ************************\r\n");
    IMUHelper::AutoScanSensor(sensorPath);

        while(serial_read_data(IMUHelper::fd, cBuff, 1)){
            WitSerialDataIn(cBuff[0]);
        }

        // printf("\n");
        IMUHelper::Delayms(500);

        if(IMUHelper::s_cDataUpdate){
            // When data needs to be updated
            IMUHelper::add_to_memory();
        }
    serial_close(fd);
    return;
}

int IMUHelper::add_to_memory()
{
    /*
     *               TESTING CODE
     * 1) To publish random data (not using the sensor):
     *   a) Uncomment all lines
     * 2) To publish data from the sensor:
     *   a) Comment all lines before the first if statement that set IMUHelper::someValue
     */  
    // IMUHelper::acc_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::acc_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::acc_z = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::angle_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::angle_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::angle_z = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::gyro_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::gyro_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::gyro_z = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::h_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::h_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    // IMUHelper::h_z = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    if (IMUHelper::s_cDataUpdate)
    {

        if (IMUHelper::s_cDataUpdate & ACC_UPDATE)
        {
            IMUHelper::acc_x = sReg[AX] / 32768.0f * 16.0f;
            IMUHelper::acc_y = sReg[AX + 1] / 32768.0f * 16.0f;
            IMUHelper::acc_z = sReg[AX + 2] / 32768.0f * 16.0f;

            if (IS_VALID_FLOAT(IMUHelper::acc_x) && IS_VALID_FLOAT(IMUHelper::acc_y) && IS_VALID_FLOAT(IMUHelper::acc_z))
            {
                IMUHelper::s_cDataUpdate &= ~ACC_UPDATE;
            }
        }

        // // Update gyroscope data
        if (IMUHelper::s_cDataUpdate & GYRO_UPDATE)
        {
            IMUHelper::gyro_x = sReg[GX] / 32768.0f * 2000.0f;
            IMUHelper::gyro_y = sReg[GX + 1] / 32768.0f * 2000.0f;
            IMUHelper::gyro_z = sReg[GX + 2] / 32768.0f * 2000.0f;

            if (IS_VALID_FLOAT(IMUHelper::gyro_x) && IS_VALID_FLOAT(IMUHelper::gyro_y) && IS_VALID_FLOAT(IMUHelper::gyro_z))
            {
                IMUHelper::s_cDataUpdate &= ~GYRO_UPDATE;
            }
        }

        // // Update angle data
        if (s_cDataUpdate & ANGLE_UPDATE)
        {
            IMUHelper::angle_x = sReg[Roll] / 32768.0f * 180.0f;
            IMUHelper::angle_y = sReg[Roll + 1] / 32768.0f * 180.0f;
            IMUHelper::angle_z = sReg[Roll + 2] / 32768.0f * 180.0f;

            if (IS_VALID_FLOAT(IMUHelper::angle_x) && IS_VALID_FLOAT(IMUHelper::angle_y) && IS_VALID_FLOAT(IMUHelper::angle_z))
            {
                IMUHelper::s_cDataUpdate &= ~ANGLE_UPDATE;
            }
        }

        // // Update magnetometer data
        if (IMUHelper::s_cDataUpdate & MAG_UPDATE)
        {

            IMUHelper::h_x = float(sReg[HX]);
            IMUHelper::h_y = float(sReg[HY]);
            IMUHelper::h_z = float(sReg[HZ]);

            if (IS_VALID_FLOAT(IMUHelper::h_x) && IS_VALID_FLOAT(IMUHelper::h_y) && IS_VALID_FLOAT(IMUHelper::h_z))
            {
                IMUHelper::s_cDataUpdate &= ~MAG_UPDATE;
            }
        }
    }
}
void IMUHelper::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case AX:
        case AY:
        case AZ:
            IMUHelper::s_cDataUpdate |= ACC_UPDATE;
            break;
        case GX:
        case GY:
        case GZ:
            IMUHelper::s_cDataUpdate |= GYRO_UPDATE;
            break;
        case HX:
        case HY:
        case HZ:
            IMUHelper::s_cDataUpdate |= MAG_UPDATE;
            break;
        case Roll:
        case Pitch:
        case Yaw:
            IMUHelper::s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            IMUHelper::s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

void IMUHelper::Delayms(uint16_t ucMs)
{
    usleep(ucMs * 500);
}

void IMUHelper::AutoScanSensor(unsigned char *dev)
{
    int i, iRetry;
    unsigned char cBuff[1];

    for (i = 1; i < 10; i++)
    {
        serial_close(fd);
        IMUHelper::s_iCurBaud = IMUHelper::c_uiBaud[i];
        fd = serial_open(dev, IMUHelper::c_uiBaud[i]);

        iRetry = 2;
        do
        {
            IMUHelper::s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            IMUHelper::Delayms(200);
            while (serial_read_data(fd, cBuff, 1))
            {
                WitSerialDataIn(cBuff[0]);
            }
            if (IMUHelper::s_cDataUpdate != 0)
            {
                printf("%d baud find sensor\r\n\r\n", IMUHelper::c_uiBaud[i]);
                return;
            }
            iRetry--;
        } while (iRetry);
    }
    printf("can not find sensor\r\n");
    printf("please check your connection\r\n");
}

std::string IMUHelper::format_timestamp(const builtin_interfaces::msg::Time &stamp)
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