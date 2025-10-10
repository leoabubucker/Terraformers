#ifndef SRC_IMU_PUBSUB_INCLUDE_IMU_PUBSUB_IMU_HELPER_H_
#define SRC_IMU_PUBSUB_INCLUDE_IMU_PUBSUB_IMU_HELPER_H_

extern "C"{
    #include "serial.h"
    #include "wit_c_sdk.h"
    #include "REG.h"

}

#include <stdint.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include "builtin_interfaces/msg/time.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <unordered_set>
#include<bits/stdc++.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"  

#include "imu_msgs/msg/imu_data.hpp"
#include <iostream>
#include <fstream>

#include <map>
#include <chrono>
#include <array>
//compile this code on the microcontroller before running
#define SHM_KEY 12345

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
#define IS_VALID_FLOAT(val) (!isnan(val))
#define IS_VALID_INT(val)   (val != 0)

class IMUHelper{
    public:
        static float acc_x;
        static float acc_y;
        static float acc_z;
        static float gyro_x;
        static float gyro_y;
        static float gyro_z;
        static float angle_x;
        static float angle_y;
        static float angle_z;
        static float h_x;
        static float h_y;
        static float h_z;

        static int initSensor();
        static void loopIMU(int fd);
        static std::string formatTimestamp(const builtin_interfaces::msg::Time &stamp);
    
    private:
        static int fd, s_iCurBaud;
        static volatile char s_cDataUpdate;
        
        const static int c_uiBaud[];
        
        static void AutoScanSensor(unsigned char* dev);
        static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
        static void Delayms(uint16_t ucMs);
        static int add_to_memory();
        static unsigned char *sensorPath;

};

#endif // SRC_IMU_PUBSUB_INCLUDE_IMU_PUBSUB_IMU_HELPER_H_
