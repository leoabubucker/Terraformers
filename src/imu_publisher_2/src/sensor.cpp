#include "imu_publisher_2/sensor.h"


int Sensor::fd, Sensor::s_iCurBaud = 9600;
volatile char Sensor::s_cDataUpdate = 0;
const int Sensor::c_uiBaud[] = {2400 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};
 
float Sensor::acc_x, Sensor::acc_y, Sensor::acc_z, Sensor::angle_x, Sensor::angle_y, Sensor::angle_z, Sensor::gyro_x, Sensor::gyro_y, Sensor::gyro_z, Sensor::h_x, Sensor::h_y, Sensor::h_z = -1.0;
    
void Sensor::initSensor(){
        Sensor::add_to_memory();
        // unsigned char *sensorPath;
        
        // if((Sensor::fd = serial_open(sensorPath , 9600)<0))
        // {
        //     printf("open %s fail\n", sensorPath);
        //     return;
        // }
        // else printf("open %s success\n", sensorPath);
        
        // float fAcc[3], fGyro[3], fAngle[3];
        // int i , ret;
        // unsigned char cBuff[1];
        
        // WitInit(WIT_PROTOCOL_NORMAL, 0x50);
        // WitRegisterCallBack(SensorDataUpdata);
        
        // // printf("\r\n********************** wit-motion Normal example  ************************\r\n");
        // AutoScanSensor(sensorPath);
        
        //     while(serial_read_data(Sensor::fd, cBuff, 1)){
        //         WitSerialDataIn(cBuff[0]);
        //     }
            
        //     // printf("\n");
        //     Sensor::Delayms(500);
            
        //     if(Sensor::s_cDataUpdate){
        //         // When data needs to be updated
        //         Sensor::add_to_memory();   
        //     }    
        // serial_close(fd);
        // return;    
    }

int Sensor::add_to_memory(){
        Sensor::acc_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::acc_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::acc_z = static_cast<float>(rand()) / RAND_MAX * 10.0f; 
        Sensor::angle_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::angle_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::angle_z = static_cast<float>(rand()) / RAND_MAX * 10.0f; 
        Sensor::gyro_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::gyro_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::gyro_z = static_cast<float>(rand()) / RAND_MAX * 10.0f; 
        Sensor::h_x = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::h_y = static_cast<float>(rand()) / RAND_MAX * 10.0f;
        Sensor::h_z = static_cast<float>(rand()) / RAND_MAX * 10.0f; 
            if (Sensor::s_cDataUpdate && false)
            {
                
                if (Sensor::s_cDataUpdate & ACC_UPDATE)
                {
                    Sensor::acc_x = sReg[AX] / 32768.0f * 16.0f;
                    Sensor::acc_y = sReg[AX + 1] / 32768.0f * 16.0f;
                    Sensor::acc_z = sReg[AX + 2] / 32768.0f * 16.0f;
                    
                    if (IS_VALID_FLOAT(Sensor::acc_x) && IS_VALID_FLOAT(Sensor::acc_y) && IS_VALID_FLOAT(Sensor::acc_z))
                    {
                        Sensor::s_cDataUpdate &= ~ACC_UPDATE;
                    }
                }
                
                // // Update gyroscope data
                if (Sensor::s_cDataUpdate & GYRO_UPDATE)
                {
                    Sensor::gyro_x = sReg[GX] / 32768.0f * 2000.0f;
                    Sensor::gyro_y = sReg[GX + 1] / 32768.0f * 2000.0f;
                    Sensor::gyro_z = sReg[GX + 2] / 32768.0f * 2000.0f;
                    
                    if (IS_VALID_FLOAT(Sensor::gyro_x) && IS_VALID_FLOAT(Sensor::gyro_y) && IS_VALID_FLOAT(Sensor::gyro_z))
                    {
                        Sensor::s_cDataUpdate &= ~GYRO_UPDATE;
                    }
                }
            
                // // Update angle data
                if (s_cDataUpdate & ANGLE_UPDATE)
                {
                    Sensor::angle_x = sReg[Roll] / 32768.0f * 180.0f;
                    Sensor::angle_y = sReg[Roll + 1] / 32768.0f * 180.0f;
                    Sensor::angle_z = sReg[Roll + 2] / 32768.0f * 180.0f;
                    
                    if (IS_VALID_FLOAT(Sensor::angle_x) && IS_VALID_FLOAT(Sensor::angle_y) && IS_VALID_FLOAT(Sensor::angle_z))
                    {
                        Sensor::s_cDataUpdate &= ~ANGLE_UPDATE;
                    }
                }
                
                // // Update magnetometer data
                if (Sensor::s_cDataUpdate & MAG_UPDATE)
                {
                    
                    Sensor::h_x = float(sReg[HX]);
                    Sensor::h_y = float(sReg[HY]);
                    Sensor::h_z = float(sReg[HZ]);
                    
                    if (IS_VALID_FLOAT(Sensor::h_x) && IS_VALID_FLOAT(Sensor::h_y) && IS_VALID_FLOAT(Sensor::h_z))
                    {
                        Sensor::s_cDataUpdate &= ~MAG_UPDATE;
                    }
                }
            }
        }
void Sensor::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
        {
            int i;
            for(i = 0; i < uiRegNum; i++)
            {
                switch(uiReg)
                {
                    case AX:
                    case AY:
                    case AZ:
                    Sensor::s_cDataUpdate |= ACC_UPDATE;
                    break;
                    case GX:
                    case GY:
                    case GZ:
                    Sensor::s_cDataUpdate |= GYRO_UPDATE;
                    break;
                    case HX:
                    case HY:
                    case HZ:
                    Sensor::s_cDataUpdate |= MAG_UPDATE;
                    break;
                    case Roll:
                    case Pitch:
                    case Yaw:
                    Sensor::s_cDataUpdate |= ANGLE_UPDATE;
                    break;
                    default:
                    Sensor::s_cDataUpdate |= READ_UPDATE;
                    break;
                }
                uiReg++;
            }
    }

void Sensor::Delayms(uint16_t ucMs)
    { 
        usleep(ucMs*500);
        }
        
void Sensor::AutoScanSensor(unsigned char* dev)
        {
            int i, iRetry;
            unsigned char cBuff[1];
            
            for(i = 1; i < 10; i++)
            {
                 serial_close(fd);
                 Sensor::s_iCurBaud = Sensor::c_uiBaud[i];
                 fd = serial_open(dev , Sensor::c_uiBaud[i]);
                
                 iRetry = 2;
                 do
                 {
                     Sensor::s_cDataUpdate = 0;
                    WitReadReg(AX, 3);
                     Sensor::Delayms(200);
                     while(serial_read_data(fd, cBuff, 1))
                     {
                         WitSerialDataIn(cBuff[0]);
                     }
                     if(Sensor::s_cDataUpdate != 0)
                     {
                         printf("%d baud find sensor\r\n\r\n", Sensor::c_uiBaud[i]);
                         return ;
                     }
                     iRetry--;
                 }while(iRetry);		
             }
            printf("can not find sensor\r\n");
            printf("please check your connection\r\n");
        }

    