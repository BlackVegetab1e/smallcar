#ifndef _MCU_COM_H
#define _MCU_COM_H

#include<iostream>
#include<serial/serial.h>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<tf/tf.h>
#include<geometry_msgs/Quaternion.h>
#include<signal.h>
#include<std_msgs/Float32.h>

#define PI 3.14159265358979323846264338327

class mcu_com
{
    public:

        mcu_com(std::string portName, int baudRate)
        {
            this->portName = portName;
            this->baudRate = baudRate;
        }
        void startMCU(void);
        void shutdownMCU(void);
        serial::Serial serialPort;
    private:

        std::string portName;
        int baudRate;
        serial::Timeout Timeout = serial::Timeout::simpleTimeout(100);
        int16_t factory_crc16 (uint8_t *bufData, uint16_t buflen);
        void initSerial(std::string portName, int baudRate);
        void getImuData(void);
        void pubImu(sensor_msgs::Imu);
        void pubVoltage(double volt);
        ros::Publisher ImuPublisher;
        ros::Publisher VoltagePublisher;
        ros::NodeHandle _n;

        
};


double I_ang_x;
double I_ang_y;
double I_ang_z;
double last_time;


boost::array<double, 9> covariance_li = {{-1,0,0,0,0,0,0,0,0}};
boost::array<double, 9> covariance_or = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-9}};
boost::array<double, 9> covariance_an = {{-1,0,0,0,0,0,0,0,0}};





#endif