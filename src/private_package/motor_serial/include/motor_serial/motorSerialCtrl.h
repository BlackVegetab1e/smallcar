#ifndef _MOTORCTRL_H
#define _MOTORCTRL_H

#include<ros/ros.h>
#include<serial/serial.h>
#include<iostream>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>
#include<signal.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<geometry_msgs/TwistWithCovariance.h>
#include "tf2_ros/transform_broadcaster.h"
#include<std_msgs/Bool.h>

#define PI 3.141592653589793


class motorSerialPort
{
    public:

        motorSerialPort(std::string portName, int baudRate)
        {
            this->portName = portName;
            this->baudRate = baudRate;
        }
        void set_vel(int motor1_vel, int motor2_vel);
        void startMotor(void);
        void shutdownMotor(void);
        void encoder_order(void);
        void CarBreak(void);
        
        serial::Serial serialPort;
        
    private:
        int motor1Mode = 0;
        int motor2Mode = 0;

        std::string portName;
        int baudRate;
        serial::Timeout Timeout = serial::Timeout::simpleTimeout(100);
        int16_t factory_crc16 (uint8_t *bufData, uint16_t buflen);
        void initSerial(std::string portName, int baudRate);
        
};



motorSerialPort *portArray;

double wheelDistance;
double wheelRadius;


bool left_inited  = false;
bool right_inited = false;


const unsigned long  maxencoder = 0xFFFFFFFF;
int oneCircleEncoder = 440;
long long last_leftEncoder = 0x0000;
long long last_rightEncoder = 0x0000;
long delta_left_pulse = 0x00;
long delta_right_pulse = 0x00;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double lase_time;
tf::TransformBroadcaster *odom_broadcasterArray;
ros::Publisher *pubOdomArray;



boost::array<double, 36UL> ODOM_POSE_COVARIANCE = 
                        {1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3};

boost::array<double, 36UL> ODOM_POSE_COVARIANCE2 = 
                        {1e-9, 0, 0, 0, 0, 0, 
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-9};
boost::array<double, 36UL> ODOM_TWIST_COVARIANCE = 
                        {1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3};
boost::array<double, 36UL>  ODOM_TWIST_COVARIANCE2 = 
                        {1e-9, 0, 0, 0, 0, 0, 
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-9};




#endif