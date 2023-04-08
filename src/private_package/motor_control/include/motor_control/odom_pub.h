#ifndef _ODOM_PUB_H
#define _ODOM_PUB_H
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
#include "tf2_ros/transform_broadcaster.h"
#include<geometry_msgs/TwistWithCovariance.h>
#include<std_msgs/Bool.h>

#define PI 3.141592653589793


class odom_pub
{
    public:
        odom_pub(std::string portName, int baudRate);
        void start_odom(ros::NodeHandle &nh);
        void shutdown_odom(void);

    private:
        ros::NodeHandle* _nh;
        serial::Serial _serialPort;
        serial::Timeout _Timeout = serial::Timeout::simpleTimeout(100);
        std::string _portName;
        int _baudRate;
        

        ros::Publisher _odom_publisher;
        tf2_ros::TransformBroadcaster broadcaster;

        std::string _odom_topic;
        int _odom_rate;
        double _wheelRadius;
        double _wheelDistance;
        int _oneCircleEncoder;

        int _last_left_encoder;
        int _last_right_encoder;
        bool _inited;
        double _th;
        double _x;
        double _y;
        double _last_time;

        int16_t factory_crc16 (uint8_t *bufData, uint16_t buflen);

        void pub_odom(uint16_t left, uint16_t right);
        void initSerial();
        void order_odom(const ros::TimerEvent& time_event);
        void encoder_order();
};


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


#endif   //_ODOM_PUB_H