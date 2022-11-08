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
#include<geometry_msgs/TwistWithCovariance.h>
#include "tf2_ros/transform_broadcaster.h"
#include<std_msgs/Bool.h>

#define PI 3.141592653589793


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