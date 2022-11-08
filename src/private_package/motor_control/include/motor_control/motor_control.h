#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include<ros/ros.h>
#include<serial/serial.h>
#include<iostream>
#include<geometry_msgs/Twist.h>
#include<signal.h>
#include<turtlesim/Pose.h>


#define PI 3.141592653589793


class motor_control
{
    public:

        motor_control(std::string portName, int baudRate);
        
        
        void startMotor(ros::NodeHandle &nh);
        void shutdownMotor(void);
        
        // 如果你不知道你在干什么,请不要调用这个函数
        // 注意:仅在第一次配置电机时使用,首先打开EEPROM写保护, 将模式设置为速度模式,然后关机, 
        // 检查一下开机时是不是速度模式了. 如果完成设置,再将EEPROM写保护关掉.
        void change_motor_mode(uint8_t mode);
        

    private:
        serial::Serial serialPort;
        serial::Timeout Timeout = serial::Timeout::simpleTimeout(100);
        std::string portName;

        ros::NodeHandle* _nh;
        ros::Subscriber _vel_sub;
        void receive_vel(const geometry_msgs::Twist::ConstPtr& msg);

        double wheel_diatance;
        double wheel_diameter;

        int baudRate;
        void encoder_order(void);
        int16_t factory_crc16 (uint8_t *bufData, uint16_t buflen);
        void initSerial();
        void shutdownSerial();
        void enable_motor(bool enable);
        void set_vel(uint16_t motor1_vel, uint16_t motor2_vel);



        // 注意:仅在第一次配置电机时使用,首先打开EEPROM写保护, 将模式设置为速度模式,然后关机, 
        // 检查一下开机时是不是速度模式了. 如果完成设置,再将EEPROM写保护关掉.
        void set_motor_mode(uint8_t mode);
        void enable_EEPROM(bool enable);
};
#endif   //_MOTOR_CONTROL_H