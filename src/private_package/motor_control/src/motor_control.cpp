#include<motor_control/motor_control.h>


motor_control::motor_control(std::string portName, int baudRate)
{
    this->portName = portName;
    this->baudRate = baudRate;
    this->initSerial();
}

void motor_control::startMotor(ros::NodeHandle &nh)
{
    ros::param::get("wheel_diatance", this->wheel_diatance);
    ros::param::get("wheel_diameter", this->wheel_diameter);
    this->set_vel(0,0);
    this->enable_motor(false);
    this->set_vel(0,0);
    this->enable_motor(true);
    
    ROS_INFO("Motor Started");
    this->_nh = &nh;
    this->_vel_sub = this->_nh->subscribe("/cmd_vel", 10, &motor_control::receive_vel, this);

    ros::spin();
}

void motor_control::receive_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // ROS_INFO("%.3f %.3f", msg->linear.x, msg->angular.z);
    double V_right = (msg->linear.x + msg->angular.z * this->wheel_diatance)/2; //电机转速与小车速度和角速度关系
    double V_left  = (msg->linear.x - msg->angular.z * this->wheel_diatance)/2;

    double N_right = 60*V_right/(PI * this->wheel_diameter);  //换算成每秒转速
    double N_left  = 60*V_left /(PI * this->wheel_diameter);

    uint16_t right_speed = (uint16_t) N_right;
    uint16_t left_speed = (uint16_t) N_left;
    this->set_vel(left_speed, right_speed);
    // ROS_INFO("%d %d", left_speed, right_speed);


}


void motor_control::initSerial()
{

    this->serialPort.setPort(this->portName);

    this->serialPort.setBaudrate(this->baudRate);

    this->serialPort.setTimeout(this->Timeout);
    
    try
    {
        this->serialPort.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port.");
    }

    if(this->serialPort.isOpen())
    {
        ROS_INFO("%s is opened @ %d", this->portName.c_str(),this->baudRate );
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }

}


void motor_control::shutdownSerial()
{
    this->serialPort.close();
    ROS_WARN("%s is closed", this->portName.c_str());
}


void motor_control::set_vel(uint16_t motor1_vel, uint16_t motor2_vel)
{

    uint8_t send_msg[12] = {0x01, 0x44, 0x23, 0x18, 0x33, 0x18, 0x00 ,0x00 ,0x00 ,0x00};
    send_msg[6] = motor1_vel / 0x100;
    send_msg[7] = motor1_vel % 0x100;
    send_msg[8] = motor2_vel / 0x100;
    send_msg[9] = motor2_vel % 0x100;

    uint16_t crc16_check= this->factory_crc16(send_msg, 10);
    send_msg[10] = crc16_check%0x100;
    send_msg[11] = crc16_check/0x100;
    this->serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ros::Duration(0.003).sleep();

}


void motor_control::shutdownMotor(void)
{
    this->set_vel(0,0);
    this->enable_motor(false);
    ROS_WARN("motor disable");
    this->shutdownSerial();
}


void motor_control::encoder_order(void)
{
    enable_EEPROM(true);

    uint8_t send_msg[8] = {0x01, 0x06, 0x33, 0x28, 0x00, 0x01};

    uint16_t crc16_check= this->factory_crc16(send_msg, 6);
    send_msg[6] = crc16_check%0x100;
    send_msg[7] = crc16_check/0x100;
    this->serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ros::Duration(0.1).sleep();


    enable_EEPROM(true);
}


void motor_control::enable_motor(bool enable)
{

    uint8_t send_msg[12] = {0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00 ,0x01 ,0x00 ,0x01};
    send_msg[7] = enable?0x01:0x00;
    send_msg[9] = enable?0x01:0x00;
    uint16_t crc16_check= this->factory_crc16(send_msg, 10);
    send_msg[10] = crc16_check%0x100;
    send_msg[11] = crc16_check/0x100;
    this->serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ros::Duration(0.05).sleep();

}


int16_t motor_control::factory_crc16 (uint8_t *bufData, uint16_t buflen)
{
    uint16_t TCPCRC = 0xffff;
    uint16_t POLYNOMIAL = 0xa001;
    uint8_t i, j;
   
    for (i = 0; i < buflen; i++)
    {
        TCPCRC ^= bufData[i];
        for (j = 0; j < 8; j++)
        {
            if ((TCPCRC & 0x0001) != 0)
            {
                TCPCRC >>= 1;
                TCPCRC ^= POLYNOMIAL;
            }
            else
            {
                TCPCRC >>= 1;
            }
        }
    }
    return TCPCRC;
}


void motor_control::change_motor_mode(uint8_t mode)
{
    this->enable_EEPROM(true);
    this->set_motor_mode(mode);
    
    this->enable_EEPROM(true);
    // this->enable_EEPROM(false);
}


void motor_control::set_motor_mode(uint8_t mode)
{
    uint8_t send_msg[8] = {0x01, 0x06, 0x21, 0x02, 0x00, 0x01};
    send_msg[5] = mode;
    uint16_t crc16_check= this->factory_crc16(send_msg, 6);
    send_msg[6] = crc16_check%0x100;
    send_msg[7] = crc16_check/0x100;
    this->serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ros::Duration(0.1).sleep();

    uint8_t send_msg1[8] = {0x01, 0x06, 0x31, 0x02, 0x00, 0x01};
    send_msg1[5] = mode;
    uint16_t crc16_check1= this->factory_crc16(send_msg1, 6);
    send_msg1[6] = crc16_check1%0x100;
    send_msg1[7] = crc16_check1/0x100;
    this->serialPort.write(send_msg1, sizeof(send_msg1)/sizeof(send_msg1[0]));
    ROS_WARN("now mode is %d", mode);
    ros::Duration(0.1).sleep();
}


void motor_control::enable_EEPROM(bool enable)
{
    uint8_t send_msg[8] = {0x01, 0x06, 0x45, 0x0A, 0x00, 0x01};
    send_msg[5] = enable?0x01:0x00;
    uint16_t crc16_check= this->factory_crc16(send_msg, 6);
    send_msg[6] = crc16_check%0x100;
    send_msg[7] = crc16_check/0x100;
    this->serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ROS_WARN("EEPROM is now %d to write", enable);
    ros::Duration(0.1).sleep();
}