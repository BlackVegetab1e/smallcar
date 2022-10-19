#include<ros/ros.h>
#include<serial/serial.h>
#include<Arm_Ctrl/arm_ctrl.h>


// bode port
arm_ctrl::arm_ctrl(int bode, std::string port)
{
    this->bode = bode;
    this->port = port;
    this->init_port(bode, port);
}
void arm_ctrl::init_port(const int &bode, const std::string &port)
{
    ROS_WARN("Arm Serial Port:%s",port.c_str());
    ROS_WARN("Arm Serial Baudrate%d",bode);

    this->serialPort.setPort(port);

    this->serialPort.setBaudrate(bode);

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
        ROS_INFO("%s is opened", port);
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }
}


void arm_ctrl::set_arm_angle(const int &level)
{
    uint8_t set_arm_angle[5]={0xAA, 0x00, 0x00, 0x00, 0x99};
    u_int8_t mode = 0x00;
    u_int8_t arm_level = level;
    u_int8_t check = mode^arm_level;
    set_arm_angle[1] = mode;
    set_arm_angle[2] = level;
    set_arm_angle[3] = check;

    this->serialPort.write(set_arm_angle,sizeof(set_arm_angle)/sizeof(set_arm_angle[0]));
}

void arm_ctrl::set_lock(const int &level)
{
    uint8_t set_lock_angle[5]={0xAA, 0x00, 0x00, 0x00, 0x99};
    u_int8_t mode = 0x11;
    u_int8_t lock_level = level;
    u_int8_t check = mode^lock_level;
    set_lock_angle[1] = mode;
    set_lock_angle[2] = level;
    set_lock_angle[3] = check;
    this->serialPort.write(set_lock_angle,sizeof(set_lock_angle)/sizeof(set_lock_angle[0]));
}



void arm_ctrl::shutdown_arm(void)
{
    this->serialPort.close();
}
