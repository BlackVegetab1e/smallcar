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
        ROS_INFO("%s is opened", port.c_str());
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
    set_arm_angle[2] = arm_level;
    set_arm_angle[3] = check;
    ROS_INFO("%d %d %d %d %d",set_arm_angle[0],set_arm_angle[1], set_arm_angle[2],set_arm_angle[3],set_arm_angle[4]);
    this->serialPort.write(set_arm_angle,sizeof(set_arm_angle)/sizeof(set_arm_angle[0]));
}

void arm_ctrl::set_lock(const int &level)
{
    
    uint8_t set_lock_angle[5]={0xAA, 0x00, 0x00, 0x00, 0x99};
    u_int8_t mode = 0x01;
    u_int8_t lock_level = 0;
    u_int8_t check = mode^lock_level;

    if(level>0)
    {
        lock_level = 0x01;
    }
    else
    {
        lock_level = 0x00;
    }
    check = mode^lock_level;
    set_lock_angle[1] = mode;
    set_lock_angle[2] = lock_level;
    set_lock_angle[3] = check;
    ROS_INFO("%d %d %d %d %d",set_lock_angle[0],set_lock_angle[1], set_lock_angle[2],set_lock_angle[3],set_lock_angle[4]);
    this->serialPort.write(set_lock_angle,sizeof(set_lock_angle)/sizeof(set_lock_angle[0]));
}



void arm_ctrl::shutdown_arm(void)
{
    this->serialPort.close();
}
