/*************所有的头文件，全局变量，宏定义*************/
#include<motor_serial/motorSerialCtrl.h>
/*******************都在这个头文件中********************/


void motorSerialPort::startMotor(void)
{
    this->initSerial(this->portName, this->baudRate);
    this->set_vel(0,0);

    ROS_WARN("Motro enable, %s open",this->portName.c_str());
}

void motorSerialPort::shutdownMotor(void)
{
    
    this->set_vel(0,0);
    ros::Duration(0.01).sleep();
    this->serialPort.close();
    ROS_WARN("Motro disable, %s shutdown",this->portName.c_str());
    
}


void motorSerialPort::initSerial(std::string portName, int baudRate)
{
    ROS_WARN("Motor Serial Port:%s",portName.c_str());
    ROS_WARN("Motor Serial Baudrate%d",baudRate);

    this->serialPort.setPort(portName);

    this->serialPort.setBaudrate(baudRate);

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
        ROS_INFO("/dev/ttyS0 is opened");
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }


}




void motorSerialPort::set_vel(int motor1_vel, int motor2_vel)
{
    u_int16_t u_motor1_vel;
    u_int16_t u_motor2_vel;
    if(motor1_vel>=0)
    {
        if(!(this->motor1Mode > 0))
        {
            uint8_t set_mode[8]={0x01, 0x06, 0x20, 0x00, 0x00, 0x01};
            uint16_t CRC = this->factory_crc16(set_mode,6);
            set_mode[6] = CRC%0x100;
            set_mode[7] = CRC/0x100;

            this->serialPort.write(set_mode,sizeof(set_mode)/sizeof(set_mode[0]));
            ros::Duration(0.02).sleep();
        }
        this->motor1Mode = 1;
        
    }
    else if(motor1_vel<0)
    {   
        if(!(this->motor1Mode < 0))
        {
            uint8_t set_mode[8]={0x01, 0x06, 0x20, 0x00, 0x00, 0x02};
            uint16_t CRC = this->factory_crc16(set_mode,6);
            set_mode[6] = CRC%0x100;
            set_mode[7] = CRC/0x100;
            this->serialPort.write(set_mode,sizeof(set_mode)/sizeof(set_mode[0]));
            ros::Duration(0.02).sleep();
        }
        this->motor1Mode = -1;
    }


    
    if(motor2_vel<=0)
    {
        if(!(this->motor2Mode < 0))
        {
            uint8_t set_mode[8]={0x02, 0x06, 0x20, 0x00, 0x00, 0x02};
            uint16_t CRC = this->factory_crc16(set_mode,6);
            set_mode[6] = CRC%0x100;
            set_mode[7] = CRC/0x100;
            this->serialPort.write(set_mode,sizeof(set_mode)/sizeof(set_mode[0]));
            ros::Duration(0.02).sleep();
        }
        this->motor2Mode = -1;
    }
    else if(motor2_vel>0)
    {
        if(!(this->motor2Mode > 0))
        {
            uint8_t set_mode[8]={0x02, 0x06, 0x20, 0x00, 0x00, 0x01};
            uint16_t CRC = this->factory_crc16(set_mode,6);
            set_mode[6] = CRC%0x100;
            set_mode[7] = CRC/0x100;
            this->serialPort.write(set_mode,sizeof(set_mode)/sizeof(set_mode[0]));
            ros::Duration(0.02).sleep();
        }
        this->motor2Mode = 1;
    }



    u_motor1_vel = abs(motor1_vel*15);
    u_motor2_vel = abs(motor2_vel*15);

    
    uint8_t set_vel1[8]={0x01, 0x06, 0x20, 0x01};
    set_vel1[4] = u_motor1_vel/0x100;
    set_vel1[5] = u_motor1_vel%0x100;

    uint16_t CRC1 = this->factory_crc16(set_vel1,6);
    set_vel1[6] = CRC1%0x100;
    set_vel1[7] = CRC1/0x100;

    



    uint8_t set_vel2[8]={0x02, 0x06, 0x20, 0x01};
    set_vel2[4] = u_motor2_vel/0x100;
    set_vel2[5] = u_motor2_vel%0x100;

    uint16_t CRC2 = this->factory_crc16(set_vel2,6);
    set_vel2[6] = CRC2%0x100;
    set_vel2[7] = CRC2/0x100;

    this->serialPort.write(set_vel1,sizeof(set_vel1)/sizeof(set_vel1[0]));
    ros::Duration(0.015).sleep();
    this->serialPort.write(set_vel2,sizeof(set_vel2)/sizeof(set_vel2[0]));
    ros::Duration(0.015).sleep();
}



int16_t motorSerialPort::factory_crc16 ( uint8_t *bufData, uint16_t buflen)
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


void motorSerialPort::encoder_order(void)
{
    // 0x01 43 50 04 51 04 28 97 //读 0x5004 和 0x5104 的值
    uint8_t encode1[8]={0x01, 0x03, 0x30, 0x13, 0x00, 0x02, 0x3A, 0xCE};
    this->serialPort.write(encode1,sizeof(encode1)/sizeof(encode1[0]));
    ros::Duration(0.03).sleep();

    uint8_t encode2[8]={0x02, 0x03, 0x30, 0x13, 0x00, 0x02, 0x3A, 0xFD};
    this->serialPort.write(encode2,sizeof(encode2)/sizeof(encode2[0]));
    ros::Duration(0.03).sleep();

}


void motorSerialPort::CarBreak(void)
{
    this->motor1Mode = 0;
    this->motor2Mode = 0;

    uint8_t encode1[8]={0x01, 0x06, 0x20, 0x00, 0x00, 0x09, 0x42, 0x0c};
    this->serialPort.write(encode1,sizeof(encode1)/sizeof(encode1[0]));
    ros::Duration(0.03).sleep();

    uint8_t encode2[8]={0x02, 0x06, 0x20, 0x00, 0x00, 0x09, 0x42, 0x3F};
    this->serialPort.write(encode2,sizeof(encode2)/sizeof(encode2[0]));
    ros::Duration(0.03).sleep();
}