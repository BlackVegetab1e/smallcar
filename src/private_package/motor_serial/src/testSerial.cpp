#include<ros/ros.h>
#include<serial/serial.h>
#include<iostream>





serial::Serial sp;
serial::Timeout to = serial::Timeout::simpleTimeout(100);

int16_t crc16 ( uint8_t *bufData, uint16_t buflen)
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_test");
    ros::NodeHandle n;
    
    sp.setPort("/dev/ttyS2");
    sp.setBaudrate(19200);
    sp.setTimeout(to);


    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if(sp.isOpen())
    {
        ROS_INFO("/dev/ttyS0 is opened");
    }
    else
    {
        return -1;
    }
   

    ros::Rate loop_rate(1);

    // uint8_t sp_buff[12] ={0x01, 0x03, 0x30, 0x13, 0x00, 0x02};
    // uint16_t CRC = crc16(sp_buff, 6);
    // sp_buff[6] = CRC%0x100;
    // sp_buff[7] = CRC/0x100;


    // sp.write(sp_buff,8);

    loop_rate.sleep();
    double begain = ros::Time().now().toSec();
    
    while(ros::ok())
    {
        // uint8_t sp_buff[12] ={0x02, 0x06, 0x20, 0x00, 0x00, 0x05};
        // uint8_t sp_buff[12] ={0x02, 0x06, 0x20, 0x05, 0x00, 0x02};
        // uint16_t CRC = crc16(sp_buff, 6);
        // sp_buff[6] = CRC%0x100;
        // sp_buff[7] = CRC/0x100;

        ros::Duration(0.1).sleep();

        // sp.write(sp_buff,8);

    
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            double end = ros::Time().now().toSec();

            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
        }
        // loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
    
    
    sp.close();
    return 0;



}



