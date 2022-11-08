#include<motor_control/odom_pub.h>


void mySignitHandler(int sig)  
{
    ROS_ERROR("Motor: Ctrl+c detected");
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_pub_node");
    ros::NodeHandle n;
    signal(SIGINT, mySignitHandler);


    serial::Serial serialPort;
    serial::Timeout Timeout = serial::Timeout::simpleTimeout(100);

    serialPort.setPort("/dev/ttyS1");

    serialPort.setBaudrate(115200);

    serialPort.setTimeout(Timeout);
    
    try
    {
        serialPort.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port.");
    }

    if(serialPort.isOpen())
    {
        ROS_INFO("%s is opened @ %d", "/dev/ttyS1",115200 );
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }


    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = serialPort.available();
        if(n>0)
        {
            uint8_t buffer[1024];
            n = serialPort.read(buffer, n);
            for(int i=0; i<n; i++)
            {
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
        }
        
    }




    return 0;
}
