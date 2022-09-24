/*************所有的头文件，全局变量，宏定义*************/
#include <mcu_rec/mcu_com.h>
/*******************都在这个头文件中********************/



double init_ang_z = 999.999;
double add_ang_z = 0.0;

void mcu_com::getImuData(void)
{

    // 注意，这里不能直接使用IMU的角度，
    // 因为IMU的角度是融合了磁力计的,所以在遇到磁场干扰时误差特别大!!
    sensor_msgs::Imu pubData;
    uint16_t u_acc_x, u_acc_y, u_acc_z;
    uint16_t u_ang_v_x, u_ang_v_y, u_ang_v_z;
    uint16_t u_voltage;
    int16_t s_acc_x, s_acc_y, s_acc_z;
    int16_t s_ang_x, s_ang_y, s_ang_z;
    int16_t s_ang_v_x, s_ang_v_y, s_ang_v_z;
    int16_t s_voltage;
    
    double acc_x, acc_y, acc_z;
    double ang_v_x, ang_v_y, ang_v_z;
    double voltage;
    double ang_z;

    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = this->serialPort.available();
        if(n>=21)
        {
            uint8_t buffer[1024];
            n = this->serialPort.read(buffer, n);

            

            
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                
                if (buffer[i] == 0x7B && buffer[i+21] == 0x7D)
                {
                    // for(int j = i; j<=i+21; j++)
                    // {
                    //     std::cout << std::hex << (buffer[j] & 0xff) << " ";
                    // }
                    


                    u_acc_x = buffer[i+1]*0x100 + buffer[i+2];
                    u_acc_y = buffer[i+3]*0x100 + buffer[i+4];
                    u_acc_z = buffer[i+5]*0x100 + buffer[i+6];

                    u_ang_v_x = buffer[i+7]*0x100 + buffer[i+8];
                    // ROS_ERROR("i+7:%d:%d    i+8:%d:%d  ",i+7, buffer[i+7],i+8, buffer[i+8]);
                    u_ang_v_y = buffer[i+9]*0x100 + buffer[i+10];
                    u_ang_v_z = buffer[i+11]*0x100 + buffer[i+12];

                    u_voltage = buffer[i+19]*0x100 + buffer[i+20];
                    

                    s_acc_x = u_acc_x;
                    s_acc_y = u_acc_y;
                    s_acc_z = u_acc_z;
                    s_ang_v_x = u_ang_v_x;
                    s_ang_v_y = u_ang_v_y;
                    s_ang_v_z = u_ang_v_z;
                    s_voltage = u_voltage;
                    
                    acc_x = (double(s_acc_x)/32768.0)*16.0;
                    acc_y = (double(s_acc_y)/32768.0)*16.0;
                    acc_z = (double(s_acc_z)/32768.0)*16.0;

     

                    ang_v_x = (double(s_ang_v_x)/32768.0)*2000.0;
                    ang_v_y = (double(s_ang_v_y)/32768.0)*2000.0;
                    ang_v_z = (double(s_ang_v_z)/32768.0)*2000.0;

                    voltage = (double(s_voltage))/1000.0;

                    add_ang_z += ang_v_z/20;

                    // ROS_ERROR("%d",s_voltage);

                    // ROS_WARN("%d %d %d",buffer[i+7],buffer[i+8],u_ang_v_x );
                    // ROS_ERROR("%d %d %f",u_ang_v_x,s_ang_v_x,ang_v_x );

                    // ROS_INFO("acc:%f,%f,%f",acc_x,acc_y,acc_z);
                    // ROS_INFO("ang_v:%f,%f,%f",ang_v_x,ang_v_y,ang_v_z);


                    ang_z = add_ang_z;

                    if(ang_z>180.0)
                    {
                        ang_z -= 360.0;
                    }
                    if(ang_z<-180.0)
                    {
                        ang_z += 360.0;
                    }




                    pubData.header.frame_id ="imu_raw";
                    pubData.header.stamp = ros::Time::now();
                    pubData.angular_velocity.x = ang_v_x;
                    pubData.angular_velocity.y = ang_v_y;
                    pubData.angular_velocity.z = ang_v_z;
                    pubData.linear_acceleration.x = acc_x;
                    pubData.linear_acceleration.y = acc_y;
                    pubData.linear_acceleration.z = acc_z;
                    
                  
                    
                    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, ang_z*PI/180);
                    pubData.orientation.x = orientation.x;
                    pubData.orientation.y = orientation.y;
                    pubData.orientation.z = orientation.z;
                    pubData.orientation.w = orientation.w;


                    pubData.orientation_covariance = covariance_or;
                    pubData.angular_velocity_covariance = covariance_an;
                    pubData.linear_acceleration_covariance = covariance_li;



                    this->pubImu(pubData);
                    this->pubVoltage(voltage);
                    

                }
            }
            
            // loop_rate.sleep();
        }
    }
        
}


void mcu_com::startMCU(void)
{
    this->initSerial(this->portName, this->baudRate);
    ROS_WARN("Mcu enable, %s open",this->portName.c_str());
    ros::Duration(0.5).sleep();

    std::string pubName;
    ros::param::get("imu_topic_name",pubName);
    this->ImuPublisher = this->_n.advertise<sensor_msgs::Imu>(pubName, 50);
    this->VoltagePublisher = this->_n.advertise<std_msgs::Float32>("/voltage_V",50);

  
    last_time = ros::Time::now().toSec();
    this->getImuData();
    
}


void mcu_com::shutdownMCU(void)
{
    this->serialPort.close();

    ROS_WARN("Mcu disable, %s shutdown",this->portName.c_str());
    
}


void mcu_com::initSerial(std::string portName, int baudRate)
{
    ROS_WARN("MCU Serial Port:%s",portName.c_str());
    ROS_WARN("MCU Serial Baudrate%d",baudRate);

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
        ROS_INFO("/dev/ttyS1 is opened");
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }


}


void mcu_com::pubImu(sensor_msgs::Imu rawData)
{
    this->ImuPublisher.publish(rawData);
}

void mcu_com::pubVoltage(double volt)
{
    std_msgs::Float32 voltP;
    voltP.data = volt;
    this->VoltagePublisher.publish(voltP);
}




int16_t mcu_com::factory_crc16 ( uint8_t *bufData, uint16_t buflen)
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

