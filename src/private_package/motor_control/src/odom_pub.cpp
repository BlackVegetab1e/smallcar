#include <motor_control/odom_pub.h>



odom_pub::odom_pub(std::string portName, int baudRate):
_portName(portName),
_baudRate(baudRate)
{
     _oneCircleEncoder = 5600;
     _odom_rate = 10;
     _wheelRadius = 0;
     _wheelDistance = 0;

     _last_left_encoder = 0;
     _last_right_encoder = 0;
     _inited = 0;
     _th = 0;
     _x = 0;
     _y = 0;
     _last_time = 0;

    this->initSerial();
}

void odom_pub::start_odom(ros::NodeHandle &nh)
{
    ros::param::get("odom_pub_topic", _odom_topic);
    ros::param::get("encoder_number", _oneCircleEncoder);
    ros::param::get("odom_rate", _odom_rate);
    ros::param::get("wheel_diameter", _wheelRadius);
    _wheelRadius/=2;
    ros::param::get("wheel_diatance",_wheelDistance);


    ros::Duration(0.2).sleep();
    double odomDurition = 1.0/_odom_rate;
    ROS_INFO("odom rate:%d", _odom_rate);

    _odom_publisher = nh.advertise<nav_msgs::Odometry>(_odom_topic, 20);
    ros::Timer odom_order = nh.createTimer(ros::Duration(odomDurition), &odom_pub::order_odom, this);
    ros::spin();
}

void odom_pub::shutdown_odom(void)
{
    _serialPort.close();
    ROS_WARN("%s is closed", this->_portName.c_str());
}

void odom_pub::order_odom(const ros::TimerEvent& time_event)
{
    encoder_order();
    size_t n = _serialPort.available();
    while(n<12 && ros::ok())
    {
        n = _serialPort.available();
    }

    if(n>10)
    {
        uint8_t buffer[1024];
        n = _serialPort.read(buffer, n);
        bool catchit = false;
        int first_byte_at;
        for(first_byte_at = 0; first_byte_at <n; first_byte_at++)
        {
            if(buffer[first_byte_at] = 0x01)
            {
                if(buffer[first_byte_at+1] == 0x43)
                {
                    if(buffer[first_byte_at+2] == 0x50 && buffer[first_byte_at+3] == 0x04 
                         && buffer[first_byte_at+4] == 0x51 && buffer[first_byte_at+5] == 0x04)
                    {
                        catchit = true;
                        break;
                    }
                }
            }
        }
        if(catchit)
        {
            uint16_t left_encoder  = buffer[first_byte_at+6]*0x100 + buffer[first_byte_at+7];
            uint16_t right_encoder = buffer[first_byte_at+8]*0x100 + buffer[first_byte_at+9];

            this->pub_odom(left_encoder, right_encoder);
        }
    }
        

}

void odom_pub::encoder_order()
{
    uint8_t send_msg[8] = {0x01, 0x43, 0x50, 0x04, 0x51, 0x04, 0x28 ,0x97};

    this->_serialPort.write(send_msg, sizeof(send_msg)/sizeof(send_msg[0]));
    ros::Duration(0.005).sleep();

}

int16_t odom_pub::factory_crc16 (uint8_t *bufData, uint16_t buflen)
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



void odom_pub::initSerial()
{

    this->_serialPort.setPort(this->_portName);

    this->_serialPort.setBaudrate(this->_baudRate);

    this->_serialPort.setTimeout(this->_Timeout);
    
    try
    {
        this->_serialPort.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port.");
    }

    if(this->_serialPort.isOpen())
    {
        ROS_INFO("%s is opened @ %d", this->_portName.c_str(),this->_baudRate );
    }
    else
    {
        ROS_ERROR("Unable to open port.");
    }

}

void odom_pub::pub_odom(uint16_t left, uint16_t right)
{

    
    if(!_inited)
    {
        _last_left_encoder = left;
        _last_right_encoder = right;
        _inited = true;
    }

    int delta_left_pulse;
    delta_left_pulse = -(int)left + (int)_last_left_encoder;
    

    if (delta_left_pulse>_oneCircleEncoder/2)
    {

        delta_left_pulse = -_oneCircleEncoder + delta_left_pulse;
    }
    else if(delta_left_pulse<-_oneCircleEncoder/2)
    {
        delta_left_pulse =  _oneCircleEncoder + delta_left_pulse;
    }
    
    

    _last_left_encoder = left;

    int delta_right_pulse;
    delta_right_pulse = (int)right - (int)_last_right_encoder;
    

    if (delta_right_pulse>_oneCircleEncoder/2)
    {
        delta_right_pulse = -_oneCircleEncoder + delta_right_pulse;
    }
    else if(delta_right_pulse<-_oneCircleEncoder/2)
    {
        delta_right_pulse = _oneCircleEncoder + delta_right_pulse;
    }
    // ROS_WARN("%d %d %d", right, _last_right_encoder, delta_right_pulse);

    _last_right_encoder = right;
    

    double wheelCircle = 2 * PI * _wheelRadius;
    double leftDistance  = (((double)delta_left_pulse) /((double)_oneCircleEncoder)) * wheelCircle;
    double rightDistance = (((double)delta_right_pulse)/((double)_oneCircleEncoder)) * wheelCircle;



    double dxy_ave = (leftDistance + rightDistance) / 2;		  //中心点移动距离
    double dth = (rightDistance - leftDistance) / _wheelDistance; //中心点转动角度
    
    

    double delta_x = dxy_ave * cos(_th + (dth / 2.0));
    double delta_y = dxy_ave * sin(_th + (dth / 2.0));

    
    

    _x += delta_x; // base_footprint与odom坐标系的偏差
    _y += delta_y;
        
    _th += dth;
    // ROS_INFO("%.3f %.3f",_th, PI/2);
    if(_th>PI)
    {
        _th -= 2*PI;
    }
    else if (_th<-PI)
    {
        _th += 2*PI;
    }
    double nowTime =ros::Time::now().toSec();
    double dt = nowTime - _last_time;
    _last_time = ros::Time::now().toSec();



    double vxy = dxy_ave / dt;		 //线速度
    double vth = dth / dt;			 //角速度

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);    //转换成四元数



    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom_combined";

    odom.pose.pose.position.x = _x;
    odom.pose.pose.position.y = _y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vxy;    //两轮差动小车只有线速度x
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    
    if (abs(vxy) <= 1e-5 && abs(vth) <= 1e-5)
    {
        odom.pose.covariance = ODOM_POSE_COVARIANCE2;
        odom.twist.covariance = ODOM_TWIST_COVARIANCE2;
    }
    else
    {
        odom.pose.covariance = ODOM_POSE_COVARIANCE;
        odom.twist.covariance = ODOM_TWIST_COVARIANCE;
    }
            

    // ROS_WARN("%ld %ld",delta_left_pulse,delta_right_pulse);
    _odom_publisher.publish(odom);    //发布odom数据
}