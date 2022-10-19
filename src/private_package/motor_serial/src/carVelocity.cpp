/*************所有的头文件，全局变量，宏定义*************/
#include<motor_serial/motorSerialCtrl.h>
/*******************都在这个头文件中********************/


bool breaking = false;

// 用于接受ctrl+c信号，停止程序运行。
void mySignitHandler(int sig)  
{
    ROS_ERROR("Motor: Ctrl+c detected");
    portArray->shutdownMotor();
    ros::shutdown();

}


void breakGet(std_msgs::Bool breaksig)
{
    if(breaksig.data == true)
    {
        breaking = true;
        portArray->CarBreak();
    }
    else
    {
        if(breaking == true)
        {
            portArray->set_vel(0,0);
        }
        breaking = false;
    }
}


// 接收速度的回调函数，收到要求的速度，并且控制电机运转。
void recVel(const geometry_msgs::Twist::ConstPtr& velReceive) 
{
    if(!breaking)
    {
        double lineX = velReceive->linear.x;
        double angularZ = velReceive->angular.z;
        double vel_left =  lineX-angularZ*wheelDistance/2;
        double vel_right = lineX+angularZ*wheelDistance/2;
        int rpm_left = int((30/PI)*vel_left/wheelRadius);
        int rpm_right = int((30/PI)*vel_right/wheelRadius);
        portArray->set_vel(rpm_left,-rpm_right);
        // ROS_INFO("%.3f   %.3f",vel_left, vel_right);
    }
    else
    {
        portArray->CarBreak();
    }
    
}


// 接收电机的里程计信息，并且转换，发送到/odom话题上
void orderOdom(const ros::TimerEvent& event)
{
    portArray->encoder_order();
    ros::Duration(0.01).sleep();
    //获取缓冲区内的字节数
    
    size_t n = portArray->serialPort.available();
    if(n>=12)
    {
        uint8_t buffer[1024];
        //读出数据
        n = portArray->serialPort.read(buffer, n);
  
        for(int i=0; i<n-6; i++){
            if (buffer[i] == 0x01){
                if(buffer[i+1] == 0x03){
                    if(buffer[i+2] == 0x04 ){
                        
                        long long leftEncoder = buffer[i+3]*0x1000000+buffer[i+4]*0x10000+buffer[i+5]*0x100+buffer[i+6];
                        if(!left_inited)
                        {
                            last_leftEncoder = leftEncoder;
                            lase_time = ros::Time::now().toSec();
                            left_inited = true;
                            ROS_INFO("Left odom init, start working!");
                            return;
                        }
                        
                        delta_left_pulse = -(int)leftEncoder + (int)last_leftEncoder;
                        
                        if (delta_left_pulse>300000)
                        {
                            delta_left_pulse = maxencoder - delta_left_pulse;
                        }
                        else if(delta_left_pulse<-300000)
                        {
                            delta_left_pulse = - maxencoder - delta_left_pulse;
                        }
                        
                        last_leftEncoder = leftEncoder;
                        
                        }}}
        }

        for(int i=0; i<n-6; i++){
            if (buffer[i] == 0x02){
                if(buffer[i+1] == 0x03){
                    if(buffer[i+2] == 0x04 ){
                    
                        long long rightEncoder = buffer[i+3]*0x1000000+buffer[i+4]*0x10000+buffer[i+5]*0x100+buffer[i+6];
                        if(!right_inited)
                        {
                            last_rightEncoder = rightEncoder;
                            lase_time = ros::Time::now().toSec();
                            right_inited = true;
                            ROS_INFO("Right odom init, start working!");
                            return;
                        }
                        
                        delta_right_pulse = -(int)rightEncoder + (int)last_rightEncoder;
                        
                        if (delta_right_pulse>300000)
                        {
                            delta_right_pulse = maxencoder - delta_right_pulse;
                        }
                        else if(delta_right_pulse<-300000)
                        {
                            delta_right_pulse = - maxencoder - delta_right_pulse;
                        }
                        
                        last_rightEncoder = rightEncoder;
                        }}}

        }
        ROS_INFO("left$%lld$ right#%lld#", last_leftEncoder, last_rightEncoder);



    }
}


void pubOdom(const ros::TimerEvent& event)
{

    double wheelCircle = 2 * PI * wheelRadius;
    double leftDistance  =  -(((double)delta_left_pulse) /((double)oneCircleEncoder)) * wheelCircle;
    double rightDistance = (((double)delta_right_pulse)/((double)oneCircleEncoder)) * wheelCircle;
    
    double dxy_ave = ((double)leftDistance + (double)rightDistance) / 2;		 //中心点移动距离
    double dth = double(rightDistance - leftDistance) / wheelDistance; //中心点转动角度
    

    double delta_x = dxy_ave * cos(th + (dth / 2.0));
    double delta_y = dxy_ave * sin(th + (dth / 2.0));
    double delta_th = dth;
    // ROS_INFO("dxy_ave:%.3f, th:%.3f, dth:%.3f,delta_y%.3f, y:%.3f", dxy_ave,th, dth, delta_y);

    x += delta_x; // base_footprint与odom坐标系的偏差
    y += delta_y;
    th += delta_th;
    if(th>PI)
    {
        th -= 2*PI;
    }
    else if (th<-PI)
    {
        th += 2*PI;
    }
    double nowTime =ros::Time::now().toSec();
    double dt = nowTime - lase_time;
    lase_time = ros::Time::now().toSec();



    double vxy = dxy_ave / dt;		 //线速度
    double vth = dth / dt;			 //角速度

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);    //转换成四元数

    // static tf2_ros::TransformBroadcaster broadcaster;
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = ros::Time::now();
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_footprint";
    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;
    

    // broadcaster.sendTransform(odom_trans);    //发布odom和base_footprint的tf数

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
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
    // ROS_ERROR("%.3f %.3f",x,y);
    pubOdomArray->publish(odom);    //发布odom数据

    
                

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "carVelocity");
    ros::NodeHandle nh;

    signal(SIGINT, mySignitHandler);


    std::string serialPort;
    int baudRate;
    ros::param::get("Port",serialPort);
    ros::param::get("Baudrate",baudRate);
    ros::param::get("wheelRadius",wheelRadius);
    ros::param::get("wheelDistance",wheelDistance);

    motorSerialPort Motor(serialPort, baudRate);
    Motor.startMotor();
    portArray = &Motor;



    double odomRate;
    ros::param::get("odomRate",odomRate);
    double odomDurition = 1/odomRate;
    ROS_INFO("odom rate:%.1f", odomRate);
    ros::Timer odomOrder = nh.createTimer(ros::Duration(odomDurition),orderOdom);

    ros::Timer odomPub = nh.createTimer(ros::Duration(odomDurition),pubOdom);

    
    std::string pubName;
    ros::param::get("odomTopicName", pubName);

    ros::Publisher  pubOdom = nh.advertise<nav_msgs::Odometry>(pubName, 50);
    pubOdomArray = &pubOdom;



    ros::Subscriber pub = nh.subscribe<geometry_msgs::Twist>("/smooth_cmd_vel",100, recVel);
    ros::Subscriber CarBreak = nh.subscribe<std_msgs::Bool>("/CarBreak",10, breakGet);
    ros::spin();
    return 0;


}
