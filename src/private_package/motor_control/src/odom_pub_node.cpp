#include<motor_control/odom_pub.h>

odom_pub * syntron_odom_ptr;

void mySignitHandler(int sig)  
{
    ROS_ERROR("Odom: Ctrl+c detected");
    syntron_odom_ptr->shutdown_odom();
    ros::shutdown();
}


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "odom_pub_node");


    ros::NodeHandle n;
    signal(SIGINT, mySignitHandler);



    odom_pub syntron_odom("/dev/ttyS1", 115200);
    syntron_odom_ptr = &syntron_odom;
    syntron_odom.start_odom(n);



    return 0;
}
