#include<motor_control/odom_pub.h>

odom_pub syntron_odom("/dev/ttyS1", 115200);

void mySignitHandler(int sig)  
{
    ROS_ERROR("Odom: Ctrl+c detected");
    syntron_odom.shutdown_odom();
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_pub_node");
    ros::NodeHandle n;
    signal(SIGINT, mySignitHandler);

    syntron_odom.start_odom(n);



    return 0;
}
