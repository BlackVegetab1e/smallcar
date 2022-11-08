#include<motor_control/motor_control.h>

motor_control sytron_motor("/dev/ttyS1", 115200);

void mySignitHandler(int sig)  
{
    ROS_ERROR("Motor: Ctrl+c detected");
    sytron_motor.shutdownMotor();
    ros::shutdown();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle n;
    signal(SIGINT, mySignitHandler);

    sytron_motor.startMotor(n);

    return 0;
}
