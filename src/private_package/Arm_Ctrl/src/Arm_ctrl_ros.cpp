#include<ros/ros.h>
#include<signal.h>
#include<Arm_Ctrl/arm_ctrl.h>

#include<Arm_Ctrl/arm_service.h>


arm_ctrl* my_arm_ctrl;

void mySignitHandler(int sig)
{
    my_arm_ctrl->shutdown_arm();
    ros::shutdown();

}


bool process_Arm(Arm_Ctrl::arm_service::Request& req, Arm_Ctrl::arm_service::Response& resp)
{
    if(req.type == "arm")
    {
        my_arm_ctrl->set_arm_angle(req.value);
    }
    else if(req.type == "lock")
    {
        my_arm_ctrl->set_lock(req.value);
    }
    return true;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Arm_ctrl_node");
    ros::NodeHandle n;

    signal(SIGINT, mySignitHandler);

    int baudRate = 0;
    std::string port;
    ros::param::get("Arm_baudRate", baudRate);
    ros::param::get("Arm_port", port);
    
    arm_ctrl arm(baudRate, port);
    my_arm_ctrl = &arm;

    

    ros::ServiceServer arm_service = n.advertiseService("Arm_actions", process_Arm);

    ros::spin();
}