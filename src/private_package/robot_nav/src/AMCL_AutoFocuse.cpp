#include<ros/ros.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<std_srvs/Empty.h>

ros::ServiceClient *FocusOrderPointer;


void startFocusing(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& InitPose)
{
    // ROS_ERROR("IN");
    for(int i = 0 ; i< 25; i++)
    {
        ROS_INFO("%d", i);
        std_srvs::Empty message;
        FocusOrderPointer->call(message);
        ros::Duration(0.1).sleep();
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AMCL_AutoFocuse");
    ros::NodeHandle n;

    ros::ServiceClient FocusOrder;
    FocusOrder = n.serviceClient<std_srvs::Empty>("/request_nomotion_update");
    ros::service::waitForService("/request_nomotion_update");
    FocusOrderPointer = &FocusOrder;
    

    ros::Subscriber initPose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",100, startFocusing);
    ros::spin();

}

