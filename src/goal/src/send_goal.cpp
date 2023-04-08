#include"ros/ros.h"
#include <fstream>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Twist.h"
using namespace std;
template <class Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;    
}
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"send_goals_node");
    ros::NodeHandle nh;
    ros::Publisher pub =nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    geometry_msgs::Twist _goal_vel;
    ifstream fin;  
    int longth;
    bool goal_flag = 0;
    string line;
    double txt[2]={0.3,0.3};
    // txt[1]=1000;
    // while(abs(txt[1])>=0.1)// 有该文件
    // {   
    //     fin.open("/home/smallcar/smallCar/src/result.txt");
    //     int row = 0;
    //     while (getline (fin, line)) // line中不包括每行的换行符
    //     {
	// 	    if (row < 2) 
    //         {
	//             txt[row]=stringToNum<double>(line);
	// 	        row++;
    //     	} 
    //      }
    //      fin.close();
    //     if(txt[1]>0.1)
    //     {
    //         _goal_vel.angular.z=0.2;
    //         pub.publish(_goal_vel);
    //     }
    //     if(txt[1]<-0.1)
    //     {
    //         _goal_vel.angular.z=-0.2;
    //         pub.publish(_goal_vel);
    //     }
    //     ros::Duration( 0.5 );
    //     _goal_vel.angular.z=0;
    //     pub.publish(_goal_vel);
    // }
    ROS_INFO("读入的数据为x:%f,y:%f",txt[0],txt[1]);
    if(goal_flag)
    {
        geometry_msgs::TransformStamped ts;
        ts.header.frame_id = "map";
        ts.header.stamp=ros::Time::now();
        ts.child_frame_id="camera";
        ts.transform.translation.x=txt[0];
        ts.transform.translation.y=txt[1];
        ts.transform.translation.z=0;
        /*
            位姿信息中没有四元数，但是有个偏航角度，有因为其是2D没有翻滚和俯仰角，欧拉角为（0，0，theta）
        */
        tf2::Quaternion qtn;
        qtn.setRPY(0,0,0);
        ts.transform.rotation.w=qtn.getW();
        ts.transform.rotation.x=qtn.getX();
        ts.transform.rotation.y=qtn.getY();
        ts.transform.rotation.z=qtn.getZ();
        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer( ros::Duration( 5.0 ) )){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        move_base_msgs::MoveBaseGoal goal;

        // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
        goal.target_pose.pose.position.x = ts.transform.translation.x;
        goal.target_pose.pose.position.y =  ts.transform.translation.y;
        goal.target_pose.pose.orientation.z = ts.transform.rotation.z;  
        goal.target_pose.pose.orientation.w = ts.transform.rotation.w;  
        ROS_INFO(" Init success!!! ");
   
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal);
        ROS_INFO("Send Goal !!!" );
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The Goal achieved success !!!" );
        else
            ROS_WARN("The  Goal Planning Failed for some reason"); 
    }
    return 0;
}
