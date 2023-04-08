#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/String.h"
#include<nav_msgs/Odometry.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <std_msgs/Float64MultiArray.h>
#include<string>  
#include<sstream>
#include<iostream>
#include<fstream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"send_goal");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);

    double x_pos,y_pos,z_pos;
    x_pos=0.0;
    y_pos=0.0;
    z_pos = 0.0;
    std::ifstream input_file("/home/smallcar/smallCar/src/result.txt");
    if (!input_file.is_open()) {
        ROS_ERROR("Failed to open file.");
        return -1;
    }
    double x, y, z;
    input_file >> x >> y >> z;
    input_file.close();


    geometry_msgs::PointStamped boal;
    boal.header.stamp = ros::Time::now();
    boal.header.frame_id = "camera";
    boal.point.x=x_pos;
    boal.point.y=y_pos;
    boal.point.z=z_pos;

    geometry_msgs::PointStamped target_pose = buffer.transform(boal,"base_footprint");
	
    // ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("move_base/goal", 100);
    // /* 
    // PoseStamped代表一个三维空间中的姿态（位置和方向），其中header字段包含时间戳和坐标系信息，pose字段则包含位置和方向信息。
    // 而PointStamped代表一个三维空间中的点，其中header字段包含时间戳和坐标系信息，point字段则包含点的三个坐标值。
    // */
    // geometry_msgs::PoseStamped msg;
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "map";
    // msg.pose.position.x = target_pose.point.x;
    // msg.pose.position.y = target_pose.point.y;
    // msg.pose.position.z = target_pose.point.z;
    // msg.pose.orientation.x = 0;
    // msg.pose.orientation.y = 0;
    // msg.pose.orientation.z = 0;
    // msg.pose.orientation.w = 1;
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // 创建一个MoveBaseGoal对象
    move_base_msgs::MoveBaseGoal goal;

    // 设置目标点的位置
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target_pose.point.x;
    goal.target_pose.pose.position.y = target_pose.point.y;
    goal.target_pose.pose.position.z = 0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = qtn.getW();

    // 发送目标点到move_base
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // 等待move_base完成任务
    ac.waitForResult();

    // 输出move_base的执行结果
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the robot reached the goal");
    } else {
        ROS_INFO("The base failed to move to the goal for some reason");
    }
    // 发布消息
    // pub.publish(msg);
    ros::spin();
    return 0;
}
