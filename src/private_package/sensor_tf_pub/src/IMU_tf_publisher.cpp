#include<iostream>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "IMU_tf_publisher");
    ros::NodeHandle n;

    tf2_ros::StaticTransformBroadcaster Imu_broadcaster;
    geometry_msgs::TransformStamped imu_trans;
    imu_trans.header.stamp = ros::Time::now();
    imu_trans.header.frame_id = "base_footprint";
    imu_trans.child_frame_id = "imu_raw";

    imu_trans.transform.translation.x = 0.008;
    imu_trans.transform.translation.y = 0.002;
    imu_trans.transform.translation.z = 0.0;
    imu_trans.transform.rotation.x = 0.0;
    imu_trans.transform.rotation.y = 0.0;
    imu_trans.transform.rotation.z = 0.0;
    imu_trans.transform.rotation.w = 1.0;

    Imu_broadcaster.sendTransform(imu_trans);  





    ros::spin();




}

