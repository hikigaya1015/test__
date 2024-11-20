#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
 
// 回调函数定义
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Received Odometry message: Position x: %f, y: %f, z: %f",
             msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z);
}
 
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("Received Imu message: Linear acceleration x: %f, y: %f, z: %f",
             msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
    // 可以添加更多字段的处理，例如角速度等
}
 
void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received LaserScan message: Number of ranges: %zu", msg->ranges.size());
    // 可以打印第一个范围或平均值等作为示例
    if (!msg->ranges.empty()) {
        ROS_INFO("First range: %f", msg->ranges[0]);
    }
}
 
void magfieldCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    ROS_INFO("Received MagneticField message: Magnetic field strength x: %f, y: %f, z: %f",
             msg->magnetic_field.x,
             msg->magnetic_field.y,
             msg->magnetic_field.z);
}
 
int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;
 
    // 订阅话题
    ros::Subscriber odomSub = nh.subscribe("/driver/encoder", 10, odometryCallback);
    ros::Subscriber imuSub1 = nh.subscribe("/driver/eul", 10, imuCallback);
    ros::Subscriber imuSub2 = nh.subscribe("/driver/imu", 10, imuCallback); // 注意：通常不会同时订阅两个IMU话题
    ros::Subscriber scanSub = nh.subscribe("/driver/scan", 10, laserscanCallback);
    ros::Subscriber magSub = nh.subscribe("/driver/mag", 10, magfieldCallback);
 
    // 处理ROS循环
    ros::spin();
 
    return 0;
}