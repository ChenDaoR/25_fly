#include <ros/ros.h>
#include "flight_ctrl/SetFlightTask.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "activate_takeoff_node");
    ros::NodeHandle nh;

    ros::Publisher takeoff_pub = nh.advertise<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask", 10);
    ROS_INFO("[activate_takeoff_node] Waiting for subscriber connection...");  // 新增

    // 等待订阅者连接（避免消息丢失）
    while (takeoff_pub.getNumSubscribers() == 0) {
        ros::Duration(0.5).sleep();
    }

    flight_ctrl::SetFlightTask msg;
    msg.FlightTask = "Takeoff";
    ROS_INFO("[activate_takeoff_node] Publishing Takeoff command");  // 新增

    takeoff_pub.publish(msg);
    ros::Duration(1.0).sleep();  // 确保消息发送

    return 0;
}