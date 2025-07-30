#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        ROS_INFO("ros::ok() is true");
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN("ros::ok() is false, exiting...");
    return 0;
}