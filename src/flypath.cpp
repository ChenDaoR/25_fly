#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "pid.hpp"
#include <vector>


// 变量
struct Point
{
   float x, y, z;
};

// 定义路径点
std::vector<Point> waypoints = {
    {0.0, 0.0, 0.5},
    {0.5, 0.0, 0.5},
    {0.5, 0.5, 0.5},
    {0.0, 0.0, 0.5}
};


const double TIME = 7; // 每个点停留的时间
mavros_msgs::State current_state; // 状态
geometry_msgs::PoseStamped cuposition; // 当前位置
geometry_msgs::PoseStamped Goalposition; // 目标位置
int flag = 0; // 标记位
pid cx, cy, cz; // 控制器
int t = 0;

// 回调函数 更新状态
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 回调函数 更新位置
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cuposition = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flypath");
    ros::NodeHandle nh;

    // 状态订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 速度发布
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    // 位置订阅
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // 控制频率
    ros::Rate rate(200);

    // 等待 FCU 连接
    ROS_INFO("Waiting for FCU...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 设置初始目标位置
    Goalposition.pose.position.x = waypoints[0].x;
    Goalposition.pose.position.y = waypoints[0].y;
    Goalposition.pose.position.z = waypoints[0].z;

    // 时间步长
    cz.set_dt(0.005);
    cy.set_dt(0.005);
    cx.set_dt(0.005);

    // 初始值
    cz.setvalue(4, 0, 0, 5);
    cx.setvalue(0.3, 0, 0, 1);
    cy.setvalue(0.3, 0, 0, 1);

    // 输出限幅
    cz.set_maxoutput(5);
    cx.set_maxoutput(1);
    cy.set_maxoutput(1);

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    // 发送几个初始速度点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_setPoint = ros::Time::now();
    int index = 0;

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    last_setPoint = ros::Time::now();
                }

                last_request = ros::Time::now();
            }
        }

        if((ros::Time::now() - last_setPoint > ros::Duration(TIME)) && (index < waypoints.size()))
        {
            ROS_INFO("Point set");
            Goalposition.pose.position.x = waypoints[index].x;
            Goalposition.pose.position.y = waypoints[index].y;
            Goalposition.pose.position.z = waypoints[index].z;
            index++;
            last_setPoint = ros::Time::now();
        }


        if(cuposition.pose.position.z > 0.4)    cz.setvalue(0.5,0,0,1.5);
        cx.setGoal(Goalposition.pose.position.x);
        cy.setGoal(Goalposition.pose.position.y);
        cz.setGoal(Goalposition.pose.position.z);
        vel.linear.x = cx.compute(cuposition.pose.position.x);
        vel.linear.y = cy.compute(cuposition.pose.position.y);
        vel.linear.z = cz.compute(cuposition.pose.position.z);

        local_vel_pub.publish(vel);


        if( t >= 100)
        {
            t = 0;
            ROS_INFO("Now position x: %.3f,y: %.3f,z: %.3f",cuposition.pose.position.x,cuposition.pose.position.y,cuposition.pose.position.z);   
        }
        else    t++;    


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}