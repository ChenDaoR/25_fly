 #include <ros/ros.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/Twist.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/SetMode.h>
 #include <mavros_msgs/State.h>
 #include "dynamic_reconfigure/server.h"
 #include "flight_ctrl/dy_positionConfig.h"
 #include "pid.hpp"
 #include <vector>
 
 //变量
 mavros_msgs::State current_state; //状态
 geometry_msgs::PoseStamped cuposition; //位置
 geometry_msgs::PoseStamped Goalposition; // 目标位置

 double max = 0;
 int t = 0;
 pid cx,cy,cz; //控制器

//回调函数 更新状态
 void state_cb(const mavros_msgs::State::ConstPtr& msg)
 {
     current_state = *msg;
 }
 
 //回调函数 更新位置
 void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
 {
    cuposition = *msg;

    if(cuposition.pose.position.z > max)    max = cuposition.pose.position.z;

 }

  //回调函数 更新位置参数
 void update_position_cb(dy_position::dy_positionConfig& config, uint32_t level)
 {

    Goalposition.pose.position.x = config.p_x;
    Goalposition.pose.position.y = config.p_y;
    Goalposition.pose.position.z = config.p_z;

 }


 int main(int argc, char **argv)
 {
     ros::init(argc, argv, "dypo");
     ros::NodeHandle nh;
     
     //状态订阅
     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
             ("mavros/state", 10, state_cb);
     //速度发布
     ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
             ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
     //位置订阅
     ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
             ("mavros/local_position/pose", 10, position_cb);

     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
             ("mavros/cmd/arming");
     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
             ("mavros/set_mode");


     dynamic_reconfigure::Server<dy_position::dy_positionConfig> server;
     dynamic_reconfigure::Server<dy_position::dy_positionConfig>::CallbackType cbType;
     cbType = boost::bind(&update_position_cb, _1, _2);
     server.setCallback(cbType);
    
    //控制频率
     ros::Rate rate(200);

 
     // wait for FCU connection
     ROS_INFO("Waiting for FCU...");
     while(ros::ok() && !current_state.connected){
         ros::spinOnce();
         rate.sleep();
     }
     ROS_INFO("FCU OK!!!");

     //目标点
     Goalposition.pose.position.x = 0;
     Goalposition.pose.position.y = 0;
     Goalposition.pose.position.z = 0.1;

     //时间步长
     cz.set_dt(0.005);
     cy.set_dt(0.005);
     cx.set_dt(0.005);

     //初始值
     cz.setvalue(4,0,0,5);
     cx.setvalue(0.55,0,0,1);
     cy.setvalue(0.55,0,0,1);

     //输出限幅
     cz.set_maxoutput(5);
     cx.set_maxoutput(1);
     cy.set_maxoutput(1);

     
     geometry_msgs::Twist vel;
     vel.linear.x = 0;
     vel.linear.y = 0;
     vel.linear.z = 0;
 
     //send a few setpoints before starting
     ROS_INFO("Presend Points");
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

     ROS_INFO("Okay!!!");
 
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
                 }

                 last_request = ros::Time::now();
             }
         }

             

            if(cuposition.pose.position.z > 0.4)    cx.setvalue(0.55,0,0,1.5);
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
                ROS_INFO("%.3f",max);
            }
            else    t++;

         ros::spinOnce();
         rate.sleep();
     }
 
     return 0;
 }